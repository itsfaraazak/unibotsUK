/**
 * bt_game_node — C++ Behaviour Tree game controller.
 *
 * Subscribes:
 *   /spatial_memory/ball_map  (unibots_msgs/BallMap)
 *   /vision/balls             (unibots_msgs/BallArray)
 *   /odom/filtered            (nav_msgs/Odometry)
 *   /match/start              (std_msgs/Bool, latched)
 *   /intake/beam_broken       (std_msgs/Bool)
 *
 * Publishes:
 *   /game/target              (geometry_msgs/PoseStamped)
 *   /game/state               (std_msgs/String)   — only on state change
 *   /servo/command            (std_msgs/String)   — OPEN / CLOSE
 *   /game/ball_collected      (std_msgs/UInt32)   — track_id after capture
 *
 * Tree (Fallback = OR, Sequence = AND):
 *
 *   Sequence [ROOT — must be started]
 *   ├── Condition: match_started?
 *   └── Fallback [GAME]
 *       ├── Sequence [STOP]:   elapsed >= t_stop_s  → hold in place
 *       ├── Sequence [PARK]:   elapsed >= t_park_s  → navigate home wall
 *       ├── Sequence [DEPOSIT]: balls_held >= capacity
 *       │   ├── Action: NAV_HOME    — drive to home wall
 *       │   ├── Action: ALIGN_NET  — timeout-based alignment wait
 *       │   └── Action: DUMP       — servo sequence + reset counter
 *       └── Fallback [HUNT_OR_SEARCH]
 *           ├── Sequence [HUNT]: spatial_memory has a valid target
 *           │   ├── Condition: target exists
 *           │   └── Fallback [APPROACH_STRATEGY]
 *           │       ├── Sequence [CAPTURE]: ball in intake blind-spot
 *           │       │   ├── Condition: ball pixel_y > blindspot threshold
 *           │       │   └── Action: wait_beam → publish collected
 *           │       ├── Sequence [SERVO]: ball within close range
 *           │       │   ├── Condition: ball distance < approach_dist_cm
 *           │       │   └── Action: drive toward ball world position
 *           │       └── Action: APPROACH — drive to predicted ball position
 *           └── Action: SEARCH — rotate in place to scan arena
 */

#include <rclcpp/rclcpp.hpp>

#include <unibots_msgs/msg/ball_array.hpp>
#include <unibots_msgs/msg/ball_map.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using BallArray   = unibots_msgs::msg::BallArray;
using BallMap     = unibots_msgs::msg::BallMap;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Odometry    = nav_msgs::msg::Odometry;
using Bool        = std_msgs::msg::Bool;
using String      = std_msgs::msg::String;
using UInt32      = std_msgs::msg::UInt32;

// ──────────────────────────────────────────────────────────────────────────────
// Minimal Behaviour Tree framework (no external deps)
// ──────────────────────────────────────────────────────────────────────────────
enum class Status { SUCCESS, FAILURE, RUNNING };

struct BTNode {
    virtual Status tick() = 0;
    virtual ~BTNode() = default;
};

// Sequence: returns SUCCESS only if ALL children succeed in order.
struct Sequence : BTNode {
    std::vector<std::unique_ptr<BTNode>> children;
    Status tick() override {
        for (auto& c : children) {
            auto s = c->tick();
            if (s != Status::SUCCESS) return s;
        }
        return Status::SUCCESS;
    }
};

// Fallback: returns SUCCESS on the first child that does not fail.
struct Fallback : BTNode {
    std::vector<std::unique_ptr<BTNode>> children;
    Status tick() override {
        for (auto& c : children) {
            auto s = c->tick();
            if (s != Status::FAILURE) return s;
        }
        return Status::FAILURE;
    }
};

struct Condition : BTNode {
    std::function<bool()> fn;
    Status tick() override { return fn() ? Status::SUCCESS : Status::FAILURE; }
};

struct Action : BTNode {
    std::function<Status()> fn;
    Status tick() override { return fn(); }
};

// Helper factories.
static std::unique_ptr<BTNode> cond(std::function<bool()> fn)
{
    auto n = std::make_unique<Condition>();
    n->fn = std::move(fn);
    return n;
}
static std::unique_ptr<BTNode> act(std::function<Status()> fn)
{
    auto n = std::make_unique<Action>();
    n->fn = std::move(fn);
    return n;
}

// Variadic make_node: avoids initializer_list which requires copyable elements.
// Each child is individually moved into the composite node's children vector.
template<typename T, typename... Children>
static std::unique_ptr<BTNode> make_node(Children&&... kids)
{
    auto n = std::make_unique<T>();
    n->children.reserve(sizeof...(kids));
    (n->children.push_back(std::forward<Children>(kids)), ...);
    return n;
}

// ──────────────────────────────────────────────────────────────────────────────
// Home-wall lookup table (arena: 2 m × 2 m, SW corner = 0,0)
// ──────────────────────────────────────────────────────────────────────────────
struct HomeGoal { float x, y, yaw; };
static HomeGoal home_for_zone(const std::string& zone)
{
    if (zone == "south") return {1.0f, 0.20f, static_cast<float>(-M_PI_2)};
    if (zone == "east")  return {1.80f, 1.0f, 0.0f};
    if (zone == "west")  return {0.20f, 1.0f, static_cast<float>(M_PI)};
    return {1.0f, 1.80f, static_cast<float>(M_PI_2)}; // default: north
}

// ──────────────────────────────────────────────────────────────────────────────
static float wrap_angle(float a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

// ──────────────────────────────────────────────────────────────────────────────
class BtGameNode : public rclcpp::Node
{
public:
    BtGameNode()
    : Node("bt_game_node")
    {
        // ── Declare parameters ────────────────────────────────────────────
        declare_parameter("home_zone",              std::string("north"));
        declare_parameter("capacity",               6);
        declare_parameter("bt_frequency_hz",        20.0);
        declare_parameter("approach_dist_cm",       150.0);
        declare_parameter("servo_blindspot_frac",   0.88);
        declare_parameter("frame_height_px",        720);
        declare_parameter("search_yaw_step_deg",    30.0);
        declare_parameter("t_park_s",               170.0);
        declare_parameter("t_stop_s",               185.0);
        declare_parameter("dump_duration_s",        2.5);
        declare_parameter("near_wall_dist_m",       0.30);
        declare_parameter("align_timeout_s",        10.0);
        declare_parameter("align_standoff_m",       0.15);
        declare_parameter("min_track_frames",       3);
        declare_parameter("use_predicted_position", true);

        std::string zone     = get_parameter("home_zone").as_string();
        capacity_            = get_parameter("capacity").as_int();
        bt_freq_             = get_parameter("bt_frequency_hz").as_double();
        approach_dist_cm_    = static_cast<float>(get_parameter("approach_dist_cm").as_double());
        blindspot_frac_      = static_cast<float>(get_parameter("servo_blindspot_frac").as_double());
        frame_h_             = get_parameter("frame_height_px").as_int();
        search_step_rad_     = static_cast<float>(get_parameter("search_yaw_step_deg").as_double()
                                                  * M_PI / 180.0);
        t_park_s_            = get_parameter("t_park_s").as_double();
        t_stop_s_            = get_parameter("t_stop_s").as_double();
        dump_dur_s_          = get_parameter("dump_duration_s").as_double();
        near_wall_m_         = static_cast<float>(get_parameter("near_wall_dist_m").as_double());
        align_timeout_s_     = get_parameter("align_timeout_s").as_double();
        align_standoff_m_    = static_cast<float>(get_parameter("align_standoff_m").as_double());
        min_track_frames_    = get_parameter("min_track_frames").as_int();
        use_predicted_       = get_parameter("use_predicted_position").as_bool();

        HomeGoal hg = home_for_zone(zone);
        home_x_ = hg.x; home_y_ = hg.y; home_yaw_ = hg.yaw;

        // ── Subscriptions ─────────────────────────────────────────────────
        ball_map_sub_ = create_subscription<BallMap>(
            "/spatial_memory/ball_map", 10,
            [this](BallMap::SharedPtr m){ ball_map_ = m; });

        balls_sub_ = create_subscription<BallArray>(
            "/vision/balls", 10,
            [this](BallArray::SharedPtr m){ balls_ = m; });

        odom_sub_ = create_subscription<Odometry>(
            "/odom/filtered", 10,
            [this](Odometry::SharedPtr m){ on_odom(m); });

        start_sub_ = create_subscription<Bool>(
            "/match/start", rclcpp::QoS(1).transient_local(),
            [this](Bool::SharedPtr m){
                if (!match_started_ && m->data) {
                    match_started_  = true;
                    match_start_    = now();
                    set_state("STARTUP");
                    RCLCPP_INFO(get_logger(), "Match started!");
                }
            });

        beam_sub_ = create_subscription<Bool>(
            "/intake/beam_broken", 10,
            [this](Bool::SharedPtr m){ beam_broken_ = m->data; });

        // ── Publishers ────────────────────────────────────────────────────
        target_pub_    = create_publisher<PoseStamped>("/game/target",         10);
        state_pub_     = create_publisher<String>     ("/game/state",          10);
        servo_pub_     = create_publisher<String>     ("/servo/command",       10);
        collected_pub_ = create_publisher<UInt32>     ("/game/ball_collected", 10);

        // ── Build BT ──────────────────────────────────────────────────────
        tree_ = build_tree();

        // ── Tick timer ────────────────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / bt_freq_),
            [this](){ tree_->tick(); });

        RCLCPP_INFO(get_logger(),
            "BtGameNode up — home=%s (%.2f,%.2f) cap=%d",
            zone.c_str(), home_x_, home_y_, capacity_);
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────
    int    capacity_, frame_h_, min_track_frames_;
    double bt_freq_, t_park_s_, t_stop_s_, dump_dur_s_, align_timeout_s_;
    float  approach_dist_cm_, blindspot_frac_, near_wall_m_, align_standoff_m_;
    float  home_x_, home_y_, home_yaw_;
    float  search_step_rad_;
    bool   use_predicted_;

    // ── State ─────────────────────────────────────────────────────────────────
    bool   match_started_{false};
    bool   beam_broken_{false};
    bool   has_pose_{false};
    float  rx_{0.f}, ry_{0.f}, ryaw_{0.f};
    int    balls_held_{0};
    float  search_yaw_{0.f};
    std::string cur_state_;

    // ALIGN_NET timer.
    bool          align_active_{false};
    rclcpp::Time  align_start_{0,0,RCL_ROS_TIME};

    // DUMP timer.
    bool         dumping_{false};
    bool         dump_opened_{false};
    rclcpp::Time dump_start_{0,0,RCL_ROS_TIME};

    rclcpp::Time match_start_{0,0,RCL_ROS_TIME};

    // ── ROS I/O ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<BallMap>::SharedPtr   ball_map_sub_;
    rclcpp::Subscription<BallArray>::SharedPtr balls_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr  odom_sub_;
    rclcpp::Subscription<Bool>::SharedPtr      start_sub_;
    rclcpp::Subscription<Bool>::SharedPtr      beam_sub_;

    rclcpp::Publisher<PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<String>::SharedPtr      state_pub_;
    rclcpp::Publisher<String>::SharedPtr      servo_pub_;
    rclcpp::Publisher<UInt32>::SharedPtr      collected_pub_;

    rclcpp::TimerBase::SharedPtr              timer_;

    BallMap::SharedPtr   ball_map_;
    BallArray::SharedPtr balls_;

    std::unique_ptr<BTNode> tree_;

    // ── Helpers ───────────────────────────────────────────────────────────────

    void on_odom(Odometry::SharedPtr msg)
    {
        auto& p = msg->pose.pose.position;
        auto& q = msg->pose.pose.orientation;
        rx_ = static_cast<float>(p.x);
        ry_ = static_cast<float>(p.y);
        double siny = 2.0*(q.w*q.z + q.x*q.y);
        double cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
        ryaw_ = static_cast<float>(std::atan2(siny, cosy));
        has_pose_ = true;
    }

    double elapsed_s() const
    {
        if (!match_started_) return 0.0;
        return (now() - match_start_).seconds();
    }

    void set_state(const std::string& s)
    {
        if (s == cur_state_) return;
        cur_state_ = s;
        String msg;
        msg.data = s;
        state_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "State → %s", s.c_str());
    }

    void publish_goal(float x, float y, float yaw)
    {
        PoseStamped ps;
        ps.header.stamp    = now();
        ps.header.frame_id = "map";
        ps.pose.position.x = static_cast<double>(x);
        ps.pose.position.y = static_cast<double>(y);
        ps.pose.position.z = 0.0;
        // Convert yaw to quaternion (z-axis rotation).
        ps.pose.orientation.w = std::cos(static_cast<double>(yaw) / 2.0);
        ps.pose.orientation.z = std::sin(static_cast<double>(yaw) / 2.0);
        target_pub_->publish(ps);
    }

    void publish_servo(const std::string& cmd)
    {
        String msg;
        msg.data = cmd;
        servo_pub_->publish(msg);
    }

    float dist_to(float x, float y) const
    {
        float dx = rx_ - x, dy = ry_ - y;
        return std::sqrt(dx*dx + dy*dy);
    }

    bool has_target() const
    {
        if (!ball_map_) return false;
        const auto& t = ball_map_->selected_target;
        if (t.track_id == 0) return false;
        if (t.status == "COLLECTED") return false;
        return (t.status == "VISIBLE" || t.status == "OCCLUDED");
    }

    // Nearest visible ball with pixel_y > threshold (in camera blind spot).
    bool ball_in_blindspot() const
    {
        if (!balls_) return false;
        float thresh = blindspot_frac_ * static_cast<float>(frame_h_);
        for (const auto& b : balls_->balls) {
            if (b.pixel_y > thresh) return true;
        }
        return false;
    }

    // Nearest visible ball closer than approach_dist_cm.
    bool ball_in_servo_range() const
    {
        if (!balls_) return false;
        for (const auto& b : balls_->balls) {
            if (b.distance_cm < approach_dist_cm_) return true;
        }
        return false;
    }

    // World position of the nearest visible ball (for servo approach).
    std::optional<std::pair<float,float>> nearest_ball_world() const
    {
        if (!balls_ || !has_pose_) return std::nullopt;
        float best_d = 1e9f;
        std::optional<std::pair<float,float>> res;
        for (const auto& b : balls_->balls) {
            if (b.distance_cm < best_d) {
                best_d = b.distance_cm;
                float br  = b.bearing_deg * static_cast<float>(M_PI / 180.0);
                float dm  = b.distance_cm / 100.0f;
                float wx  = rx_ + dm * std::cos(ryaw_ + br);
                float wy  = ry_ + dm * std::sin(ryaw_ + br);
                res = {wx, wy};
            }
        }
        return res;
    }

    // ── BT Action implementations ─────────────────────────────────────────────

    Status stop_action()
    {
        set_state("STOP");
        publish_goal(rx_, ry_, ryaw_);   // hold current pose
        return Status::RUNNING;
    }

    Status park_action()
    {
        set_state("PARK");
        publish_goal(home_x_, home_y_, home_yaw_);
        return Status::RUNNING;   // keep navigating even after arrival (safe end state)
    }

    Status nav_home_action()
    {
        set_state("NAV_HOME");
        float d = dist_to(home_x_, home_y_);
        publish_goal(home_x_, home_y_, home_yaw_);
        if (d < near_wall_m_) return Status::SUCCESS;
        return Status::RUNNING;
    }

    Status align_net_action()
    {
        set_state("ALIGN_NET");
        if (!align_active_) {
            align_start_  = now();
            align_active_ = true;
        }
        double elapsed = (now() - align_start_).seconds();
        if (elapsed > align_timeout_s_) {
            align_active_ = false;
            return Status::SUCCESS;
        }
        // Approach to standoff position.
        float tx = home_x_ - align_standoff_m_ * std::cos(home_yaw_);
        float ty = home_y_ - align_standoff_m_ * std::sin(home_yaw_);
        publish_goal(tx, ty, home_yaw_);
        return Status::RUNNING;
    }

    Status dump_action()
    {
        set_state("DUMP");
        if (!dumping_) {
            dumping_    = true;
            dump_opened_= false;
            dump_start_ = now();
        }
        double elapsed = (now() - dump_start_).seconds();
        if (!dump_opened_) {
            publish_servo("OPEN");
            dump_opened_ = true;
        }
        if (elapsed < dump_dur_s_) return Status::RUNNING;
        // Sequence complete.
        publish_servo("CLOSE");
        balls_held_ = 0;
        dumping_    = false;
        return Status::SUCCESS;
    }

    Status wait_beam_action()
    {
        set_state("CAPTURE");
        if (!beam_broken_) return Status::RUNNING;
        // Beam broken → ball captured.
        if (ball_map_ && ball_map_->selected_target.track_id != 0) {
            UInt32 msg;
            msg.data = ball_map_->selected_target.track_id;
            collected_pub_->publish(msg);
        }
        ++balls_held_;
        RCLCPP_INFO(get_logger(), "Ball captured! held=%d", balls_held_);
        return Status::SUCCESS;
    }

    Status servo_approach_action()
    {
        set_state("SERVO");
        auto wp = nearest_ball_world();
        if (wp) publish_goal(wp->first, wp->second, ryaw_);
        return Status::RUNNING;
    }

    Status approach_action()
    {
        set_state("APPROACH");
        if (!ball_map_ || ball_map_->selected_target.track_id == 0) {
            return Status::FAILURE;
        }
        const auto& t = ball_map_->selected_target;
        float gx = use_predicted_ ? t.predicted_x : t.world_x;
        float gy = use_predicted_ ? t.predicted_y : t.world_y;
        // Face the ball as we approach.
        float dx = gx - rx_, dy = gy - ry_;
        float face_yaw = std::atan2(dy, dx);
        publish_goal(gx, gy, face_yaw);
        return Status::RUNNING;
    }

    Status search_action()
    {
        set_state("SEARCH");
        search_yaw_ = wrap_angle(search_yaw_ + search_step_rad_);
        publish_goal(rx_, ry_, search_yaw_);
        return Status::RUNNING;
    }

    // ── Build tree ────────────────────────────────────────────────────────────

    std::unique_ptr<BTNode> build_tree()
    {
        // Capture / Servo / Approach sub-tree (inner Fallback).
        auto capture_seq = make_node<Sequence>(
            cond([this]{ return ball_in_blindspot(); }),
            act ([this]{ return wait_beam_action(); })
        );
        auto servo_seq = make_node<Sequence>(
            cond([this]{ return ball_in_servo_range(); }),
            act ([this]{ return servo_approach_action(); })
        );
        auto approach_strategy = make_node<Fallback>(
            std::move(capture_seq),
            std::move(servo_seq),
            act([this]{ return approach_action(); })
        );

        // Hunt sequence: target exists → try approach strategy.
        auto hunt_seq = make_node<Sequence>(
            cond([this]{ return has_target(); }),
            std::move(approach_strategy)
        );

        // Hunt-or-search.
        auto hunt_or_search = make_node<Fallback>(
            std::move(hunt_seq),
            act([this]{ return search_action(); })
        );

        // Deposit sequence.
        auto deposit_seq = make_node<Sequence>(
            cond([this]{ return balls_held_ >= capacity_; }),
            act ([this]{ return nav_home_action(); }),
            act ([this]{ return align_net_action(); }),
            act ([this]{ return dump_action(); })
        );

        // Park sequence.
        auto park_seq = make_node<Sequence>(
            cond([this]{ return elapsed_s() >= t_park_s_; }),
            act ([this]{ return park_action(); })
        );

        // Stop sequence.
        auto stop_seq = make_node<Sequence>(
            cond([this]{ return elapsed_s() >= t_stop_s_; }),
            act ([this]{ return stop_action(); })
        );

        // Game fallback (runs when match is started).
        auto game_fb = make_node<Fallback>(
            std::move(stop_seq),
            std::move(park_seq),
            std::move(deposit_seq),
            std::move(hunt_or_search)
        );

        // Root: only tick game tree once match has started.
        auto root = make_node<Sequence>(
            cond([this]{ return match_started_; }),
            std::move(game_fb)
        );

        return root;
    }
};

// ──────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BtGameNode>());
    rclcpp::shutdown();
    return 0;
}
