/**
 * spatial_memory_node — persistent Kalman ball tracker with prediction.
 *
 * Subscribes:
 *   /vision/balls             (unibots_msgs/BallArray)
 *   /odom/filtered            (nav_msgs/Odometry)
 *   /game/ball_collected      (std_msgs/UInt32)  — track_id to mark collected
 *
 * Publishes:
 *   /spatial_memory/ball_map      (unibots_msgs/BallMap)
 *   /spatial_memory/prediction_error (std_msgs/Float32)  — re-acquisition error
 *   /spatial_memory/debug_markers (visualization_msgs/MarkerArray)
 */

#include <rclcpp/rclcpp.hpp>

#include <unibots_msgs/msg/ball_array.hpp>
#include <unibots_msgs/msg/ball_map.hpp>
#include <unibots_msgs/msg/world_ball.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using BallArray   = unibots_msgs::msg::BallArray;
using BallMap     = unibots_msgs::msg::BallMap;
using WorldBall   = unibots_msgs::msg::WorldBall;
using Odometry    = nav_msgs::msg::Odometry;
using Float32     = std_msgs::msg::Float32;
using UInt32      = std_msgs::msg::UInt32;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker      = visualization_msgs::msg::Marker;

// ──────────────────────────────────────────────────────────────────────────────
// BallTrack: Kalman state + lifecycle for one tracked ball.
// ──────────────────────────────────────────────────────────────────────────────
struct BallTrack {
    uint32_t    id;
    std::string type;       // "ping_pong" | "steel"
    std::string status;     // "VISIBLE" | "OCCLUDED" | "COLLECTED"
    int         missed_frames{0};
    int         seen_frames{0};

    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
    rclcpp::Time collected_at{0, 0, RCL_ROS_TIME};

    // Kalman state [px, py, vx, vy] in arena/map frame.
    Eigen::Vector4f x{Eigen::Vector4f::Zero()};
    Eigen::Matrix4f P{Eigen::Matrix4f::Identity()};

    // Previous measurement for motion gating.
    float prev_mx{0.0f}, prev_my{0.0f};
    bool  has_prev{false};
};

// ──────────────────────────────────────────────────────────────────────────────
class SpatialMemoryNode : public rclcpp::Node
{
public:
    SpatialMemoryNode()
    : Node("spatial_memory_node"), next_id_(1)
    {
        // ── Declare parameters ────────────────────────────────────────────
        declare_parameter("assoc_radius_m",          0.25);
        declare_parameter("max_occlusion_frames",    8);
        declare_parameter("pred_horizon_s",          0.5);
        declare_parameter("prediction_mode",         std::string("friction"));
        declare_parameter("kalman_q_pos",            0.01);
        declare_parameter("kalman_q_vel_ping_pong",  1.0);
        declare_parameter("kalman_q_vel_steel",      0.3);
        declare_parameter("kalman_r_pos",            0.05);
        declare_parameter("motion_gate_m",           0.03);
        declare_parameter("friction_coeff_ping_pong",0.15);
        declare_parameter("friction_coeff_steel",    0.05);
        declare_parameter("density_radius_m",        0.50);
        declare_parameter("density_bonus",           0.4);
        declare_parameter("min_confidence",          0.35);
        declare_parameter("publish_rate_hz",         20.0);
        declare_parameter("collect_expiry_s",        45.0);

        assoc_radius_    = get_parameter("assoc_radius_m").as_double();
        max_occ_frames_  = get_parameter("max_occlusion_frames").as_int();
        pred_horizon_    = static_cast<float>(get_parameter("pred_horizon_s").as_double());
        pred_mode_       = get_parameter("prediction_mode").as_string();
        q_pos_           = static_cast<float>(get_parameter("kalman_q_pos").as_double());
        q_vel_pp_        = static_cast<float>(get_parameter("kalman_q_vel_ping_pong").as_double());
        q_vel_steel_     = static_cast<float>(get_parameter("kalman_q_vel_steel").as_double());
        r_pos_           = static_cast<float>(get_parameter("kalman_r_pos").as_double());
        motion_gate_     = static_cast<float>(get_parameter("motion_gate_m").as_double());
        fric_pp_         = static_cast<float>(get_parameter("friction_coeff_ping_pong").as_double());
        fric_steel_      = static_cast<float>(get_parameter("friction_coeff_steel").as_double());
        density_radius_  = static_cast<float>(get_parameter("density_radius_m").as_double());
        density_bonus_   = static_cast<float>(get_parameter("density_bonus").as_double());
        min_conf_        = static_cast<float>(get_parameter("min_confidence").as_double());
        pub_rate_hz_     = get_parameter("publish_rate_hz").as_double();
        collect_expiry_  = get_parameter("collect_expiry_s").as_double();

        // ── Subscriptions ─────────────────────────────────────────────────
        balls_sub_ = create_subscription<BallArray>(
            "/vision/balls", 10,
            [this](BallArray::SharedPtr m){ on_balls(m); });

        odom_sub_ = create_subscription<Odometry>(
            "/odom/filtered", 10,
            [this](Odometry::SharedPtr m){ on_odom(m); });

        collected_sub_ = create_subscription<UInt32>(
            "/game/ball_collected", 10,
            [this](UInt32::SharedPtr m){ on_collected(m); });

        // ── Publishers ────────────────────────────────────────────────────
        map_pub_     = create_publisher<BallMap>("/spatial_memory/ball_map", 10);
        error_pub_   = create_publisher<Float32>("/spatial_memory/prediction_error", 10);
        markers_pub_ = create_publisher<MarkerArray>("/spatial_memory/debug_markers", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / pub_rate_hz_),
            [this](){ publish_tick(); });

        RCLCPP_INFO(get_logger(),
            "SpatialMemoryNode up — mode=%s horizon=%.2fs rate=%.0fHz",
            pred_mode_.c_str(), pred_horizon_, pub_rate_hz_);
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────
    double      assoc_radius_;
    int         max_occ_frames_;
    float       pred_horizon_;
    std::string pred_mode_;
    float       q_pos_, q_vel_pp_, q_vel_steel_, r_pos_;
    float       motion_gate_;
    float       fric_pp_, fric_steel_;
    float       density_radius_, density_bonus_, min_conf_;
    double      pub_rate_hz_;
    double      collect_expiry_;

    // ── State ─────────────────────────────────────────────────────────────────
    std::vector<BallTrack> tracks_;
    uint32_t               next_id_;
    bool                   has_pose_{false};
    float                  rx_{0.f}, ry_{0.f}, ryaw_{0.f};

    // ── ROS I/O ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<BallArray>::SharedPtr  balls_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<UInt32>::SharedPtr     collected_sub_;
    rclcpp::Publisher<BallMap>::SharedPtr       map_pub_;
    rclcpp::Publisher<Float32>::SharedPtr       error_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr   markers_pub_;
    rclcpp::TimerBase::SharedPtr                timer_;

    // ── Helpers ───────────────────────────────────────────────────────────────

    float q_vel(const BallTrack& t) const {
        return (t.type == "steel") ? q_vel_steel_ : q_vel_pp_;
    }

    float friction(const BallTrack& t) const {
        return (t.type == "steel") ? fric_steel_ : fric_pp_;
    }

    void kalman_predict(BallTrack& t, float dt)
    {
        Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
        Q(0,0) = q_pos_; Q(1,1) = q_pos_;
        Q(2,2) = q_vel(t); Q(3,3) = q_vel(t);

        t.x = F * t.x;
        t.P = F * t.P * F.transpose() + Q;
    }

    void kalman_update(BallTrack& t, float mx, float my, bool update_velocity)
    {
        Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
        H(0,0) = 1.f; H(1,1) = 1.f;

        Eigen::Matrix2f R = Eigen::Matrix2f::Zero();
        R(0,0) = r_pos_; R(1,1) = r_pos_;

        Eigen::Matrix2f S = H * t.P * H.transpose() + R;
        Eigen::Matrix<float, 4, 2> K = t.P * H.transpose() * S.inverse();

        if (!update_velocity) {
            K.row(2).setZero();
            K.row(3).setZero();
        }

        Eigen::Vector2f innov;
        innov << mx - t.x[0], my - t.x[1];

        t.x += K * innov;
        t.P = (Eigen::Matrix4f::Identity() - K * H) * t.P;
    }

    std::pair<float,float> predict_pos(const BallTrack& t) const
    {
        if (pred_mode_ == "none") {
            return {t.x[0], t.x[1]};
        } else if (pred_mode_ == "constant_velocity") {
            return {t.x[0] + t.x[2] * pred_horizon_,
                    t.x[1] + t.x[3] * pred_horizon_};
        } else {
            // Friction: per-step velocity decay.
            float fc = friction(t);
            float dt = static_cast<float>(1.0 / pub_rate_hz_);
            int   steps = static_cast<int>(pred_horizon_ / dt);
            float px = t.x[0], py = t.x[1];
            float vx = t.x[2], vy = t.x[3];
            float decay = 1.0f - fc * dt;
            for (int i = 0; i < steps; ++i) {
                px += vx * dt;
                py += vy * dt;
                vx *= decay;
                vy *= decay;
            }
            return {px, py};
        }
    }

    // Project bearing_deg + distance_m from robot pose to arena world frame.
    std::pair<float,float> project_world(float bearing_deg, float distance_m) const
    {
        float bearing_rad = bearing_deg * static_cast<float>(M_PI / 180.0);
        float wx = rx_ + distance_m * std::cos(ryaw_ + bearing_rad);
        float wy = ry_ + distance_m * std::sin(ryaw_ + bearing_rad);
        return {wx, wy};
    }

    // Return index of nearest track within assoc_radius_, or -1.
    int nearest_track(float wx, float wy, const std::string& type) const
    {
        int   best = -1;
        float best_d2 = static_cast<float>(assoc_radius_ * assoc_radius_);
        for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
            const auto& t = tracks_[i];
            if (t.type != type) continue;
            if (t.status == "COLLECTED") continue;
            float dx = t.x[0] - wx, dy = t.x[1] - wy;
            float d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        return best;
    }

    int count_nearby(int idx) const
    {
        int count = 0;
        float x0 = tracks_[idx].x[0], y0 = tracks_[idx].x[1];
        for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
            if (i == idx) continue;
            if (tracks_[i].status == "COLLECTED") continue;
            float dx = tracks_[i].x[0] - x0, dy = tracks_[i].x[1] - y0;
            if (std::sqrt(dx*dx + dy*dy) < density_radius_) ++count;
        }
        return count;
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────

    void on_odom(Odometry::SharedPtr msg)
    {
        auto& p = msg->pose.pose.position;
        auto& q = msg->pose.pose.orientation;
        rx_  = static_cast<float>(p.x);
        ry_  = static_cast<float>(p.y);
        // yaw from quaternion
        double siny = 2.0 * (q.w*q.z + q.x*q.y);
        double cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
        ryaw_    = static_cast<float>(std::atan2(siny, cosy));
        has_pose_ = true;
    }

    void on_collected(UInt32::SharedPtr msg)
    {
        uint32_t tid = msg->data;
        for (auto& t : tracks_) {
            if (t.id == tid) {
                t.status       = "COLLECTED";
                t.collected_at = now();
                RCLCPP_DEBUG(get_logger(), "Track %u marked COLLECTED", tid);
                return;
            }
        }
    }

    void on_balls(BallArray::SharedPtr msg)
    {
        if (!has_pose_) return;

        rclcpp::Time stamp = msg->header.stamp;
        std::vector<bool> matched(tracks_.size(), false);

        for (const auto& det : msg->balls) {
            if (det.confidence < min_conf_) continue;

            float dist_m = det.distance_cm / 100.0f;
            auto [wx, wy] = project_world(det.bearing_deg, dist_m);

            int idx = nearest_track(wx, wy, det.ball_type);

            if (idx >= 0) {
                // ── Update existing track ───────────────────────────────
                auto& t = tracks_[idx];

                // Measurement prediction error for re-acquired OCCLUDED tracks.
                if (t.status == "OCCLUDED") {
                    auto [px, py] = predict_pos(t);
                    float err = std::sqrt((px-wx)*(px-wx) + (py-wy)*(py-wy));
                    Float32 emsg;
                    emsg.data = err;
                    error_pub_->publish(emsg);
                }

                // Kalman predict with dt since last seen.
                float dt = static_cast<float>((stamp - t.last_seen).seconds());
                if (dt > 0.f && dt < 2.f) kalman_predict(t, dt);

                // Motion gate: only update velocity if ball moved enough.
                bool update_vel = true;
                if (t.has_prev) {
                    float ddx = wx - t.prev_mx, ddy = wy - t.prev_my;
                    update_vel = (std::sqrt(ddx*ddx + ddy*ddy) > motion_gate_);
                }
                kalman_update(t, wx, wy, update_vel);

                t.prev_mx     = wx; t.prev_my = wy; t.has_prev = true;
                t.status       = "VISIBLE";
                t.last_seen    = stamp;
                t.missed_frames= 0;
                ++t.seen_frames;
                matched[idx]   = true;

            } else {
                // ── Create new track ─────────────────────────────────────
                BallTrack nt;
                nt.id     = next_id_++;
                nt.type   = det.ball_type;
                nt.status = "VISIBLE";
                nt.last_seen = stamp;
                nt.seen_frames = 1;

                nt.x << wx, wy, 0.f, 0.f;
                nt.P = Eigen::Matrix4f::Identity() * 0.5f;

                nt.prev_mx = wx; nt.prev_my = wy; nt.has_prev = true;

                tracks_.push_back(std::move(nt));
                matched.push_back(true);
            }
        }

        // ── Mark unmatched tracks as OCCLUDED / expire ──────────────────
        rclcpp::Time now_t = now();
        for (int i = static_cast<int>(tracks_.size()) - 1; i >= 0; --i) {
            auto& t = tracks_[i];
            if (matched[static_cast<size_t>(i)]) continue;
            if (t.status == "COLLECTED") {
                // Expire old COLLECTED tracks so the BT can hunt them again later.
                double age = (now_t - t.collected_at).seconds();
                if (age > collect_expiry_) tracks_.erase(tracks_.begin() + i);
                continue;
            }
            ++t.missed_frames;
            if (t.missed_frames > max_occ_frames_) {
                tracks_.erase(tracks_.begin() + i);
            } else {
                t.status = "OCCLUDED";
            }
        }
    }

    // ── Publish tick ──────────────────────────────────────────────────────────

    void publish_tick()
    {
        BallMap bmap;
        bmap.header.stamp    = now();
        bmap.header.frame_id = "map";

        // Target selection: highest score among VISIBLE/OCCLUDED non-COLLECTED.
        int    best_idx = -1;
        float  best_score = -1.f;

        for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
            const auto& t = tracks_[i];
            if (t.status == "COLLECTED") continue;
            if (t.seen_frames < 1) continue;

            auto [px, py] = predict_pos(t);
            int density = count_nearby(i);

            float speed = std::sqrt(t.x[2]*t.x[2] + t.x[3]*t.x[3]);
            float conf  = (t.status == "VISIBLE") ? 1.0f : 0.5f;
            float score = conf * (1.f + density_bonus_ * static_cast<float>(density));

            WorldBall wb;
            wb.ball_type   = t.type;
            wb.world_x     = t.x[0];
            wb.world_y     = t.x[1];
            wb.confidence  = conf;
            wb.status      = t.status;
            wb.last_seen   = t.last_seen;
            wb.track_id    = t.id;
            wb.predicted_x = px;
            wb.predicted_y = py;
            wb.speed_m_s   = speed;
            wb.density     = density;
            bmap.balls.push_back(wb);

            if (score > best_score) {
                best_score = score;
                best_idx   = static_cast<int>(bmap.balls.size()) - 1;
            }
        }

        if (best_idx >= 0) {
            bmap.selected_target = bmap.balls[static_cast<size_t>(best_idx)];
        }

        map_pub_->publish(bmap);
        publish_markers(bmap);
    }

    void publish_markers(const BallMap& bmap)
    {
        MarkerArray ma;
        int marker_id = 0;

        for (const auto& wb : bmap.balls) {
            // Sphere at current position.
            Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp    = bmap.header.stamp;
            sphere.ns              = "balls";
            sphere.id              = marker_id++;
            sphere.type            = Marker::SPHERE;
            sphere.action          = Marker::ADD;
            sphere.pose.position.x = static_cast<double>(wb.world_x);
            sphere.pose.position.y = static_cast<double>(wb.world_y);
            sphere.pose.position.z = 0.04;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
            sphere.color.a = (wb.status == "VISIBLE") ? 1.0f : 0.4f;
            sphere.color.r = (wb.ball_type == "steel") ? 0.7f : 1.0f;
            sphere.color.g = (wb.ball_type == "ping_pong") ? 0.5f : 0.7f;
            sphere.color.b = 0.0f;
            sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
            ma.markers.push_back(sphere);

            // Arrow to predicted position.
            if (wb.predicted_x != wb.world_x || wb.predicted_y != wb.world_y) {
                Marker arrow;
                arrow.header = sphere.header;
                arrow.ns     = "predictions";
                arrow.id     = marker_id++;
                arrow.type   = Marker::ARROW;
                arrow.action = Marker::ADD;
                geometry_msgs::msg::Point p0, p1;
                p0.x = static_cast<double>(wb.world_x);
                p0.y = static_cast<double>(wb.world_y);
                p0.z = 0.04;
                p1.x = static_cast<double>(wb.predicted_x);
                p1.y = static_cast<double>(wb.predicted_y);
                p1.z = 0.04;
                arrow.points = {p0, p1};
                arrow.scale.x = 0.01; arrow.scale.y = 0.02; arrow.scale.z = 0.02;
                arrow.color.a = 0.8f; arrow.color.r = 1.0f; arrow.color.g = 1.0f;
                arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
                ma.markers.push_back(arrow);
            }
        }

        markers_pub_->publish(ma);
    }
};

// ──────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpatialMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
