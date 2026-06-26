// bt_game_node — Unibots 2026 high-level game controller.
//
// A BehaviorTree.CPP v4 tree (bt/game_tree.xml) is ticked at tick_rate_hz. The
// tree's leaf nodes (include/unibots_bt/bt_nodes.hpp) read and write one shared
// GameContext placed on the blackboard. This node owns only the ROS plumbing:
// it fills the context from subscriptions and exposes the publish helpers.
//
// Motion output is ONLY /game/target (a goal pose); the MPC/APF controller turns
// it into wheel commands. This node never publishes /cmd_vel.

#include "unibots_bt/bt_nodes.hpp"
#include "unibots_bt/game_context.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace unibots_bt {

using Bool = std_msgs::msg::Bool;

class BtGameNode : public rclcpp::Node {
public:
  BtGameNode() : Node("bt_game_node"), ctx_(this) {
    loadParams();

    ctx_.setPublishers(
        create_publisher<PoseStamped>("/game/target", 10),
        create_publisher<String>("/servo/command", 10),
        create_publisher<String>("/game/state", 10),
        create_publisher<UInt32>("/game/ball_collected", 10));

    ball_map_sub_ = create_subscription<BallMap>(
        "/spatial_memory/ball_map", 10,
        [this](BallMap::SharedPtr m) { ctx_.ball_map = m; });

    odom_sub_ = create_subscription<Odometry>(
        "/odom/filtered", 10,
        [this](Odometry::SharedPtr m) { onOdom(*m); });

    start_sub_ = create_subscription<Bool>(
        "/match/start", rclcpp::QoS(1).transient_local(),
        [this](Bool::SharedPtr m) { onStart(m->data); });

    buildTree();

    const auto period = std::chrono::duration<double>(1.0 / ctx_.p.tick_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { tree_.tickOnce(); });

    const HomePose h = ctx_.homePose();
    RCLCPP_INFO(get_logger(),
                "bt_game_node ready — zone=%s home=(%.2f,%.2f,%.2f) cap=%d "
                "endgame=%.0fs tick=%.0fHz",
                ctx_.p.home_zone.c_str(), h.x, h.y, h.yaw,
                ctx_.p.storage_capacity, ctx_.p.endgame_enter_s, ctx_.p.tick_rate_hz);
  }

private:
  void loadParams() {
    ctx_.p.home_zone   = declare_parameter("home_zone", ctx_.p.home_zone);
    ctx_.p.tick_rate_hz = declare_parameter("tick_rate_hz", ctx_.p.tick_rate_hz);
    ctx_.p.match_duration_s = declare_parameter("match_duration_s", ctx_.p.match_duration_s);
    ctx_.p.endgame_enter_s  = declare_parameter("endgame_enter_s", ctx_.p.endgame_enter_s);
    ctx_.p.goal_tol_m       = declare_parameter("goal_tol_m", ctx_.p.goal_tol_m);
    ctx_.p.capture_radius_m = declare_parameter("capture_radius_m", ctx_.p.capture_radius_m);
    ctx_.p.storage_capacity = declare_parameter("storage_capacity", ctx_.p.storage_capacity);
    ctx_.p.weight_ping      = declare_parameter("weight_ping", ctx_.p.weight_ping);
    ctx_.p.weight_steel     = declare_parameter("weight_steel", ctx_.p.weight_steel);
    ctx_.p.density_bonus    = declare_parameter("density_bonus", ctx_.p.density_bonus);
    ctx_.p.occluded_penalty = declare_parameter("occluded_penalty", ctx_.p.occluded_penalty);
    ctx_.p.use_predicted_position =
        declare_parameter("use_predicted_position", ctx_.p.use_predicted_position);
    ctx_.p.scoop_duration_s = declare_parameter("scoop_duration_s", ctx_.p.scoop_duration_s);
    ctx_.p.dump_duration_s  = declare_parameter("dump_duration_s", ctx_.p.dump_duration_s);
    ctx_.p.wall_offset_m    = declare_parameter("wall_offset_m", ctx_.p.wall_offset_m);
    ctx_.p.arena_min        = declare_parameter("arena_min", ctx_.p.arena_min);
    ctx_.p.arena_max        = declare_parameter("arena_max", ctx_.p.arena_max);
    ctx_.p.patrol_waypoints = declare_parameter(
        "patrol_waypoints",
        std::vector<double>{0.4, 0.4, 1.0, 0.4, 1.6, 0.4, 1.6, 1.0, 1.0, 1.0,
                            0.4, 1.0, 0.4, 1.6, 1.0, 1.6, 1.6, 1.6});
  }

  void onOdom(const Odometry& msg) {
    const auto& pos = msg.pose.pose.position;
    const auto& q   = msg.pose.pose.orientation;
    ctx_.robot_x = pos.x;
    ctx_.robot_y = pos.y;
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    ctx_.robot_yaw = std::atan2(siny, cosy);
    ctx_.has_pose  = true;
  }

  // Latch the first rising edge so a collision-reset re-press cannot re-anchor
  // the 180 s match clock.
  void onStart(bool data) {
    if (ctx_.match_started || !data) return;
    ctx_.match_started    = true;
    ctx_.match_start_time = now();
    RCLCPP_INFO(get_logger(), "Match started!");
  }

  void buildTree() {
    registerNodes(factory_);
    auto blackboard = BT::Blackboard::create();
    blackboard->set<GameContext*>("ctx", &ctx_);

    const std::string xml =
        ament_index_cpp::get_package_share_directory("unibots_bt") +
        "/bt/game_tree.xml";
    tree_ = factory_.createTreeFromFile(xml, blackboard);
    RCLCPP_INFO(get_logger(), "Loaded behaviour tree: %s", xml.c_str());
  }

  GameContext ctx_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  rclcpp::Subscription<BallMap>::SharedPtr  ball_map_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Bool>::SharedPtr     start_sub_;
  rclcpp::TimerBase::SharedPtr              timer_;
};

}  // namespace unibots_bt

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unibots_bt::BtGameNode>());
  rclcpp::shutdown();
  return 0;
}
