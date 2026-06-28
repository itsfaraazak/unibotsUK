// game_context.hpp — shared blackboard state for the Unibots behaviour tree.
//
// One GameContext instance is created by bt_game_node, placed on the BT.CPP
// blackboard, and read/written by every custom node (see bt_nodes.hpp). It owns
// the latest ROS inputs (pose, ball map, match clock, hopper count, latches) and
// the publish helpers — the leaf nodes never touch rclcpp directly.
//
// Coordinate frame: arena "map", origin = SW corner, metres (rulebook geometry).

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <unibots_msgs/msg/ball_map.hpp>
#include <unibots_msgs/msg/world_ball.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace unibots_bt {

using BallMap     = unibots_msgs::msg::BallMap;
using WorldBall   = unibots_msgs::msg::WorldBall;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Odometry    = nav_msgs::msg::Odometry;
using String      = std_msgs::msg::String;
using UInt32      = std_msgs::msg::UInt32;

// Deposit/park pose for one scoring wall (yaw points into the net).
struct HomePose { double x, y, yaw; };

// Tunable parameters, declared once in the node and copied here (cheap reads).
struct Params {
  std::string home_zone        = "north";
  double tick_rate_hz          = 20.0;
  double match_duration_s      = 180.0;
  double endgame_enter_s       = 158.0;
  double goal_tol_m            = 0.08;
  double capture_radius_m      = 0.12;
  int    storage_capacity      = 6;
  double weight_ping           = 4.0;
  double weight_steel          = 2.0;
  double density_bonus         = 0.4;
  double occluded_penalty      = 0.6;
  bool   use_predicted_position = true;
  double scoop_duration_s      = 1.4;
  double dump_duration_s       = 2.5;
  double wall_offset_m         = 0.15;
  double arena_min             = 0.15;
  double arena_max             = 1.85;
  std::vector<double> patrol_waypoints;  // flat [x0,y0, x1,y1, ...]
};

class GameContext {
public:
  explicit GameContext(rclcpp::Node* node) : node_(node) {}

  // ── Live inputs (written by node subscription callbacks) ──────────────────
  bool          has_pose      = false;
  double        robot_x       = 0.0;
  double        robot_y       = 0.0;
  double        robot_yaw     = 0.0;
  BallMap::SharedPtr ball_map;            // nullable until first message

  // ── Match clock / latches ─────────────────────────────────────────────────
  bool          match_started = false;    // latched on first /match/start rising edge
  rclcpp::Time  match_start_time{0, 0, RCL_ROS_TIME};

  // ── Game counters / cursors (written by nodes) ────────────────────────────
  int           balls_held    = 0;
  std::size_t   patrol_index  = 0;

  // Current hunt target, written by SelectTarget, read by Approach / capture.
  bool          has_target    = false;
  uint32_t      target_id     = 0;
  double        target_x      = 0.0;
  double        target_y      = 0.0;

  Params        p;

  // ── Derived helpers ───────────────────────────────────────────────────────
  rclcpp::Node* node() const { return node_; }
  rclcpp::Time  now()  const { return node_->now(); }

  double elapsed_s() const {
    if (!match_started) return 0.0;
    return (now() - match_start_time).seconds();
  }

  double distanceTo(double x, double y) const {
    const double dx = robot_x - x, dy = robot_y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  HomePose homePose() const {
    const double off = p.wall_offset_m;
    if (p.home_zone == "south") return {1.0, off,       -M_PI_2};
    if (p.home_zone == "east")  return {2.0 - off, 1.0,  0.0};
    if (p.home_zone == "west")  return {off, 1.0,        M_PI};
    return {1.0, 2.0 - off, M_PI_2};  // north (default)
  }

  // Point value used by SelectTarget — matches perception ball_type strings
  // ("ping_pong" / "steel") published by spatial_memory_node.
  double ballValue(const WorldBall& b) const {
    return (b.ball_type == "ping_pong") ? p.weight_ping : p.weight_steel;
  }

  // ── Publish helpers (the only motion / actuation outputs) ─────────────────
  void publishTarget(double x, double y, double yaw) {
    x = clampArena(x);
    y = clampArena(y);
    PoseStamped ps;
    ps.header.stamp        = now();
    ps.header.frame_id     = "map";
    ps.pose.position.x     = x;
    ps.pose.position.y     = y;
    ps.pose.orientation.z  = std::sin(yaw / 2.0);
    ps.pose.orientation.w  = std::cos(yaw / 2.0);
    target_pub_->publish(ps);
  }

  // Hold position: command current pose so the controller settles to zero vel.
  void holdPosition() { publishTarget(robot_x, robot_y, robot_yaw); }

  void publishServo(const std::string& cmd) {
    String m; m.data = cmd;
    servo_pub_->publish(m);
  }

  void publishCollected(uint32_t track_id) {
    UInt32 m; m.data = track_id;
    collected_pub_->publish(m);
  }

  // Publish the current state every tick as a heartbeat (consumers such as the
  // controller's open-loop-search fallback use its freshness to know the BT is
  // alive), but only LOG on a transition — cheap, clean logs.
  void publishState(const std::string& s) {
    String m; m.data = s;
    state_pub_->publish(m);
    if (s == cur_state_) return;
    cur_state_ = s;
    RCLCPP_INFO(node_->get_logger(), "State → %s  [t=%.1fs  held=%d]",
                s.c_str(), elapsed_s(), balls_held);
  }

  // ── Publisher wiring (called once by the node) ────────────────────────────
  void setPublishers(rclcpp::Publisher<PoseStamped>::SharedPtr target,
                     rclcpp::Publisher<String>::SharedPtr      servo,
                     rclcpp::Publisher<String>::SharedPtr      state,
                     rclcpp::Publisher<UInt32>::SharedPtr      collected) {
    target_pub_    = std::move(target);
    servo_pub_     = std::move(servo);
    state_pub_     = std::move(state);
    collected_pub_ = std::move(collected);
  }

private:
  double clampArena(double v) const {
    return std::min(std::max(v, p.arena_min), p.arena_max);
  }

  rclcpp::Node* node_;
  std::string   cur_state_;

  rclcpp::Publisher<PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<String>::SharedPtr      servo_pub_;
  rclcpp::Publisher<String>::SharedPtr      state_pub_;
  rclcpp::Publisher<UInt32>::SharedPtr      collected_pub_;
};

}  // namespace unibots_bt
