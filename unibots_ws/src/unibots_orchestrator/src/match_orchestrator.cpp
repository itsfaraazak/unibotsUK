// match_orchestrator.cpp — Unibots 2026 high-level strategy state machine (C++).
//
// One rclcpp node ticking at 20 Hz that turns the existing perception/kalman
// (spatial_memory), navigation (MPC) and actuation (servo) stacks into a full
// autonomous match. It owns DECISIONS and GOALS only — the MPC computes motion.
// The node takes direct control of /cmd_vel for exactly two manoeuvres the MPC
// cannot express: in-place 360 spins and the timed wall-flush.
//
//   SLEEP --button--> STARTUP --> SEARCH <-> CHASE
//                                   |          |
//                                   +--> DEPOSIT --(t>=150)--> SLEEP
//                                               +--(t<150)---> SEARCH
//   ToF trigger (any running state) --> STOP + SCOOP --> resume prior state
//
// Pure C++/rclcpp for minimum-latency reaction (no Python GIL, no py_trees tick).
//
// ToF robustness (claw can cross the beam): a real capture must hold for
// `tof_consecutive` samples, ToF is ignored entirely while scooping, and a
// post-scoop inhibit window covers the claw retracting back through the beam.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "unibots_msgs/msg/ball_map.hpp"
#include "unibots_msgs/msg/world_ball.hpp"

namespace
{
// Servo command strings (match hardware_servo_node / CLAUDE.md table).
constexpr const char * SERVO_SCOOP = "SCOOP";   // claw grab + scoop into hopper
constexpr const char * SERVO_OPEN = "OPEN";     // trapdoor open -> dump into net
constexpr const char * SERVO_CLOSE = "CLOSE";   // trapdoor return

constexpr double TWO_PI = 2.0 * M_PI;

// Rulebook net points per ball type, used as priority "value".
double ball_value(const std::string & type)
{
  if (type == "ping_pong") {return 4.0;}
  if (type == "steel") {return 2.0;}
  return 1.0;
}

double wrap_angle(double a) {return std::atan2(std::sin(a), std::cos(a));}

double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}
}  // namespace

enum class State { SLEEP, STARTUP, SEARCH, CHASE, DEPOSIT };

// Result of one in-place-spin tick. The caller must NOT run its own "spin
// finished" transition unless the spin actually COMPLETED — a PREEMPTED spin has
// already transitioned the FSM into CHASE.
enum class SpinResult { SPINNING, COMPLETED, PREEMPTED };

static const char * state_name(State s)
{
  switch (s) {
    case State::SLEEP: return "SLEEP";
    case State::STARTUP: return "STARTUP";
    case State::SEARCH: return "SEARCH";
    case State::CHASE: return "CHASE";
    case State::DEPOSIT: return "DEPOSIT";
  }
  return "?";
}

class MatchOrchestrator : public rclcpp::Node
{
public:
  MatchOrchestrator()
  : rclcpp::Node("match_orchestrator")
  {
    // ---------------- parameters --------------------------------------------
    home_name_ = declare_parameter<std::string>("home", "south");
    arena_ = declare_parameter<double>("arena_size_m", 2.0);
    const double rate = declare_parameter<double>("tick_rate_hz", 20.0);

    match_duration_ = declare_parameter<double>("match_duration_s", 180.0);
    endgame_s_ = declare_parameter<double>("endgame_deposit_s", 150.0);

    goal_tol_ = declare_parameter<double>("goal_tolerance_m", 0.08);
    stability_buf_ = declare_parameter<double>("goal_stability_buffer_m", 0.02);
    startup_frac_ = declare_parameter<double>("startup_forward_frac", 0.25);
    waypoints_flat_ = declare_parameter<std::vector<double>>(
      "waypoints",
      {0.4, 0.4, 1.0, 0.4, 1.6, 0.4, 1.6, 1.0, 1.0, 1.0,
        0.4, 1.0, 0.4, 1.6, 1.0, 1.6, 1.6, 1.6});

    spin_speed_ = declare_parameter<double>("spin_speed_rad_s", 1.2);
    spin_target_ = declare_parameter<double>("spin_target_rad", TWO_PI);
    flush_speed_ = declare_parameter<double>("flush_speed_m_s", 0.15);
    flush_duration_ = declare_parameter<double>("flush_duration_s", 2.0);

    tof_trigger_ = declare_parameter<double>("tof_trigger_m", 0.05);
    tof_rearm_ = declare_parameter<double>("tof_rearm_m", 0.12);
    tof_consecutive_ = declare_parameter<int>("tof_consecutive", 3);
    tof_inhibit_after_scoop_s_ =
      declare_parameter<double>("tof_inhibit_after_scoop_s", 0.8);
    scoop_dwell_ = declare_parameter<double>("scoop_dwell_s", 1.4);
    capacity_ = declare_parameter<int>("storage_capacity", 6);

    dump_open_dwell_ = declare_parameter<double>("dump_open_dwell_s", 1.5);
    dump_close_dwell_ = declare_parameter<double>("dump_close_dwell_s", 0.8);

    use_pred_ = declare_parameter<bool>("use_predicted_position", true);
    loss_grace_ = declare_parameter<double>("loss_grace_s", 1.0);
    density_bonus_ = declare_parameter<double>("density_bonus", 0.4);
    ball_stale_ = declare_parameter<double>("ball_stale_s", 1.0);

    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom/filtered");
    const auto ballmap_topic =
      declare_parameter<std::string>("ball_map_topic", "/spatial_memory/ball_map");
    const auto tof_topic = declare_parameter<std::string>("tof_topic", "/sensors/tof_distance");
    const auto button_topic = declare_parameter<std::string>("button_topic", "/match/button");
    const auto start_topic = declare_parameter<std::string>("start_topic", "/match/start");
    const auto target_topic = declare_parameter<std::string>("target_topic", "/game/target");
    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto servo_topic = declare_parameter<std::string>("servo_topic", "/servo/command");
    const auto collected_topic =
      declare_parameter<std::string>("collected_topic", "/game/ball_collected");
    const auto mpc_enable_topic =
      declare_parameter<std::string>("mpc_enable_topic", "/mpc/enable");

    // AprilTag map (Section 1): accessible config — id list + flat xy list.
    // Defaults to the wall/corner survey if not provided in YAML.
    const auto tag_ids = declare_parameter<std::vector<int64_t>>(
      "apriltag_ids", {0, 1, 2, 3, 4, 5, 6, 7});
    const auto tag_xy = declare_parameter<std::vector<double>>(
      "apriltag_xy",
      {1, 0, 2, 1, 1, 2, 0, 1, 0, 0, 2, 0, 2, 2, 0, 2});
    for (size_t i = 0; i < tag_ids.size() && 2 * i + 1 < tag_xy.size(); ++i) {
      apriltags_[static_cast<int>(tag_ids[i])] = {tag_xy[2 * i], tag_xy[2 * i + 1]};
    }

    // resolve home tile ----------------------------------------------------
    home_ = resolve_home(home_name_);
    center_ = {arena_ / 2.0, arena_ / 2.0};

    // parse waypoints into pairs ------------------------------------------
    for (size_t i = 0; i + 1 < waypoints_flat_.size(); i += 2) {
      waypoints_.push_back({waypoints_flat_[i], waypoints_flat_[i + 1]});
    }

    // ---------------- QoS ---------------------------------------------------
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    // ---------------- subscriptions ----------------------------------------
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        px_ = m->pose.pose.position.x;
        py_ = m->pose.pose.position.y;
        yaw_ = yaw_from_quat(m->pose.pose.orientation);
        have_pose_ = true;
      });
    ballmap_sub_ = create_subscription<unibots_msgs::msg::BallMap>(
      ballmap_topic, sensor_qos,
      [this](unibots_msgs::msg::BallMap::SharedPtr m) {balls_ = m->balls;});
    tof_sub_ = create_subscription<sensor_msgs::msg::Range>(
      tof_topic, sensor_qos,
      [this](sensor_msgs::msg::Range::SharedPtr m) {tof_range_ = m->range;});
    button_sub_ = create_subscription<std_msgs::msg::Bool>(
      button_topic, sensor_qos,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        if (!m->data) {return;}
        if (state_ == State::SLEEP) {start_match();} else {go_sleep("button");}
      });
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_topic, latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data && state_ == State::SLEEP) {start_match();} else if (!m->data &&
          state_ != State::SLEEP) {go_sleep("start_topic");}
      });

    // ---------------- publishers -------------------------------------------
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, sensor_qos);
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, sensor_qos);
    servo_pub_ = create_publisher<std_msgs::msg::String>(servo_topic, sensor_qos);
    collected_pub_ = create_publisher<std_msgs::msg::UInt32>(collected_topic, sensor_qos);
    mpc_pub_ = create_publisher<std_msgs::msg::Bool>(mpc_enable_topic, latched_qos);
    state_pub_ = create_publisher<std_msgs::msg::String>("/game/state", latched_qos);

    phase_t0_ = now();
    match_t0_ = now();
    lock_last_seen_ = now();
    collect_t0_ = now();
    tof_inhibit_until_ = now();

    set_mpc_enabled(true);
    publish_state();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this]() {tick();});

    RCLCPP_INFO(
      get_logger(),
      "MatchOrchestrator (C++) up. home=%s(%.1f,%.1f), %zu waypoints, capacity=%d. "
      "Press /match/button to start.",
      home_name_.c_str(), home_.first, home_.second, waypoints_.size(), capacity_);
  }

private:
  // ================= time helpers ========================================
  double elapsed(const rclcpp::Time & t0) const {return (now() - t0).seconds();}

  // ================= transitions =========================================
  void start_match()
  {
    match_t0_ = now();
    ball_count_ = 0;
    wp_index_ = 0;
    force_deposit_ = false;
    clear_lock();
    startup_goal_.reset();
    send_servo(SERVO_CLOSE);
    enter(State::STARTUP, 0);
    RCLCPP_INFO(get_logger(), "MATCH START.");
  }

  void go_sleep(const std::string & why)
  {
    stop_broadcast();
    set_mpc_enabled(false);
    hard_stop();
    collecting_ = false;
    state_ = State::SLEEP;
    phase_ = 0;
    publish_state();
    RCLCPP_INFO(get_logger(), "SLEEP (%s).", why.c_str());
  }

  void enter(State s, int phase)
  {
    state_ = s;
    phase_ = phase;
    phase_t0_ = now();
    publish_state();
  }

  void begin_spin(State next)
  {
    spin_accum_ = 0.0;
    spin_prev_yaw_ = have_pose_ ? std::optional<double>(yaw_) : std::nullopt;
    spin_next_ = next;
    stop_broadcast();
    set_mpc_enabled(false);   // we own /cmd_vel for the spin
  }

  // ================= output helpers ======================================
  void set_mpc_enabled(bool en)
  {
    if (mpc_enabled_.has_value() && *mpc_enabled_ == en) {return;}
    mpc_enabled_ = en;
    std_msgs::msg::Bool b;
    b.data = en;
    mpc_pub_->publish(b);
  }

  void publish_state()
  {
    const std::string s = state_name(state_);
    if (published_state_ == s) {return;}
    published_state_ = s;
    std_msgs::msg::String m;
    m.data = s;
    state_pub_->publish(m);
  }

  void send_servo(const char * cmd)
  {
    std_msgs::msg::String m;
    m.data = cmd;
    servo_pub_->publish(m);
  }

  void hard_stop()
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    cmd_pub_->publish(t);
  }

  void drive_cmd(double vx, double vy, double omega)
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.twist.linear.x = vx;
    t.twist.linear.y = vy;
    t.twist.angular.z = omega;
    cmd_pub_->publish(t);
  }

  // Section 2: CONTINUOUSLY stream the goal to the MPC (every tick), honouring
  // the 2 cm stability buffer. The buffer filters jitter — a new candidate
  // within 2 cm of the last streamed goal is ignored and the PREVIOUS goal is
  // (re)published — but we keep publishing so the MPC always has a live goal and
  // late subscribers never miss it. Broadcasting is stopped only by advancing
  // state + stop_broadcast() (Section 2: stop the moment the goal is reached).
  void broadcast_goal(double x, double y)
  {
    set_mpc_enabled(true);
    if (last_broadcast_.has_value()) {
      const double dx = x - last_broadcast_->first;
      const double dy = y - last_broadcast_->second;
      if (std::hypot(dx, dy) < stability_buf_) {
        x = last_broadcast_->first;   // hold previous goal (jitter rejected)...
        y = last_broadcast_->second;
      } else {
        last_broadcast_ = {x, y};     // ...else accept the new goal
      }
    } else {
      last_broadcast_ = {x, y};
    }
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now();
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = y;
    if (have_pose_) {p.pose.orientation = quat_from_yaw(std::atan2(y - py_, x - px_));}
    target_pub_->publish(p);   // publish EVERY tick -> continuous goal stream
  }

  void stop_broadcast() {last_broadcast_.reset();}

  // ================= geometry / selection ================================
  double dist_to(double x, double y) const {return std::hypot(x - px_, y - py_);}
  bool at_goal(double x, double y) const {return dist_to(x, y) <= goal_tol_;}

  std::pair<double, double> ball_pos(const unibots_msgs::msg::WorldBall & b) const
  {
    if (use_pred_ && (b.predicted_x != 0.0f || b.predicted_y != 0.0f)) {
      return {b.predicted_x, b.predicted_y};
    }
    return {b.world_x, b.world_y};
  }

  bool is_fresh(const unibots_msgs::msg::WorldBall & b) const
  {
    if (b.status == "COLLECTED") {return false;}
    const double seen = b.last_seen.sec + b.last_seen.nanosec * 1e-9;
    return (now().seconds() - seen) <= ball_stale_;
  }

  const unibots_msgs::msg::WorldBall * find_ball(uint32_t id) const
  {
    for (const auto & b : balls_) {if (b.track_id == id) {return &b;}}
    return nullptr;
  }

  // Section 5: highest-utility fresh ball, value*density*vis/distance.
  const unibots_msgs::msg::WorldBall * select_target() const
  {
    if (!have_pose_) {return nullptr;}
    const unibots_msgs::msg::WorldBall * best = nullptr;
    double best_u = -1.0;
    for (const auto & b : balls_) {
      if (!is_fresh(b)) {continue;}
      const auto [x, y] = ball_pos(b);
      const double d = std::max(dist_to(x, y), 0.05);
      const double density = 1.0 + density_bonus_ * std::max<int>(b.density, 0);
      const double vis = (b.status == "VISIBLE") ? 1.0 : 0.5;
      const double u = ball_value(b.ball_type) * density * vis / d;
      if (u > best_u) {best_u = u; best = &b;}
    }
    return best;
  }

  // ================= main tick ===========================================
  void tick()
  {
    if (!have_pose_) {return;}

    // hard match end (180 s) -> stop everything
    if (state_ != State::SLEEP && elapsed(match_t0_) >= match_duration_) {
      go_sleep("match time up (180s)");
      return;
    }

    // global ToF interrupt (Section 7) — runs in every non-sleep state.
    if (collecting_) {tick_collect(); return;}
    if (state_ != State::SLEEP && tof_triggered()) {begin_collect(); return;}

    // endgame override (Section 8): 150 s -> drop tasks, deposit.
    // A ToF collection already in progress finished above (collecting_ branch)
    // before we reach here, satisfying "finish busy servo first".
    if (state_ != State::SLEEP && state_ != State::DEPOSIT &&
      elapsed(match_t0_) >= endgame_s_)
    {
      if (!force_deposit_) {
        RCLCPP_WARN(get_logger(), "ENDGAME (150s): dropping tasks -> DEPOSIT.");
      }
      force_deposit_ = true;
      enter(State::DEPOSIT, 0);
      return;
    }

    switch (state_) {
      case State::SLEEP: break;
      case State::STARTUP: tick_startup(); break;
      case State::SEARCH: tick_search(); break;
      case State::CHASE: tick_chase(); break;
      case State::DEPOSIT: tick_deposit(); break;
    }
  }

  // ================= ToF collection overlay (Section 7) ==================
  // Hardened against the claw crossing the beam:
  //   * require tof_consecutive samples below trigger (reject transient spikes),
  //   * hysteresis re-arm above tof_rearm,
  //   * post-scoop inhibit window covering claw retract,
  //   * never sample ToF while DEPOSITing (trapdoor/claw motion at the wall).
  bool tof_triggered()
  {
    if (state_ == State::DEPOSIT) {return false;}
    if (now() < tof_inhibit_until_) {tof_below_count_ = 0; return false;}

    if (tof_range_ >= tof_rearm_) {tof_armed_ = true;}
    if (!tof_armed_) {return false;}

    if (tof_range_ <= tof_trigger_) {
      if (++tof_below_count_ >= tof_consecutive_) {
        tof_below_count_ = 0;
        tof_armed_ = false;     // require clearing past rearm before next fire
        return true;
      }
    } else {
      tof_below_count_ = 0;
    }
    return false;
  }

  void begin_collect()
  {
    resume_state_ = state_;
    resume_phase_ = phase_;
    collecting_ = true;
    collect_t0_ = now();
    stop_broadcast();
    set_mpc_enabled(false);
    hard_stop();
    send_servo(SERVO_SCOOP);   // claw grab + scoop (actuate + retract cycle)
    RCLCPP_INFO(get_logger(), "ToF TRIGGER -> STOP + SCOOP.");
  }

  void tick_collect()
  {
    hard_stop();   // hold still through the scoop
    if (elapsed(collect_t0_) < scoop_dwell_) {return;}

    // scoop done: inhibit ToF while the claw retracts back through the beam.
    tof_inhibit_until_ = now() + rclcpp::Duration::from_seconds(tof_inhibit_after_scoop_s_);
    tof_below_count_ = 0;
    tof_armed_ = false;

    ++ball_count_;
    if (resume_state_ == State::CHASE && lock_id_.has_value()) {
      std_msgs::msg::UInt32 u;
      u.data = *lock_id_;
      collected_pub_->publish(u);   // spatial_memory marks it COLLECTED
      clear_lock();
    }
    RCLCPP_INFO(get_logger(), "Collected. count=%d/%d.", ball_count_, capacity_);
    collecting_ = false;

    if (ball_count_ >= capacity_) {
      RCLCPP_INFO(get_logger(), "Hopper FULL -> DEPOSIT.");
      enter(State::DEPOSIT, 0);
    } else if (elapsed(match_t0_) >= endgame_s_) {
      force_deposit_ = true;
      enter(State::DEPOSIT, 0);
    } else {
      // Resume exactly. If we were chasing, the target is now collected/gone, so
      // the resumed CHASE tick detects loss and reverts to SEARCH.
      enter(resume_state_, resume_phase_);
    }
  }

  // ================= STARTUP (Section 2) =================================
  void tick_startup()
  {
    if (phase_ == 0) {
      if (!startup_goal_.has_value()) {
        const double vx = center_.first - home_.first;
        const double vy = center_.second - home_.second;
        const double n = std::max(std::hypot(vx, vy), 1e-9);
        const double step = startup_frac_ * arena_;
        startup_goal_ = {home_.first + vx / n * step, home_.second + vy / n * step};
      }
      const auto [gx, gy] = *startup_goal_;
      if (at_goal(gx, gy)) {
        stop_broadcast();
        begin_spin(State::SEARCH);
        enter(State::STARTUP, 1);
      } else {
        broadcast_goal(gx, gy);
      }
    } else {  // phase 1: spin 360, then SEARCH (ball mid-spin -> CHASE)
      if (tick_spin() == SpinResult::COMPLETED) {enter(State::SEARCH, 0);}
    }
  }

  // ================= SEARCH (Section 3) =================================
  void tick_search()
  {
    if (const auto * tgt = select_target()) {
      lock_target(*tgt);
      stop_broadcast();
      enter(State::CHASE, 0);
      return;
    }
    if (waypoints_.empty()) {return;}
    const auto & wp = waypoints_[wp_index_];
    if (phase_ == 0) {
      if (at_goal(wp.first, wp.second)) {
        stop_broadcast();
        begin_spin(State::SEARCH);
        enter(State::SEARCH, 1);
      } else {
        broadcast_goal(wp.first, wp.second);
      }
    } else {  // phase 1: spin 360, then queue next waypoint
      if (tick_spin() == SpinResult::COMPLETED) {
        wp_index_ = (wp_index_ + 1) % waypoints_.size();
        enter(State::SEARCH, 0);
      }
    }
  }

  // ================= CHASE (Section 5/6) ================================
  void lock_target(const unibots_msgs::msg::WorldBall & b)
  {
    lock_id_ = b.track_id;
    lock_goal_ = ball_pos(b);
    lock_last_seen_ = now();
    RCLCPP_INFO(
      get_logger(),
      "LOCK ball #%u (%s) -> CHASE. [priority algorithm disabled until resolved]",
      b.track_id, b.ball_type.c_str());
  }

  void clear_lock() {lock_id_.reset(); lock_goal_.reset();}

  void tick_chase()
  {
    // Section 5 safeguard APPLIED: no re-selection mid-chase (prevents target
    // thrash). Track only the locked ball.
    const unibots_msgs::msg::WorldBall * b =
      lock_id_.has_value() ? find_ball(*lock_id_) : nullptr;

    if (b != nullptr && is_fresh(*b)) {
      lock_goal_ = ball_pos(*b);
      lock_last_seen_ = now();
      broadcast_goal(lock_goal_->first, lock_goal_->second);
      return;
    }

    // Section 6: target no longer published -> coast to last known goal for
    // loss_grace_s (MPC's own 1 s coast mirrors this); ToF can still fire.
    if (lock_goal_.has_value() && elapsed(lock_last_seen_) < loss_grace_) {
      broadcast_goal(lock_goal_->first, lock_goal_->second);
      return;
    }

    // coast expired, no collection -> ball gone. Mandatory 360 spin, then SEARCH.
    RCLCPP_INFO(get_logger(), "Target lost (coast expired) -> SEARCH (spin first).");
    clear_lock();
    stop_broadcast();
    begin_spin(State::SEARCH);
    enter(State::SEARCH, 1);
  }

  // ================= DEPOSIT (Section 8) ================================
  void tick_deposit()
  {
    const double hx = home_.first, hy = home_.second;
    if (phase_ == 0) {                       // drive home (ends facing wall)
      if (at_goal(hx, hy)) {
        stop_broadcast();
        set_mpc_enabled(false);
        hard_stop();
        enter(State::DEPOSIT, 1);
      } else {
        broadcast_goal(hx, hy);
      }
    } else if (phase_ == 1) {                // manual FORWARD flush to the wall
      if (elapsed(phase_t0_) < flush_duration_) {
        drive_cmd(0.0, flush_speed_, 0.0);   // body-forward = +y
      } else {
        hard_stop();
        send_servo(SERVO_OPEN);
        enter(State::DEPOSIT, 2);
      }
    } else if (phase_ == 2) {                // wait for drop, then close
      hard_stop();
      if (elapsed(phase_t0_) >= dump_open_dwell_) {
        send_servo(SERVO_CLOSE);
        enter(State::DEPOSIT, 3);
      }
    } else {                                 // wait for close, evaluate
      hard_stop();
      if (elapsed(phase_t0_) >= dump_close_dwell_) {
        ball_count_ = 0;
        if (elapsed(match_t0_) >= endgame_s_) {
          go_sleep("post-deposit, endgame");
        } else {
          RCLCPP_INFO(get_logger(), "Deposited -> back to SEARCH.");
          begin_spin(State::SEARCH);
          enter(State::SEARCH, 1);
        }
      }
    }
  }

  // ================= shared in-place 360 spin ============================
  // Direct /cmd_vel. A ball found mid-spin preempts straight into CHASE.
  SpinResult tick_spin()
  {
    if (spin_next_ == State::SEARCH) {
      if (const auto * tgt = select_target()) {
        lock_target(*tgt);
        hard_stop();
        set_mpc_enabled(true);
        enter(State::CHASE, 0);
        return SpinResult::PREEMPTED;
      }
    }
    if (!spin_prev_yaw_.has_value()) {spin_prev_yaw_ = yaw_;}
    spin_accum_ += std::abs(wrap_angle(yaw_ - *spin_prev_yaw_));
    spin_prev_yaw_ = yaw_;

    if (spin_accum_ >= spin_target_) {
      hard_stop();
      set_mpc_enabled(true);
      return SpinResult::COMPLETED;
    }
    drive_cmd(0.0, 0.0, spin_speed_);
    return SpinResult::SPINNING;
  }

  // ================= home resolution =====================================
  std::pair<double, double> resolve_home(const std::string & name) const
  {
    // Valid start/home tiles (Section 1): wall midpoints.
    static const std::map<std::string, std::pair<double, double>> tiles = {
      {"south", {1.0, 0.0}}, {"east", {2.0, 1.0}},
      {"north", {1.0, 2.0}}, {"west", {0.0, 1.0}}};
    auto it = tiles.find(name);
    if (it == tiles.end()) {
      RCLCPP_ERROR(
        get_logger(), "Unknown home '%s'; defaulting to south(1,0).", name.c_str());
      return {1.0, 0.0};
    }
    return it->second;
  }

  // ================= members =============================================
  // params
  std::string home_name_;
  std::pair<double, double> home_, center_;
  double arena_{2.0};
  double match_duration_{180.0}, endgame_s_{150.0};
  double goal_tol_{0.08}, stability_buf_{0.02}, startup_frac_{0.25};
  std::vector<double> waypoints_flat_;
  std::vector<std::pair<double, double>> waypoints_;
  double spin_speed_{1.2}, spin_target_{TWO_PI}, flush_speed_{0.15}, flush_duration_{2.0};
  double tof_trigger_{0.05}, tof_rearm_{0.12}, tof_inhibit_after_scoop_s_{0.8};
  int tof_consecutive_{3};
  double scoop_dwell_{1.4};
  int capacity_{6};
  double dump_open_dwell_{1.5}, dump_close_dwell_{0.8};
  bool use_pred_{true};
  double loss_grace_{1.0}, density_bonus_{0.4}, ball_stale_{1.0};
  std::map<int, std::pair<double, double>> apriltags_;

  // world state
  double px_{0}, py_{0}, yaw_{0};
  bool have_pose_{false};
  std::vector<unibots_msgs::msg::WorldBall> balls_;
  double tof_range_{std::numeric_limits<double>::infinity()};

  // FSM
  State state_{State::SLEEP};
  int phase_{0};
  rclcpp::Time phase_t0_, match_t0_;
  int ball_count_{0};
  size_t wp_index_{0};
  bool force_deposit_{false};

  // chase lock
  std::optional<uint32_t> lock_id_;
  std::optional<std::pair<double, double>> lock_goal_;
  rclcpp::Time lock_last_seen_;

  // spin
  double spin_accum_{0.0};
  std::optional<double> spin_prev_yaw_;
  State spin_next_{State::SEARCH};

  // startup / broadcast
  std::optional<std::pair<double, double>> startup_goal_;
  std::optional<std::pair<double, double>> last_broadcast_;

  // collect overlay + ToF guard
  bool collecting_{false};
  bool tof_armed_{true};
  int tof_below_count_{0};
  rclcpp::Time tof_inhibit_until_, collect_t0_;
  State resume_state_{State::SEARCH};
  int resume_phase_{0};

  // misc
  std::optional<bool> mpc_enabled_;
  std::string published_state_;

  // ROS handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<unibots_msgs::msg::BallMap>::SharedPtr ballmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_, start_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr servo_pub_, state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr collected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mpc_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatchOrchestrator>());
  rclcpp::shutdown();
  return 0;
}
