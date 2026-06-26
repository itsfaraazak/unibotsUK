// bt_nodes.hpp — custom BehaviorTree.CPP v4 leaf nodes for the Unibots game tree.
//
// Conditions are pure (no side effects); actions own the side effects (publish a
// goal pose, fire a servo, advance a cursor). Every node reaches shared state
// through a GameContext* fetched once from the blackboard key "ctx".
//
// Multi-tick actions derive from BT::StatefulActionNode (onStart/onRunning/
// onHalted); instantaneous ones use the onStart→SUCCESS path.

#pragma once

#include "unibots_bt/game_context.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace unibots_bt {

// ── Bases that grab the shared GameContext from the blackboard ───────────────

class CtxCondition : public BT::ConditionNode {
public:
  CtxCondition(const std::string& name, const BT::NodeConfig& cfg)
      : BT::ConditionNode(name, cfg),
        ctx_(cfg.blackboard->get<GameContext*>("ctx")) {}
  static BT::PortsList providedPorts() { return {}; }
protected:
  GameContext* ctx_;
};

class CtxAction : public BT::StatefulActionNode {
public:
  CtxAction(const std::string& name, const BT::NodeConfig& cfg)
      : BT::StatefulActionNode(name, cfg),
        ctx_(cfg.blackboard->get<GameContext*>("ctx")) {}
  static BT::PortsList providedPorts() { return {}; }
  void onHalted() override {}  // most actions need no teardown
protected:
  GameContext* ctx_;
};

// ── Conditions ───────────────────────────────────────────────────────────────

class MatchNotStarted : public CtxCondition {
public:
  using CtxCondition::CtxCondition;
  BT::NodeStatus tick() override {
    return ctx_->match_started ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class MatchTimeExpired : public CtxCondition {
public:
  using CtxCondition::CtxCondition;
  BT::NodeStatus tick() override {
    return (ctx_->match_started && ctx_->elapsed_s() >= ctx_->p.match_duration_s)
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class InEndgameWindow : public CtxCondition {
public:
  using CtxCondition::CtxCondition;
  BT::NodeStatus tick() override {
    return (ctx_->match_started && ctx_->elapsed_s() >= ctx_->p.endgame_enter_s)
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class StorageFull : public CtxCondition {
public:
  using CtxCondition::CtxCondition;
  BT::NodeStatus tick() override {
    return (ctx_->balls_held >= ctx_->p.storage_capacity)
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// True if at least one collectable ball is known.
class BallKnown : public CtxCondition {
public:
  using CtxCondition::CtxCondition;
  BT::NodeStatus tick() override {
    if (!ctx_->ball_map) return BT::NodeStatus::FAILURE;
    for (const auto& b : ctx_->ball_map->balls) {
      if (b.track_id == 0 || b.status == "COLLECTED") continue;
      if (b.status == "VISIBLE" || b.status == "OCCLUDED")
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

// True once the selected target is inside the intake blind-spot.
class WithinCaptureRadius : public CtxCondition {
public:
  using CtxCondition::CtxCondition;
  BT::NodeStatus tick() override {
    if (!ctx_->has_target || !ctx_->has_pose) return BT::NodeStatus::FAILURE;
    return (ctx_->distanceTo(ctx_->target_x, ctx_->target_y) < ctx_->p.capture_radius_m)
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ── Actions ──────────────────────────────────────────────────────────────────

// PRE-START: hold still, status IDLE.
class HoldIdle : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override {
    ctx_->publishState("IDLE");
    if (ctx_->has_pose) ctx_->holdPosition();
    return BT::NodeStatus::RUNNING;
  }
};

// TIME-UP: stop and hold forever, status STOPPED.
class FullStop : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override {
    ctx_->publishState("STOPPED");
    if (ctx_->has_pose) ctx_->holdPosition();
    return BT::NodeStatus::RUNNING;
  }
};

// FAILSAFE: nothing else applied — stop and hold, status STOPPED.
class FailsafeStop : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override {
    ctx_->publishState("FAILSAFE");
    if (ctx_->has_pose) ctx_->holdPosition();
    return BT::NodeStatus::RUNNING;
  }
};

// Drive to the home/net pose. SUCCESS once within home_arrive_m of it.
class NavToHome : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override {
    ctx_->publishState("NAV_HOME");
    if (!ctx_->has_pose) return BT::NodeStatus::RUNNING;
    const HomePose h = ctx_->homePose();
    // Arrival a touch looser than goal tolerance so the controller settles.
    const double arrive = std::max(ctx_->p.goal_tol_m, ctx_->p.wall_offset_m);
    if (ctx_->distanceTo(h.x, h.y) < arrive) return BT::NodeStatus::SUCCESS;
    ctx_->publishTarget(h.x, h.y, h.yaw);
    return BT::NodeStatus::RUNNING;
  }
};

// Open the trapdoor, hold at the wall for dump_duration_s, close, empty hopper.
class Deposit : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override {
    ctx_->publishState("DUMP");
    ctx_->publishServo("OPEN");
    start_ = ctx_->now();
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const HomePose h = ctx_->homePose();
    ctx_->publishTarget(h.x, h.y, h.yaw);  // stay pressed to the net
    if ((ctx_->now() - start_).seconds() < ctx_->p.dump_duration_s)
      return BT::NodeStatus::RUNNING;
    ctx_->publishServo("CLOSE");
    ctx_->balls_held = 0;
    RCLCPP_INFO(ctx_->node()->get_logger(), "Deposit complete — hopper emptied");
    return BT::NodeStatus::SUCCESS;
  }
private:
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
};

// PARK & HOLD: sit on the scoring wall until the match ends (always RUNNING).
class ParkAndHold : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override {
    ctx_->publishState("PARKED");
    const HomePose h = ctx_->homePose();
    ctx_->publishTarget(h.x, h.y, h.yaw);
    return BT::NodeStatus::RUNNING;
  }
};

// Pick the highest-utility ball and store it on the context. FAILURE if none.
class SelectTarget : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return pick(); }
  BT::NodeStatus onRunning() override { return pick(); }
private:
  BT::NodeStatus pick() {
    ctx_->has_target = false;
    if (!ctx_->ball_map || !ctx_->has_pose) return BT::NodeStatus::FAILURE;

    const WorldBall* best = nullptr;
    double best_u = -1.0;
    for (const auto& b : ctx_->ball_map->balls) {
      if (b.track_id == 0 || b.status == "COLLECTED") continue;
      if (b.status != "VISIBLE" && b.status != "OCCLUDED") continue;

      const double bx = ctx_->p.use_predicted_position ? b.predicted_x : b.world_x;
      const double by = ctx_->p.use_predicted_position ? b.predicted_y : b.world_y;
      const double dist = ctx_->distanceTo(bx, by);
      const double vis  = (b.status == "OCCLUDED") ? ctx_->p.occluded_penalty : 1.0;
      const double dens = 1.0 + ctx_->p.density_bonus * std::max(0, b.density);
      const double u = ctx_->ballValue(b) * dens * vis / (dist + 1e-3);
      if (u > best_u) { best_u = u; best = &b; }
    }
    if (!best) return BT::NodeStatus::FAILURE;

    ctx_->has_target = true;
    ctx_->target_id  = best->track_id;
    ctx_->target_x   = ctx_->p.use_predicted_position ? best->predicted_x : best->world_x;
    ctx_->target_y   = ctx_->p.use_predicted_position ? best->predicted_y : best->world_y;
    ctx_->publishState("HUNT");
    return BT::NodeStatus::SUCCESS;
  }
};

// Drive toward the selected target (always RUNNING; capture handled by sibling).
class Approach : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override {
    if (!ctx_->has_target || !ctx_->has_pose) return BT::NodeStatus::FAILURE;
    const double yaw = std::atan2(ctx_->target_y - ctx_->robot_y,
                                  ctx_->target_x - ctx_->robot_x);
    ctx_->publishTarget(ctx_->target_x, ctx_->target_y, yaw);
    return BT::NodeStatus::RUNNING;
  }
};

// Grab the ball: SCOOP, register collection, hold for scoop_duration_s.
class FireScoop : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override {
    ctx_->publishState("CAPTURE");
    ctx_->publishServo("SCOOP");
    if (ctx_->has_target) ctx_->publishCollected(ctx_->target_id);
    ++ctx_->balls_held;
    ctx_->has_target = false;
    start_ = ctx_->now();
    RCLCPP_INFO(ctx_->node()->get_logger(),
                "SCOOP ball #%u — held=%d", ctx_->target_id, ctx_->balls_held);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (ctx_->has_pose) ctx_->holdPosition();
    return ((ctx_->now() - start_).seconds() >= ctx_->p.scoop_duration_s)
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
private:
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
};

// SEARCH: drive a covering patrol; advance waypoint on arrival. Always RUNNING.
class Patrol : public CtxAction {
public:
  using CtxAction::CtxAction;
  BT::NodeStatus onStart() override { return onRunning(); }
  BT::NodeStatus onRunning() override {
    ctx_->publishState("SEARCH");
    const auto& wp = ctx_->p.patrol_waypoints;
    if (wp.size() < 2 || !ctx_->has_pose) {
      if (ctx_->has_pose) ctx_->holdPosition();
      return BT::NodeStatus::RUNNING;
    }
    const std::size_t n = wp.size() / 2;
    if (ctx_->patrol_index >= n) ctx_->patrol_index = 0;
    const double gx = wp[2 * ctx_->patrol_index];
    const double gy = wp[2 * ctx_->patrol_index + 1];
    if (ctx_->distanceTo(gx, gy) < ctx_->p.goal_tol_m)
      ctx_->patrol_index = (ctx_->patrol_index + 1) % n;
    const double yaw = std::atan2(gy - ctx_->robot_y, gx - ctx_->robot_x);
    ctx_->publishTarget(gx, gy, yaw);
    return BT::NodeStatus::RUNNING;
  }
};

// ── Registration ─────────────────────────────────────────────────────────────

inline void registerNodes(BT::BehaviorTreeFactory& f) {
  f.registerNodeType<MatchNotStarted>("MatchNotStarted");
  f.registerNodeType<MatchTimeExpired>("MatchTimeExpired");
  f.registerNodeType<InEndgameWindow>("InEndgameWindow");
  f.registerNodeType<StorageFull>("StorageFull");
  f.registerNodeType<BallKnown>("BallKnown");
  f.registerNodeType<WithinCaptureRadius>("WithinCaptureRadius");

  f.registerNodeType<HoldIdle>("HoldIdle");
  f.registerNodeType<FullStop>("FullStop");
  f.registerNodeType<FailsafeStop>("FailsafeStop");
  f.registerNodeType<NavToHome>("NavToHome");
  f.registerNodeType<Deposit>("Deposit");
  f.registerNodeType<ParkAndHold>("ParkAndHold");
  f.registerNodeType<SelectTarget>("SelectTarget");
  f.registerNodeType<Approach>("Approach");
  f.registerNodeType<FireScoop>("FireScoop");
  f.registerNodeType<Patrol>("Patrol");
}

}  // namespace unibots_bt
