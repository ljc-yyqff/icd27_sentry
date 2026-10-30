// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "icd2027sentry_nav2_plugins/tunnel_config.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace sentry_tunnel
{
using Steady = std::chrono::steady_clock;

// A BT control node with two ordinary Nav2 action children: the planner and
// FollowPath. It owns the passage stages; no background process chooses when
// to lower, cross, or raise the robot. The output guard only enforces policy.
class TunnelNavigation : public BT::ControlNode
{
public:
  TunnelNavigation(const std::string & name, const BT::NodeConfiguration & options)
  : BT::ControlNode(name, options)
  {
    const auto parent = options.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string file;
    parent->get_parameter_or("tunnel_config_file", file, std::string{});
    config_ = Config::load(file);
    rclcpp::NodeOptions node_options;
    node_options.use_global_arguments(false);
    bool sim_time = false;
    parent->get_parameter_or("use_sim_time", sim_time, false);
    node_options.parameter_overrides({rclcpp::Parameter("use_sim_time", sim_time)});
    io_ = std::make_shared<rclcpp::Node>("tunnel_bt_io", parent->get_namespace(), node_options);
    policy_pub_ = io_->create_publisher<std_msgs::msg::Float64MultiArray>("tunnel_policy", 10);
    lift_pub_ = io_->create_publisher<std_msgs::msg::Float64>("chassis_lift_cmd", 10);
    gimbal_pub_ = io_->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);
    phase_pub_ = io_->create_publisher<std_msgs::msg::String>(
      "tunnel_navigation_state", rclcpp::QoS(1).transient_local());
    state_sub_ = io_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "tunnel_guard_state", 10, [this](const std_msgs::msg::Float64MultiArray & message) {
        if (
          message.data.size() >= 6 && std::all_of(
                                        message.data.begin(), message.data.begin() + 6,
                                        [](double v) { return std::isfinite(v); })) {
          state_ = message.data;
          state_received_ = Steady::now();
        }
      });
    joints_sub_ = io_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & msg) {
        for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
          if (msg.name[i] == "chassis_lift_joint" && std::isfinite(msg.position[i])) {
            lift_position_ = msg.position[i];
            lift_received_ = Steady::now();
          }
          if (msg.name[i] == "gimbal_pitch_joint" && std::isfinite(msg.position[i])) {
            pitch_ = msg.position[i];
            pitch_received_ = Steady::now();
          }
        }
      });
    ready_sub_ = io_->create_subscription<std_msgs::msg::Bool>(
      "tunnel_local_clearance_ready", 10, [this](const std_msgs::msg::Bool & msg) {
        clearance_ready_ = msg.data;
        clearance_received_ = Steady::now();
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("planning_goal"),
      BT::OutputPort<nav_msgs::msg::Path>("execution_path"),
      BT::OutputPort<std::string>("controller_id"),
      BT::OutputPort<std::string>("goal_checker_id")};
  }

  void halt() override
  {
    // The simulator position controller retains its last target. Never send a
    // raising command here: halt is also called on cancellation INSIDE a roof.
    publishPolicy(STOP);
    haltChildren();
    phase_ = Phase::PLAN;
    initialized_ = false;
    waiting_for_state_ = false;
    user_goals_.clear();
    waypoint_index_ = 0;
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  enum class Phase {
    PLAN,
    PLAN_APPROACH,
    APPROACH,
    NORMAL_FOLLOW,
    STOPPING,
    ALIGNING,
    LOWERING,
    CLEARANCE,
    CENTERING,
    CROSSING,
    GUIDE_EXIT,
    RAISING
  };

  static double age(Steady::time_point point)
  {
    return std::chrono::duration<double>(Steady::now() - point).count();
  }
  bool stateFresh() const
  {
    return state_.size() >= 6 && age(state_received_) < config_.joint_timeout;
  }
  bool lowReady() const
  {
    return stateFresh() && state_[5] > 0.5 && age(lift_received_) < config_.joint_timeout &&
           age(pitch_received_) < config_.joint_timeout &&
           std::abs(lift_position_ - config_.lift_target) <= config_.lift_tolerance &&
           std::abs(pitch_) <= config_.pitch_tolerance;
  }
  const Tunnel * overlapping() const
  {
    if (!stateFresh()) {
      return nullptr;
    }
    for (const auto & tunnel : config_.tunnels) {
      if (config_.overlaps(tunnel, state_[0], state_[1], state_[2])) {
        return &tunnel;
      }
    }
    return nullptr;
  }
  const Tunnel * guideAt(double x, double y, bool goal_target = false) const
  {
    for (const auto & tunnel : config_.tunnels) {
      // The side-wall guide is longer than the roof. Exclude its open outer
      // endpoints so a completed escape cannot immediately trigger again.
      const double inset = goal_target ? -1e-6 : 0.03;
      if (
        x > tunnel.x_min - config_.wait_offset + inset &&
        x < tunnel.x_max + config_.exit_offset - inset &&
        std::abs(y - tunnel.centerY()) <= config_.goal_lateral_tolerance) {
        return &tunnel;
      }
    }
    return nullptr;
  }
  const Tunnel * currentGuide() const
  {
    return stateFresh() ? guideAt(state_[0], state_[1]) : nullptr;
  }
  double stagingY() const
  {
    return active_->centerY() - std::copysign(config_.staging_lateral_offset, active_->centerY());
  }
  void phase(Phase next, const std::string & label)
  {
    phase_ = next;
    clearance_paused_ = false;
    phase_started_ = Steady::now();
    stable_since_ = Steady::time_point{};
    std_msgs::msg::String message;
    message.data = label + (active_ ? ":" + active_->id : "");
    phase_pub_->publish(message);
    RCLCPP_INFO(io_->get_logger(), "Tunnel BT: %s", message.data.c_str());
  }
  void publishPolicy(Policy policy)
  {
    std_msgs::msg::Float64MultiArray message;
    double speed = config_.speed;
    double heading = heading_;
    if (policy == NORMAL) {
      // The open staging point is reached using ordinary, collision-checked
      // navigation. Holding one arrival heading for metres can lock the
      // simplified wheel cylinders against CAD floor seams. Only a narrow
      // guide owns the axis-aligned heading override.
      speed = phase_ == Phase::GUIDE_EXIT ? config_.approach_speed : 0.0;
    } else if (phase_ == Phase::CENTERING) {
      speed = config_.centering_speed;
    }
    message.data = {static_cast<double>(policy), heading, speed};
    policy_pub_->publish(message);
  }
  void commandLift(double position)
  {
    std_msgs::msg::Float64 message;
    message.data = position;
    lift_pub_->publish(message);
  }
  void levelGimbal()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = io_->now();
    message.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
    message.position = {0.0, 0.0};
    gimbal_pub_->publish(message);
  }
  bool settled(bool condition)
  {
    if (!condition) {
      stable_since_ = Steady::time_point{};
      return false;
    }
    if (stable_since_ == Steady::time_point{}) {
      stable_since_ = Steady::now();
    }
    return age(stable_since_) >= config_.settle_time;
  }
  geometry_msgs::msg::PoseStamped pose(double x, double y) const
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = config_.frame;
    result.header.stamp = io_->now();
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.z = std::sin(heading_ / 2.0);
    result.pose.orientation.w = std::cos(heading_ / 2.0);
    return result;
  }
  bool sameGoal(
    const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b) const
  {
    return a.header.frame_id == b.header.frame_id &&
           std::hypot(
             a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y) <
             0.001 &&
           std::abs(a.pose.orientation.z - b.pose.orientation.z) < 0.001 &&
           std::abs(a.pose.orientation.w - b.pose.orientation.w) < 0.001;
  }
  bool readGoal(geometry_msgs::msg::PoseStamped & goal)
  {
    if (getInput("goal", goal)) {
      return true;
    }
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    if (!getInput("goals", goals) || goals.empty()) {
      return false;
    }
    // Execute user waypoints one by one. Internal tunnel stage points are never
    // passed through Nav2 RemovePassedGoals' large default radius.
    if (goals != user_goals_) {
      user_goals_ = goals;
      waypoint_index_ = 0;
    }
    if (waypoint_index_ >= user_goals_.size()) {
      return false;
    }
    goal = user_goals_[waypoint_index_];
    return true;
  }
  BT::NodeStatus failure(const char * reason)
  {
    RCLCPP_ERROR(io_->get_logger(), "Tunnel BT stopped: %s", reason);
    publishPolicy(STOP);
    haltChildren();
    initialized_ = false;
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus finish()
  {
    publishPolicy(STOP);
    haltChildren();
    if (!user_goals_.empty() && waypoint_index_ + 1 < user_goals_.size()) {
      ++waypoint_index_;
      initialized_ = false;
      phase(Phase::PLAN, "next_waypoint");
      return BT::NodeStatus::RUNNING;
    }
    initialized_ = false;
    user_goals_.clear();
    waypoint_index_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
  void startPlan(const geometry_msgs::msg::PoseStamped & target, Phase next)
  {
    haltChild(0);
    auto planning_target = target;
    if (next == Phase::PLAN) {
      if (const auto * tunnel = guideAt(target.pose.position.x, target.pose.position.y, true)) {
        // The physical-footprint planner must receive the same safe target as
        // the guide follower. Keep final_goal_ untouched: completion still
        // checks the original user request within its ordinary XY tolerance.
        planning_target.pose.position.y = tunnel->centerY();
      }
    }
    setOutput("planning_goal", planning_target);
    phase(next, next == Phase::PLAN ? "planning" : "planning_approach");
  }
  void follow(const nav_msgs::msg::Path & path, bool tunnel)
  {
    execution_ = path;
    execution_.header.stamp = io_->now();
    setOutput("execution_path", execution_);
    setOutput(
      "controller_id",
      std::string(tunnel ? "TunnelFollowPath" : (active_ ? "TunnelApproach" : "FollowPath")));
    setOutput(
      "goal_checker_id",
      std::string(tunnel || active_ ? "tunnel_goal_checker" : "general_goal_checker"));
    haltChild(1);
  }
  const Tunnel * findCrossing(const nav_msgs::msg::Path & path) const
  {
    // Test segments, not only planner vertices: Theta* may output a long
    // straight segment with no vertex inside the 35-cm-long roof.
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      const int samples =
        std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.02)));
      for (int j = 0; j <= samples; ++j) {
        const double ratio = static_cast<double>(j) / samples;
        for (const auto & tunnel : config_.tunnels) {
          const double x = a.x + ratio * (b.x - a.x);
          const double y = a.y + ratio * (b.y - a.y);
          // A goal just outside a portal can still leave the vehicle's front
          // or rear under the roof. Detect the vehicle, not only its centre.
          if (y >= tunnel.y_min && y <= tunnel.y_max && config_.overlaps(tunnel, x, y, 0.0)) {
            return &tunnel;
          }
        }
      }
    }
    return nullptr;
  }
  void selectHeading()
  {
    heading_ =
      std::abs(angleDifference(0.0, state_[2])) < std::acos(-1.0) * 0.5 ? 0.0 : std::acos(-1.0);
  }
  bool normalRideHeight() const
  {
    return age(lift_received_) < config_.joint_timeout &&
           std::abs(lift_position_) <= config_.lift_tolerance;
  }
  bool beginGuideEscape(const Tunnel * tunnel)
  {
    active_ = tunnel;
    selectHeading();
    if (
      config_.overlaps(*active_, state_[0], state_[1], state_[2]) ||
      std::abs(angleDifference(heading_, state_[2])) >= 0.025) {
      // An arbitrary turn inside a narrow guide is not an escape strategy.
      return false;
    }
    direction_ = state_[0] > (active_->x_min + active_->x_max) * 0.5 ? 1 : -1;
    guide_only_ = normalRideHeight();
    guide_escape_attempted_ = true;
    phase(Phase::STOPPING, "stopping_to_leave_guide");
    return true;
  }
  bool startGuideExit()
  {
    const auto & requested = final_goal_.pose.position;
    double target_x =
      direction_ > 0 ? active_->x_max + config_.exit_offset : active_->x_min - config_.wait_offset;
    const double target_y = active_->centerY();
    const bool current_right = state_[0] > (active_->x_min + active_->x_max) * 0.5;
    const bool goal_right = requested.x > (active_->x_min + active_->x_max) * 0.5;
    guide_ends_at_goal_ = std::abs(requested.y - target_y) <= config_.goal_lateral_tolerance &&
                          requested.x >= active_->x_min - config_.wait_offset - 1e-6 &&
                          requested.x <= active_->x_max + config_.exit_offset + 1e-6 &&
                          current_right == goal_right &&
                          !config_.overlaps(*active_, requested.x, target_y, heading_);
    if (guide_ends_at_goal_) {
      target_x = requested.x;
    }
    if (
      config_.overlaps(*active_, state_[0], state_[1], state_[2]) ||
      config_.overlaps(*active_, target_x, target_y, heading_) ||
      (target_x > (active_->x_min + active_->x_max) * 0.5) != current_right ||
      std::abs(angleDifference(heading_, state_[2])) >= 0.025) {
      return false;
    }
    // A monotonic segment between these same-side endpoints cannot enter the
    // roof. The actual high-body polygon and all obstacles are still checked
    // by both the follower and the final output guard.
    crossing_goal_ = final_goal_;
    nav_msgs::msg::Path path;
    path.header.frame_id = config_.frame;
    path.header.stamp = io_->now();
    // First translate laterally at the open staging X, then travel axially.
    // A diagonal from the off-wall alignment area can cut the platform corner.
    double from_x = state_[0], from_y = state_[1];
    for (const auto & endpoint :
         {std::pair<double, double>{state_[0], target_y},
          std::pair<double, double>{target_x, target_y}}) {
      const int count = std::max(
        1, static_cast<int>(
             std::ceil(std::hypot(endpoint.first - from_x, endpoint.second - from_y) / 0.02)));
      for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / count;
        path.poses.push_back(
          pose(from_x + t * (endpoint.first - from_x), from_y + t * (endpoint.second - from_y)));
      }
      from_x = endpoint.first;
      from_y = endpoint.second;
    }
    follow(path, false);
    phase(Phase::GUIDE_EXIT, "following_axis_aligned_guide");
    return true;
  }
  bool startCrossing()
  {
    crossing_goal_ = final_goal_;
    double target_x =
      direction_ > 0 ? active_->x_max + config_.exit_offset : active_->x_min - config_.exit_offset;
    double target_y = active_->centerY();
    const auto & requested = final_goal_.pose.position;
    const bool goal_overlaps_roof = config_.overlaps(*active_, requested.x, requested.y, heading_);
    const bool close_to_centerline =
      std::abs(requested.y - target_y) <= config_.goal_lateral_tolerance;
    if (goal_overlaps_roof && !close_to_centerline) {
      // A diagonal target beside a roof must not be approached by cutting
      // through its side wall, nor by repeatedly leaving and re-entering it.
      return false;
    }
    crossing_ends_at_goal_ =
      close_to_centerline &&
      (goal_overlaps_roof || (direction_ * (requested.x - state_[0]) >= -1e-6 &&
                              direction_ * (requested.x - target_x) <= 1e-6));
    if (crossing_ends_at_goal_) {
      target_x = final_goal_.pose.position.x;
      // Honour the ordinary user-goal XY tolerance without steering 2--10 cm
      // sideways into the guide. The internal follower still reaches its
      // centreline target with the stricter tunnel goal checker.
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = config_.frame;
    path.header.stamp = io_->now();
    const double distance = std::hypot(target_x - state_[0], target_y - state_[1]);
    const int count = std::max(1, static_cast<int>(std::ceil(distance / 0.02)));
    for (int i = 0; i <= count; ++i) {
      const double t = static_cast<double>(i) / count;
      path.poses.push_back(
        pose(state_[0] + t * (target_x - state_[0]), state_[1] + t * (target_y - state_[1])));
    }
    follow(path, true);
    phase(Phase::CROSSING, "crossing");
    return true;
  }

  void startCentering()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = config_.frame;
    path.header.stamp = io_->now();
    path.poses = {pose(state_[0], state_[1]), pose(state_[0], active_->centerY())};
    follow(path, false);
    phase(Phase::CENTERING, "centering_before_entry");
  }

  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::RuntimeError("TunnelNavigation needs planner and follower");
    }
    rclcpp::spin_some(io_);
    geometry_msgs::msg::PoseStamped requested;
    if (!readGoal(requested)) {
      return failure("missing goal/goals input");
    }
    if (requested.header.frame_id != config_.frame) {
      return failure("navigation goals must be in the configured map frame");
    }
    // Do not select PLAN versus resume-inside until a fresh physical pose is
    // available. In particular, the first tick after a cancellation may happen
    // before this new IO subscription has received the guard heartbeat.
    if (!config_.tunnels.empty() && !stateFresh()) {
      if (!waiting_for_state_) {
        waiting_for_state_ = true;
        state_wait_started_ = Steady::now();
      }
      publishPolicy(STOP);
      if (age(state_wait_started_) > 5.0) {
        return failure("pose / guard feedback unavailable");
      }
      return BT::NodeStatus::RUNNING;
    }
    waiting_for_state_ = false;
    if (!initialized_) {
      initialized_ = true;
      final_goal_ = requested;
      active_ = nullptr;
      crossing_ends_at_goal_ = false;
      finish_after_raise_ = false;
      guide_only_ = false;
      guide_escape_attempted_ = false;
      guide_goal_attempted_ = false;
      phase_started_ = Steady::now();
      if (const auto * tunnel = overlapping()) {
        active_ = tunnel;
        direction_ = final_goal_.pose.position.x >= state_[0] ? 1 : -1;
        selectHeading();
        phase(Phase::STOPPING, "resume_inside_tunnel");
      } else {
        startPlan(final_goal_, Phase::PLAN);
      }
    } else if (!sameGoal(requested, final_goal_)) {
      final_goal_ = requested;
      guide_goal_attempted_ = false;
      finish_after_raise_ = false;
      const bool committed = phase_ == Phase::STOPPING || phase_ == Phase::ALIGNING ||
                             phase_ == Phase::LOWERING || phase_ == Phase::CLEARANCE ||
                             phase_ == Phase::CENTERING || phase_ == Phase::CROSSING ||
                             phase_ == Phase::GUIDE_EXIT || phase_ == Phase::RAISING;
      if (!committed && !overlapping()) {
        haltChildren();
        active_ = nullptr;
        guide_escape_attempted_ = false;
        guide_only_ = false;
        startPlan(final_goal_, Phase::PLAN);
      } else {
        RCLCPP_INFO(
          io_->get_logger(),
          "Tunnel BT: updated goal deferred until the committed passage safely exits");
      }
      // Once the maneuver is committed, also protect the narrow side-wall
      // approach before the roof. Never resume ordinary spin or change the
      // lift target there merely because a new goal arrived.
    }

    switch (phase_) {
      case Phase::PLAN:
      case Phase::PLAN_APPROACH: {
        publishPolicy(STOP);
        const auto result = children_nodes_[0]->executeTick();
        if (result == BT::NodeStatus::FAILURE) {
          haltChild(0);
          if (phase_ == Phase::PLAN && !guide_escape_attempted_) {
            if (const auto * tunnel = currentGuide()) {
              if (!beginGuideEscape(tunnel)) {
                return failure("cannot safely realign or escape the narrow guide");
              }
              return BT::NodeStatus::RUNNING;
            }
          }
          return failure("global planning failed");
        }
        if (result != BT::NodeStatus::SUCCESS) {
          return BT::NodeStatus::RUNNING;
        }
        nav_msgs::msg::Path path;
        if (!getInput("path", path) || path.poses.empty()) {
          return failure("empty planner path");
        }
        // A completed asynchronous BT action must be reset before the next
        // request, even if the following stage will execute a different child.
        haltChild(0);
        if (phase_ == Phase::PLAN_APPROACH) {
          // Use the metric waiting point rather than a rounded Theta* vertex.
          path.poses.back() = approach_goal_;
          follow(path, false);
          phase(Phase::APPROACH, "approach_wait_point");
          last_replan_ = Steady::now();
        } else if ((active_ = findCrossing(path))) {
          guide_only_ = false;
          direction_ = state_[0] < (active_->x_min + active_->x_max) * 0.5 ? 1 : -1;
          selectHeading();
          // A second goal can begin on the already-centred approach after a
          // previous crossing. Do not drive back out to the distant staging
          // point through a point-planner curve that clips the narrow walls.
          if (
            state_[0] >= active_->x_min - config_.wait_offset - 0.03 &&
            state_[0] <= active_->x_max + config_.exit_offset + 0.03 &&
            std::abs(state_[1] - active_->centerY()) <= 0.008 &&
            std::abs(angleDifference(heading_, state_[2])) < 0.025) {
            phase(Phase::STOPPING, "stopping_before_tunnel");
            return BT::NodeStatus::RUNNING;
          }
          const auto wait_pose = pose(
            direction_ > 0 ? active_->x_min - config_.wait_offset
                           : active_->x_max + config_.wait_offset,
            stagingY());
          approach_goal_ = wait_pose;
          startPlan(wait_pose, Phase::PLAN_APPROACH);
        } else if (const auto * tunnel = currentGuide()) {
          if (guide_escape_attempted_ || !beginGuideEscape(tunnel)) {
            return failure("cannot safely realign or escape the narrow guide");
          }
        } else if (
          const auto * tunnel =
            guideAt(final_goal_.pose.position.x, final_goal_.pose.position.y, true)) {
          active_ = tunnel;
          selectHeading();
          if (guide_goal_attempted_) {
            return failure("guide goal was not reached after its axis maneuver");
          }
          guide_goal_attempted_ = true;
          if (config_.overlaps(
                *active_, final_goal_.pose.position.x, active_->centerY(), heading_)) {
            return failure("planner did not provide a safe low-roof approach");
          }
          direction_ =
            final_goal_.pose.position.x > (active_->x_min + active_->x_max) * 0.5 ? 1 : -1;
          guide_only_ = normalRideHeight();
          approach_goal_ = pose(
            direction_ > 0 ? active_->x_max + config_.exit_offset
                           : active_->x_min - config_.wait_offset,
            stagingY());
          startPlan(approach_goal_, Phase::PLAN_APPROACH);
        } else {
          active_ = nullptr;
          guide_only_ = false;
          follow(path, false);
          phase(Phase::NORMAL_FOLLOW, "normal_follow");
          last_replan_ = Steady::now();
        }
        return BT::NodeStatus::RUNNING;
      }
      case Phase::APPROACH:
      case Phase::NORMAL_FOLLOW: {
        publishPolicy(NORMAL);
        const auto result = children_nodes_[1]->executeTick();
        if (result == BT::NodeStatus::FAILURE) {
          return failure("path following failed");
        }
        if (result == BT::NodeStatus::SUCCESS) {
          haltChild(1);
          if (phase_ == Phase::APPROACH) {
            phase(Phase::STOPPING, "stopping_before_tunnel");
          } else if (
            age(lift_received_) < config_.joint_timeout && lift_position_ > 0.005 &&
            !overlapping()) {
            finish_after_raise_ = true;
            phase(Phase::RAISING, "restore_at_goal");
          } else {
            return finish();
          }
        } else if (age(last_replan_) > 0.333) {
          // Ordinary navigation and the open staging approach may replan.
          // Committed alignment / low passage never use this branch.
          const auto planned = children_nodes_[0]->executeTick();
          if (planned == BT::NodeStatus::SUCCESS) {
            nav_msgs::msg::Path path;
            if (getInput("path", path) && !path.poses.empty()) {
              if (phase_ == Phase::NORMAL_FOLLOW && findCrossing(path)) {
                haltChild(1);
                startPlan(final_goal_, Phase::PLAN);
              } else {
                execution_ = path;
                setOutput("execution_path", execution_);
              }
            }
            haltChild(0);
            last_replan_ = Steady::now();
          } else if (planned == BT::NodeStatus::FAILURE) {
            haltChild(0);
            last_replan_ = Steady::now();
          }
        }
        return BT::NodeStatus::RUNNING;
      }
      case Phase::STOPPING:
        publishPolicy(STOP);
        if (settled(state_[3] < 0.025 && std::abs(state_[4]) < 0.05)) {
          phase(Phase::ALIGNING, "align_chassis_and_gimbal");
        } else if (age(phase_started_) > config_.lift_timeout) {
          return failure("robot did not stop");
        }
        break;
      case Phase::ALIGNING:
        publishPolicy(ALIGN);
        levelGimbal();
        if (settled(
              std::abs(angleDifference(heading_, state_[2])) < 0.025 &&
              std::abs(state_[4]) < 0.05 && age(pitch_received_) < config_.joint_timeout &&
              std::abs(pitch_) < config_.pitch_tolerance)) {
          if (guide_only_) {
            if (!startGuideExit()) {
              return failure("high-body guide path would enter the roof or require a turn");
            }
          } else {
            phase(Phase::LOWERING, "lowering");
          }
        } else if (age(phase_started_) > 30.0) {
          return failure("alignment timed out");
        }
        break;
      case Phase::LOWERING:
        publishPolicy(STOP);
        levelGimbal();
        commandLift(config_.lift_target);
        if (settled(lowReady())) {
          phase(Phase::CLEARANCE, "wait_local_clearance");
        } else if (age(phase_started_) > config_.lift_timeout) {
          return failure("lift did not reach lowered target");
        }
        break;
      case Phase::CLEARANCE:
        publishPolicy(STOP);
        commandLift(config_.lift_target);
        levelGimbal();
        if (settled(
              lowReady() && clearance_ready_ && clearance_received_ > phase_started_ &&
              age(clearance_received_) < 0.5)) {
          if (!overlapping() && std::abs(state_[1] - active_->centerY()) > 0.008) {
            startCentering();
          } else {
            if (!startCrossing()) {
              return failure("goal under roof is outside the safe centreline tolerance");
            }
          }
        } else if (age(phase_started_) > config_.lift_timeout) {
          return failure("local clearance map not ready");
        }
        break;
      case Phase::CENTERING:
      case Phase::CROSSING: {
        commandLift(config_.lift_target);
        levelGimbal();
        if (!lowReady() || !clearance_ready_ || age(clearance_received_) >= 0.5) {
          publishPolicy(STOP);
          if (!clearance_paused_) {
            haltChild(1);
            clearance_paused_ = true;
            clearance_pause_started_ = Steady::now();
            RCLCPP_WARN(
              io_->get_logger(),
              "Tunnel BT: paused low for valid lift, gimbal and fresh local clearance");
          }
          clearance_resume_since_ = Steady::time_point{};
          if (age(clearance_pause_started_) > config_.lift_timeout) {
            return failure("lift / gimbal / local clearance did not recover; held low");
          }
          return BT::NodeStatus::RUNNING;
        }
        if (clearance_paused_) {
          publishPolicy(STOP);
          if (clearance_resume_since_ == Steady::time_point{}) {
            clearance_resume_since_ = Steady::now();
          }
          if (age(clearance_resume_since_) < config_.settle_time) {
            return BT::NodeStatus::RUNNING;
          }
          clearance_paused_ = false;
          // Rebuild only the remaining straight stage. Re-sending a 4.15 m
          // original path late in the passage can exceed the controller's
          // nearest-point search horizon and make it target a point behind us.
          const auto target = execution_.poses.back().pose.position;
          nav_msgs::msg::Path remaining;
          remaining.header.frame_id = config_.frame;
          remaining.header.stamp = io_->now();
          const int count = std::max(
            1, static_cast<int>(
                 std::ceil(std::hypot(target.x - state_[0], target.y - state_[1]) / 0.02)));
          for (int i = 0; i <= count; ++i) {
            const double t = static_cast<double>(i) / count;
            remaining.poses.push_back(
              pose(state_[0] + t * (target.x - state_[0]), state_[1] + t * (target.y - state_[1])));
          }
          follow(remaining, phase_ != Phase::CENTERING);
          RCLCPP_INFO(
            io_->get_logger(), "Tunnel BT: valid passage feedback restored, resuming low");
          return BT::NodeStatus::RUNNING;
        }
        publishPolicy(TRAVERSE);
        const auto result = children_nodes_[1]->executeTick();
        if (result == BT::NodeStatus::FAILURE) {
          return failure("passage blocked; held low, no blind backup");
        }
        if (result == BT::NodeStatus::SUCCESS) {
          haltChild(1);
          publishPolicy(STOP);
          if (phase_ == Phase::CENTERING) {
            if (!startCrossing()) {
              return failure("goal under roof is outside the safe centreline tolerance");
            }
            break;
          }
          if (overlapping()) {
            if (crossing_ends_at_goal_) {
              if (sameGoal(crossing_goal_, final_goal_)) {
                return finish();
              }
              // The old requested stop was under the roof. A replacement goal
              // cannot succeed there: continue while low toward its safe exit
              // (or its new inside target), preserving the physical heading.
              direction_ = final_goal_.pose.position.x >= state_[0] ? 1 : -1;
              if (!startCrossing()) {
                return failure("goal under roof is outside the safe centreline tolerance");
              }
              return BT::NodeStatus::RUNNING;
            }
            return failure("follower stopped before complete footprint exited");
          }
          // A goal on the safe exit centreline needs no ordinary follower:
          // that follower would reintroduce the sentry's automatic spin while
          // the robot can still be between the approach side walls.
          finish_after_raise_ = crossing_ends_at_goal_ && sameGoal(crossing_goal_, final_goal_);
          phase(Phase::RAISING, "raising_after_exit");
        }
        break;
      }
      case Phase::GUIDE_EXIT: {
        if (
          overlapping() || std::abs(angleDifference(heading_, state_[2])) > 0.05 ||
          !normalRideHeight()) {
          return failure("high-body guide pose or height became unsafe");
        }
        levelGimbal();
        publishPolicy(NORMAL);
        const auto result = children_nodes_[1]->executeTick();
        if (result == BT::NodeStatus::FAILURE) {
          return failure("guide path blocked; no blind turn or backup");
        }
        if (result == BT::NodeStatus::SUCCESS) {
          haltChild(1);
          publishPolicy(STOP);
          if (guide_ends_at_goal_ && sameGoal(crossing_goal_, final_goal_)) {
            return finish();
          }
          if (currentGuide()) {
            // A replacement goal arrived while stopping at an old guide goal.
            // Keep the axis controller until its new target or the open exit.
            if (!startGuideExit()) {
              return failure("updated guide goal cannot be reached safely");
            }
          } else {
            active_ = nullptr;
            guide_only_ = false;
            startPlan(final_goal_, Phase::PLAN);
          }
        }
        break;
      }
      case Phase::RAISING:
        publishPolicy(STOP);
        if (overlapping()) {
          return failure("refusing to raise under a roof");
        }
        commandLift(0.0);
        if (settled(
              age(lift_received_) < config_.joint_timeout &&
              std::abs(lift_position_) <= config_.lift_tolerance)) {
          active_ = nullptr;
          if (finish_after_raise_) {
            return finish();
          }
          startPlan(final_goal_, Phase::PLAN);
        } else if (age(phase_started_) > config_.lift_timeout) {
          return failure("restoring ride height timed out");
        }
        break;
    }
    return BT::NodeStatus::RUNNING;
  }

  Config config_;
  Phase phase_{Phase::PLAN};
  const Tunnel * active_{nullptr};
  int direction_{1};
  double heading_{0.0}, lift_position_{0.0}, pitch_{0.0};
  bool initialized_{false}, clearance_ready_{false}, waiting_for_state_{false};
  bool crossing_ends_at_goal_{false}, finish_after_raise_{false};
  bool guide_only_{false}, guide_ends_at_goal_{false}, guide_escape_attempted_{false};
  bool guide_goal_attempted_{false};
  bool clearance_paused_{false};
  size_t waypoint_index_{0};
  geometry_msgs::msg::PoseStamped final_goal_, crossing_goal_, approach_goal_;
  std::vector<geometry_msgs::msg::PoseStamped> user_goals_;
  nav_msgs::msg::Path execution_;
  std::vector<double> state_;
  Steady::time_point state_received_{}, lift_received_{}, pitch_received_{};
  Steady::time_point phase_started_{}, stable_since_{}, last_replan_{}, clearance_received_{};
  Steady::time_point state_wait_started_{};
  Steady::time_point clearance_pause_started_{}, clearance_resume_since_{};
  rclcpp::Node::SharedPtr io_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr policy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
};
}  // namespace sentry_tunnel

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_tunnel::TunnelNavigation>("TunnelNavigation");
}
