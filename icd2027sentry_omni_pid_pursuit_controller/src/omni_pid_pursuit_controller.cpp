// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "icd2027sentry_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "icd2027sentry_omni_pid_pursuit_controller/swept_collision_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace icd2027sentry_omni_pid_pursuit_controller
{

void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_approach_scaling", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".tunnel_mode", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_footprint_collision", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_regulated_lookahead", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_min_lookahead_dist", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".chassis_frame", rclcpp::ParameterValue("chassis"));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_gain", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_prediction_time", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_path_horizon", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_allow_unknown", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_footprint", rclcpp::ParameterValue(std::vector<double>{}));

  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".linear_approach_scaling", linear_approach_scaling_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_,
      "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
  node->get_parameter(plugin_name_ + ".tunnel_mode", tunnel_mode_);
  node->get_parameter(plugin_name_ + ".use_footprint_collision", use_footprint_collision_);
  node->get_parameter(
    plugin_name_ + ".collision_regulated_lookahead", collision_regulated_lookahead_);
  node->get_parameter(
    plugin_name_ + ".collision_min_lookahead_dist", collision_min_lookahead_dist_);
  // Tunnel mode cannot fall back to a point check even if explicitly disabled.
  use_footprint_collision_ = use_footprint_collision_ || tunnel_mode_;
  node->get_parameter(plugin_name_ + ".chassis_frame", chassis_frame_);
  node->get_parameter(plugin_name_ + ".yaw_tolerance", yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_gain", yaw_gain_);
  node->get_parameter(plugin_name_ + ".collision_prediction_time", collision_prediction_time_);
  node->get_parameter(plugin_name_ + ".collision_path_horizon", collision_path_horizon_);
  node->get_parameter(plugin_name_ + ".collision_allow_unknown", collision_allow_unknown_);
  std::vector<double> footprint_xy;
  node->get_parameter(plugin_name_ + ".collision_footprint", footprint_xy);
  collision_footprint_.clear();
  if (footprint_xy.size() % 2 != 0) {
    throw std::invalid_argument("collision_footprint must contain x,y pairs");
  }
  for (size_t i = 0; i < footprint_xy.size(); i += 2) {
    geometry_msgs::msg::Point point;
    point.x = footprint_xy[i];
    point.y = footprint_xy[i + 1];
    collision_footprint_.push_back(point);
  }
  if (use_footprint_collision_) {
    validateFootprint();
  }

  node->get_parameter("controller_frequency", control_frequency);
  if (
    control_frequency <= 0.0 || v_linear_max_ <= 0.0 || yaw_tolerance_ <= 0.0 || yaw_gain_ <= 0.0 ||
    collision_prediction_time_ <= 0.0 || collision_path_horizon_ <= 0.0) {
    throw std::invalid_argument(
      "Controller frequency, speed and collision horizons must be positive");
  }
  if (!std::isfinite(collision_min_lookahead_dist_) || collision_min_lookahead_dist_ <= 0.0) {
    throw std::invalid_argument("collision_min_lookahead_dist must be positive and finite");
  }

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;
  speed_limit_ = 0.0;
  speed_limit_is_percentage_ = false;
  last_velocity_scaling_factor_ = 0.0;

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()
      ->create_publisher<visualization_msgs::msg::MarkerArray>(  // 初始化 MarkerArray Publisher
        "curvature_points_marker_array", rclcpp::QoS(10));

  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " icd2027sentry_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
}

void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(costmap_ros_->getGlobalFrameID(), pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot into costmap frame");
  }
  auto chassis_pose = robot_pose;
  if (use_footprint_collision_) {
    geometry_msgs::msg::PoseStamped origin;
    origin.header.frame_id = chassis_frame_;
    origin.header.stamp = pose.header.stamp;
    origin.pose.orientation.w = 1.0;
    if (!transformPose(costmap_ros_->getGlobalFrameID(), origin, chassis_pose)) {
      throw nav2_core::PlannerException(
        "Footprint collision checking requires the actual chassis transform");
    }
  }

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(velocity);

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

  if (tunnel_mode_) {
    // The local path is expressed in the virtual (gimbal_yaw_fake) frame.
    // Its orientation encodes the requested *physical chassis* heading.
    angle_to_goal = chassisHeadingError(
      angle_to_goal, tf2::getYaw(robot_pose.pose.orientation),
      tf2::getYaw(chassis_pose.pose.orientation));
    if (std::abs(angle_to_goal) > yaw_tolerance_) {
      lin_dist = 0.0;
    }
  } else if (use_rotate_to_heading_) {
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
      lin_dist = 0;
    }
  }

  auto lin_vel = move_pid_->calculate(lin_dist, 0);
  auto angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;
  if (tunnel_mode_) {
    angular_vel = std::clamp(yaw_gain_ * angle_to_goal, v_angular_min_, v_angular_max_);
  }

  applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);

  applyApproachVelocityScaling(transformed_plan, lin_vel);
  lin_vel = std::clamp(lin_vel, 0.0, effectiveLinearSpeedLimit(v_linear_max_));
  if (tunnel_mode_ && std::abs(angle_to_goal) > yaw_tolerance_) {
    // Clear any remaining integral output while alignment is required.
    lin_vel = 0.0;
    move_pid_->setSumError(0.0);
  }

  const bool regulate_command =
    use_footprint_collision_ && collision_regulated_lookahead_ && !tunnel_mode_;
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = transformed_plan.header;
  cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
  cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);
  cmd_vel.twist.angular.z = angular_vel;
  if (regulate_command) {
    // Checking a fixed one-metre portion of the global path as a hard gate
    // prevents even a safe approach to a corner. Instead certify the actual
    // executable velocity, using successively nearer interpolated carrots.
    // Tunnel trajectories deliberately keep their stricter straight-path gate.
    std::vector<geometry_msgs::msg::Point> carrots;
    for (double distance = lookahead_dist; distance > collision_min_lookahead_dist_;
         distance *= 0.5) {
      carrots.push_back(getLookAheadPoint(distance, transformed_plan, true).pose.position);
    }
    carrots.push_back(
      getLookAheadPoint(collision_min_lookahead_dist_, transformed_plan, true).pose.position);
    geometry_msgs::msg::Twist regulated;
    if (!collisionRegulatedTranslation(
          cmd_vel.twist, carrots,
          [&](const auto & candidate) {
            return !isCommandCollisionDetected(
              chassis_pose, candidate, tf2::getYaw(robot_pose.pose.orientation), false);
          },
          regulated)) {
      move_pid_->setSumError(0.0);
      throw nav2_core::PlannerException("No collision-free physical-footprint pursuit command");
    }
    cmd_vel.twist = regulated;
    return cmd_vel;
  }

  // Transform local frame to global frame to use in collision checking.
  nav_msgs::msg::Path costmap_frame_local_plan;
  costmap_frame_local_plan.header = chassis_pose.header;
  costmap_frame_local_plan.poses.push_back(chassis_pose);
  double checked_distance = 0.0;
  geometry_msgs::msg::Point previous;
  for (const auto & local_pose : transformed_plan.poses) {
    const auto & position = local_pose.pose.position;
    const double segment_length = std::hypot(position.x - previous.x, position.y - previous.y);
    const double remaining = collision_path_horizon_ - checked_distance;
    auto checked_pose = local_pose;
    if (segment_length > remaining) {
      const double ratio = remaining / segment_length;
      checked_pose.pose.position.x = previous.x + ratio * (position.x - previous.x);
      checked_pose.pose.position.y = previous.y + ratio * (position.y - previous.y);
    }
    geometry_msgs::msg::PoseStamped map_pose;
    if (!transformPose(costmap_ros_->getGlobalFrameID(), checked_pose, map_pose)) {
      throw nav2_core::PlannerException("Unable to transform collision path into costmap frame");
    }
    // Tunnel path orientations explicitly request physical headings. In a
    // non-tunnel holonomic approach the guard holds current physical yaw, so
    // ordinary planner orientations must not rotate the collision footprint.
    costmap_frame_local_plan.poses.push_back(chassisCollisionPose(
      map_pose, robot_pose, chassis_pose, tunnel_mode_ || !use_footprint_collision_));
    checked_distance += segment_length;
    if (checked_distance >= collision_path_horizon_) {
      break;
    }
    previous = position;
  }

  if (isCollisionDetected(costmap_frame_local_plan)) {
    throw nav2_core::PlannerException("Collision detected in the trajectory. Stopping the robot!");
  }
  if (tunnel_mode_) {
    geometry_msgs::msg::Twist regulated;
    if (!collisionRegulatedSpeed(
          cmd_vel.twist,
          [&](const auto & candidate) {
            return !isCommandCollisionDetected(
              chassis_pose, candidate, tf2::getYaw(robot_pose.pose.orientation), false);
          },
          regulated)) {
      throw nav2_core::PlannerException("No safe physical tunnel speed with required heading");
    }
    cmd_vel.twist = regulated;
  } else if (isCommandCollisionDetected(
               chassis_pose, cmd_vel.twist, tf2::getYaw(robot_pose.pose.orientation))) {
    throw nav2_core::PlannerException("Collision detected in the trajectory. Stopping the robot!");
  }

  return cmd_vel;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path.poses.empty() || path.header.frame_id.empty()) {
    throw nav2_core::PlannerException("Received empty path or path without a frame");
  }
  global_plan_ = path;
  if (use_footprint_collision_ && collision_regulated_lookahead_ && !tunnel_mode_) {
    // Nearest-vertex pruning on a two-pose path otherwise keeps the old start
    // behind the robot. Short adaptive carrots then target that point and
    // repeatedly reverse instead of making forward progress.
    global_plan_ = densifyTrackingPath(path, std::min(0.02, costmap_->getResolution() * 0.5));
  }
  if (tunnel_mode_) {
    move_pid_->setSumError(0.0);
    heading_pid_->setSumError(0.0);
    last_velocity_scaling_factor_ = 0.0;
  }
}

void OmniPidPursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    RCLCPP_WARN(logger_, "Ignoring invalid speed limit");
    return;
  }
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

double OmniPidPursuitController::effectiveLinearSpeedLimit(double configured_maximum) const
{
  if (speed_limit_ == 0.0) {
    return configured_maximum;
  }
  const double limit =
    speed_limit_is_percentage_ ? configured_maximum * speed_limit_ / 100.0 : speed_limit_;
  return std::min(configured_maximum, limit);
}

double OmniPidPursuitController::chassisHeadingError(
  double local_path_yaw, double virtual_base_yaw, double chassis_yaw)
{
  return std::remainder(local_path_yaw + virtual_base_yaw - chassis_yaw, 2.0 * M_PI);
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // Lambda to transform a PoseStamped from global frame to local
  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
      throw nav2_core::PlannerException("Unable to transform a global path pose");
    }
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan,
  bool force_interpolation)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (
    (force_interpolation || use_interpolation_) && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    pose.pose.orientation = goal_pose_it->pose.orientation;
    return pose;
  }

  return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  if (dr2 < 1e-12) {
    return p2;
  }
  double d = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(std::max(0.0, r * r * dr2 - d * d));
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}
bool OmniPidPursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool OmniPidPursuitController::isCollisionDetected(
  const nav_msgs::msg::Path & path, bool log_collision)
{
  auto costmap = costmap_ros_->getCostmap();
  if (use_footprint_collision_) {
    CollisionSample collision;
    const bool blocked = sweptFootprintCollision(
      *costmap, collision_footprint_, path, collision_allow_unknown_, &collision);
    if (blocked && log_collision) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "Footprint collision cost %u at (%.3f, %.3f), controller %s",
        collision.cost, collision.x, collision.y, plugin_name_.c_str());
    }
    return blocked;
  }
  if (path.poses.empty()) {
    return true;
  }
  // Preserve the ordinary controller's center/inflation semantics, but sample
  // every half cell rather than just ten planner poses. An out-of-map pose must
  // never hide a later obstacle or be treated as verified free space.
  auto previous = path.poses.front().pose.position;
  for (const auto & pose : path.poses) {
    const auto & point = pose.pose.position;
    const size_t samples = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(
           std::hypot(point.x - previous.x, point.y - previous.y) /
           (costmap->getResolution() * 0.5))));
    for (size_t i = 0; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / samples;
      unsigned int mx, my;
      if (!costmap->worldToMap(
            previous.x + ratio * (point.x - previous.x),
            previous.y + ratio * (point.y - previous.y), mx, my)) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000, "Collision sample is outside the local costmap (%.3f, %.3f)",
          previous.x + ratio * (point.x - previous.x), previous.y + ratio * (point.y - previous.y));
        return true;
      }
      const auto cost = costmap->getCost(mx, my);
      if (
        (cost == nav2_costmap_2d::NO_INFORMATION && !collision_allow_unknown_) ||
        (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
         cost != nav2_costmap_2d::NO_INFORMATION)) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000, "Collision sample cost %u at (%.3f, %.3f), controller %s",
          static_cast<unsigned int>(cost), previous.x + ratio * (point.x - previous.x),
          previous.y + ratio * (point.y - previous.y), plugin_name_.c_str());
        return true;
      }
    }
    previous = point;
  }
  return false;
}

bool OmniPidPursuitController::isCommandCollisionDetected(
  const geometry_msgs::msg::PoseStamped & chassis_pose, const geometry_msgs::msg::Twist & command,
  double command_frame_yaw, bool log_collision)
{
  nav_msgs::msg::Path predicted;
  predicted.header = chassis_pose.header;
  predicted.poses.push_back(chassis_pose);
  const double vx =
    command.linear.x * std::cos(command_frame_yaw) - command.linear.y * std::sin(command_frame_yaw);
  const double vy =
    command.linear.x * std::sin(command_frame_yaw) + command.linear.y * std::cos(command_frame_yaw);
  const double initial_yaw = tf2::getYaw(chassis_pose.pose.orientation);
  const size_t steps = std::max<size_t>(
    1, static_cast<size_t>(std::ceil(collision_prediction_time_ / control_duration_)));
  for (size_t i = 1; i <= steps; ++i) {
    const double time = collision_prediction_time_ * static_cast<double>(i) / steps;
    auto predicted_pose = chassis_pose;
    // The virtual frame cancels chassis yaw, so its translation axes remain
    // fixed while the physical chassis may rotate underneath it.
    predicted_pose.pose.position.x += vx * time;
    predicted_pose.pose.position.y += vy * time;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, initial_yaw + command.angular.z * time);
    predicted_pose.pose.orientation = tf2::toMsg(orientation);
    predicted.poses.push_back(predicted_pose);
  }
  return isCollisionDetected(predicted, log_collision);
}

void OmniPidPursuitController::validateFootprint() const
{
  if (collision_footprint_.size() < 3) {
    throw std::invalid_argument(
      "Physical footprint collision checking requires an explicit collision_footprint");
  }
  double sign = 0.0;
  for (size_t i = 0; i < collision_footprint_.size(); ++i) {
    const auto & a = collision_footprint_[i];
    const auto & b = collision_footprint_[(i + 1) % collision_footprint_.size()];
    const auto & c = collision_footprint_[(i + 2) % collision_footprint_.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y)) {
      throw std::invalid_argument("collision_footprint contains a non-finite coordinate");
    }
    const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    if (std::abs(cross) > 1e-10) {
      if (sign != 0.0 && sign * cross < 0.0) {
        throw std::invalid_argument("collision_footprint must be a convex polygon");
      }
      sign = cross;
    }
  }
  if (sign == 0.0) {
    throw std::invalid_argument("collision_footprint has zero area");
  }
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;

  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  if (linear_approach_scaling_) {
    linear_vel = linearApproachVelocity(
      linear_vel, v_linear_max_, min_approach_linear_velocity_, velocity_scaling);
    return;
  }
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

double OmniPidPursuitController::linearApproachVelocity(
  double velocity, double configured_maximum, double minimum_approach,
  double distance_scaling_factor)
{
  if (
    !std::isfinite(velocity) || !std::isfinite(configured_maximum) ||
    !std::isfinite(minimum_approach) || !std::isfinite(distance_scaling_factor) ||
    velocity <= 0.0 || configured_maximum <= 0.0 || minimum_approach < 0.0 ||
    distance_scaling_factor < 0.0) {
    return 0.0;
  }
  const double cap =
    std::max(minimum_approach, configured_maximum * std::min(1.0, distance_scaling_factor));
  return std::min(velocity, cap);
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  linear_vel = std::min(linear_vel, scaled_linear_vel);
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  double curvature = 1.0 / curvature_radius;
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

};  // namespace icd2027sentry_omni_pid_pursuit_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  icd2027sentry_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
