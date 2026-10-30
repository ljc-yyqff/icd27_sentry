// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0

#include "icd2027sentry_nav2_plugins/planners/footprint_grid_planner.hpp"

#include <mutex>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace icd2027sentry_nav2_plugins
{
void FootprintGridPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  const auto node = parent_.lock();
  if (!node || !tf_ || !costmap_ros_) {
    throw nav2_core::PlannerException("FootprintGridPlanner has no lifecycle node, TF or costmap");
  }
  const auto declare = [this, &node](const std::string & suffix, const auto & value) {
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + "." + suffix, rclcpp::ParameterValue(value));
  };
  declare("tunnel_config_file", std::string());
  declare("chassis_frame", chassis_frame_);
  declare("pose_timeout", pose_timeout_);
  declare("cost_weight", cost_weight_);
  declare("guide_half_width", guide_half_width_);
  std::string config_file;
  node->get_parameter(name_ + ".tunnel_config_file", config_file);
  node->get_parameter(name_ + ".chassis_frame", chassis_frame_);
  node->get_parameter(name_ + ".pose_timeout", pose_timeout_);
  node->get_parameter(name_ + ".cost_weight", cost_weight_);
  node->get_parameter(name_ + ".guide_half_width", guide_half_width_);
  config_ = sentry_tunnel::Config::load(config_file);
  if (
    config_.footprint.size() < 3 || config_.frame != costmap_ros_->getGlobalFrameID() ||
    !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0 || guide_half_width_ < 0.0 ||
    guide_half_width_ > 2.0) {
    throw nav2_core::PlannerException("FootprintGridPlanner needs a valid shared physical profile");
  }
  RCLCPP_INFO(
    node->get_logger(), "%s uses continuous full-footprint configuration-space A*", name_.c_str());
}

void FootprintGridPlanner::cleanup()
{
  tf_.reset();
  costmap_ros_.reset();
  parent_.reset();
}

nav_msgs::msg::Path FootprintGridPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  const auto node = parent_.lock();
  if (!node || !tf_ || !costmap_ros_) {
    throw nav2_core::PlannerException("FootprintGridPlanner is not configured");
  }
  const auto now = node->now();
  const std::string frame = costmap_ros_->getGlobalFrameID();
  if (start.header.frame_id != frame || goal.header.frame_id != frame) {
    throw nav2_core::PlannerException("FootprintGridPlanner endpoints must be in the global frame");
  }
  double yaw;
  try {
    const auto pose = tf_->lookupTransform(frame, chassis_frame_, tf2::TimePointZero);
    const double age = (now - rclcpp::Time(pose.header.stamp, now.get_clock_type())).seconds();
    if (age < -0.05 || age > pose_timeout_) {
      throw nav2_core::PlannerException(
        "Physical chassis TF is stale; refusing an assumed footprint yaw");
    }
    yaw = tf2::getYaw(pose.transform.rotation);
  } catch (const tf2::TransformException & error) {
    throw nav2_core::PlannerException(
      std::string("Physical chassis TF unavailable: ") + error.what());
  }
  auto * map = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  std::vector<sentry_tunnel::GridPose> route;
  try {
    sentry_tunnel::FootprintGridSearch search(*map, config_, yaw, cost_weight_, guide_half_width_);
    route = search.plan(
      start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
  } catch (const std::runtime_error & error) {
    throw nav2_core::PlannerException(error.what());
  }
  nav_msgs::msg::Path result;
  result.header.frame_id = frame;
  result.header.stamp = now;
  result.poses.reserve(route.size());
  for (const auto & point : route) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.z = std::sin(point.yaw * 0.5);
    pose.pose.orientation.w = std::cos(point.yaw * 0.5);
    result.poses.push_back(pose);
  }
  // Never silently move a requested goal to a nearby free cell. Path headings
  // are a control hint; the exact endpoint messages remain the user's poses.
  result.poses.front() = start;
  result.poses.back() = goal;
  return result;
}
}  // namespace icd2027sentry_nav2_plugins

PLUGINLIB_EXPORT_CLASS(icd2027sentry_nav2_plugins::FootprintGridPlanner, nav2_core::GlobalPlanner)
