// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>

#include "icd2027sentry_nav2_plugins/footprint_grid_search.hpp"
#include "nav2_core/global_planner.hpp"

namespace icd2027sentry_nav2_plugins
{
class FootprintGridPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override {}
  void deactivate() override {}
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  sentry_tunnel::Config config_;
  std::string name_, chassis_frame_{"chassis"};
  double pose_timeout_{0.5}, cost_weight_{2.0}, guide_half_width_{0.05};
};
}  // namespace icd2027sentry_nav2_plugins
