// Copyright 2026 ICD Sentry Contributors
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

#ifndef ICD2027SENTRY_OMNI_PID_PURSUIT_CONTROLLER__SWEPT_COLLISION_CHECKER_HPP_
#define ICD2027SENTRY_OMNI_PID_PURSUIT_CONTROLLER__SWEPT_COLLISION_CHECKER_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace icd2027sentry_omni_pid_pursuit_controller
{
struct CollisionSample
{
  double x{0.0}, y{0.0};
  unsigned int cost{255};
};

// Keep a tunnel's required heading, while regulating a constant-velocity
// prediction that would otherwise continue past its goal into a distant wall.
// The path itself remains a separate hard gate; every candidate uses the full
// existing physical polygon and prediction duration.
template <typename IsSafe>
bool collisionRegulatedSpeed(
  const geometry_msgs::msg::Twist & requested, IsSafe is_safe, geometry_msgs::msg::Twist & output)
{
  output = geometry_msgs::msg::Twist();
  if (
    !std::isfinite(requested.linear.x) || !std::isfinite(requested.linear.y) ||
    !std::isfinite(requested.angular.z)) {
    return false;
  }
  if (is_safe(requested)) {
    output = requested;
    return true;
  }
  auto stopped = requested;
  stopped.linear.x = stopped.linear.y = 0.0;
  if (!is_safe(stopped)) {
    return false;
  }
  output = stopped;
  double safe = 0.0, unsafe = 1.0;
  for (int i = 0; i < 8; ++i) {
    const double scale = (safe + unsafe) * 0.5;
    auto candidate = requested;
    candidate.linear.x *= scale;
    candidate.linear.y *= scale;
    if (is_safe(candidate)) {
      output = candidate;
      safe = scale;
    } else {
      unsafe = scale;
    }
  }
  auto margin = output;
  margin.linear.x *= 0.9;
  margin.linear.y *= 0.9;
  if (is_safe(margin)) {
    output = margin;
  }
  return true;
}

inline nav_msgs::msg::Path densifyTrackingPath(const nav_msgs::msg::Path & path, double spacing)
{
  if (path.poses.empty() || !std::isfinite(spacing) || spacing <= 0.0) {
    throw std::invalid_argument("Invalid tracking path or spacing");
  }
  auto result = path;
  result.poses.clear();
  result.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & from = path.poses[i - 1];
    const auto & to = path.poses[i];
    const double dx = to.pose.position.x - from.pose.position.x;
    const double dy = to.pose.position.y - from.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) {
      throw std::invalid_argument("Nonfinite tracking path");
    }
    const double yaw = tf2::getYaw(from.pose.orientation);
    const double angle = std::remainder(tf2::getYaw(to.pose.orientation) - yaw, 2.0 * M_PI);
    const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / spacing)));
    for (size_t j = 1; j <= steps; ++j) {
      const double t = static_cast<double>(j) / steps;
      auto point = to;
      point.pose.position.x = from.pose.position.x + t * dx;
      point.pose.position.y = from.pose.position.y + t * dy;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw + t * angle);
      point.pose.orientation = tf2::toMsg(orientation);
      result.poses.push_back(point);
    }
  }
  return result;
}

// A holonomic pursuit command can cut across the inside of a corner although
// the planner's polyline is clear. Try progressively nearer points ON that
// polyline and reduce speed only if their complete predicted sweep is safe.
// This is not a wall-clearing or arbitrary-heading escape: every accepted
// command is checked with the same physical polygon and collision horizon.
template <typename IsSafe>
bool collisionRegulatedTranslation(
  const geometry_msgs::msg::Twist & requested,
  const std::vector<geometry_msgs::msg::Point> & carrots, IsSafe is_safe,
  geometry_msgs::msg::Twist & output)
{
  const double speed = std::hypot(requested.linear.x, requested.linear.y);
  if (!std::isfinite(speed) || !std::isfinite(requested.angular.z) || carrots.empty()) {
    return false;
  }
  if (speed < 1e-8) {
    output = requested;
    return is_safe(output);
  }
  for (const double scale : {1.0, 0.75, 0.5, 0.25, 0.125, 0.0625}) {
    for (const auto & carrot : carrots) {
      const double distance = std::hypot(carrot.x, carrot.y);
      if (!std::isfinite(distance) || distance < 1e-8) {
        continue;
      }
      auto candidate = requested;
      candidate.linear.x = speed * scale * carrot.x / distance;
      candidate.linear.y = speed * scale * carrot.y / distance;
      if (is_safe(candidate)) {
        output = candidate;
        return true;
      }
    }
  }
  return false;
}

// Planner poses refer to the navigation's virtual base, while the physical
// footprint is centered on the chassis. During a guarded holonomic approach,
// the chassis holds its current heading even when the path moves sideways;
// a planner's path orientation must not invent a physical chassis rotation.
inline geometry_msgs::msg::PoseStamped chassisCollisionPose(
  const geometry_msgs::msg::PoseStamped & path_pose,
  const geometry_msgs::msg::PoseStamped & virtual_base_pose,
  const geometry_msgs::msg::PoseStamped & chassis_pose, bool use_path_heading)
{
  auto result = path_pose;
  result.pose.position.x += chassis_pose.pose.position.x - virtual_base_pose.pose.position.x;
  result.pose.position.y += chassis_pose.pose.position.y - virtual_base_pose.pose.position.y;
  if (!use_path_heading) {
    result.pose.orientation = chassis_pose.pose.orientation;
  }
  return result;
}

// Fill the polygon, rather than only testing its outline: an obstacle entirely
// underneath the robot must also stop it. Footprints must be convex.
inline bool footprintCollisionAtPose(
  nav2_costmap_2d::Costmap2D & costmap, const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw, bool allow_unknown = false, CollisionSample * collision = nullptr)
{
  if (footprint.size() < 3) {
    return true;
  }
  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(x, y, yaw, footprint, oriented);
  std::vector<nav2_costmap_2d::MapLocation> polygon;
  for (const auto & vertex : oriented) {
    nav2_costmap_2d::MapLocation cell;
    if (!costmap.worldToMap(vertex.x, vertex.y, cell.x, cell.y)) {
      if (collision) {
        *collision = {vertex.x, vertex.y, 256};
      }
      return true;
    }
    polygon.push_back(cell);
  }
  std::vector<nav2_costmap_2d::MapLocation> filled;
  costmap.convexFillCells(polygon, filled);
  for (const auto & cell : filled) {
    const auto cost = costmap.getCost(cell.x, cell.y);
    if (
      cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown)) {
      if (collision) {
        costmap.mapToWorld(cell.x, cell.y, collision->x, collision->y);
        collision->cost = cost;
      }
      return true;
    }
  }
  return false;
}

// A path supplied by a planner can have arbitrarily sparse poses. Interpolate
// both translation and rotation so a footprint vertex moves at most half a cell
// per check, including during a turn in place.
inline bool sweptFootprintCollision(
  nav2_costmap_2d::Costmap2D & costmap, const std::vector<geometry_msgs::msg::Point> & footprint,
  const nav_msgs::msg::Path & path, bool allow_unknown = false,
  CollisionSample * collision = nullptr)
{
  if (path.poses.empty() || footprint.size() < 3) {
    return true;
  }
  double radius = 0.0;
  for (const auto & vertex : footprint) {
    radius = std::max(radius, std::hypot(vertex.x, vertex.y));
  }
  const double step = std::max(1e-4, costmap.getResolution() * 0.5);
  auto previous = path.poses.front().pose;
  if (footprintCollisionAtPose(
        costmap, footprint, previous.position.x, previous.position.y,
        tf2::getYaw(previous.orientation), allow_unknown, collision)) {
    return true;
  }
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & next = path.poses[i].pose;
    const double previous_yaw = tf2::getYaw(previous.orientation);
    const double delta_yaw =
      std::remainder(tf2::getYaw(next.orientation) - previous_yaw, 2.0 * M_PI);
    const double distance =
      std::hypot(next.position.x - previous.position.x, next.position.y - previous.position.y);
    const size_t samples = std::max<size_t>(
      1, static_cast<size_t>(std::ceil((distance + radius * std::abs(delta_yaw)) / step)));
    for (size_t j = 1; j <= samples; ++j) {
      const double ratio = static_cast<double>(j) / samples;
      if (footprintCollisionAtPose(
            costmap, footprint,
            previous.position.x + ratio * (next.position.x - previous.position.x),
            previous.position.y + ratio * (next.position.y - previous.position.y),
            previous_yaw + ratio * delta_yaw, allow_unknown, collision)) {
        return true;
      }
    }
    previous = next;
  }
  return false;
}

}  // namespace icd2027sentry_omni_pid_pursuit_controller

#endif  // ICD2027SENTRY_OMNI_PID_PURSUIT_CONTROLLER__SWEPT_COLLISION_CHECKER_HPP_
