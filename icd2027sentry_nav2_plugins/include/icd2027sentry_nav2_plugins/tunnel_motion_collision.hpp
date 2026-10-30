// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "icd2027sentry_nav2_plugins/tunnel_clearance.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace sentry_tunnel
{
// Ordinary sentry navigation adds a continuous spin after the controller. It
// may resume only when the *whole* physical rotation is safe, including after
// leaving a roof but while still between the approach side walls.
template <typename IsSafe>
PlanarVelocity collisionCheckedVelocity(
  Policy policy, double speed_limit, PlanarVelocity output, IsSafe is_safe,
  double actual_yaw_rate = 0.0)
{
  if (
    !std::isfinite(output.x) || !std::isfinite(output.y) || !std::isfinite(output.yaw_rate) ||
    !std::isfinite(actual_yaw_rate)) {
    return PlanarVelocity();
  }
  if (is_safe(output)) {
    return output;
  }
  if (policy == NORMAL && speed_limit == 0.0) {
    output.yaw_rate = 0.0;
    if (std::abs(actual_yaw_rate) > 0.05) {
      // First stop residual physical spin; a straight-line prediction cannot
      // certify translation while the chassis is still rotating underneath.
      return PlanarVelocity();
    }
    if (is_safe(output)) {
      return output;
    }
    // A constant-velocity extrapolation can hit the wall beyond a path bend
    // even though a shorter step along the current heading is safe. Returning
    // zero for every such command deadlocks the controller before it reaches
    // the bend. Retain the FULL sweep duration and physical footprint, reducing
    // only ordinary translation. Each accepted candidate certifies a prefix
    // of the exact same straight sweep; never invent a new motion direction or
    // suppress the BT's tunnel heading controller.
    if ((output.x != 0.0 || output.y != 0.0) && is_safe(PlanarVelocity())) {
      double safe_scale = 0.0;
      double unsafe_scale = 1.0;
      PlanarVelocity selected;
      for (int iteration = 0; iteration < 8; ++iteration) {
        const double scale = 0.5 * (safe_scale + unsafe_scale);
        const PlanarVelocity candidate{output.x * scale, output.y * scale, 0.0};
        if (is_safe(candidate)) {
          selected = candidate;
          safe_scale = scale;
        } else {
          unsafe_scale = scale;
        }
      }
      return selected;
    }
  }
  if (policy == TRAVERSE || (policy == NORMAL && speed_limit > 0.0)) {
    // The final guard observes a newer physical pose than the controller and
    // runs after the velocity smoother. Re-certify a slower translation here
    // instead of an abrupt all-stop when the previous command predicts past
    // the goal into the perimeter. Never discard the required heading.
    const PlanarVelocity stationary{0.0, 0.0, output.yaw_rate};
    if ((output.x != 0.0 || output.y != 0.0) && is_safe(stationary)) {
      double safe_scale = 0.0, unsafe_scale = 1.0;
      auto selected = stationary;
      for (int iteration = 0; iteration < 8; ++iteration) {
        const double scale = 0.5 * (safe_scale + unsafe_scale);
        const PlanarVelocity candidate{output.x * scale, output.y * scale, output.yaw_rate};
        if (is_safe(candidate)) {
          selected = candidate;
          safe_scale = scale;
        } else {
          unsafe_scale = scale;
        }
      }
      return selected;
    }
  }
  return PlanarVelocity();
}

struct MotionCollision
{
  enum class Reason { NONE, INVALID, OUTSIDE_MAP, EMPTY_FOOTPRINT, LETHAL, UNKNOWN };
  Reason reason{Reason::NONE};
  unsigned int cell_x{0}, cell_y{0};
  double x{0.0}, y{0.0}, time{0.0};
};

inline const char * collisionReasonName(MotionCollision::Reason reason)
{
  switch (reason) {
    case MotionCollision::Reason::NONE:
      return "none";
    case MotionCollision::Reason::INVALID:
      return "invalid geometry";
    case MotionCollision::Reason::OUTSIDE_MAP:
      return "outside local map";
    case MotionCollision::Reason::EMPTY_FOOTPRINT:
      return "empty footprint";
    case MotionCollision::Reason::LETHAL:
      return "lethal obstacle";
    case MotionCollision::Reason::UNKNOWN:
      return "unknown cell";
  }
  return "invalid reason";
}

inline bool freshMotionCostmap(int64_t stamp_ns, int64_t received_ns, int64_t now_ns)
{
  const double source_age = (now_ns - stamp_ns) * 1e-9;
  const double received_age = (now_ns - received_ns) * 1e-9;
  return stamp_ns > 0 && received_ns > 0 && source_age >= -0.05 && source_age <= 0.5 &&
         received_age >= 0.0 && received_age <= 0.5;
}

inline bool physicalFootprintCollision(
  nav2_costmap_2d::Costmap2D & costmap, const std::vector<std::pair<double, double>> & footprint,
  double x, double y, double yaw, MotionCollision * collision = nullptr)
{
  if (collision != nullptr) {
    *collision = MotionCollision();
    collision->x = x;
    collision->y = y;
  }
  if (footprint.size() < 3 || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    if (collision != nullptr) {
      collision->reason = MotionCollision::Reason::INVALID;
    }
    return true;
  }
  std::vector<geometry_msgs::msg::Point> shape;
  for (const auto & vertex : footprint) {
    geometry_msgs::msg::Point point;
    point.x = vertex.first;
    point.y = vertex.second;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      if (collision != nullptr) {
        collision->reason = MotionCollision::Reason::INVALID;
      }
      return true;
    }
    shape.push_back(point);
  }
  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(x, y, yaw, shape, oriented);
  std::vector<nav2_costmap_2d::MapLocation> polygon;
  for (const auto & vertex : oriented) {
    nav2_costmap_2d::MapLocation cell;
    if (!costmap.worldToMap(vertex.x, vertex.y, cell.x, cell.y)) {
      if (collision != nullptr) {
        collision->reason = MotionCollision::Reason::OUTSIDE_MAP;
        collision->x = vertex.x;
        collision->y = vertex.y;
      }
      return true;
    }
    polygon.push_back(cell);
  }
  std::vector<nav2_costmap_2d::MapLocation> filled;
  costmap.convexFillCells(polygon, filled);
  if (filled.empty()) {
    if (collision != nullptr) {
      collision->reason = MotionCollision::Reason::EMPTY_FOOTPRINT;
    }
    return true;
  }
  for (const auto & cell : filled) {
    const auto value = costmap.getCost(cell.x, cell.y);
    // Inflation is already accounted for by the explicit physical polygon.
    if (value == nav2_costmap_2d::LETHAL_OBSTACLE || value == nav2_costmap_2d::NO_INFORMATION) {
      if (collision != nullptr) {
        collision->reason = value == nav2_costmap_2d::LETHAL_OBSTACLE
                              ? MotionCollision::Reason::LETHAL
                              : MotionCollision::Reason::UNKNOWN;
        collision->cell_x = cell.x;
        collision->cell_y = cell.y;
        costmap.mapToWorld(cell.x, cell.y, collision->x, collision->y);
      }
      return true;
    }
  }
  return false;
}

// The velocity's translation is already rotated into the fixed costmap frame.
// Sample at most half a cell of translation + footprint vertex rotation, using
// the FINAL guard output rather than the controller's pre-guard angular speed.
inline bool sweptMotionCollision(
  nav2_costmap_2d::Costmap2D & costmap, const std::vector<std::pair<double, double>> & footprint,
  double x, double y, double yaw, const PlanarVelocity & velocity, double duration = 1.0,
  MotionCollision * collision = nullptr)
{
  if (collision != nullptr) {
    *collision = MotionCollision();
  }
  if (
    !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.yaw_rate) ||
    !std::isfinite(duration) || duration <= 0.0 || costmap.getResolution() <= 0.0) {
    if (collision != nullptr) {
      collision->reason = MotionCollision::Reason::INVALID;
    }
    return true;
  }
  double radius = 0.0;
  for (const auto & point : footprint) {
    radius = std::max(radius, std::hypot(point.first, point.second));
  }
  const double distance =
    duration * (std::hypot(velocity.x, velocity.y) + radius * std::abs(velocity.yaw_rate));
  const double step = std::max(1e-4, costmap.getResolution() * 0.5);
  const size_t samples = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / step)));
  for (size_t i = 0; i <= samples; ++i) {
    const double t = duration * static_cast<double>(i) / samples;
    if (physicalFootprintCollision(
          costmap, footprint, x + velocity.x * t, y + velocity.y * t, yaw + velocity.yaw_rate * t,
          collision)) {
      if (collision != nullptr) {
        collision->time = t;
      }
      return true;
    }
  }
  return false;
}
}  // namespace sentry_tunnel
