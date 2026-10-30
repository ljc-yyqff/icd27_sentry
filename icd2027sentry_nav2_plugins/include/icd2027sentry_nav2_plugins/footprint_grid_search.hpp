// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include "icd2027sentry_nav2_plugins/tunnel_motion_collision.hpp"

namespace sentry_tunnel
{
struct GridPose
{
  double x{}, y{}, yaw{};
};

// Configuration-space search for a holonomic chassis. Inflation remains a
// preference, not a second physical robot envelope. Every grid vertex, edge,
// exact endpoint and simplified segment checks the FILLED physical polygon.
class FootprintGridSearch
{
public:
  FootprintGridSearch(
    nav2_costmap_2d::Costmap2D & map, const Config & config, double actual_yaw,
    double cost_weight = 2.0, double guide_half_width = 0.05, bool axis_guides = true)
  : map_(map),
    config_(config),
    actual_yaw_(actual_yaw),
    cost_weight_(cost_weight),
    guide_half_width_(guide_half_width),
    axis_guides_(axis_guides),
    width_(map.getSizeInCellsX()),
    height_(map.getSizeInCellsY()),
    validity_(width_ * height_, -1)
  {
    if (
      config_.footprint.size() < 3 || map_.getResolution() <= 0.0 || !std::isfinite(actual_yaw_) ||
      !std::isfinite(cost_weight_) || cost_weight_ < 0.0 || !std::isfinite(guide_half_width_) ||
      guide_half_width_ < 0.0) {
      throw std::runtime_error("Invalid footprint grid planner geometry or parameters");
    }
    axis_yaw_ =
      std::abs(angleDifference(0.0, actual_yaw_)) <= std::acos(-1.0) * 0.5 ? 0.0 : std::acos(-1.0);
  }

  double configurationYaw(double x, double y) const
  {
    if (!axis_guides_) {
      return actual_yaw_;
    }
    for (const auto & tunnel : config_.tunnels) {
      if (
        x >= tunnel.x_min - config_.wait_offset - 1e-6 &&
        x <= tunnel.x_max + config_.exit_offset + 1e-6 &&
        std::abs(y - tunnel.centerY()) <= guide_half_width_ + 1e-9) {
        // This is an explicit BT alignment guide, not an obstacle exemption.
        // Physical collisions and rotation sweeps still apply in this region.
        return axis_yaw_;
      }
    }
    return actual_yaw_;
  }

  bool safeSegment(const GridPose & from, const GridPose & to) const
  {
    if (
      !std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(from.yaw) ||
      !std::isfinite(to.x) || !std::isfinite(to.y) || !std::isfinite(to.yaw)) {
      return false;
    }
    const double angle = angleDifference(to.yaw, from.yaw);
    if (
      std::abs(angle) > 1e-6 && (!rotationSafeAt(from.x, from.y, from.yaw, to.yaw) ||
                                 !rotationSafeAt(to.x, to.y, from.yaw, to.yaw))) {
      // The BT stops before aligning. A transition which is safe ONLY while
      // translating and turning together cannot certify that stationary
      // alignment. Require the complete rotation at both edge endpoints.
      return false;
    }
    const double travel =
      std::hypot(to.x - from.x, to.y - from.y) + config_.radius() * std::abs(angle);
    const size_t samples =
      std::max<size_t>(1, static_cast<size_t>(std::ceil(travel / (map_.getResolution() * 0.5))));
    for (size_t i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      if (physicalFootprintCollision(
            map_, config_.footprint, from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t,
            from.yaw + angle * t)) {
        return false;
      }
    }
    return true;
  }

  std::vector<GridPose> plan(double start_x, double start_y, double goal_x, double goal_y)
  {
    if (
      !std::isfinite(start_x) || !std::isfinite(start_y) || !std::isfinite(goal_x) ||
      !std::isfinite(goal_y)) {
      throw std::runtime_error("Non-finite global planner endpoint");
    }
    try {
      return planProfile(start_x, start_y, goal_x, goal_y);
    } catch (const std::runtime_error &) {
      if (!axis_guides_) {
        throw;
      }
      // An axis profile can itself be unreachable although a normal route
      // exists, e.g. a non-axis-shaped obstacle outside the real passage.
      FootprintGridSearch ordinary(
        map_, config_, actual_yaw_, cost_weight_, guide_half_width_, false);
      return ordinary.plan(start_x, start_y, goal_x, goal_y);
    }
  }

private:
  bool rotationSafeAt(double x, double y, double from_yaw, double to_yaw) const
  {
    const double angle = angleDifference(to_yaw, from_yaw);
    const size_t samples = std::max<size_t>(
      1, static_cast<size_t>(
           std::ceil(config_.radius() * std::abs(angle) / (map_.getResolution() * 0.5))));
    for (size_t i = 0; i <= samples; ++i) {
      const double yaw = from_yaw + angle * static_cast<double>(i) / samples;
      if (physicalFootprintCollision(map_, config_.footprint, x, y, yaw)) {
        return false;
      }
    }
    return true;
  }

  std::vector<GridPose> planProfile(double start_x, double start_y, double goal_x, double goal_y)
  {
    unsigned int sx, sy, gx, gy;
    if (!map_.worldToMap(start_x, start_y, sx, sy) || !map_.worldToMap(goal_x, goal_y, gx, gy)) {
      throw std::runtime_error("Start or goal is outside the global costmap");
    }
    const GridPose start{start_x, start_y, actual_yaw_};
    const GridPose goal{goal_x, goal_y, configurationYaw(goal_x, goal_y)};
    if (!safeSegment(start, start) || !safeSegment(goal, goal)) {
      throw std::runtime_error("Start or goal collides with the physical chassis footprint");
    }
    const size_t start_index = index(sx, sy), goal_index = index(gx, gy);
    if (
      !cellSafe(start_index) || !cellSafe(goal_index) ||
      !safeSegment(start, cellPose(start_index)) || !safeSegment(cellPose(goal_index), goal)) {
      throw std::runtime_error("Exact start or goal has no safe connector to the grid");
    }

    struct QueueEntry
    {
      double estimate, cost;
      size_t index;
    };
    const auto later = [](const QueueEntry & a, const QueueEntry & b) {
      return a.estimate > b.estimate;
    };
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(later)> queue(later);
    std::vector<double> distances(width_ * height_, std::numeric_limits<double>::infinity());
    std::vector<size_t> parents(width_ * height_, width_ * height_);
    distances[start_index] = 0.0;
    const GridPose grid_goal = cellPose(goal_index);
    const auto heuristic = [&grid_goal](const GridPose & point) {
      return std::hypot(point.x - grid_goal.x, point.y - grid_goal.y);
    };
    queue.push({heuristic(cellPose(start_index)), 0.0, start_index});
    constexpr std::array<int, 8> dx{{1, 0, -1, 0, 1, -1, -1, 1}};
    constexpr std::array<int, 8> dy{{0, 1, 0, -1, 1, 1, -1, -1}};
    bool found = false;
    while (!queue.empty()) {
      const auto current = queue.top();
      queue.pop();
      if (current.cost > distances[current.index]) {
        continue;
      }
      if (current.index == goal_index) {
        found = true;
        break;
      }
      const int x = current.index % width_, y = current.index / width_;
      const auto from = cellPose(current.index);
      for (size_t direction = 0; direction < dx.size(); ++direction) {
        const int nx = x + dx[direction], ny = y + dy[direction];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) {
          continue;
        }
        const size_t next = index(nx, ny);
        if (!cellSafe(next)) {
          continue;
        }
        // A diagonal may not pass between two occupied configuration cells.
        if (
          dx[direction] != 0 && dy[direction] != 0 &&
          (!cellSafe(index(nx, y)) || !cellSafe(index(x, ny)))) {
          continue;
        }
        const auto to = cellPose(next);
        if (!safeSegment(from, to)) {
          continue;
        }
        const double candidate = current.cost + segmentCost(from, to);
        if (candidate + 1e-12 < distances[next]) {
          distances[next] = candidate;
          parents[next] = current.index;
          queue.push({candidate + heuristic(to), candidate, next});
        }
      }
    }
    if (!found) {
      throw std::runtime_error("No full-footprint collision-free global path");
    }

    std::vector<GridPose> route;
    for (size_t cell = goal_index;; cell = parents[cell]) {
      route.push_back(cellPose(cell));
      if (cell == start_index) {
        break;
      }
    }
    std::reverse(route.begin(), route.end());
    route.insert(route.begin(), start);
    route.push_back(goal);
    if (axis_guides_ && !btOwnsAxisGuide(route)) {
      // The BT aligns for a roof crossing, a current-guide escape, or a guide
      // goal. A route that merely clips a guide midway has no such action.
      // Ordinary FollowPath holds the ACTUAL yaw, so do not certify it using
      // an unexecuted rotation. Replan with that yaw everywhere instead.
      FootprintGridSearch ordinary(
        map_, config_, actual_yaw_, cost_weight_, guide_half_width_, false);
      return ordinary.plan(start_x, start_y, goal_x, goal_y);
    }
    return densify(simplify(route));
  }

  bool btOwnsAxisGuide(const std::vector<GridPose> & route) const
  {
    bool changes_yaw = false;
    for (const auto & pose : route) {
      changes_yaw = changes_yaw || std::abs(angleDifference(pose.yaw, actual_yaw_)) > 1e-6;
    }
    if (!changes_yaw) {
      return true;
    }
    for (const auto & tunnel : config_.tunnels) {
      for (const auto * endpoint : {&route.front(), &route.back()}) {
        const bool goal_endpoint = endpoint == &route.back();
        const double inset = goal_endpoint ? -1e-6 : 0.03;
        if (
          endpoint->x > tunnel.x_min - config_.wait_offset + inset &&
          endpoint->x < tunnel.x_max + config_.exit_offset - inset &&
          std::abs(endpoint->y - tunnel.centerY()) <= config_.goal_lateral_tolerance) {
          return true;
        }
      }
      // Raw A* vertices are at most one diagonal cell apart. Match the BT's
      // 2 cm segment samples rather than checking only the robot centre.
      for (size_t i = 1; i < route.size(); ++i) {
        const auto & from = route[i - 1];
        const auto & to = route[i];
        const int samples =
          std::max(1, static_cast<int>(std::ceil(std::hypot(to.x - from.x, to.y - from.y) / 0.02)));
        for (int j = 0; j <= samples; ++j) {
          const double ratio = static_cast<double>(j) / samples;
          const double x = from.x + ratio * (to.x - from.x);
          const double y = from.y + ratio * (to.y - from.y);
          if (y >= tunnel.y_min && y <= tunnel.y_max && config_.overlaps(tunnel, x, y, 0.0)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  size_t index(unsigned int x, unsigned int y) const { return y * width_ + x; }

  GridPose cellPose(size_t cell) const
  {
    GridPose result;
    map_.mapToWorld(cell % width_, cell / width_, result.x, result.y);
    result.yaw = configurationYaw(result.x, result.y);
    return result;
  }

  bool cellSafe(size_t cell)
  {
    if (validity_[cell] < 0) {
      const auto pose = cellPose(cell);
      validity_[cell] =
        physicalFootprintCollision(map_, config_.footprint, pose.x, pose.y, pose.yaw) ? 0 : 1;
    }
    return validity_[cell] != 0;
  }

  double segmentCost(const GridPose & from, const GridPose & to) const
  {
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const size_t samples =
      std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / (map_.getResolution() * 0.5))));
    double cost = 0.0;
    for (size_t i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      unsigned int x, y;
      if (!map_.worldToMap(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, x, y)) {
        return std::numeric_limits<double>::infinity();
      }
      const double scaled = map_.getCost(x, y) / 253.0;
      cost += 1.0 + cost_weight_ * scaled * scaled;
    }
    return distance * cost / (samples + 1);
  }

  std::vector<GridPose> simplify(const std::vector<GridPose> & route) const
  {
    std::vector<double> accumulated(route.size(), 0.0);
    for (size_t i = 1; i < route.size(); ++i) {
      accumulated[i] = accumulated[i - 1] + segmentCost(route[i - 1], route[i]);
    }
    std::vector<GridPose> result{route.front()};
    for (size_t i = 0; i + 1 < route.size();) {
      size_t selected = i + 1;
      // Keep safety-sensitive yaw/profile boundaries explicit. A shortcut may
      // never silently bypass an axis-guide alignment or leave/re-enter it.
      for (size_t j = i + 2; j < route.size(); ++j) {
        if (
          std::abs(angleDifference(route[j - 1].yaw, route[i].yaw)) > 1e-6 ||
          std::abs(angleDifference(route[j].yaw, route[i].yaw)) > 1e-6) {
          break;
        }
        if (
          segmentCost(route[i], route[j]) <= (accumulated[j] - accumulated[i]) * 1.02 + 1e-9 &&
          safeSegment(route[i], route[j])) {
          selected = j;
        }
      }
      result.push_back(route[selected]);
      i = selected;
    }
    return result;
  }

  std::vector<GridPose> densify(const std::vector<GridPose> & route) const
  {
    std::vector<GridPose> result{route.front()};
    const double step = std::min(0.02, map_.getResolution() * 0.5);
    for (size_t i = 1; i < route.size(); ++i) {
      const auto & from = route[i - 1];
      const auto & to = route[i];
      const double angle = angleDifference(to.yaw, from.yaw);
      const double travel =
        std::hypot(to.x - from.x, to.y - from.y) + config_.radius() * std::abs(angle);
      const size_t samples = std::max<size_t>(1, static_cast<size_t>(std::ceil(travel / step)));
      for (size_t j = 1; j <= samples; ++j) {
        const double t = static_cast<double>(j) / samples;
        result.push_back(
          {from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.yaw + angle * t});
      }
    }
    return result;
  }

  nav2_costmap_2d::Costmap2D & map_;
  const Config & config_;
  double actual_yaw_, cost_weight_, guide_half_width_, axis_yaw_{};
  bool axis_guides_;
  unsigned int width_, height_;
  std::vector<int8_t> validity_;
};
}  // namespace sentry_tunnel
