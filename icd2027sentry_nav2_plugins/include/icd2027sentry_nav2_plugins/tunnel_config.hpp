// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace sentry_tunnel
{
// Axis-aligned, CAD-verified low-clearance passage, in Config::frame.
struct Tunnel
{
  std::string id;
  double x_min{}, x_max{}, y_min{}, y_max{}, floor_z{}, ceiling_z{};
  double center_y{};
  bool contains(double x, double y, double margin = 0.0) const
  {
    return x >= x_min + margin && x <= x_max - margin && y >= y_min + margin && y <= y_max - margin;
  }
  double centerY() const { return center_y; }
};

struct Config
{
  std::string frame{"map"};
  double normal_height{0.288}, lowered_height{0.233}, height_margin{0.010};
  double lift_target{0.055}, lift_tolerance{0.002}, joint_timeout{0.5};
  double lift_timeout{5.0}, settle_time{0.25}, pitch_tolerance{0.08};
  // speed is the aligned passage cruise limit. Keep legacy speed-only files
  // usable, but let staging and precision centering have independent limits.
  double speed{0.2}, approach_speed{0.2}, centering_speed{0.2};
  // A user goal near the narrow guide is reached on its verified centreline.
  // This must remain no greater than the normal Nav2 XY goal tolerance.
  double goal_lateral_tolerance{0.10};
  double wait_offset{0.9}, exit_offset{0.9};
  double staging_lateral_offset{0.0};
  std::vector<Tunnel> tunnels;
  std::vector<std::pair<double, double>> footprint;

  static Config load(const std::string & path)
  {
    Config result;
    if (path.empty()) {
      return result;
    }
    const auto root = YAML::LoadFile(path);
    if (root["frame"]) {
      result.frame = root["frame"].as<std::string>();
    }
#define SENTRY_TUNNEL_LOAD(name)            \
  if (root[#name]) {                        \
    result.name = root[#name].as<double>(); \
  }
    SENTRY_TUNNEL_LOAD(normal_height)
    SENTRY_TUNNEL_LOAD(lowered_height)
    SENTRY_TUNNEL_LOAD(height_margin)
    SENTRY_TUNNEL_LOAD(lift_target)
    SENTRY_TUNNEL_LOAD(lift_tolerance)
    SENTRY_TUNNEL_LOAD(joint_timeout)
    SENTRY_TUNNEL_LOAD(lift_timeout)
    SENTRY_TUNNEL_LOAD(settle_time)
    SENTRY_TUNNEL_LOAD(pitch_tolerance)
    SENTRY_TUNNEL_LOAD(speed)
    result.approach_speed = result.speed;
    result.centering_speed = std::min(result.speed, 0.2);
    SENTRY_TUNNEL_LOAD(approach_speed)
    SENTRY_TUNNEL_LOAD(centering_speed)
    SENTRY_TUNNEL_LOAD(goal_lateral_tolerance)
    SENTRY_TUNNEL_LOAD(wait_offset)
    SENTRY_TUNNEL_LOAD(exit_offset)
    SENTRY_TUNNEL_LOAD(staging_lateral_offset)
#undef SENTRY_TUNNEL_LOAD
    for (const auto & point : root["footprint"]) {
      result.footprint.emplace_back(point[0].as<double>(), point[1].as<double>());
    }
    for (const auto & entry : root["tunnels"]) {
      Tunnel tunnel;
      tunnel.id = entry["id"].as<std::string>();
      tunnel.x_min = entry["x_min"].as<double>();
      tunnel.x_max = entry["x_max"].as<double>();
      tunnel.y_min = entry["y_min"].as<double>();
      tunnel.y_max = entry["y_max"].as<double>();
      tunnel.floor_z = entry["floor_z"].as<double>();
      tunnel.ceiling_z = entry["ceiling_z"].as<double>();
      tunnel.center_y =
        entry["center_y"] ? entry["center_y"].as<double>() : (tunnel.y_min + tunnel.y_max) * 0.5;
      if (!(tunnel.x_min < tunnel.x_max && tunnel.y_min < tunnel.y_max &&
            tunnel.ceiling_z - tunnel.floor_z > result.lowered_height + result.height_margin)) {
        throw std::runtime_error("Invalid or insufficient tunnel clearance: " + tunnel.id);
      }
      result.tunnels.push_back(tunnel);
    }
    if (
      !result.tunnels.empty() &&
      (result.footprint.size() < 3 || !std::isfinite(result.speed) || result.speed <= 0.0 ||
       !std::isfinite(result.approach_speed) || result.approach_speed <= 0.0 ||
       !std::isfinite(result.centering_speed) || result.centering_speed <= 0.0 ||
       !std::isfinite(result.goal_lateral_tolerance) || result.goal_lateral_tolerance < 0.0 ||
       result.goal_lateral_tolerance > 0.15 || !std::isfinite(result.staging_lateral_offset) ||
       result.staging_lateral_offset < 0.0 || result.staging_lateral_offset > 2.0 ||
       result.lift_target <= 0.0 || result.joint_timeout <= 0.0)) {
      throw std::runtime_error("Invalid tunnel robot dimensions or control limits");
    }
    return result;
  }

  double radius() const
  {
    double radius = 0.0;
    for (const auto & point : footprint) {
      radius = std::max(radius, std::hypot(point.first, point.second));
    }
    return radius;
  }

  // Conservative footprint AABB overlap, including all orientations and a
  // localization margin. False positives keep the robot low a little longer.
  bool overlaps(const Tunnel & tunnel, double x, double y, double yaw, double margin = 0.01) const
  {
    double lo_x = x, hi_x = x, lo_y = y, hi_y = y;
    for (const auto & point : footprint) {
      const double px = x + std::cos(yaw) * point.first - std::sin(yaw) * point.second;
      const double py = y + std::sin(yaw) * point.first + std::cos(yaw) * point.second;
      lo_x = std::min(lo_x, px);
      hi_x = std::max(hi_x, px);
      lo_y = std::min(lo_y, py);
      hi_y = std::max(hi_y, py);
    }
    return hi_x + margin >= tunnel.x_min && lo_x - margin <= tunnel.x_max &&
           hi_y + margin >= tunnel.y_min && lo_y - margin <= tunnel.y_max;
  }
};

enum Policy { NORMAL = 0, STOP = 1, ALIGN = 2, TRAVERSE = 3 };

inline double angleDifference(double target, double current)
{
  return std::remainder(target - current, 2.0 * std::acos(-1.0));
}
}  // namespace sentry_tunnel
