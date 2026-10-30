// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "icd2027sentry_nav2_plugins/tunnel_config.hpp"

namespace sentry_tunnel
{
struct JointSample
{
  double position{0.0};
  int64_t received_ns{0};
  int64_t stamp_ns{0};
  bool valid{false};
};

inline bool freshJoint(const JointSample & sample, int64_t now_ns, double timeout)
{
  const double received_age = (now_ns - sample.received_ns) * 1e-9;
  const double source_age = (now_ns - sample.stamp_ns) * 1e-9;
  return sample.valid && std::isfinite(sample.position) && sample.stamp_ns > 0 &&
         received_age >= 0.0 && received_age <= timeout && source_age >= -0.05 &&
         source_age <= timeout;
}

inline bool loweredReady(
  const Config & config, const JointSample & lift, const JointSample & pitch, int64_t now_ns)
{
  return freshJoint(lift, now_ns, config.joint_timeout) &&
         freshJoint(pitch, now_ns, config.joint_timeout) &&
         std::abs(lift.position - config.lift_target) <= config.lift_tolerance &&
         std::abs(pitch.position) <= config.pitch_tolerance;
}

inline bool knownRoofPoint(
  const Config & config, double map_x, double map_y, double map_z, double robot_height)
{
  if (!std::isfinite(map_x) || !std::isfinite(map_y) || !std::isfinite(map_z)) {
    return false;
  }
  for (const auto & tunnel : config.tunnels) {
    // The short axis is the travel direction for these CAD roof rectangles.
    // Include its entrance/exit edges: retaining either edge marks a wall across
    // the passage. Keep the lateral boundary strips to preserve the side walls.
    constexpr double boundary_inset = 0.01;
    constexpr double entrance_tolerance = 0.01;
    constexpr double roof_height_tolerance = 0.005;
    constexpr double roof_max_thickness = 0.055;
    const bool travel_along_x = tunnel.x_max - tunnel.x_min < tunnel.y_max - tunnel.y_min;
    const double x_margin = travel_along_x ? -entrance_tolerance : boundary_inset;
    const double y_margin = travel_along_x ? boundary_inset : -entrance_tolerance;
    if (
      map_x <= tunnel.x_min + x_margin || map_x >= tunnel.x_max - x_margin ||
      map_y <= tunnel.y_min + y_margin || map_y >= tunnel.y_max - y_margin) {
      continue;
    }
    // Never clear an XY column: lower objects and geometry outside the measured
    // roof height band must continue to mark the obstacle layer.
    if (
      map_z >= tunnel.floor_z + robot_height + config.height_margin &&
      map_z >= tunnel.ceiling_z - roof_height_tolerance &&
      map_z <= tunnel.ceiling_z + roof_max_thickness) {
      return true;
    }
  }
  return false;
}
struct PlanarVelocity
{
  double x{0.0}, y{0.0}, yaw_rate{0.0};
};

struct GuardConditions
{
  bool configured{false}, pose_valid{false}, policy_live{false}, command_live{false};
  bool lowered_ready{false}, clearance_ready{false}, inside_roof{false}, policy_seen{false};
  Policy policy{NORMAL};
  double yaw{0.0}, target_yaw{0.0}, speed_limit{0.0}, tunnel_speed{0.2};
};

// Pure policy enforcement; the supplied predictor consumes the already-limited
// velocity, in the unchanged command frame, and checks the future footprint.
template <typename PredictOverlap>
PlanarVelocity guardedVelocity(
  const GuardConditions & conditions, const PlanarVelocity & incoming,
  PredictOverlap predict_overlap)
{
  PlanarVelocity output;
  if (
    (conditions.configured && !conditions.pose_valid) ||
    ((conditions.policy != NORMAL || (conditions.configured && conditions.policy_seen)) &&
     !conditions.policy_live)) {
    return output;
  }
  if (conditions.policy == NORMAL && conditions.command_live) {
    output = incoming;
    if (conditions.configured && conditions.speed_limit > 0.0) {
      // A positive NORMAL speed limit is the BT's tunnel approach policy.
      // Disable the incoming chassis spin already at the approach, and align
      // in the open space rather than arriving at the portal still spinning.
      const double error = angleDifference(conditions.target_yaw, conditions.yaw);
      output.yaw_rate = std::clamp(2.0 * error, -0.3, 0.3);
      if (std::abs(error) > 0.05) {
        output.x = 0.0;
        output.y = 0.0;
      }
    }
  } else if (conditions.policy == ALIGN && conditions.pose_valid) {
    output.yaw_rate =
      std::clamp(2.0 * angleDifference(conditions.target_yaw, conditions.yaw), -0.3, 0.3);
  } else if (
    conditions.policy == TRAVERSE && conditions.command_live && conditions.lowered_ready &&
    conditions.clearance_ready) {
    output = incoming;
    output.yaw_rate =
      std::clamp(2.0 * angleDifference(conditions.target_yaw, conditions.yaw), -0.3, 0.3);
  }
  const double limit =
    conditions.policy == TRAVERSE
      ? std::min(
          conditions.tunnel_speed,
          conditions.speed_limit > 0.0 ? conditions.speed_limit : conditions.tunnel_speed)
      : conditions.speed_limit;
  const double norm = std::hypot(output.x, output.y);
  if (limit > 0.0 && norm > limit) {
    output.x *= limit / norm;
    output.y *= limit / norm;
  }
  if (conditions.configured) {
    if (conditions.policy == NORMAL && conditions.inside_roof) {
      return PlanarVelocity();
    }
    if (!conditions.lowered_ready && predict_overlap(output)) {
      if (conditions.policy != NORMAL || conditions.speed_limit <= 0.0) {
        // Only the BT-owned approach / guide may approach a roof at normal
        // height. Ordinary navigation still stops and must let the BT decide
        // whether this is a passage, not silently enter one while high.
        return PlanarVelocity();
      }
      const PlanarVelocity stationary{0.0, 0.0, output.yaw_rate};
      if ((output.x == 0.0 && output.y == 0.0) || predict_overlap(stationary)) {
        // A speed reduction cannot make an already-overlapping body or its
        // stationary rotation safe. Do not discard the BT's heading override.
        return PlanarVelocity();
      }
      double safe_scale = 0.0;
      double unsafe_scale = 1.0;
      PlanarVelocity selected = stationary;
      for (int iteration = 0; iteration < 8; ++iteration) {
        const double scale = 0.5 * (safe_scale + unsafe_scale);
        const PlanarVelocity candidate{
          output.x * scale, output.y * scale, output.yaw_rate};
        if (!predict_overlap(candidate)) {
          safe_scale = scale;
          selected = candidate;
        } else {
          unsafe_scale = scale;
        }
      }
      // The predictor still certifies its complete one-second roof sweep.
      // Selecting only a safe prefix of translation avoids a permanent stop
      // before a high-body goal that is itself completely outside the roof.
      return selected;
    }
  }
  return output;
}
}  // namespace sentry_tunnel
