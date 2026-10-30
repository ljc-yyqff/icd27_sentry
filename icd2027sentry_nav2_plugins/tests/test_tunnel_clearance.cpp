// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0

#include <limits>

#include "gtest/gtest.h"
#include "icd2027sentry_nav2_plugins/tunnel_clearance.hpp"
#include "tf2/LinearMath/Transform.h"

namespace
{
sentry_tunnel::Config tunnelConfig()
{
  sentry_tunnel::Config config;
  config.tunnels = {
    {"red", -2.675, -2.325, 3.06, 3.86, 0.0, 0.25},
    {"blue", 2.325, 2.675, -3.86, -3.06, 0.0, 0.25}};
  return config;
}

TEST(TunnelClearance, RoofOnlyClearsWithEnoughVerticalClearance)
{
  const auto config = tunnelConfig();
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 0.25, config.lowered_height));
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 0.27, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 0.25, config.normal_height));
  // A box beneath the roof and a separate high structure must remain obstacles.
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 0.20, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 0.40, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 2.5, 0.25, config.lowered_height));
}

TEST(TunnelClearance, EntranceEdgesClearButLateralWallBoundariesRemain)
{
  const auto config = tunnelConfig();
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(config, -2.675, 3.5, 0.25, config.lowered_height));
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(config, -2.325, 3.5, 0.25, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.065, 0.25, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.855, 0.25, config.lowered_height));
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(config, 2.5, -3.5, 0.25, config.lowered_height));
}

TEST(TunnelClearance, ClearanceIsRelativeToTheConfiguredFloor)
{
  auto config = tunnelConfig();
  config.tunnels[0].floor_z = 1.0;
  config.tunnels[0].ceiling_z = 1.25;
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 1.25, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 0.25, config.lowered_height));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(config, -2.5, 3.5, 1.20, config.lowered_height));
}

TEST(TunnelClearance, CloudCoordinatesMustBeTransformedIntoMap)
{
  const auto config = tunnelConfig();
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, 0.7);
  const tf2::Transform map_from_odom(rotation, tf2::Vector3(-1.2, 0.5, 0.1));
  const tf2::Vector3 map_roof(-2.5, 3.5, 0.25);
  const auto odom_roof = map_from_odom.inverse() * map_roof;
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(
    config, odom_roof.x(), odom_roof.y(), odom_roof.z(), config.lowered_height));
  const auto transformed_roof = map_from_odom * odom_roof;
  EXPECT_TRUE(sentry_tunnel::knownRoofPoint(
    config, transformed_roof.x(), transformed_roof.y(), transformed_roof.z(),
    config.lowered_height));
}

TEST(TunnelClearance, BothJointsMustBeFreshAndAtTheRequiredPose)
{
  const auto config = tunnelConfig();
  constexpr int64_t sample_ns = 10000000000;
  constexpr int64_t now_ns = 10200000000;
  sentry_tunnel::JointSample lift{0.055, sample_ns, sample_ns, true};
  sentry_tunnel::JointSample pitch{0.0, sample_ns, sample_ns, true};
  EXPECT_TRUE(sentry_tunnel::loweredReady(config, lift, pitch, now_ns));
  lift.position = 0.052;
  EXPECT_FALSE(sentry_tunnel::loweredReady(config, lift, pitch, now_ns));
  lift.position = 0.055;
  pitch.position = 0.09;
  EXPECT_FALSE(sentry_tunnel::loweredReady(config, lift, pitch, now_ns));
  pitch.position = 0.0;
  pitch.valid = false;
  EXPECT_FALSE(sentry_tunnel::loweredReady(config, lift, pitch, now_ns));
  pitch.valid = true;
  EXPECT_FALSE(sentry_tunnel::loweredReady(config, lift, pitch, 11000000000));
  pitch.received_ns = now_ns;
  pitch.stamp_ns = 9000000000;
  EXPECT_FALSE(sentry_tunnel::loweredReady(config, lift, pitch, now_ns));
}

TEST(TunnelClearance, TimeJumpsAndInvalidNumbersFailClosed)
{
  const auto config = tunnelConfig();
  sentry_tunnel::JointSample joint{0.055, 10000000000, 10000000000, true};
  EXPECT_FALSE(sentry_tunnel::freshJoint(joint, 9000000000, config.joint_timeout));
  joint.stamp_ns = 0;
  EXPECT_FALSE(sentry_tunnel::freshJoint(joint, 10200000000, config.joint_timeout));
  joint.stamp_ns = 10000000000;
  joint.position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sentry_tunnel::freshJoint(joint, 10200000000, config.joint_timeout));
  EXPECT_FALSE(sentry_tunnel::knownRoofPoint(
    config, -2.5, 3.5, std::numeric_limits<double>::quiet_NaN(), config.lowered_height));
}
}  // namespace
