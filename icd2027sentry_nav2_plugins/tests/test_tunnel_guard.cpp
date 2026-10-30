// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0

#include <functional>
#include <limits>

#include "gtest/gtest.h"
#include "icd2027sentry_nav2_plugins/tunnel_clearance.hpp"
#include "icd2027sentry_nav2_plugins/tunnel_motion_collision.hpp"

namespace
{
using sentry_tunnel::GuardConditions;
using sentry_tunnel::PlanarVelocity;

GuardConditions liveConditions()
{
  GuardConditions conditions;
  conditions.configured = true;
  conditions.pose_valid = true;
  conditions.policy_live = true;
  conditions.command_live = true;
  conditions.lowered_ready = true;
  conditions.clearance_ready = true;
  return conditions;
}

const auto no_overlap = [](const PlanarVelocity &) { return false; };

void expectStopped(const PlanarVelocity & command)
{
  EXPECT_DOUBLE_EQ(command.x, 0.0);
  EXPECT_DOUBLE_EQ(command.y, 0.0);
  EXPECT_DOUBLE_EQ(command.yaw_rate, 0.0);
}

TEST(TunnelGuard, NoCommandNeverCreatesSpin)
{
  auto conditions = liveConditions();
  conditions.command_live = false;
  conditions.policy_live = false;
  const PlanarVelocity old_command{0.0, 0.0, 3.14};
  expectStopped(sentry_tunnel::guardedVelocity(conditions, old_command, no_overlap));
}

TEST(TunnelGuard, StopAndLostTraverseHeartbeatOverrideIncomingSpin)
{
  auto conditions = liveConditions();
  const PlanarVelocity command{2.0, 1.0, 3.14};
  conditions.policy = sentry_tunnel::STOP;
  expectStopped(sentry_tunnel::guardedVelocity(conditions, command, no_overlap));
  conditions.policy = sentry_tunnel::TRAVERSE;
  conditions.policy_live = false;
  expectStopped(sentry_tunnel::guardedVelocity(conditions, command, no_overlap));
}

TEST(TunnelGuard, AlignmentDiscardsTranslationAndUsesActualHeading)
{
  auto conditions = liveConditions();
  conditions.policy = sentry_tunnel::ALIGN;
  conditions.yaw = 1.0;
  conditions.target_yaw = 1.1;
  const auto output =
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{2.0, 1.0, 3.14}, no_overlap);
  EXPECT_DOUBLE_EQ(output.x, 0.0);
  EXPECT_DOUBLE_EQ(output.y, 0.0);
  EXPECT_NEAR(output.yaw_rate, 0.2, 1e-12);
  conditions.target_yaw = 2.0;
  EXPECT_DOUBLE_EQ(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity(), no_overlap).yaw_rate, 0.3);
}

TEST(TunnelGuard, TraversePreservesCommandCoordinatesAndLimitsTheirNorm)
{
  auto conditions = liveConditions();
  conditions.policy = sentry_tunnel::TRAVERSE;
  conditions.speed_limit = 0.2;
  conditions.target_yaw = 0.05;
  const auto output =
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{3.0, 4.0, 3.14}, no_overlap);
  EXPECT_NEAR(output.x, 0.12, 1e-12);
  EXPECT_NEAR(output.y, 0.16, 1e-12);
  EXPECT_NEAR(output.yaw_rate, 0.1, 1e-12);
  conditions.speed_limit = 0.0;
  EXPECT_NEAR(
    std::hypot(
      sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{3.0, 4.0, 3.14}, no_overlap).x,
      sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{3.0, 4.0, 3.14}, no_overlap).y),
    0.2, 1e-12);
}

TEST(TunnelGuard, MissingPoseOrLiftFeedbackStopsTraversal)
{
  auto conditions = liveConditions();
  conditions.policy = sentry_tunnel::TRAVERSE;
  conditions.lowered_ready = false;
  expectStopped(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.1, 0.0, 0.0}, no_overlap));
  conditions.lowered_ready = true;
  conditions.pose_valid = false;
  expectStopped(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.1, 0.0, 0.0}, no_overlap));
}

TEST(TunnelGuard, LostLocalMapReadinessStopsWithLiftAndPoseStillValid)
{
  auto conditions = liveConditions();
  conditions.policy = sentry_tunnel::TRAVERSE;
  conditions.clearance_ready = false;
  expectStopped(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.1, 0.0, 0.0}, no_overlap));
}

TEST(TunnelGuard, LostNavigatorHeartbeatStopsNormalAfterPolicyWasSeen)
{
  auto conditions = liveConditions();
  conditions.policy_seen = true;
  conditions.policy_live = false;
  expectStopped(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.5, 0.0, 3.14}, no_overlap));
  // Before the navigator ever publishes a policy, manual commands remain possible.
  conditions.policy_seen = false;
  EXPECT_DOUBLE_EQ(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.5, 0.0, 0.0}, no_overlap).x, 0.5);
}

TEST(TunnelGuard, ApproachPredictionUsesLimitedSpeedAndStopsBeforeRoof)
{
  auto conditions = liveConditions();
  conditions.lowered_ready = false;
  conditions.speed_limit = 0.2;
  bool predictor_called = false;
  const auto output = sentry_tunnel::guardedVelocity(
    conditions, PlanarVelocity{3.0, 4.0, 0.0},
    [&predictor_called](const PlanarVelocity & proposed) {
      if (!predictor_called) {
        EXPECT_NEAR(std::hypot(proposed.x, proposed.y), 0.2, 1e-12);
      }
      predictor_called = true;
      return true;
    });
  EXPECT_TRUE(predictor_called);
  expectStopped(output);
}

TEST(TunnelGuard, NormalCannotResumeSpinInsideRoofAfterRestart)
{
  auto conditions = liveConditions();
  conditions.inside_roof = true;
  expectStopped(
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.1, 0.0, 3.14}, no_overlap));
}

TEST(TunnelGuard, ApproachDisablesSpinAndAlignsBeforeTranslating)
{
  auto conditions = liveConditions();
  conditions.speed_limit = 0.2;
  auto output =
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.2, 0.0, 3.14}, no_overlap);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(output.x, 0.2);
  conditions.yaw = 0.2;
  output = sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{0.2, 0.0, 3.14}, no_overlap);
  EXPECT_DOUBLE_EQ(output.x, 0.0);
  EXPECT_DOUBLE_EQ(output.y, 0.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, -0.3);
}

TEST(TunnelGuard, EmptyConfigurationPreservesOrdinaryNavigation)
{
  GuardConditions conditions;
  conditions.command_live = true;
  const auto output =
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity{1.0, -0.3, 0.2}, no_overlap);
  EXPECT_DOUBLE_EQ(output.x, 1.0);
  EXPECT_DOUBLE_EQ(output.y, -0.3);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.2);
}

void obstacle(nav2_costmap_2d::Costmap2D & costmap, double x, double y)
{
  unsigned int mx, my;
  ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TunnelGuard, FinalSweepChecksAlignmentCreatedFromZeroInput)
{
  auto conditions = liveConditions();
  conditions.policy = sentry_tunnel::ALIGN;
  conditions.target_yaw = 0.3;
  const auto final_output =
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity(), no_overlap);
  EXPECT_DOUBLE_EQ(final_output.yaw_rate, 0.3);
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1.0, -1.0);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.4, -0.08}, {0.4, -0.08}, {0.4, 0.08}, {-0.4, 0.08}};
  obstacle(map, 0.36, 0.13);
  EXPECT_FALSE(sentry_tunnel::physicalFootprintCollision(map, footprint, 0.0, 0.0, 0.0));
  EXPECT_FALSE(
    sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, PlanarVelocity()));
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, final_output));
}

TEST(TunnelGuard, FinalSweepChecksApproachYawOverrideNotControllerAngularSpeed)
{
  auto conditions = liveConditions();
  conditions.speed_limit = 0.2;
  conditions.target_yaw = 0.04;
  const auto final_output =
    sentry_tunnel::guardedVelocity(conditions, PlanarVelocity(), no_overlap);
  EXPECT_DOUBLE_EQ(final_output.yaw_rate, 0.08);
  nav2_costmap_2d::Costmap2D map(400, 400, 0.005, -1.0, -1.0);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.4, -0.08}, {0.4, -0.08}, {0.4, 0.08}, {-0.4, 0.08}};
  obstacle(map, 0.39, 0.103);
  EXPECT_FALSE(
    sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, PlanarVelocity()));
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, final_output));
}

TEST(TunnelGuard, FinalSweepAllowsInflationButRejectsWallsUnknownAndMapEdges)
{
  nav2_costmap_2d::Costmap2D map(
    200, 200, 0.02, -2.0, -2.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.36, -0.36}, {0.36, -0.36}, {0.36, 0.36}, {-0.36, 0.36}};
  const PlanarVelocity motion{0.2, 0.0, 0.0};
  EXPECT_FALSE(sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, motion));
  obstacle(map, 0.5, 0.0);
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, motion));
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(0.5, 0.0, mx, my));
  map.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, motion));
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(map, footprint, 1.5, 0.0, 0.0, motion));
}

TEST(TunnelGuard, FinalSweepUsesActualChassisPositionAndYaw)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.4, -0.08}, {0.4, -0.08}, {0.4, 0.08}, {-0.4, 0.08}};
  obstacle(map, 0.31, 0.0);
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, PlanarVelocity()));
  EXPECT_FALSE(
    sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, M_PI_2, PlanarVelocity()));
  EXPECT_FALSE(
    sentry_tunnel::sweptMotionCollision(map, footprint, -0.5, 0.0, 0.0, PlanarVelocity()));
}

TEST(TunnelGuard, CostmapRequiresFreshSourceAndReceiptTimes)
{
  EXPECT_TRUE(sentry_tunnel::freshMotionCostmap(1000000000, 1200000000, 1490000000));
  EXPECT_FALSE(sentry_tunnel::freshMotionCostmap(1000000000, 1490000000, 1510000000));
  EXPECT_FALSE(sentry_tunnel::freshMotionCostmap(1510000000, 1000000000, 1510000000));
  EXPECT_FALSE(sentry_tunnel::freshMotionCostmap(1600000000, 1490000000, 1510000000));
  EXPECT_FALSE(sentry_tunnel::freshMotionCostmap(0, 1490000000, 1510000000));
}

TEST(TunnelGuard, OrdinarySpinIsSuppressedWhileSafeTranslationContinues)
{
  const PlanarVelocity input{0.2, 0.0, 3.14};
  auto output = sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, input,
    [](const PlanarVelocity & velocity) { return velocity.yaw_rate == 0.0; });
  EXPECT_DOUBLE_EQ(output.x, input.x);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
  output = sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, input, [](const PlanarVelocity &) { return false; });
  expectStopped(output);
  output = sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, input, [](const PlanarVelocity &) { return true; });
  EXPECT_DOUBLE_EQ(output.yaw_rate, input.yaw_rate);
  output = sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, input,
    [](const PlanarVelocity & velocity) { return velocity.yaw_rate == 0.0; }, 0.2);
  expectStopped(output);
}

TEST(TunnelGuard, TunnelAlignmentAndTraversalCannotSilentlyDropTheirHeadingControl)
{
  const PlanarVelocity input{0.2, 0.0, 0.1};
  const auto only_translation_safe = [](const PlanarVelocity & velocity) {
    return velocity.yaw_rate == 0.0;
  };
  for (const auto policy : {sentry_tunnel::ALIGN, sentry_tunnel::TRAVERSE}) {
    expectStopped(
      sentry_tunnel::collisionCheckedVelocity(policy, 0.2, input, only_translation_safe));
  }
  expectStopped(sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.2, input, only_translation_safe));
}

TEST(TunnelGuard, OrdinaryTranslationSlowsWithinTheSameFullHorizonInsteadOfDeadlocking)
{
  nav2_costmap_2d::Costmap2D map(600, 600, 0.01, -3.0, -3.0);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  obstacle(map, 0.8, 0.0);
  const PlanarVelocity requested{1.5, 0.0, 0.0};
  const auto is_safe = [&](const PlanarVelocity & velocity) {
    return !sentry_tunnel::sweptMotionCollision(map, footprint, 0.0, 0.0, 0.0, velocity);
  };
  ASSERT_FALSE(is_safe(requested));
  const auto output =
    sentry_tunnel::collisionCheckedVelocity(sentry_tunnel::NORMAL, 0.0, requested, is_safe);
  EXPECT_GT(output.x, 0.65);
  EXPECT_LT(output.x, 0.7);
  EXPECT_DOUBLE_EQ(output.y, 0.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
  EXPECT_TRUE(is_safe(output));
  // The horizon was not shortened: one full second of the selected output is
  // clear, while a slightly faster output intersects the same real wall.
  EXPECT_FALSE(is_safe(PlanarVelocity{output.x + 0.03, 0.0, 0.0}));
}

TEST(TunnelGuard, TunnelFinalSpeedRegulationPreservesRequiredHeading)
{
  const PlanarVelocity request{1.8, -0.2, 0.04};
  const auto safe = [](const PlanarVelocity & value) {
    return value.yaw_rate == 0.04 && std::hypot(value.x, value.y) <= 0.8;
  };
  for (const auto policy : {sentry_tunnel::NORMAL, sentry_tunnel::TRAVERSE}) {
    const auto result = sentry_tunnel::collisionCheckedVelocity(policy, 1.8, request, safe);
    EXPECT_TRUE(safe(result));
    EXPECT_GT(result.x, 0.75);
    EXPECT_NEAR(result.y / result.x, request.y / request.x, 1e-12);
    EXPECT_DOUBLE_EQ(result.yaw_rate, request.yaw_rate);
    expectStopped(sentry_tunnel::collisionCheckedVelocity(
      policy, 1.8, request, [](const PlanarVelocity &) { return false; }));
  }
}

TEST(TunnelGuard, AdaptiveTranslationNeverAdvancesThroughWallAndRecoversOnChangedPath)
{
  nav2_costmap_2d::Costmap2D map(600, 600, 0.01, -3.0, -3.0);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  unsigned int wall_x, wall_y;
  ASSERT_TRUE(map.worldToMap(0.8, 0.0, wall_x, wall_y));
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    map.setCost(wall_x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  double x = 0.0;
  const auto is_safe = [&](const PlanarVelocity & velocity) {
    return !sentry_tunnel::sweptMotionCollision(map, footprint, x, 0.0, 0.0, velocity);
  };
  PlanarVelocity output;
  for (int tick = 0; tick < 250; ++tick) {
    output = sentry_tunnel::collisionCheckedVelocity(
      sentry_tunnel::NORMAL, 0.0, PlanarVelocity{1.5, 0.0, 0.0}, is_safe);
    ASSERT_TRUE(is_safe(output));
    x += output.x * 0.05;
    ASSERT_LT(x, 0.7);
  }
  EXPECT_GT(x, 0.6);
  expectStopped(output);
  // Once the controller turns along the bend, the guard accepts full speed
  // again, without a latched stop or an artificial minimum crawl speed.
  output = sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, PlanarVelocity{0.0, 1.5, 0.0}, is_safe);
  EXPECT_DOUBLE_EQ(output.x, 0.0);
  EXPECT_DOUBLE_EQ(output.y, 1.5);
}

TEST(TunnelGuard, AdaptiveTranslationPreservesDirectionAndDoesNotCreateMotion)
{
  const auto is_safe = [](const PlanarVelocity & velocity) {
    return velocity.yaw_rate == 0.0 && std::hypot(velocity.x, velocity.y) <= 0.5;
  };
  const auto output = sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, PlanarVelocity{0.6, -0.8, 3.14}, is_safe);
  EXPECT_GT(output.x, 0.0);
  EXPECT_LT(output.y, 0.0);
  EXPECT_NEAR(output.y / output.x, -4.0 / 3.0, 1e-12);
  EXPECT_LE(std::hypot(output.x, output.y), 0.5);
  expectStopped(
    sentry_tunnel::collisionCheckedVelocity(sentry_tunnel::NORMAL, 0.0, PlanarVelocity(), is_safe));
  expectStopped(sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, PlanarVelocity{0.6, -0.8, 0.0},
    [](const PlanarVelocity &) { return false; }));
}

TEST(TunnelGuard, AdaptiveTranslationCannotBypassResidualSpinOrNonFiniteFeedback)
{
  const auto half_speed_safe = [](const PlanarVelocity & velocity) {
    return velocity.yaw_rate == 0.0 && std::hypot(velocity.x, velocity.y) <= 0.5;
  };
  expectStopped(sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, PlanarVelocity{1.0, 0.0, 0.0}, half_speed_safe, 0.2));
  expectStopped(sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, PlanarVelocity{1.0, 0.0, 0.0},
    [](const PlanarVelocity &) { return true; }, std::numeric_limits<double>::quiet_NaN()));
  expectStopped(sentry_tunnel::collisionCheckedVelocity(
    sentry_tunnel::NORMAL, 0.0, PlanarVelocity{std::numeric_limits<double>::infinity(), 0.0, 0.0},
    [](const PlanarVelocity &) { return true; }));
  for (const auto policy : {sentry_tunnel::NORMAL, sentry_tunnel::ALIGN, sentry_tunnel::TRAVERSE}) {
    expectStopped(sentry_tunnel::collisionCheckedVelocity(
      policy, 0.2, PlanarVelocity{1.0, 0.0, 0.1}, half_speed_safe));
  }
}

TEST(TunnelGuard, CollisionDiagnosticsIdentifyObstacleCellAndFirstPredictionTime)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1.0, -1.0);
  const std::vector<std::pair<double, double>> footprint = {
    {-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  obstacle(map, 0.5, 0.0);
  sentry_tunnel::MotionCollision collision;
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(
    map, footprint, 0.0, 0.0, 0.0, PlanarVelocity{0.5, 0.0, 0.0}, 1.0, &collision));
  EXPECT_EQ(collision.reason, sentry_tunnel::MotionCollision::Reason::LETHAL);
  EXPECT_GE(collision.time, 0.75);
  EXPECT_LE(collision.time, 0.85);
  EXPECT_NEAR(collision.x, 0.505, 1e-12);
  map.setCost(collision.cell_x, collision.cell_y, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(
    map, footprint, 0.5, 0.0, 0.0, PlanarVelocity(), 1.0, &collision));
  EXPECT_EQ(collision.reason, sentry_tunnel::MotionCollision::Reason::UNKNOWN);
  EXPECT_DOUBLE_EQ(collision.time, 0.0);
  EXPECT_TRUE(sentry_tunnel::sweptMotionCollision(
    map, footprint, 0.95, 0.0, 0.0, PlanarVelocity(), 1.0, &collision));
  EXPECT_EQ(collision.reason, sentry_tunnel::MotionCollision::Reason::OUTSIDE_MAP);
  EXPECT_DOUBLE_EQ(collision.time, 0.0);
}

struct BlueRoofPrediction
{
  BlueRoofPrediction()
  {
    config.footprint = {{-0.36, -0.36}, {0.36, -0.36}, {0.36, 0.36}, {-0.36, 0.36}};
    tunnel.x_min = 2.325;
    tunnel.x_max = 2.675;
    tunnel.y_min = -3.86;
    tunnel.y_max = -3.06;
  }
  bool operator()(const PlanarVelocity & velocity) const
  {
    for (int i = 0; i <= 200; ++i) {
      const double time = static_cast<double>(i) / 200.0;
      if (config.overlaps(
            tunnel, x + velocity.x * time, y + velocity.y * time, yaw + velocity.yaw_rate * time)) {
        return true;
      }
    }
    return false;
  }
  sentry_tunnel::Config config;
  sentry_tunnel::Tunnel tunnel;
  double x{0.425}, y{-3.475}, yaw{0.0};
};

TEST(TunnelGuard, HighGuideApproachesSafeBlueEndpointWithoutRoofPredictionDeadlock)
{
  auto conditions = liveConditions();
  conditions.lowered_ready = false;
  conditions.speed_limit = 1.8;
  BlueRoofPrediction prediction;
  const auto initial = sentry_tunnel::guardedVelocity(
    conditions, PlanarVelocity{1.8, 0.0, 3.14}, std::ref(prediction));
  EXPECT_GT(initial.x, 1.5);
  EXPECT_LT(initial.x, 1.53);
  EXPECT_FALSE(prediction(initial));
  const double target_x = 1.8;
  for (int tick = 0; tick < 250; ++tick) {
    const double requested_speed = std::min(1.8, 3.0 * (target_x - prediction.x));
    const auto output = sentry_tunnel::guardedVelocity(
      conditions, PlanarVelocity{requested_speed, 0.0, 3.14}, std::ref(prediction));
    ASSERT_FALSE(prediction(output));
    ASSERT_GE(output.x, 0.0);
    ASSERT_LE(output.x, requested_speed);
    prediction.x += output.x * 0.02;
    ASSERT_LE(prediction.x, target_x);
  }
  EXPECT_NEAR(prediction.x, target_x, 0.005);
}

TEST(TunnelGuard, HighGuideSafePrefixCannotCrossRoofAfterRepeatedCommands)
{
  auto conditions = liveConditions();
  conditions.lowered_ready = false;
  conditions.speed_limit = 1.8;
  BlueRoofPrediction prediction;
  PlanarVelocity output;
  for (int tick = 0; tick < 250; ++tick) {
    output = sentry_tunnel::guardedVelocity(
      conditions, PlanarVelocity{1.8, 0.0, 3.14}, std::ref(prediction));
    ASSERT_FALSE(prediction(output));
    prediction.x += output.x * 0.05;
    ASSERT_FALSE(prediction(PlanarVelocity()));
    ASSERT_LT(prediction.x + 0.36 + 0.01, prediction.tunnel.x_min);
  }
  EXPECT_GT(prediction.x, 1.9);
  expectStopped(output);
  conditions.inside_roof = true;
  expectStopped(sentry_tunnel::guardedVelocity(
    conditions, PlanarVelocity{-0.2, 0.0, 0.0}, std::ref(prediction)));
}

TEST(TunnelGuard, HighGuideSafePrefixPreservesHeadingAndNeverCreatesTranslation)
{
  auto conditions = liveConditions();
  conditions.lowered_ready = false;
  conditions.speed_limit = 1.8;
  conditions.target_yaw = 0.02;
  const auto limited = sentry_tunnel::guardedVelocity(
    conditions, PlanarVelocity{1.8, -0.9, 3.14}, [](const PlanarVelocity & velocity) {
      EXPECT_NEAR(velocity.yaw_rate, 0.04, 1e-12);
      return std::hypot(velocity.x, velocity.y) > 0.5;
    });
  EXPECT_GT(limited.x, 0.0);
  EXPECT_NEAR(limited.y / limited.x, -0.5, 1e-12);
  EXPECT_LE(std::hypot(limited.x, limited.y), 0.5);
  EXPECT_NEAR(limited.yaw_rate, 0.04, 1e-12);
  // An unsafe stationary rotation may not be converted into a safe-looking
  // straight command by silently dropping the BT's heading control.
  expectStopped(sentry_tunnel::guardedVelocity(
    conditions, PlanarVelocity{0.2, 0.0, 3.14},
    [](const PlanarVelocity & velocity) { return velocity.yaw_rate != 0.0; }));
  conditions.target_yaw = 0.0;
  expectStopped(sentry_tunnel::guardedVelocity(
    conditions, PlanarVelocity(), [](const PlanarVelocity &) { return false; }));
  // A possible safe prefix never authorizes ordinary navigation to make its
  // own tunnel decision, or turns ALIGN / TRAVERSE into high-body travel.
  for (const auto policy : {sentry_tunnel::NORMAL, sentry_tunnel::ALIGN, sentry_tunnel::TRAVERSE}) {
    conditions.policy = policy;
    conditions.speed_limit = policy == sentry_tunnel::NORMAL ? 0.0 : 1.8;
    expectStopped(sentry_tunnel::guardedVelocity(
      conditions, PlanarVelocity{1.8, 0.0, 0.0},
      [](const PlanarVelocity & velocity) { return std::hypot(velocity.x, velocity.y) > 0.5; }));
  }
}
}  // namespace
