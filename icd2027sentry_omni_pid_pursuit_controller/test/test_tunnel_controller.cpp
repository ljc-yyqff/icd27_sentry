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

#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "icd2027sentry_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"
#include "icd2027sentry_omni_pid_pursuit_controller/swept_collision_checker.hpp"

namespace icd2027sentry_omni_pid_pursuit_controller
{
namespace
{

class LimitController : public OmniPidPursuitController
{
public:
  using OmniPidPursuitController::chassisHeadingError;
  using OmniPidPursuitController::effectiveLinearSpeedLimit;
  using OmniPidPursuitController::getLookAheadPoint;
  using OmniPidPursuitController::linearApproachVelocity;
};

std::vector<geometry_msgs::msg::Point> rectangle(double half_x, double half_y)
{
  std::vector<geometry_msgs::msg::Point> result(4);
  result[0].x = result[3].x = -half_x;
  result[1].x = result[2].x = half_x;
  result[0].y = result[1].y = -half_y;
  result[2].y = result[3].y = half_y;
  return result;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(orientation);
  return result;
}

void obstacle(nav2_costmap_2d::Costmap2D & costmap, double x, double y)
{
  unsigned int mx, my;
  ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TunnelController, SpeedLimitsAreAbsoluteOrRelativeToConfiguredMaximum)
{
  LimitController controller;
  controller.setSpeedLimit(0.2, false);
  EXPECT_DOUBLE_EQ(controller.effectiveLinearSpeedLimit(2.5), 0.2);
  controller.setSpeedLimit(20.0, true);
  EXPECT_DOUBLE_EQ(controller.effectiveLinearSpeedLimit(2.5), 0.5);
  controller.setSpeedLimit(50.0, true);
  EXPECT_DOUBLE_EQ(controller.effectiveLinearSpeedLimit(2.5), 1.25);
  controller.setSpeedLimit(10.0, false);
  EXPECT_DOUBLE_EQ(controller.effectiveLinearSpeedLimit(2.5), 2.5);
  controller.setSpeedLimit(0.0, false);
  EXPECT_DOUBLE_EQ(controller.effectiveLinearSpeedLimit(2.5), 2.5);
  controller.setSpeedLimit(0.1, false);
  controller.setSpeedLimit(std::numeric_limits<double>::quiet_NaN(), false);
  EXPECT_DOUBLE_EQ(controller.effectiveLinearSpeedLimit(2.5), 0.1);
}

TEST(TunnelController, SparseCenteringPathDoesNotOfferABehindRobotAdaptiveCarrot)
{
  nav_msgs::msg::Path sparse;
  sparse.poses = {pose(0, 0), pose(0, 0.8)};
  auto dense = densifyTrackingPath(sparse, 0.02);
  ASSERT_GE(dense.poses.size(), 41u);
  // Match transformGlobalPlan's nearest-vertex pruning after moving 6 cm.
  const double robot_y = 0.06;
  auto nearest = std::min_element(
    dense.poses.begin(), dense.poses.end(), [robot_y](const auto & a, const auto & b) {
      return std::abs(a.pose.position.y - robot_y) < std::abs(b.pose.position.y - robot_y);
    });
  dense.poses.erase(dense.poses.begin(), nearest);
  for (auto & point : dense.poses) {
    point.pose.position.y -= robot_y;
  }
  LimitController controller;
  const auto carrot = controller.getLookAheadPoint(0.05, dense, true);
  EXPECT_GT(carrot.pose.position.y, 0.0);
  EXPECT_NEAR(carrot.pose.position.y, 0.05, 1e-9);
}

TEST(TunnelController, TunnelSpeedRegulatesFullHorizonBeforeWallWithoutDroppingHeading)
{
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 1.8;
  requested.angular.z = 0.12;
  ASSERT_TRUE(collisionRegulatedSpeed(
    requested,
    [](const auto & candidate) {
      return std::abs(candidate.angular.z - 0.12) < 1e-12 && candidate.linear.x <= 0.6;
    },
    output));
  EXPECT_GT(output.linear.x, 0.45);
  EXPECT_LT(output.linear.x, 0.6);
  EXPECT_DOUBLE_EQ(output.angular.z, 0.12);
  ASSERT_TRUE(collisionRegulatedSpeed(
    requested, [](const auto &) { return true; }, output));
  EXPECT_DOUBLE_EQ(output.linear.x, 1.8);
}

TEST(TunnelController, TunnelSpeedCannotBypassBodyCollisionOrUnsafeRequiredRotation)
{
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 1.8;
  requested.angular.z = 0.2;
  EXPECT_FALSE(collisionRegulatedSpeed(
    requested, [](const auto &) { return false; }, output));
  EXPECT_FALSE(collisionRegulatedSpeed(
    requested, [](const auto & candidate) { return candidate.angular.z == 0.0; }, output));
}

TEST(TunnelController, HeadingErrorUsesThePhysicalChassisInsteadOfTheVirtualBase)
{
  EXPECT_NEAR(LimitController::chassisHeadingError(0.7, 0.5, 1.1), 0.1, 1e-12);
  EXPECT_NEAR(LimitController::chassisHeadingError(0.0, 0.0, M_PI_2), -M_PI_2, 1e-12);
  EXPECT_NEAR(LimitController::chassisHeadingError(-M_PI + 0.05, 0.0, M_PI - 0.05), 0.1, 1e-12);
}

TEST(TunnelController, DetectsAnObstacleInsideTheFootprintNotJustOnItsOutline)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const auto footprint = rectangle(0.36, 0.36);
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0));
  obstacle(map, 0.01, 0.01);
  EXPECT_TRUE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0));
}

TEST(TunnelController, UsesThePhysicalChassisOrientation)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const auto footprint = rectangle(0.4, 0.1);
  obstacle(map, 0.32, 0.01);
  EXPECT_TRUE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0));
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, M_PI_2));
}

TEST(TunnelController, FindsAnObstacleBetweenSparsePlannerPoses)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const auto footprint = rectangle(0.1, 0.1);
  nav_msgs::msg::Path path;
  path.poses = {pose(-1.0, 0.0), pose(1.0, 0.0)};
  obstacle(map, 0.01, 0.01);
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, -1.0, 0.0, 0.0));
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 1.0, 0.0, 0.0));
  EXPECT_TRUE(sweptFootprintCollision(map, footprint, path));
}

TEST(TunnelController, FindsAnObstacleDuringRotationBetweenClearEndpointPoses)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1.0, -1.0);
  const auto footprint = rectangle(0.4, 0.08);
  nav_msgs::msg::Path path;
  path.poses = {pose(0.0, 0.0, 0.0), pose(0.0, 0.0, M_PI_2)};
  obstacle(map, 0.26, 0.26);
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0));
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, M_PI_2));
  EXPECT_TRUE(sweptFootprintCollision(map, footprint, path));
}

TEST(TunnelController, RejectsOutOfMapAndUnknownSpace)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.02, -1.0, -1.0);
  const auto footprint = rectangle(0.36, 0.36);
  EXPECT_TRUE(footprintCollisionAtPose(map, footprint, 0.8, 0.0, 0.0));
  map.setCost(50, 50, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0));
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0, true));
  map.setCost(50, 50, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.0));
}

TEST(TunnelController, ApproachFootprintAllowsInflationButNeverAnActualWall)
{
  nav2_costmap_2d::Costmap2D map(
    200, 200, 0.02, -2.0, -2.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const auto footprint = rectangle(0.36, 0.36);
  nav_msgs::msg::Path path;
  path.poses = {pose(-0.8, 0.0), pose(0.8, 0.0)};
  // A center-point check would reject every cell here even though the actual
  // 0.72 m footprint fits between these physical walls.
  for (double x = -1.2; x <= 1.2; x += 0.01) {
    obstacle(map, x, -0.40);
    obstacle(map, x, 0.40);
  }
  EXPECT_FALSE(sweptFootprintCollision(map, footprint, path));
  obstacle(map, 0.01, 0.01);
  EXPECT_TRUE(sweptFootprintCollision(map, footprint, path));
}

TEST(TunnelController, ApproachFootprintUsesActualChassisOffsetAndHeldHeading)
{
  const auto virtual_base = pose(0.2, -0.1, -0.7);
  const auto chassis = pose(0.0, 0.0, M_PI_2);
  const auto planner_pose = pose(0.6, -0.1, 0.0);
  const auto collision_pose = chassisCollisionPose(planner_pose, virtual_base, chassis, false);
  EXPECT_NEAR(collision_pose.pose.position.x, 0.4, 1e-12);
  EXPECT_NEAR(collision_pose.pose.position.y, 0.0, 1e-12);
  EXPECT_NEAR(tf2::getYaw(collision_pose.pose.orientation), M_PI_2, 1e-12);

  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const auto footprint = rectangle(0.30, 0.08);
  obstacle(map, 0.4, 0.25);
  EXPECT_FALSE(footprintCollisionAtPose(map, footprint, 0.6, -0.1, 0.0));
  EXPECT_TRUE(footprintCollisionAtPose(
    map, footprint, collision_pose.pose.position.x, collision_pose.pose.position.y,
    tf2::getYaw(collision_pose.pose.orientation)));

  // Tunnel-mode paths deliberately encode a requested physical heading.
  const auto tunnel_pose = chassisCollisionPose(planner_pose, virtual_base, chassis, true);
  EXPECT_NEAR(tf2::getYaw(tunnel_pose.pose.orientation), 0.0, 1e-12);
}

TEST(TunnelController, ApproachDoesNotRotateFootprintWithHolonomicPathDirection)
{
  nav2_costmap_2d::Costmap2D map(400, 400, 0.01, -2.0, -2.0);
  const auto footprint = rectangle(0.4, 0.08);
  const auto actual_chassis = pose(0.0, -0.2);
  nav_msgs::msg::Path path;
  path.poses = {
    chassisCollisionPose(pose(0.0, -0.2, M_PI_2), actual_chassis, actual_chassis, false),
    chassisCollisionPose(pose(0.0, 0.2, M_PI_2), actual_chassis, actual_chassis, false)};
  obstacle(map, 0.02, 0.49);
  EXPECT_FALSE(sweptFootprintCollision(map, footprint, path));
  // Using the path's +90 degree heading would invent a collision ahead of
  // the actual sideways-moving chassis.
  path.poses[1] = pose(0.0, 0.2, M_PI_2);
  EXPECT_TRUE(sweptFootprintCollision(map, footprint, path));
}

TEST(TunnelController, FitsTheTunnelOnlyWhenAligned)
{
  nav2_costmap_2d::Costmap2D map(400, 400, 0.01, -2.0, -2.0);
  const auto footprint = rectangle(0.36, 0.36);
  for (double x = -1.0; x <= 1.0; x += 0.005) {
    obstacle(map, x, -0.40);
    obstacle(map, x, 0.40);
  }
  nav_msgs::msg::Path path;
  path.poses = {pose(-0.8, 0.0), pose(0.8, 0.0)};
  EXPECT_FALSE(sweptFootprintCollision(map, footprint, path));
  EXPECT_TRUE(footprintCollisionAtPose(map, footprint, 0.0, 0.0, 0.2));
}

TEST(TunnelController, AdaptivePursuitPrefersAFullSpeedNearCarrotBeforeCreepingTowardAFarCorner)
{
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 1.0;
  geometry_msgs::msg::Point far, near;
  far.x = far.y = 1.0;
  near.x = 0.1;
  ASSERT_TRUE(collisionRegulatedTranslation(
    requested, {far, near},
    [](const auto & command) {
      return std::abs(command.linear.y) < 1e-8 ||
             std::hypot(command.linear.x, command.linear.y) < 0.2;
    },
    output));
  EXPECT_NEAR(output.linear.x, 1.0, 1e-12);
  EXPECT_NEAR(output.linear.y, 0.0, 1e-12);
}

TEST(TunnelController, AdaptivePursuitSlowsBeforeTheEntirePhysicalSweepHitsAWall)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const auto footprint = rectangle(0.36, 0.36);
  for (double y = -1.0; y < 1.0; y += 0.01) {
    obstacle(map, 0.95, y);
  }
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 1.0;
  geometry_msgs::msg::Point carrot;
  carrot.x = 1.0;
  auto safe = [&](const auto & command) {
    nav_msgs::msg::Path predicted;
    predicted.poses = {pose(0.0, 0.0), pose(command.linear.x, command.linear.y, command.angular.z)};
    return !sweptFootprintCollision(map, footprint, predicted);
  };
  ASSERT_FALSE(safe(requested));
  ASSERT_TRUE(collisionRegulatedTranslation(requested, {carrot}, safe, output));
  EXPECT_NEAR(output.linear.x, 0.5, 1e-12);
  EXPECT_TRUE(safe(output));
}

TEST(TunnelController, AdaptivePursuitNeverEscapesUnknownOrAnObstacleAlreadyUnderTheBody)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.02, -2.0, -2.0);
  const auto footprint = rectangle(0.36, 0.36);
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 1.0;
  geometry_msgs::msg::Point carrot;
  carrot.x = 1.0;
  auto safe = [&](const auto & command) {
    nav_msgs::msg::Path predicted;
    predicted.poses = {pose(0.0, 0.0), pose(command.linear.x, command.linear.y)};
    return !sweptFootprintCollision(map, footprint, predicted);
  };
  obstacle(map, 0.0, 0.0);
  EXPECT_FALSE(collisionRegulatedTranslation(requested, {carrot}, safe, output));
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(0.0, 0.0, x, y));
  map.setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(collisionRegulatedTranslation(requested, {carrot}, safe, output));
}

TEST(TunnelController, AdaptivePursuitDoesNotInventAHeadingOutsideTheSuppliedPath)
{
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 1.0;
  geometry_msgs::msg::Point carrot;
  carrot.x = 1.0;
  EXPECT_FALSE(collisionRegulatedTranslation(
    requested, {carrot}, [](const auto & command) { return std::abs(command.linear.y) > 0.01; },
    output));
  EXPECT_FALSE(collisionRegulatedTranslation(
    requested, {}, [](const auto &) { return true; }, output));
  carrot.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(collisionRegulatedTranslation(
    requested, {carrot}, [](const auto &) { return true; }, output));
}

TEST(TunnelController, AdaptivePursuitRetainsPhysicalAngularCommandAndSpeedLimit)
{
  geometry_msgs::msg::Twist requested, output;
  requested.linear.x = 0.2;
  requested.angular.z = -0.3;
  geometry_msgs::msg::Point carrot;
  carrot.x = carrot.y = 0.5;
  ASSERT_TRUE(collisionRegulatedTranslation(
    requested, {carrot}, [](const auto &) { return true; }, output));
  EXPECT_NEAR(std::hypot(output.linear.x, output.linear.y), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(output.angular.z, -0.3);
}

TEST(TunnelController, AdaptiveCarrotsInterpolateSparsePathsRatherThanJumpingPastTheCorner)
{
  LimitController controller;
  nav_msgs::msg::Path path;
  path.poses = {pose(0.0, 0.0), pose(1.0, 0.0), pose(1.0, 1.0)};
  const auto point = controller.getLookAheadPoint(0.05, path, true);
  EXPECT_NEAR(point.pose.position.x, 0.05, 1e-12);
  EXPECT_NEAR(point.pose.position.y, 0.0, 1e-12);
}

TEST(TunnelController, LinearApproachDoesNotMultiplyTheDistanceProportionalPidTwice)
{
  // A 0.05 m goal error already produces 0.30 m/s at Kp=6. The approach
  // factor 0.05/0.5 caps it at 0.18, instead of squaring distance to 0.03.
  EXPECT_NEAR(LimitController::linearApproachVelocity(0.30, 1.8, 0.015, 0.1), 0.18, 1e-12);
  EXPECT_NEAR(LimitController::linearApproachVelocity(0.60, 1.8, 0.015, 0.2), 0.36, 1e-12);
  EXPECT_NEAR(LimitController::linearApproachVelocity(1.20, 1.8, 0.015, 0.4), 0.72, 1e-12);
}

TEST(TunnelController, LinearApproachPreservesOtherSlowdownAndTheFullConfiguredSpeed)
{
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(1.8, 1.8, 0.015, 1.0), 1.8);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.1, 1.8, 0.015, 0.5), 0.1);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.005, 1.8, 0.015, 0.001), 0.005);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, 1.8, 0.015, 0.0), 0.015);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.0, 1.8, 0.015, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, 0.0, 0.015, 1.0), 0.0);
  // A separate absolute/percentage speed limit is still applied AFTER this
  // cap in computeVelocityCommands, so the approach minimum cannot bypass it.
  LimitController controller;
  controller.setSpeedLimit(0.01, false);
  EXPECT_DOUBLE_EQ(
    std::min(
      LimitController::linearApproachVelocity(0.3, 1.8, 0.015, 0.0),
      controller.effectiveLinearSpeedLimit(1.8)),
    0.01);
}

TEST(TunnelController, LinearApproachFailsClosedForInvalidNumbers)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(nan, 1.8, 0.015, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, infinity, 0.015, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, 1.8, nan, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, 1.8, 0.015, infinity), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, 1.8, 0.015, -0.1), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(0.3, 1.8, -0.01, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(LimitController::linearApproachVelocity(-0.3, 1.8, 0.015, 0.5), 0.0);
}

}  // namespace
}  // namespace icd2027sentry_omni_pid_pursuit_controller
