// Copyright 2026 ICD Sentry contributors
// SPDX-License-Identifier: Apache-2.0

#include "gtest/gtest.h"
#include "icd2027sentry_nav2_plugins/footprint_grid_search.hpp"

namespace
{
sentry_tunnel::Config squareConfig(double half_side = 0.36)
{
  sentry_tunnel::Config config;
  config.footprint = {
    {half_side, half_side},
    {-half_side, half_side},
    {-half_side, -half_side},
    {half_side, -half_side}};
  return config;
}

void rectangle(
  nav2_costmap_2d::Costmap2D & map, double xmin, double xmax, double ymin, double ymax,
  unsigned char value = nav2_costmap_2d::LETHAL_OBSTACLE)
{
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      if (wx >= xmin && wx < xmax && wy >= ymin && wy < ymax) {
        map.setCost(x, y, value);
      }
    }
  }
}

void expectContinuousSafe(
  const std::vector<sentry_tunnel::GridPose> & path,
  const sentry_tunnel::FootprintGridSearch & search)
{
  ASSERT_GE(path.size(), 2u);
  for (size_t i = 1; i < path.size(); ++i) {
    ASSERT_TRUE(search.safeSegment(path[i - 1], path[i])) << "segment " << i;
    EXPECT_LE(std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 0.020001);
  }
}

TEST(FootprintGridPlanner, PreservesExactOffGridEndpoints)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, 0.0, 0.0, 0);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.4);
  const auto path = search.plan(1.017, 1.033, 3.913, 3.942);
  EXPECT_DOUBLE_EQ(path.front().x, 1.017);
  EXPECT_DOUBLE_EQ(path.front().y, 1.033);
  EXPECT_DOUBLE_EQ(path.back().x, 3.913);
  EXPECT_DOUBLE_EQ(path.back().y, 3.942);
  expectContinuousSafe(path, search);
}

TEST(FootprintGridPlanner, GoesAroundStaticCornerWithEntireRotatedSquare)
{
  nav2_costmap_2d::Costmap2D map(120, 100, 0.05, -3.0, -2.0, 0);
  rectangle(map, -2.0, -0.6, -2.0, 1.0);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.45);
  const auto path = search.plan(-1.3, 1.6, 0.8, -0.7);
  expectContinuousSafe(path, search);
  bool clears_corner = false;
  for (const auto & point : path) {
    if (point.y < 1.0) {
      EXPECT_GT(point.x, -0.15);
      clears_corner = true;
    }
  }
  EXPECT_TRUE(clears_corner);
}

TEST(FootprintGridPlanner, RejectsGoalWhoseCenterIsFreeButCornerHitsObstacle)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, 0.0, 0.0, 0);
  rectangle(map, 2.25, 2.35, 2.25, 2.35);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  EXPECT_THROW(search.plan(1.0, 1.0, 2.0, 2.0), std::runtime_error);
}

TEST(FootprintGridPlanner, UnknownInsidePolygonAndOutsideMapRemainBlocked)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, 0.0, 0.0, 0);
  rectangle(map, 1.95, 2.05, 1.95, 2.05, nav2_costmap_2d::NO_INFORMATION);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  EXPECT_THROW(search.plan(1.0, 1.0, 2.0, 2.0), std::runtime_error);
  EXPECT_THROW(search.plan(1.0, 1.0, 0.1, 2.0), std::runtime_error);
}

TEST(FootprintGridPlanner, ExplicitPhysicalFootprintDoesNotDoubleCountInflation253)
{
  nav2_costmap_2d::Costmap2D map(
    100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  const auto path = search.plan(1.0, 1.0, 3.0, 3.0);
  expectContinuousSafe(path, search);
}

TEST(FootprintGridPlanner, DiagonalCannotCutBetweenBlockedConfigurationCells)
{
  nav2_costmap_2d::Costmap2D map(3, 3, 1.0, 0.0, 0.0, 0);
  map.setCost(1, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  map.setCost(0, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto config = squareConfig(0.05);
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  EXPECT_THROW(search.plan(0.5, 0.5, 1.5, 1.5), std::runtime_error);
}

TEST(FootprintGridPlanner, NarrowGuideIsAxisAlignedInBothDirectionsAndRotationsAreChecked)
{
  nav2_costmap_2d::Costmap2D map(120, 100, 0.05, 0.0, 0.0, 0);
  rectangle(map, 1.5, 4.5, 0.0, 2.1);
  rectangle(map, 1.5, 4.5, 2.85, 5.0);
  auto config = squareConfig();
  config.wait_offset = 2.1;
  config.exit_offset = 2.1;
  config.tunnels.push_back({"test", 2.825, 3.175, 2.06, 2.86, 0.0, 0.25, 2.475});
  for (const bool reverse : {false, true}) {
    const double yaw = reverse ? std::acos(-1.0) + 0.4 : 0.4;
    sentry_tunnel::FootprintGridSearch search(map, config, yaw);
    const auto path = search.plan(reverse ? 5.2 : 0.8, 2.475, reverse ? 0.8 : 5.2, 2.475);
    expectContinuousSafe(path, search);
    for (const auto & point : path) {
      if (point.x >= 1.5 && point.x <= 4.5) {
        EXPECT_NEAR(std::sin(point.yaw), 0.0, 1e-9);
        EXPECT_NEAR(point.y, 2.475, 1e-9);
      }
    }
    // Being inside the guide must not make a physical 45-degree roof-side
    // rotation legal: its corners overlap the unchanged side-wall cells.
    EXPECT_FALSE(search.safeSegment({3.0, 2.475, 0.0}, {3.0, 2.475, 0.4}));
  }
}

TEST(FootprintGridPlanner, VerifiedGuideDoesNotClearAnActualLowObstacle)
{
  nav2_costmap_2d::Costmap2D map(120, 100, 0.05, 0.0, 0.0, 0);
  auto config = squareConfig();
  config.tunnels.push_back({"test", 2.825, 3.175, 2.06, 2.86, 0.0, 0.25, 2.475});
  rectangle(map, 2.95, 3.05, 2.425, 2.525);
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  EXPECT_FALSE(search.safeSegment({2.0, 2.475, 0.0}, {4.0, 2.475, 0.0}));
  const auto path = search.plan(1.0, 2.475, 5.0, 2.475);
  expectContinuousSafe(path, search);
  EXPECT_EQ(map.getCost(60, 49), nav2_costmap_2d::LETHAL_OBSTACLE);
}
TEST(FootprintGridPlanner, MidRouteGuideWithoutBtOwnershipKeepsActualYawEverywhere)
{
  nav2_costmap_2d::Costmap2D map(120, 100, 0.05, 0.0, 0.0, 0);
  auto config = squareConfig();
  config.wait_offset = 4.0;
  config.exit_offset = 0.9;
  config.tunnels.push_back({"distant_roof", 4.3, 4.65, 2.06, 2.86, 0.0, 0.25, 2.475});
  rectangle(map, 1.45, 1.55, 0.0, 2.1);
  sentry_tunnel::FootprintGridSearch search(map, config, 0.4);
  const auto path = search.plan(0.8, 1.5, 2.3, 1.5);
  expectContinuousSafe(path, search);
  for (const auto & point : path) {
    EXPECT_NEAR(point.yaw, 0.4, 1e-12);
  }
}

TEST(FootprintGridPlanner, OffWallStagingAllowsRotatedArrivalAtExactGuideEndpoint)
{
  nav2_costmap_2d::Costmap2D map(120, 100, 0.05, 0.0, -5.0, 0);
  rectangle(map, 0.0, 6.0, -5.0, -2.85);  // continuous arena perimeter
  rectangle(map, 1.5, 4.5, -2.1, 0.0);    // platform's inner side wall
  auto config = squareConfig();
  config.wait_offset = 2.1;
  config.exit_offset = 2.1;
  config.tunnels.push_back({"blue", 2.825, 3.175, -2.86, -2.06, 0.0, 0.25, -2.475});
  // The requested endpoint fits at axis yaw, but not at the incoming .4 rad.
  // Alignment must happen in the off-wall band, not at the narrow goal.
  sentry_tunnel::FootprintGridSearch search(map, config, 0.4, 0.5, 0.85);
  const auto path = search.plan(0.8, -1.3, 0.725, -2.475);
  expectContinuousSafe(path, search);
  EXPECT_NEAR(path.back().yaw, 0.0, 1e-12);
  EXPECT_NEAR(path.back().x, 0.725, 1e-12);
}

TEST(FootprintGridPlanner, NonFiniteEndpointsAndSegmentsFailClosed)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, 0.0, 0.0, 0);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(search.plan(1.0, 1.0, nan, 3.0), std::runtime_error);
  EXPECT_FALSE(search.safeSegment({1.0, 1.0, 0.0}, {1.1, 1.0, nan}));
}

TEST(FootprintGridPlanner, MovingRotationCannotStandInForBtStationaryAlignment)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, 0.0, 0.0, 0);
  rectangle(map, 2.40, 2.50, 1.70, 2.10);
  const auto config = squareConfig();
  sentry_tunnel::FootprintGridSearch search(map, config, 0.0);
  const sentry_tunnel::PlanarVelocity moving_away{-0.5, 0.0, 0.6};
  ASSERT_FALSE(
    sentry_tunnel::sweptMotionCollision(map, config.footprint, 2.0, 2.0, 0.0, moving_away));
  // Actual ALIGN cannot translate away from the nearby wall while turning.
  EXPECT_FALSE(search.safeSegment({2.0, 2.0, 0.0}, {1.5, 2.0, 0.6}));
}
}  // namespace
