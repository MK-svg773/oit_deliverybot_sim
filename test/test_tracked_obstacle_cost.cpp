#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include "oit_deliverybot_sim/tracked_obstacle_cost.hpp"

using oit_deliverybot_sim::FootprintPoint;
using oit_deliverybot_sim::footprintCircleDistance;
using oit_deliverybot_sim::ObstacleTrajectoryCost;

// Current padded footprint: base_link is ahead of the rectangle's centre.
const std::vector<FootprintPoint> footprint{{.17, .28}, {.17, -.28},
  {-.41, -.28}, {-.41, .28}};

TEST(TrackedObstacleCost, RectangleAndCircleDistance)
{
  EXPECT_NEAR(footprintCircleDistance(0, 0, 0, 1, 0, .25, footprint), .58, 1e-9);
  EXPECT_NEAR(footprintCircleDistance(0, 0, 0, -1, 0, .25, footprint), .34, 1e-9);
  EXPECT_NEAR(footprintCircleDistance(0, 0, 0, .47, .68, .25, footprint), .25, 1e-9);
  EXPECT_NEAR(footprintCircleDistance(0, 0, 0, 0, 0, .25, footprint), -.42, 1e-9);
  EXPECT_NEAR(footprintCircleDistance(0, 0, 0, .42, 0, .25, footprint), 0, 1e-9);
  auto reversed = footprint;
  std::reverse(reversed.begin(), reversed.end());
  EXPECT_NEAR(footprintCircleDistance(0, 0, 0, 0, 0, .25, reversed), -.42, 1e-9);
}

TEST(TrackedObstacleCost, RotatesAboutBaseLinkAndTranslates)
{
  EXPECT_NEAR(footprintCircleDistance(2, 3, M_PI_2, 2, 4, .25, footprint), .58, 1e-9);
  EXPECT_NEAR(footprintCircleDistance(2, 3, M_PI_2, 1, 3, .25, footprint), .47, 1e-9);
}

TEST(TrackedObstacleCost, CollisionCostOnceRegardlessOfTimesOrObstacles)
{
  ObstacleTrajectoryCost single;
  single.addStep(.1, .25);
  single.addStep(-.1, .25);
  EXPECT_NEAR(single.score(2, 20, 1, 1000), 1005, 1e-9);
  ObstacleTrajectoryCost repeated;
  // Each timestep supplies the minimum distance over all obstacles.
  repeated.addStep(-.1, .25);
  repeated.addStep(-.1, .25);
  EXPECT_NEAR(repeated.score(2, 20, 1, 1000), 1007, 1e-9);
  EXPECT_NEAR(repeated.score(2, 20, 2, 700), 749, 1e-9);
}

TEST(TrackedObstacleCost, SoftMarginDoesNotTriggerCollisionAndTouchIsNotOverlap)
{
  ObstacleTrajectoryCost soft;
  soft.addStep(.1, .25);
  soft.addStep(0, .25);
  EXPECT_NEAR(soft.score(2, 20, 1, 1000), 4, 1e-9);
}

TEST(TrackedObstacleCost, PositionAndVelocityPredictionUseAgePlusFirstDt)
{
  // A track at x=1 approaches at 1 m/s: age .3 + first dt .1 -> x=.6.
  ObstacleTrajectoryCost position, velocity;
  position.addStep(footprintCircleDistance(0, 0, 0, 1, 0, .25, footprint), .25);
  velocity.addStep(footprintCircleDistance(0, 0, 0, 1 - (.3 + .1), 0, .25, footprint), .25);
  EXPECT_DOUBLE_EQ(position.score(1, 20, 1, 1000), 0);
  EXPECT_NEAR(velocity.score(1, 20, 1, 1000), 1.4, 1e-9);
  ObstacleTrajectoryCost future;
  future.addStep(footprintCircleDistance(0, 0, 0, 1 - (.3 + .4), 0, .25, footprint), .25);
  EXPECT_NEAR(future.score(1, 20, 1, 1000), 1007.4, 1e-9);
}
