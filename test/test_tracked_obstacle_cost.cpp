#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
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

TEST(TrackedObstacleCost, LegacyPredictionAndCostsRemainBitIdentical)
{
  // Match Jazzy's float model_dt; promoting the multiplication to double changes C2.
  const float dt = .1f;
  for (const bool use_velocity : {false, true}) {
    for (const double age : {0.0, .3, .97}) {
      ObstacleTrajectoryCost original, current;
      for (size_t j = 0; j < 40; ++j) {
        const double old_time = age + (j + 1) * dt;
        const double time = oit_deliverybot_sim::trackPredictionTime(age, j + 1, dt);
        EXPECT_EQ(std::memcmp(&old_time, &time, sizeof(double)), 0);
        const double vx = use_velocity ? -.7 : 0.0;
        const double vy = use_velocity ? .1 : 0.0;
        const double old_x = 1 + vx * old_time, x = 1 + vx * time;
        const double old_y = .2 + vy * old_time, y = .2 + vy * time;
        EXPECT_EQ(std::memcmp(&old_x, &x, sizeof(double)), 0);
        EXPECT_EQ(std::memcmp(&old_y, &y, sizeof(double)), 0);
        original.addStep(footprintCircleDistance(0, 0, 0, old_x, old_y, .25, footprint), .25);
        current.addStep(footprintCircleDistance(0, 0, 0, x, y, .25, footprint), .25);
      }
      const double old_score = original.score(40, 20, 1, 1000);
      const double score = current.score(40, 20, 1, 1000);
      EXPECT_EQ(std::memcmp(&old_score, &score, sizeof(double)), 0);
    }
  }
}

TEST(TrackedObstacleCost, AgeCompensationHoldsCurrentPositionAtEveryStep)
{
  // p=(1,2), v=(-1,.5), age=.5 gives (.5,2.25), including the last horizon step.
  for (size_t step = 1; step <= 40; ++step) {
    const double time = oit_deliverybot_sim::trackPredictionTime(.5, step, .1f, false);
    EXPECT_EQ(time, .5);
    EXPECT_EQ(1 - time, .5);
    EXPECT_EQ(2 + .5 * time, 2.25);
    const double zero_age = oit_deliverybot_sim::trackPredictionTime(0, step, .1f, false);
    EXPECT_EQ(1 - zero_age, 1);
    // A zero velocity is unaffected in both modes.
    for (const bool future : {false, true}) {
      const double t = oit_deliverybot_sim::trackPredictionTime(.5, step, .1f, future);
      EXPECT_EQ(1 + 0.0 * t, 1);
    }
  }
}
