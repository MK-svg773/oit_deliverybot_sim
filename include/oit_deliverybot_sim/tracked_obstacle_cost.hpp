// Original implementation for deliverybot simulation. No third-party critic code copied.
#ifndef OIT_DELIVERYBOT_02__TRACKED_OBSTACLE_COST_HPP_
#define OIT_DELIVERYBOT_02__TRACKED_OBSTACLE_COST_HPP_
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace oit_deliverybot_sim
{
struct FootprintPoint {double x, y;};

// Keep model_dt as float, matching CriticData and the original C1/C2 arithmetic.
inline double trackPredictionTime(
  double age, size_t step, float model_dt, bool extrapolate_future = true)
{
  return extrapolate_future ? age + step * model_dt : age;
}

// Signed polygon-to-circle gap; negative means overlap, with no safety margin.
inline double footprintCircleDistance(
  double x, double y, double yaw, double ox, double oy, double obstacle_radius,
  const std::vector<FootprintPoint> & footprint)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double px = c * (ox - x) + s * (oy - y);
  const double py = -s * (ox - x) + c * (oy - y);
  double squared_distance = std::numeric_limits<double>::infinity();
  bool positive = false, negative = false;
  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & a = footprint[i];
    const auto & b = footprint[(i + 1) % footprint.size()];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t = length_squared > 0 ?
      std::clamp(((px - a.x) * dx + (py - a.y) * dy) / length_squared, 0.0, 1.0) : 0;
    const double ex = px - a.x - t * dx, ey = py - a.y - t * dy;
    squared_distance = std::min(squared_distance, ex * ex + ey * ey);
    const double cross = dx * (py - a.y) - dy * (px - a.x);
    positive |= cross > 0;
    negative |= cross < 0;
  }
  const double distance = std::sqrt(squared_distance);
  return (positive && negative ? distance : -distance) - obstacle_radius;
}

struct ObstacleTrajectoryCost
{
  double shortage_sum{0};
  bool collision{false};

  void addStep(double distance, double margin)
  {
    shortage_sum += std::max(0.0, margin - distance);
    collision |= distance < 0;
  }

  double score(size_t steps, double weight, int power, double collision_cost) const
  {
    return std::pow(weight * shortage_sum / steps, power) + (collision ? collision_cost : 0);
  }
};
}  // namespace oit_deliverybot_sim
#endif  // OIT_DELIVERYBOT_02__TRACKED_OBSTACLE_COST_HPP_
