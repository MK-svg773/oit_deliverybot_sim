// Original implementation for deliverybot simulation. No third-party critic code copied.
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include "nav2_mppi_controller/critic_function.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "oit_deliverybot_sim/tracked_obstacle_cost.hpp"

// Jazzy CriticManager always prefixes plugin names with mppi::critics::.
namespace mppi::critics
{
class TrackedObstacleCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto get = parameters_handler_->getParamGetter(name_);
    get(weight_, "cost_weight", 20.0);
    get(power_, "cost_power", 1);
    get(margin_, "safety_margin", 0.25);
    get(collision_cost_, "collision_cost", 1000.0);
    get(use_velocity_, "use_velocity", false);
    get(max_age_, "max_track_age", 1.0);
    if (weight_ < 0 || power_ < 1 || margin_ < 0 || max_age_ <= 0 || collision_cost_ < 0) {
      throw std::runtime_error("Invalid TrackedObstacleCritic parameters");
    }
    if (costmap_ros_->getRobotFootprint().size() < 3) {
      throw std::runtime_error("TrackedObstacleCritic requires a convex polygon footprint");
    }
    auto node = parent_.lock();
    subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sim/obstacle_tracks", rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(mutex_);
        tracks_ = msg;
      });
    RCLCPP_INFO(logger_,
      "TrackedObstacleCritic use_velocity=%s weight=%.2f power=%d collision_cost=%.2f",
      use_velocity_ ? "true" : "false", weight_, power_, collision_cost_);
  }

  void score(CriticData & data) override
  {
    if (!enabled_) {return;}
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      cloud = tracks_;
    }
    if (!cloud || cloud->width * cloud->height == 0) {return;}
    auto node = parent_.lock();
    const double age = (node->now() - rclcpp::Time(cloud->header.stamp)).seconds();
    if (age < 0 || age > max_age_) {
      RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 5000, "Track stamp stale/future: %.3fs",
          age);
      return;
    }
    const auto frame = costmap_ros_->getGlobalFrameID();
    // The optimizer integrates in the local costmap frame (sim/odom here).
    if (data.state.pose.header.frame_id != frame || cloud->header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(logger_, *node->get_clock(), 2000,
        "Critic frame mismatch: trajectory=%s costmap=%s tracks=%s",
        data.state.pose.header.frame_id.c_str(), frame.c_str(), cloud->header.frame_id.c_str());
      data.fail_flag = true;
      return;
    }
    double tx = 0, ty = 0, c = 1, s = 0;
    if (cloud->header.frame_id != frame) {
      try {
        // Current map->local transform places predictions in the optimizer's frame.
        auto transform = costmap_ros_->getTfBuffer()->lookupTransform(
          frame, cloud->header.frame_id, tf2::TimePointZero);
        const auto & t = transform.transform;
        tx = t.translation.x; ty = t.translation.y;
        const double yaw = tf2::getYaw(t.rotation);
        c = std::cos(yaw); s = std::sin(yaw);
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_THROTTLE(logger_, *node->get_clock(), 2000, "Track TF: %s", e.what());
        data.fail_flag = true;
        return;
      }
    }
    struct Obstacle {double x, y, vx, vy, radius;};
    std::vector<Obstacle> obstacles;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"),
      vx(*cloud, "vx"), vy(*cloud, "vy"), radius(*cloud, "radius");
      for (; x != x.end(); ++x, ++y, ++vx, ++vy, ++radius) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*vx) ||
          !std::isfinite(*vy) || !std::isfinite(*radius) || *radius <= 0) {continue;}
        obstacles.push_back({tx + c * *x - s * *y, ty + s * *x + c * *y,
            use_velocity_ ? c * *vx - s * *vy : 0.0,
            use_velocity_ ? s * *vx + c * *vy : 0.0, *radius});
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_THROTTLE(logger_, *node->get_clock(), 2000, "Invalid track fields: %s",
          e.what());
      data.fail_flag = true;
      return;
    }
    const size_t steps = data.trajectories.x.shape(1);
    if (!steps) {return;}
    std::vector<oit_deliverybot_sim::FootprintPoint> footprint;
    for (const auto & p : costmap_ros_->getRobotFootprint()) {
      footprint.push_back({p.x, p.y});
    }
    if (footprint.size() < 3) {data.fail_flag = true; return;}
    for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
      oit_deliverybot_sim::ObstacleTrajectoryCost cost;
      for (size_t j = 0; j < steps; ++j) {
        double nearest = std::numeric_limits<double>::infinity();
        const double time = age + (j + 1) * data.model_dt;
        for (const auto & obstacle : obstacles) {
          nearest = std::min(nearest, oit_deliverybot_sim::footprintCircleDistance(
                data.trajectories.x(i, j), data.trajectories.y(i, j),
                data.trajectories.yaws(i, j),
                obstacle.x + obstacle.vx * time, obstacle.y + obstacle.vy * time,
                obstacle.radius, footprint));
        }
        cost.addStep(nearest, margin_);
      }
      data.costs(i) += cost.score(steps, weight_, power_, collision_cost_);
    }
    if (!obstacles.empty()) {
      const auto & o = obstacles.front();
      RCLCPP_INFO_THROTTLE(logger_, *node->get_clock(), 2000,
        "Track prediction velocity=%s %s->%s age=%.3f now=(%.2f,%.2f) horizon=(%.2f,%.2f)",
        use_velocity_ ? "on" : "off", cloud->header.frame_id.c_str(), frame.c_str(), age,
        o.x + o.vx * age, o.y + o.vy * age,
        o.x + o.vx * (age + steps * data.model_dt),
        o.y + o.vy * (age + steps * data.model_dt));
    }
  }

private:
  double weight_{20}, margin_{0.25}, max_age_{1}, collision_cost_{1000};
  int power_{1};
  bool use_velocity_{false};
  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr tracks_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::TrackedObstacleCritic, mppi::critics::CriticFunction)
