#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <thread>
#include "nav2_mppi_controller/critic_function.hpp"
#include "oit_deliverybot_sim/tracked_obstacle_cost.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
// A fixed footprint/frame is enough: no costmap plugins, TF publisher or simulator runs.
class TestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestCostmap()
  : Costmap2DROS("calibration_test_costmap")
  {
    global_frame_ = "map";
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    for (const auto & p : std::vector<oit_deliverybot_sim::FootprintPoint>{
        {.17, .28}, {.17, -.28}, {-.41, -.28}, {-.41, .28}})
    {
      geometry_msgs::msg::Point point;
      point.x = p.x; point.y = p.y;
      padded_footprint_.push_back(point);
    }
  }
};

class CriticPlugin : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("calibration_test_controller");
    node->declare_parameter("FollowPath.time_steps", 32);
    node->declare_parameter("FollowPath.model_dt", .1);
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 10000000000LL),
      RCL_RET_OK);
    costmap = std::make_shared<TestCostmap>();
    handler = std::make_unique<mppi::ParametersHandler>(node);
  }

  std::shared_ptr<mppi::critics::CriticFunction> create(
    const std::string & name, bool velocity, bool future, bool enabled,
    const std::vector<double> & margins = {})
  {
    const auto ns = "FollowPath." + name;
    node->declare_parameter(ns + ".use_velocity", velocity);
    node->declare_parameter(ns + ".extrapolate_future", future);
    node->declare_parameter(ns + ".enabled", enabled);
    node->declare_parameter(ns + ".prediction_margins", margins);
    auto critic = loader.createSharedInstance("mppi::critics::TrackedObstacleCritic");
    critic->on_configure(node, "FollowPath", ns, costmap, handler.get());
    return critic;
  }

  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader{
    "nav2_mppi_controller", "mppi::critics::CriticFunction"};
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<TestCostmap> costmap;
  std::unique_ptr<mppi::ParametersHandler> handler;
};

TEST_F(CriticPlugin, LegacyScoresAndZeroMarginC2uAreBitIdentical)
{
  auto c0 = create("C0Disabled", false, true, false);
  auto c1 = create("C1", false, true, true);
  auto c2 = create("C2", true, true, true);
  auto age = create("CAge", true, false, true);
  auto c2u = create("C2u", true, true, true, std::vector<double>(32, 0.));
  auto wider = create("Wider", true, true, true, std::vector<double>(32, .3));

  auto sender = std::make_shared<rclcpp::Node>("calibration_test_tracks");
  auto publisher = sender->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/sim/obstacle_tracks", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = rclcpp::Time(9700000000LL);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vx", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vy", 1, sensor_msgs::msg::PointField::FLOAT32,
    "radius", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = 1.2f;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = .2f;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "vx") = -.7f;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "vy") = .1f;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "radius") = .25f;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  for (size_t attempt = 0; attempt < 100 && publisher->get_subscription_count() < 6; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 6u);
  for (size_t attempt = 0; attempt < 20; ++attempt) {
    publisher->publish(cloud);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    executor.spin_some();
  }

  mppi::models::State state;
  state.pose.header.frame_id = "map";
  mppi::models::Trajectories trajectories;
  trajectories.reset(3, 32);
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 32; ++j) {
      trajectories.x(i, j) = .03f * j;
      trajectories.y(i, j) = .15f * i;
      trajectories.yaws(i, j) = .01f * j;
    }
  }
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  float dt = .1f;
  xt::xtensor<float, 1> costs = xt::zeros<float>({3});
  mppi::CriticData data{state, trajectories, path, goal, costs, dt, false, nullptr,
    nullptr, std::nullopt, std::nullopt};
  const std::vector<oit_deliverybot_sim::FootprintPoint> footprint{
    {.17, .28}, {.17, -.28}, {-.41, -.28}, {-.41, .28}};
  for (const auto & critic : {c0, c1, c2, age, c2u, wider}) {
    costs.fill(5.0f);  // Existing standard-critic costs must survive.
    critic->score(data);
    ASSERT_FALSE(data.fail_flag);
    for (size_t i = 0; i < 3; ++i) {
      double shortage = 0.;
      bool overlap = false;
      for (size_t j = 0; j < 32; ++j) {
        const double t = critic == age ? .3 : .3 + (j + 1) * dt;
        const double vx = critic == c1 ? 0. : static_cast<double>(-.7f);
        const double vy = critic == c1 ? 0. : static_cast<double>(.1f);
        const double gap = oit_deliverybot_sim::footprintCircleDistance(
          trajectories.x(i, j), trajectories.y(i, j), trajectories.yaws(i, j),
          static_cast<double>(1.2f) + vx * t, static_cast<double>(.2f) + vy * t,
          .25, footprint);
        shortage += std::max(0., (critic == wider ? .55 : .25) - gap);
        overlap |= gap < 0.;
      }
      float expected = 5.f;
      if (critic != c0) {
        expected += std::pow(20. * shortage / 32, 1) + (overlap ? 1000. : 0.);
      }
      EXPECT_EQ(std::memcmp(&expected, &costs(i), sizeof(float)), 0) << critic->getName();
    }
  }
  dt = .2f;
  c2u->score(data);
  EXPECT_TRUE(data.fail_flag);  // Changing dt cannot silently reuse calibration.
}

TEST_F(CriticPlugin, InvalidTablesFailConfiguration)
{
  EXPECT_THROW(create("Short", true, true, true, {.1}), std::invalid_argument);
  EXPECT_THROW(create("Negative", true, true, true, std::vector<double>(32, -.1)),
    std::invalid_argument);
  EXPECT_THROW(create("Nonfinite", true, true, true, std::vector<double>(32, NAN)),
    std::invalid_argument);
  EXPECT_THROW(create("Static", false, true, true, std::vector<double>(32, .1)),
    std::runtime_error);
  EXPECT_THROW(create("AgeOnly", true, false, true, std::vector<double>(32, .1)),
    std::runtime_error);
}
}  // namespace
