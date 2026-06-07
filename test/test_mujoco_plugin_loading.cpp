#include "roadmap_test_utils.hpp"

#include <algorithm>
#include <cstdlib>
#include <array>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

namespace
{

using PluginCase = std::tuple<const char *, const char *, const char *>;

class IndependentPluginLoadTest : public ::testing::TestWithParam<PluginCase>
{
};

TEST_P(IndependentPluginLoadTest, RegistersOnlyRequiredPluginAndCompilesFixture)
{
  const auto &[library_env, plugin_name, fixture] = GetParam();
  const char *library_path = std::getenv(library_env);
  if (!library_path || std::string(library_path).empty())
  {
    GTEST_SKIP() << library_env << " is not set; modular plugin target is not available";
  }

  mj_loadPluginLibrary(library_path);
  int slot = -1;
  ASSERT_NE(mjp_getPlugin(plugin_name, &slot), nullptr);
  EXPECT_GE(slot, 0);

  EXPECT_NO_THROW(mujoco_ros_utils::test::MujocoModel model(fixture));
}

INSTANTIATE_TEST_SUITE_P(
  ModularLibraries,
  IndependentPluginLoadTest,
  ::testing::Values(
    PluginCase{
      "MUJOCO_ROS_UTILS_CLOCK_PLUGIN_LIBRARY",
      "MujocoRosUtils::ClockPublisher",
      "test_plugin_load_clock.xml"},
    PluginCase{
      "MUJOCO_ROS_UTILS_SENSOR_PLUGIN_LIBRARY",
      "MujocoRosUtils::SensorPublisher",
      "test_plugin_load_sensor.xml"},
    PluginCase{
      "MUJOCO_ROS_UTILS_ACTUATOR_PLUGIN_LIBRARY",
      "MujocoRosUtils::ActuatorCommand",
      "test_plugin_load_actuator.xml"}));

TEST(LidarPluginLoadTest, CompilesAndInitializesCurrentSample)
{
  for (const char *environment_variable : {
      "MUJOCO_ROS_UTILS_CLOCK_PLUGIN_LIBRARY",
      "MUJOCO_ROS_UTILS_POSE_PLUGIN_LIBRARY",
      "MUJOCO_ROS_UTILS_LIDAR_PLUGIN_LIBRARY"})
  {
    const char *library_path = std::getenv(environment_variable);
    if (!library_path || std::string(library_path).empty())
    {
      GTEST_SKIP() << environment_variable << " is not set";
    }
    mj_loadPluginLibrary(library_path);
  }

  int slot = -1;
  ASSERT_NE(mjp_getPlugin("MujocoRosUtils::LidarPublisher", &slot), nullptr);
  EXPECT_GE(slot, 0);
  EXPECT_NO_THROW(
    mujoco_ros_utils::test::MujocoModel model("sample_mujoco_ros_utils_lidar.xml"));
}

TEST(LidarPluginLoadTest, Vlp16RaysExcludeChassisAndHitSceneObjects)
{
  for (const char *environment_variable : {
      "MUJOCO_ROS_UTILS_CLOCK_PLUGIN_LIBRARY",
      "MUJOCO_ROS_UTILS_POSE_PLUGIN_LIBRARY",
      "MUJOCO_ROS_UTILS_LIDAR_PLUGIN_LIBRARY"})
  {
    const char *library_path = std::getenv(environment_variable);
    if (!library_path || std::string(library_path).empty())
    {
      GTEST_SKIP() << environment_variable << " is not set";
    }
    mj_loadPluginLibrary(library_path);
  }

  mujoco_ros_utils::test::MujocoModel fixture("sample_mujoco_ros_utils_lidar.xml");
  mj_forward(fixture.model(), fixture.data());
  const int site_id = mj_name2id(fixture.model(), mjOBJ_SITE, "lidar_origin");
  const int body_id = mj_name2id(fixture.model(), mjOBJ_BODY, "lidar_body");
  ASSERT_GE(site_id, 0);
  ASSERT_GE(body_id, 0);

  const mjtNum *origin = fixture.data()->site_xpos + 3 * site_id;
  const mjtNum *rotation = fixture.data()->site_xmat + 9 * site_id;
  constexpr std::array<double, 16> elevations_degrees = {
    -15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
    1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0};
  constexpr int horizontal_ray_count = 360;
  constexpr int ray_count =
    horizontal_ray_count * static_cast<int>(elevations_degrees.size());
  std::vector<mjtNum> directions(static_cast<std::size_t>(ray_count) * 3);
  int index = 0;
  for (const double elevation_degrees : elevations_degrees)
  {
    const double elevation = elevation_degrees * M_PI / 180.0;
    for (int horizontal_index = 0;
         horizontal_index < horizontal_ray_count;
         ++horizontal_index, ++index)
    {
      const double azimuth =
        2.0 * M_PI * horizontal_index / horizontal_ray_count;
      const std::array<mjtNum, 3> local = {
        static_cast<mjtNum>(std::cos(elevation) * std::cos(azimuth)),
        static_cast<mjtNum>(std::cos(elevation) * std::sin(azimuth)),
        static_cast<mjtNum>(std::sin(elevation))};
      directions[3 * index] =
        rotation[0] * local[0] + rotation[1] * local[1] + rotation[2] * local[2];
      directions[3 * index + 1] =
        rotation[3] * local[0] + rotation[4] * local[1] + rotation[5] * local[2];
      directions[3 * index + 2] =
        rotation[6] * local[0] + rotation[7] * local[1] + rotation[8] * local[2];
    }
  }

  std::vector<int> geom_ids(ray_count, -1);
  std::vector<mjtNum> distances(ray_count, -1.0);
  mj_multiRay(
    fixture.model(), fixture.data(), origin, directions.data(), nullptr, 1, body_id,
    geom_ids.data(), distances.data(), nullptr, ray_count, 100.0);

  const auto hit_count = std::count_if(
    distances.begin(), distances.end(), [](mjtNum distance) { return distance >= 0.1; });
  EXPECT_GT(hit_count, 0);
}

}  // namespace
