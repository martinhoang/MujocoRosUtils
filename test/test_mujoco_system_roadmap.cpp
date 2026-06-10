#include "mujoco_system_interface.hpp"
#include "roadmap_test_utils.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace
{

using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using hardware_interface::return_type;
using mujoco_ros2_control::MujocoSystemInterface;
using mujoco_ros_utils::test::MujocoModel;

// set_value returns bool in Jazzy+ (RCLCPP >= 28) but void in Humble
#if RCLCPP_VERSION_MAJOR >= 28
#define EXPECT_SET_VALUE(cmd, val) ASSERT_TRUE((cmd)->set_value(val))
#else
#define EXPECT_SET_VALUE(cmd, val) (cmd)->set_value(val)
#endif

template<typename Interface>
std::unordered_map<std::string, Interface *> by_name(std::vector<Interface> &interfaces)
{
  std::unordered_map<std::string, Interface *> result;
  for (auto &interface : interfaces)
  {
    result.emplace(interface.get_name(), &interface);
  }
  return result;
}

class MujocoSystemRoadmapTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    model_ = std::make_unique<MujocoModel>("test_ros2_control_interfaces.xml");
    node_ = std::make_shared<rclcpp::Node>("mujoco_system_roadmap_test");
    loader_ = std::make_unique<pluginlib::ClassLoader<MujocoSystemInterface>>(
      "mujoco_ros_utils", "mujoco_ros2_control::MujocoSystemInterface");
    system_ = loader_->createSharedInstance("mujoco_ros2_control/MujocoSystem");
    info_ = mujoco_ros_utils::test::hardware_info("test_ros2_control_interfaces.urdf");
  }

  bool initialize()
  {
    return system_->initialize(node_, model_->model(), model_->data(), info_);
  }

  std::unique_ptr<MujocoModel> model_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<pluginlib::ClassLoader<MujocoSystemInterface>> loader_;
  std::shared_ptr<MujocoSystemInterface> system_;
  hardware_interface::HardwareInfo info_;
};

TEST_F(MujocoSystemRoadmapTest, ExportsEverySensorProfileAndGpioDirection)
{
  ASSERT_TRUE(initialize());
  auto state_interfaces = system_->export_state_interfaces();
  auto command_interfaces = system_->export_command_interfaces();
  const auto states = by_name(state_interfaces);
  const auto commands = by_name(command_interfaces);

  for (const auto &name : {
      "base_imu/orientation.x", "base_imu/angular_velocity.z",
      "base_imu/linear_acceleration.y", "tool_wrench/force.x",
      "tool_wrench/torque.z", "proximity/range", "tool_pose/position.x",
      "tool_pose/orientation.w", "tool_feedback/temperature",
      "input_only/input_0", "input_only/input_1", "bidirectional/pressure",
      "bidirectional/commanded_level"})
  {
    EXPECT_TRUE(states.count(name)) << name;
  }
  EXPECT_TRUE(commands.count("output_only/level"));
  EXPECT_TRUE(commands.count("bidirectional/level"));
}

TEST_F(MujocoSystemRoadmapTest, ReadsSensorAndBidirectionalGpioState)
{
  ASSERT_TRUE(initialize());
  auto state_interfaces = system_->export_state_interfaces();
  const auto states = by_name(state_interfaces);

  const int temperature_id = mj_name2id(
    model_->model(), mjOBJ_SENSOR, "tool_temperature");
  const int digital_id = mj_name2id(model_->model(), mjOBJ_SENSOR, "digital_inputs");
  ASSERT_GE(temperature_id, 0);
  ASSERT_GE(digital_id, 0);
  model_->data()->sensordata[model_->model()->sensor_adr[temperature_id]] = 25000.0;
  model_->data()->sensordata[model_->model()->sensor_adr[digital_id]] = 1.0;
  model_->data()->sensordata[model_->model()->sensor_adr[digital_id] + 1] = 0.0;

  ASSERT_EQ(
    system_->read(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  EXPECT_DOUBLE_EQ(states.at("tool_feedback/temperature")->get_value(), 2480.0);
  EXPECT_DOUBLE_EQ(states.at("input_only/input_0")->get_value(), 1.0);
  EXPECT_DOUBLE_EQ(states.at("input_only/input_1")->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(states.at("bidirectional/pressure")->get_value(), 25.0);
}

TEST_F(MujocoSystemRoadmapTest, WritesAndClampsGpioCommands)
{
  ASSERT_TRUE(initialize());
  auto command_interfaces = system_->export_command_interfaces();
  const auto commands = by_name(command_interfaces);
  EXPECT_SET_VALUE(commands.at("output_only/level"), 2.0);
  EXPECT_SET_VALUE(commands.at("bidirectional/level"), -0.4);

  ASSERT_EQ(
    system_->write(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  const int output_id = mj_name2id(model_->model(), mjOBJ_ACTUATOR, "gpio_output");
  const int bidirectional_id = mj_name2id(
    model_->model(), mjOBJ_ACTUATOR, "gpio_bidirectional");
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[output_id], 1.0);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[bidirectional_id], -0.4);
}

TEST_F(MujocoSystemRoadmapTest, ResetClearsCommandsAndRestoresInitialValues)
{
  ASSERT_TRUE(initialize());
  auto command_interfaces = system_->export_command_interfaces();
  const auto commands = by_name(command_interfaces);
  EXPECT_SET_VALUE(commands.at("controlled_joint/position"), 1.5);
  EXPECT_SET_VALUE(commands.at("output_only/level"), 0.8);
  ASSERT_EQ(
    system_->write(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);

  ASSERT_EQ(
    system_->write(rclcpp::Time(0, 0), rclcpp::Duration::from_seconds(0.0)),
    return_type::OK);
  const int position_id = mj_name2id(
    model_->model(), mjOBJ_ACTUATOR, "controlled_position");
  const int output_id = mj_name2id(model_->model(), mjOBJ_ACTUATOR, "gpio_output");
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[position_id], 0.25);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[output_id], 0.0);
  EXPECT_DOUBLE_EQ(commands.at("controlled_joint/position")->get_value(), 0.25);
  EXPECT_DOUBLE_EQ(commands.at("output_only/level")->get_value(), 0.0);
}

TEST_F(MujocoSystemRoadmapTest, RejectsConflictingJointCommandModes)
{
  ASSERT_TRUE(initialize());
  EXPECT_EQ(
    system_->prepare_command_mode_switch(
      {"controlled_joint/position", "controlled_joint/velocity"}, {}),
    return_type::ERROR);
  EXPECT_EQ(
    system_->prepare_command_mode_switch({"controlled_joint/position"}, {}),
    return_type::OK);
  EXPECT_EQ(
    system_->perform_command_mode_switch({"controlled_joint/position"}, {}),
    return_type::OK);
}

TEST_F(MujocoSystemRoadmapTest, WritesOnlyTheActiveExplicitlyMappedActuator)
{
  ASSERT_TRUE(initialize());
  auto command_interfaces = system_->export_command_interfaces();
  const auto commands = by_name(command_interfaces);
  const int position_id = mj_name2id(
    model_->model(), mjOBJ_ACTUATOR, "controlled_position");
  const int velocity_id = mj_name2id(
    model_->model(), mjOBJ_ACTUATOR, "controlled_velocity");
  ASSERT_GE(position_id, 0);
  ASSERT_GE(velocity_id, 0);

  EXPECT_SET_VALUE(commands.at("controlled_joint/position"), 1.25);
  EXPECT_SET_VALUE(commands.at("controlled_joint/velocity"), 2.0);
  ASSERT_EQ(
    system_->write(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[position_id], 1.25);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[velocity_id], 0.0);

  const std::vector<std::string> start = {"controlled_joint/velocity"};
  const std::vector<std::string> stop = {"controlled_joint/position"};
  ASSERT_EQ(system_->prepare_command_mode_switch(start, stop), return_type::OK);
  ASSERT_EQ(system_->perform_command_mode_switch(start, stop), return_type::OK);
  EXPECT_SET_VALUE(commands.at("controlled_joint/velocity"), 2.0);
  ASSERT_EQ(
    system_->write(rclcpp::Time(2, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[position_id], 1.25);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[velocity_id], 2.0);
}

TEST_F(MujocoSystemRoadmapTest, LifecycleAllowsReadsAndGatesWrites)
{
  ASSERT_TRUE(initialize());
  auto state_interfaces = system_->export_state_interfaces();
  auto command_interfaces = system_->export_command_interfaces();
  const auto states = by_name(state_interfaces);
  const auto commands = by_name(command_interfaces);
  const int position_id = mj_name2id(
    model_->model(), mjOBJ_ACTUATOR, "controlled_position");
  const int joint_id = mj_name2id(model_->model(), mjOBJ_JOINT, "controlled_joint");
  ASSERT_GE(position_id, 0);
  ASSERT_GE(joint_id, 0);

  EXPECT_SET_VALUE(commands.at("controlled_joint/position"), 1.0);
  ASSERT_EQ(
    system_->on_deactivate(rclcpp_lifecycle::State(3, "active")),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system_->write(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[position_id], 0.25);

  model_->data()->qpos[model_->model()->jnt_qposadr[joint_id]] = 0.75;
  ASSERT_EQ(
    system_->read(rclcpp::Time(2, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  EXPECT_DOUBLE_EQ(states.at("controlled_joint/position")->get_value(), 0.75);

  ASSERT_EQ(
    system_->on_activate(rclcpp_lifecycle::State(2, "inactive")),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system_->write(rclcpp::Time(3, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::OK);
  EXPECT_DOUBLE_EQ(model_->data()->ctrl[position_id], 1.0);
}

TEST_F(MujocoSystemRoadmapTest, ExposesOptInReadWriteDiagnostics)
{
  info_.hardware_parameters["diagnostics_enabled"] = "true";
  ASSERT_TRUE(initialize());

  ASSERT_EQ(
    system_->read(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(1.0)),
    return_type::OK);
  ASSERT_EQ(
    system_->write(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(1.0)),
    return_type::OK);
  ASSERT_EQ(
    system_->write(rclcpp::Time(0, 0), rclcpp::Duration::from_seconds(0.0)),
    return_type::OK);

  const auto diagnostics = system_->diagnostics();
  EXPECT_TRUE(diagnostics.enabled);
  EXPECT_EQ(diagnostics.read_count, 1u);
  EXPECT_EQ(diagnostics.write_count, 2u);
  EXPECT_EQ(diagnostics.reset_count, 1u);
  EXPECT_GT(diagnostics.last_read_duration_ns, 0u);
  EXPECT_GE(diagnostics.max_read_duration_ns, diagnostics.last_read_duration_ns);
  EXPECT_GT(diagnostics.last_write_duration_ns, 0u);
  EXPECT_GE(diagnostics.max_write_duration_ns, diagnostics.last_write_duration_ns);
}

TEST_F(MujocoSystemRoadmapTest, RejectsAndCountsNonFiniteCommands)
{
  info_.hardware_parameters["diagnostics_enabled"] = "true";
  ASSERT_TRUE(initialize());
  auto command_interfaces = system_->export_command_interfaces();
  const auto commands = by_name(command_interfaces);
  EXPECT_SET_VALUE(commands.at("controlled_joint/position"),
      std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(
    system_->write(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.001)),
    return_type::ERROR);
  const auto diagnostics = system_->diagnostics();
  EXPECT_EQ(diagnostics.invalid_command_count, 1u);
  EXPECT_EQ(diagnostics.write_count, 1u);
}

class InvalidMappingTest : public ::testing::TestWithParam<const char *>
{
protected:
  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};

TEST_P(InvalidMappingTest, FailsBeforeInterfacesAreExported)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  MujocoModel model("test_ros2_control_interfaces.xml");
  auto node = std::make_shared<rclcpp::Node>(
    std::string("invalid_mapping_") + GetParam());
  pluginlib::ClassLoader<MujocoSystemInterface> loader(
    "mujoco_ros_utils", "mujoco_ros2_control::MujocoSystemInterface");
  auto system = loader.createSharedInstance("mujoco_ros2_control/MujocoSystem");
  const auto info = mujoco_ros_utils::test::hardware_info(
    "test_ros2_control_invalid_mappings.urdf", GetParam());
  EXPECT_FALSE(system->initialize(node, model.model(), model.data(), info));
}

INSTANTIATE_TEST_SUITE_P(
  Validation,
  InvalidMappingTest,
  ::testing::Values(
    "MissingSensorSource", "UnknownSensor", "SensorIndexOutOfRange",
    "IncompleteImuProfile", "MissingGpioMapping", "DuplicateActuatorOwner",
    "InvalidScale"));

}  // namespace
