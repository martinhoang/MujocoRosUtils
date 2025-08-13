#pragma once

#include "mujoco_system_interface.hpp"

using namespace rclcpp_lifecycle;
using namespace hardware_interface;

namespace mujoco_ros2_control
{

class MujocoSystemPrivate;

class MujocoSystem : public MujocoSystemInterface
{
public:
  /**
   * @brief Initialize the MujocoSystemInterface
   *
   * @param node  pointer to the ROS 2 node
   * @param m constant pointer to the Mujoco model
   * @param d pointer to the Mujoco data
   * @param info hardware interface information
   * @return true if initialization was successful
   * @return false if initialization failed
   */
  bool initialize(rclcpp::Node::SharedPtr node, const mjModel *m, mjData *d,
                  const hardware_interface::HardwareInfo &info) override;

  CallbackReturn on_activate(const State &previous_state);
  CallbackReturn on_deactivate(const State &previous_state);

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  void register_joints(const hardware_interface::HardwareInfo &hardware_info, const mjModel *m);
  std::unique_ptr<MujocoSystemPrivate> impl_;
};

} // namespace mujoco_ros2_control