#pragma once

#include "mujoco_system_interface.hpp"

using namespace rclcpp_lifecycle;
using namespace hardware_interface;

namespace mujoco_ros_utils
{

class MujocoSystem : public MujocoSystemInterface
{
public:
  MujocoSystem() = default;

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

  CallbackReturn on_configure(const State &previous_state);
  CallbackReturn on_cleanup(const State &previous_state);
  CallbackReturn on_shutdown(const State &previous_state);
  CallbackReturn on_activate(const State &previous_state);
  CallbackReturn on_deactivate(const State &previous_state);
  CallbackReturn on_error(const State &previous_state);

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  CallbackReturn                on_init(const HardwareInfo &hardware_info) override;
  std::vector<StateInterface>   export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;
  return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
};

} // namespace mujoco_ros_utils