#include "mujoco_system.hpp"

namespace mujoco_ros2_control
{

bool MujocoSystem::initialize(rclcpp::Node::SharedPtr node, const mjModel *m, mjData* d,
                            const hardware_interface::HardwareInfo &info)

{
  printf("Initializing 'MujocoSystem' with for: %s\n", info.name.c_str());
  return true;
}

CallbackReturn MujocoSystem::on_configure(const State &previous_state)
{
  // Setting up the communication to the hardware
  printf("Configuring 'MujocoSystem' with previous state: %s\n", previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoSystem::on_cleanup(const State &previous_state)
{
  // Do the opposite of on_configure()
  printf("Cleaning up 'MujocoSystem' with previous state: %s\n", previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoSystem::on_shutdown(const State &previous_state)
{
  printf("Shutting down 'MujocoSystem' with previous state: %s\n", previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoSystem::on_activate(const State &previous_state)
{
  // Activate the hardware power
  printf("Activating 'MujocoSystem' with previous state: %s\n", previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoSystem::on_deactivate(const State &previous_state)
{
  // Turn off the hardware power
  printf("Deactivating 'MujocoSystem' with previous state: %s\n", previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoSystem::on_error(const State &previous_state)
{
  // Handle errors
  printf("Error occurred in 'MujocoSystem' with previous state: %s\n",
         previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_init(const HardwareInfo &hardware_info)
{
  // Initialize the hardware interface from the URDF data
  hardware_interface::SystemInterface::on_init(hardware_info);
  return CallbackReturn::SUCCESS;
}
std::vector<StateInterface> MujocoSystem::export_state_interfaces()
{
  // Export state interfaces
  return std::vector<StateInterface>();
}
std::vector<CommandInterface> MujocoSystem::export_command_interfaces()
{
  // Export command interfaces
  return std::vector<CommandInterface>();
}
return_type MujocoSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  //
  return return_type::OK;
}
return_type MujocoSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  //
  return return_type::OK;
}

} // namespace mujoco_ros2_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem,
                       mujoco_ros2_control::MujocoSystemInterface)