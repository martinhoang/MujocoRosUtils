#include "mujoco_system.hpp"

namespace mujoco_ros2_control
{

bool MujocoSystem::initialize(rclcpp::Node::SharedPtr node, const mjModel *m, mjData *d,
                              const hardware_interface::HardwareInfo &info)
{
  if (!node || !m || !d)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize MujocoSystem: null pointer provided.");
    return false;
  }
  node_  = node;
  model_ = m;
  data_  = d;
  RCLCPP_INFO(node_->get_logger(), "Initializing MujocoSystem '%s'\n", info.name.c_str());
  return true;
}

CallbackReturn MujocoSystem::on_configure(const State &previous_state)
{
  // Setting up the communication to the hardware
  // Nothing to do here for Mujoco
  // RCLCPP_INFO(node_->get_logger(), "Configuring 'MujocoSystem' from previous state: %s\n",
  //             previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_cleanup(const State &previous_state)
{
  // Do the opposite of on_configure()
  // Nothing to clean up here
  // RCLCPP_INFO(node_->get_logger(), "Cleaning up 'MujocoSystem' from previous state: %s\n",
  //             previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_shutdown(const State &previous_state)
{
  // Hardware interface is ready for unloading/destruction
  RCLCPP_INFO(node_->get_logger(), "Shutting down 'MujocoSystem' from previous state: %s\n",
              previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_activate(const State &previous_state)
{
  // Activate the hardware power
  RCLCPP_INFO(node_->get_logger(), "Activating 'MujocoSystem' from previous state: %s\n",
              previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_deactivate(const State &previous_state)
{
  // Turn off the hardware power
  RCLCPP_INFO(node_->get_logger(), "Deactivating 'MujocoSystem' from previous state: %s\n",
              previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_error(const State &previous_state)
{
  // Handle errors
  RCLCPP_INFO(node_->get_logger(), "Error occurred in 'MujocoSystem' from previous state: %s\n",
              previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_init(const HardwareInfo &hardware_info)
{
  // Initialize the hardware interface from the URDF data
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> MujocoSystem::export_state_interfaces()
{
  // Export state interfaces
  return std::move(state_interfaces_);
}

std::vector<CommandInterface> MujocoSystem::export_command_interfaces()
{
  // Export command interfaces
  return std::move(command_interfaces_);
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