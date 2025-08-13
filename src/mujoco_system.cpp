#include "mujoco_system.hpp"

namespace mujoco_ros2_control
{

struct Joint
{
  std::size_t id;   // id of the joint in the Mujoco model
  std::string name; // Name of the joint
  double      joint_position;
  double      joint_velocity;
  double      joint_effort;
  double      position_cmd;
  double      velocity_cmd;
  double      effort_cmd;
  bool        is_mimic   = false;
  double      multiplier = 1.0;
  std::size_t mimicked_joint_id;
};

class MujocoSystemPrivate
{
public:
  MujocoSystemPrivate(const mjModel *m, mjData *d)
      : model_(m)
      , data_(d)
  {}

  /// \brief Mujoco model
  const mjModel *model_{nullptr};

  /// \brief Mujoco data
  mjData *data_{nullptr};

  /// \brief State interfaces
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief Command interfaces
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Joint States
  std::vector<Joint> joint_states_;
};

bool MujocoSystem::initialize(rclcpp::Node::SharedPtr node, const mjModel *m, mjData *d,
                              const hardware_interface::HardwareInfo &hardware_info)
{
  if (!node || !m || !d)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize MujocoSystem: null pointer provided.");
    return false;
  }
  node_ = node;
  impl_ = std::make_unique<MujocoSystemPrivate>(m, d);

  RCLCPP_INFO(node_->get_logger(), "Initializing MujocoSystem '%s'\n", hardware_info.name.c_str());

  try
  {
    register_joints(hardware_info, m);
  }
  catch (...)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MujocoSystem for '%s'",
                 hardware_info.name.c_str());
    return false;
  }

  return true;
}

void MujocoSystem::register_joints(const hardware_interface::HardwareInfo &hardware_info,
                                   const mjModel                          *m)
{
  // Register joints from the URDF information
  for (unsigned int i = 0; i < hardware_info.joints.size(); ++i)
  {
    auto &joint_info = hardware_info.joints[i];

    Joint new_joint;

    // Find joint name in the model
    int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_info.name.c_str());
    if (joint_id < 0)
    {
      RCLCPP_WARN(node_->get_logger(), "Joint '%s' not found in Mujoco model",
                  joint_info.name.c_str());
      std::stringstream ss;
      ss << "Joint '" << joint_info.name << "' not found in Mujoco model";
      throw std::runtime_error(ss.str());
    }

    RCLCPP_INFO(node_->get_logger(), "Registering joint '%s' id %d", joint_info.name.c_str(),
                joint_id);
    new_joint.name = joint_info.name;
    new_joint.id   = joint_id;

    auto mimicked_joint_it = joint_info.parameters.find("mimic");

    if (mimicked_joint_it != joint_info.parameters.end())
    {
      new_joint.is_mimic          = true;
      new_joint.mimicked_joint_id = mj_name2id(m, mjOBJ_JOINT, mimicked_joint_it->second.c_str());
      RCLCPP_INFO(node_->get_logger(), "Joint '%s is mimicking joint '%s'", new_joint.name.c_str(),
                  mimicked_joint_it->second.c_str());
      auto param_it = joint_info.parameters.find("multiplier");
      if (param_it != joint_info.parameters.end())
      {
        new_joint.multiplier = std::stod(param_it->second);
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Mimic joint '%s' does not have 'multiplier' parameter. Default to 1.0",
                    joint_info.name.c_str());
        new_joint.multiplier = 1.0; // Default multiplier
      }
      continue;
    }
  }
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
  return std::move(this->impl_->state_interfaces_);
}

std::vector<CommandInterface> MujocoSystem::export_command_interfaces()
{
  // Export command interfaces
  return std::move(this->impl_->command_interfaces_);
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