#include "mujoco_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

namespace mujoco_ros2_control
{

struct Joint
{
  std::size_t id;          // id of the joint in the Mujoco model
  std::size_t actuator_id; // id of the actuator for this joint in the Mujoco model
  std::string name;        // Name of the joint
  double      position;
  double      initial_position;
  double      velocity;
  double      initial_velocity;
  double      effort;
  double      initial_effort;
  double      position_cmd;
  double      velocity_cmd;
  double      effort_cmd;
  std::string control_type;
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
  std::vector<Joint> joints_;

  /// \brief Joint command publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_publisher_;
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

  impl_->joint_cmd_publisher_
    = node_->create_publisher<sensor_msgs::msg::JointState>("joint_commands", rclcpp::QoS(10));

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

void MujocoSystem::reset(void)
{
  // Reset all joint states
  std::stringstream ss;
  for (auto &joint : impl_->joints_)
  {
    const char *joint_name_char = mj_id2name(impl_->model_, mjOBJ_JOINT, joint.id);

    std::string joint_name;
    if (joint_name_char && strlen(joint_name_char) > 0)
    {
      joint_name = joint_name_char;
    }
    else
    {
      joint_name = "unknown_joint_" + std::to_string(joint.id);
    }
    ss << "Joint '" << joint_name << "' reset to: " << joint.initial_position << std::endl;
    joint.position                            = joint.initial_position;
    joint.position_cmd                        = joint.initial_position;
    joint.velocity                            = joint.initial_velocity;
    joint.velocity_cmd                        = joint.initial_velocity;
    joint.effort                              = joint.initial_effort;
    joint.effort_cmd                          = joint.initial_effort;
    int joint_data_id_in_qpos                 = impl_->model_->jnt_qposadr[joint.id];
    impl_->data_->qpos[joint_data_id_in_qpos] = joint.initial_position;

    if (joint.control_type == hardware_interface::HW_IF_POSITION)
    {
      impl_->data_->ctrl[joint.actuator_id] = joint.initial_position;
    }
    else if (joint.control_type == hardware_interface::HW_IF_VELOCITY)
    {
      impl_->data_->ctrl[joint.actuator_id] = joint.initial_velocity;
    }
    else if (joint.control_type == hardware_interface::HW_IF_EFFORT)
    {
      impl_->data_->ctrl[joint.actuator_id] = joint.initial_effort;
    }
  }

  if (ss.str().length() > 0)
  {
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "No joints to reset");
  }

  RCLCPP_INFO(node_->get_logger(), "MujocoSystem reset");
}

void MujocoSystem::register_joints(const hardware_interface::HardwareInfo &hardware_info,
                                   const mjModel                          *m)
{
  impl_->joints_.resize(hardware_info.joints.size());

  // Register joints from the URDF information
  for (unsigned int i = 0; i < hardware_info.joints.size(); ++i)
  {
    auto &joint_info = hardware_info.joints[i];

    Joint &last_joint = impl_->joints_[i];

    // Find joint name in the model
    int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_info.name.c_str());
    if (joint_id < 0)
    {
      RCLCPP_WARN(node_->get_logger(), "Joint '%s' not found in Mujoco model",
                  joint_info.name.c_str());
      throw std::runtime_error("Joint '" + joint_info.name + "' not found in Mujoco model");
    }

    last_joint.name = joint_info.name;
    last_joint.id   = joint_id;

    auto mimicked_joint_it = joint_info.parameters.find("mimic");

    if (mimicked_joint_it != joint_info.parameters.end())
    {
      last_joint.is_mimic          = true;
      last_joint.mimicked_joint_id = mj_name2id(m, mjOBJ_JOINT, mimicked_joint_it->second.c_str());
      auto param_it                = joint_info.parameters.find("multiplier");
      if (param_it != joint_info.parameters.end())
      {
        last_joint.multiplier = std::stod(param_it->second);
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Mimic joint '%s' does not have 'multiplier' parameter. Default to 1.0",
                    joint_info.name.c_str());
        last_joint.multiplier = 1.0; // Default multiplier
      }
    }

    //* Get the "position" PID actuator corresponding with this actuator-plugin-joint pair
    // since I dont know how to make a PID controller yet
    // I latched on to the logics of the existing built-in position PID controller
    // Trick: check if there is a "position" actuator for that joint already
    // by checking if gain_type is mjGAIN_FIXED and bias_type is mjBIAS_AFFINE
    for (int idx = 0; idx < m->nu; ++idx)
    {
      // If the actuator is associated with this joint
      if (m->actuator_trnid[2 * idx] == joint_id)
      {
        RCLCPP_INFO(node_->get_logger(), "Actuator %d is associated with joint id %ld - '%s'", idx,
                    last_joint.id, last_joint.name.c_str());
        int gain_type = m->actuator_gaintype[idx];
        int bias_type = m->actuator_biastype[idx];

        if (gain_type == mjGAIN_FIXED && bias_type == mjBIAS_AFFINE)
        {
          last_joint.actuator_id = idx;
          break; // No need to check further, we found the actuator
        }
      }
    }

    // impl_->joints_[i] = new_joint;
    // Joint &last_joint = impl_->joints_[i];

    if (last_joint.is_mimic)
    {
      RCLCPP_INFO(node_->get_logger(), "Registered mimic joint '%s' id %d\n", joint_info.name.c_str(),
                  joint_id);
      continue;
    }

    //* Get initial value of joint
    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort   = std::numeric_limits<double>::quiet_NaN();

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo &info) {
      double initial_value{0.0};

      if (!info.initial_value.empty())
      {
        try
        {
          // Converting string double value
          initial_value = std::stod(info.initial_value);
        }
        catch (std::invalid_argument &e)
        {
          RCLCPP_ERROR(node_->get_logger(), "Invalid initial value '%s' for joint '%s': %s",
                       info.initial_value.c_str(), info.name.c_str(), e.what());
          throw std::invalid_argument("Failed converting 'initial_value' string for joint '"
                                      + info.name + "'");
        }
      }
      return initial_value;
    };

    // * Connfiguring state interfaces internal variables
    for (const auto &state_if : joint_info.state_interfaces)
    {
      RCLCPP_INFO(node_->get_logger(), "Registering State Interface '%s' for joint '%s'",
                  state_if.name.c_str(), joint_info.name.c_str());
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        impl_->state_interfaces_.emplace_back(joint_info.name, state_if.name, &last_joint.position);
        initial_position = get_initial_value(state_if);
        if (!std::isnan(initial_position))
        {
          last_joint.initial_position               = initial_position;
          last_joint.position                       = initial_position;
          int joint_data_id_in_qpos                 = impl_->model_->jnt_qposadr[joint_id];
          impl_->data_->qpos[joint_data_id_in_qpos] = initial_position;
        }
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        impl_->state_interfaces_.emplace_back(joint_info.name, state_if.name, &last_joint.velocity);
        initial_velocity = get_initial_value(state_if);
        if (!std::isnan(initial_velocity))
        {
          last_joint.initial_velocity               = initial_velocity;
          last_joint.velocity                       = initial_velocity;
          int joint_data_id_in_qvel                 = impl_->model_->jnt_dofadr[joint_id];
          impl_->data_->qvel[joint_data_id_in_qvel] = initial_velocity;
        }
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        impl_->state_interfaces_.emplace_back(joint_info.name, state_if.name, &last_joint.effort);
        initial_effort = get_initial_value(state_if);
        if (!std::isnan(initial_effort))
        {
          last_joint.initial_effort                         = initial_effort;
          last_joint.effort                                 = initial_effort;
          int joint_data_id_in_qvel                         = impl_->model_->jnt_dofadr[joint_id];
          impl_->data_->qfrc_applied[joint_data_id_in_qvel] = initial_effort;
        }
      }
    }

    // * Configuring command interfaces internal variables
    for (const auto &cmd_if : joint_info.command_interfaces)
    {
      RCLCPP_INFO(node_->get_logger(), "Registering Command Interface '%s' for joint '%s'",
                  cmd_if.name.c_str(), joint_info.name.c_str());
      if (cmd_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
      {
        impl_->command_interfaces_.emplace_back(joint_info.name, cmd_if.name,
                                                &last_joint.position_cmd);
        last_joint.control_type = hardware_interface::HW_IF_POSITION;
        if (!std::isnan(initial_position))
        {
          last_joint.position_cmd                    = initial_position;
          impl_->data_->ctrl[last_joint.actuator_id] = initial_position;
        }
      }
      if (cmd_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      {
        impl_->command_interfaces_.emplace_back(joint_info.name, cmd_if.name,
                                                &last_joint.velocity_cmd);
        last_joint.control_type = hardware_interface::HW_IF_VELOCITY;

        if (!std::isnan(initial_velocity))
        {
          last_joint.velocity_cmd                    = initial_velocity;
          impl_->data_->ctrl[last_joint.actuator_id] = initial_velocity;
        }
      }
      if (cmd_if.name.find(hardware_interface::HW_IF_EFFORT) != std::string::npos)
      {
        impl_->command_interfaces_.emplace_back(joint_info.name, cmd_if.name,
                                                &last_joint.effort_cmd);
        last_joint.control_type = hardware_interface::HW_IF_EFFORT;
        if (!std::isnan(initial_effort))
        {
          last_joint.effort_cmd                      = initial_effort;
          impl_->data_->ctrl[last_joint.actuator_id] = initial_effort;
        }
      }
    }

    RCLCPP_INFO(node_->get_logger(), "Registered joint '%s' id %d\n", joint_info.name.c_str(),
                joint_id);
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
  RCLCPP_INFO(node_->get_logger(), "Exporting state interfaces for MujocoSystem");
  return std::move(impl_->state_interfaces_);
}

std::vector<CommandInterface> MujocoSystem::export_command_interfaces()
{
  // Export command interfaces
  RCLCPP_INFO(node_->get_logger(), "Exporting command interfaces for MujocoSystem");
  return std::move(impl_->command_interfaces_);
}

return_type MujocoSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  for (unsigned int i = 0; i < impl_->joints_.size(); ++i)
  {
    auto &joint = impl_->joints_[i];
    if (joint.is_mimic)
    {
      continue;
    }
    int joint_data_id_in_qpos = impl_->model_->jnt_qposadr[joint.id];
    joint.position            = impl_->data_->qpos[joint_data_id_in_qpos];
    int joint_data_id_in_qvel = impl_->model_->jnt_dofadr[joint.id];
    joint.velocity            = impl_->data_->qvel[joint_data_id_in_qvel];
    joint.effort              = impl_->data_->qfrc_applied[joint_data_id_in_qvel];
  }
  return return_type::OK;
}

return_type MujocoSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{

  // * If sim is reset
  if (period.seconds() <= 0)
  {
    reset();
  }

  // * Update joints command
  std::stringstream            ss;
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.name.resize(impl_->joints_.size());
  joint_state_msg.position.resize(impl_->joints_.size());
  for (unsigned int i = 0; i < impl_->joints_.size(); ++i)
  {
    auto &joint                 = impl_->joints_[i];
    joint_state_msg.name[i]     = joint.name;
    joint_state_msg.position[i] = joint.position_cmd;

    // Find the actuator id corresponding to this joint id
    if (joint.is_mimic)
    {
      // Control mimicking joint follows mimicked joint's position with the multiplier
      // impl_->data_->qpos[joint.id]
      //   = joint.multiplier * impl_->data_->qpos[joint.mimicked_joint_id];
      // impl_->data_->ctrl[joint.actuator_id]
      //   = joint.multiplier * impl_->data_->qpos[joint.mimicked_joint_id];
      continue; // logic of mimic joints is already in the MimicJoint plugin
    }
    else
    {
      // For now only allow for 1 single type of controller
      // Maybe in the future open to two ?
      if (joint.control_type == hardware_interface::HW_IF_POSITION)
      {
        ss << "Joint '" << joint.name.c_str() << "' pos_cmd: " << joint.position_cmd << std::endl;
        // RCLCPP_INFO(node_->get_logger(), "Joint '%s' pos_cmd: %.4f", joint.name.c_str(),
        // joint.position_cmd);
        impl_->data_->ctrl[joint.actuator_id] = joint.position_cmd;
      }
      else if (joint.control_type == hardware_interface::HW_IF_VELOCITY)
      {
        impl_->data_->ctrl[joint.actuator_id] = joint.velocity_cmd;
      }
      else if (joint.control_type == hardware_interface::HW_IF_EFFORT)
      {
        impl_->data_->ctrl[joint.actuator_id] = joint.effort_cmd;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Unsupported control type '%s' for joint '%s'",
                    joint.control_type.c_str(), joint.name.c_str());
        return return_type::ERROR;
      }
    }
  }

  if (impl_->joint_cmd_publisher_)
  {
    joint_state_msg.header.stamp = time;
    impl_->joint_cmd_publisher_->publish(joint_state_msg);
  }
  static rclcpp::Time previous_update_time{(uint64_t)0, RCL_ROS_TIME};

  double update_freq   = 1 / (time - previous_update_time).seconds();
  previous_update_time = time;

  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Joint
  // commands:\n%s\nUpdate frequency: %.4f", ss.str().c_str(), update_freq);
  return return_type::OK;
}

} // namespace mujoco_ros2_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem,
                       mujoco_ros2_control::MujocoSystemInterface)