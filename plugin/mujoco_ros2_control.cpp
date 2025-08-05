#include "mujoco_ros2_control.hpp"

#include <mujoco/mujoco.h>

#include "mujoco_utils.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace MujocoRosUtils
{

void MujocoRos2Control::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  print_info("Registering MujocoRos2Control plugin\n");

  plugin.name = "MujocoRosUtils::MujocoRos2Control";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char * attributes[] = { "node_name", "joints" };

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int) { return 0; };
  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };

  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id) {
    auto * plugin_instance = MujocoRos2Control::Create(m, d, plugin_id);
    if (!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id) {
    delete reinterpret_cast<MujocoRos2Control *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, void * plugin_data, int plugin_id) {
    auto * plugin_instance = reinterpret_cast<class MujocoRos2Control *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int) {
    auto * plugin_instance = reinterpret_cast<class MujocoRos2Control *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
  print_info("MujocoRos2Control plugin registered\n");
}

MujocoRos2Control * MujocoRos2Control::Create(const mjModel * m, mjData * d, int plugin_id)
{
  const char * node_name_char = mj_getPluginConfig(m, plugin_id, "node_name");
  std::string node_name = "mujoco_ros2_control";
  if (strlen(node_name_char) > 0)
  {
    node_name = std::string(node_name_char);
  }

  const char * joints_char = mj_getPluginConfig(m, plugin_id, "joints");
  if (strlen(joints_char) == 0)
  {
    mju_error("[MujocoRos2Control] `joints` attribute is missing.");
    return nullptr;
  }
  std::string joints_str = std::string(joints_char);
  std::vector<std::string> joint_names;
  std::stringstream ss(joints_str);
  std::string joint_name;
  while (ss >> joint_name)
  {
    joint_names.push_back(joint_name);
  }

  return new MujocoRos2Control(m, d, joint_names, node_name);
}

MujocoRos2Control::MujocoRos2Control(const mjModel * m,
                                     mjData * d,
                                     const std::vector<std::string> & joint_names,
                                     const std::string & node_name)
: m_(m), d_(d)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  nh_ = rclcpp::Node::make_shared(node_name);
  last_update_time_ = nh_->now();

  hardware_interface::HardwareInfo info;
  info.name = "MujocoRos2Control";
  info.type = "system";
  info.hardware_class_type = "MujocoRosUtils/MujocoRos2Control";

  for (const auto & joint_name : joint_names)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = joint_name;
    joint.type = "joint";
    joint.command_interfaces.emplace_back();
    joint.command_interfaces.back().name = "position";
    joint.state_interfaces.emplace_back();
    joint.state_interfaces.back().name = "position";
    joint.state_interfaces.emplace_back();
    joint.state_interfaces.back().name = "velocity";
    info.joints.push_back(joint);
  }

  on_init(info);

  init_ros();
}

MujocoRos2Control::~MujocoRos2Control()
{
  is_running_ = false;
  if (ros_spin_thread_ && ros_spin_thread_->joinable())
  {
    ros_spin_thread_->join();
  }
  if (executor_)
  {
    executor_->cancel();
  }
}

void MujocoRos2Control::init_ros()
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ =
      std::make_shared<controller_manager::ControllerManager>(executor_, nh_, nh_->get_name());

  executor_->add_node(nh_);
  executor_->add_node(controller_manager_);

  is_running_ = true;
  ros_spin_thread_ = std::make_unique<std::thread>([this]() {
    while (is_running_ && rclcpp::ok())
    {
      executor_->spin_once(std::chrono::milliseconds(100));
    }
  });
}

void MujocoRos2Control::reset(const mjModel *, int)
{
  if (controller_manager_)
  {
    // controller_manager_->reset();
  }
}

void MujocoRos2Control::compute(const mjModel *, mjData * d, int)
{
  if (!controller_manager_)
  {
    return;
  }
  rclcpp::Time now = nh_->now();
  rclcpp::Duration period = now - last_update_time_;
  last_update_time_ = now;

  read(now, period);
  controller_manager_->update(now, period);
  write(now, period);
}

hardware_interface::CallbackReturn MujocoRos2Control::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_pos_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_vel_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  qpos_adr_.resize(info_.joints.size());
  qvel_adr_.resize(info_.joints.size());
  ctrl_adr_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    int joint_id = mj_name2id(m_, mjOBJ_JOINT, info_.joints[i].name.c_str());
    if (joint_id == -1)
    {
      RCLCPP_ERROR(nh_->get_logger(), "Joint '%s' not found in Mujoco model", info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    qpos_adr_[i] = m_->jnt_qposadr[joint_id];
    qvel_adr_[i] = m_->jnt_dofadr[joint_id];

    int actuator_id = mj_name2id(m_, mjOBJ_ACTUATOR, info_.joints[i].name.c_str());
    if (actuator_id == -1)
    {
      RCLCPP_WARN(nh_->get_logger(), "Actuator for joint '%s' not found, will not be able to command.",
                  info_.joints[i].name.c_str());
    }
    else
    {
      ctrl_adr_[i] = actuator_id;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoRos2Control::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoRos2Control::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type MujocoRos2Control::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    hw_states_pos_[i] = d_->qpos[qpos_adr_[i]];
    hw_states_vel_[i] = d_->qvel[qvel_adr_[i]];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoRos2Control::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (ctrl_adr_[i] != -1 && !std::isnan(hw_commands_[i]))
    {
      d_->ctrl[ctrl_adr_[i]] = hw_commands_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MujocoRos2Control::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(nh_->get_logger(), "Activating... please wait...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoRos2Control::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(nh_->get_logger(), "Deactivating... please wait...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace MujocoRosUtils
