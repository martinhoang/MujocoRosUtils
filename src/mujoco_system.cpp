#include "mujoco_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mujoco_ros_utils/mujoco_bindings.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace mujoco_ros2_control
{

namespace
{

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();

double parse_number(const std::string & text, const std::string & description)
{
  try
  {
    std::size_t parsed = 0;
    const double value = std::stod(text, &parsed);
    if(parsed != text.size() || !std::isfinite(value))
    {
      throw std::invalid_argument("not a finite number");
    }
    return value;
  }
  catch(const std::exception & e)
  {
    throw std::invalid_argument(
        "Invalid " + description + " value '" + text + "': " + e.what());
  }
}

double parse_optional_number(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & key, double default_value, const std::string & component)
{
  const auto it = parameters.find(key);
  if(it == parameters.end())
  {
    return default_value;
  }
  return parse_number(it->second, "'" + key + "' for component '" + component + "'");
}

bool parse_optional_bool(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & key, bool default_value, const std::string & component)
{
  const auto it = parameters.find(key);
  if(it == parameters.end())
  {
    return default_value;
  }
  std::string value = it->second;
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if(value == "true" || value == "1" || value == "yes" || value == "on")
  {
    return true;
  }
  if(value == "false" || value == "0" || value == "no" || value == "off")
  {
    return false;
  }
  throw std::invalid_argument(
      "Invalid '" + key + "' value '" + it->second + "' for component '" + component + "'");
}

std::string required_parameter(
    const hardware_interface::ComponentInfo & component, const std::string & key)
{
  const auto it = component.parameters.find(key);
  if(it == component.parameters.end() || it->second.empty())
  {
    throw std::invalid_argument(
        "Component '" + component.name + "' requires parameter '" + key + "'");
  }
  return it->second;
}

struct SensorSource
{
  std::string name;
  std::size_t index = 0;
};

SensorSource parse_sensor_source(const std::string & expression, const std::string & context)
{
  constexpr char prefix[] = "mujoco_sensor:";
  if(expression.rfind(prefix, 0) != 0)
  {
    throw std::invalid_argument(
        context + " must use mujoco_sensor:<name>[<index>], got '" + expression + "'");
  }
  const std::string value = expression.substr(sizeof(prefix) - 1);
  const std::size_t open = value.rfind('[');
  const std::size_t close = value.rfind(']');
  if(open == std::string::npos || close != value.size() - 1 || open == 0 || close <= open + 1)
  {
    throw std::invalid_argument(
        context + " must include a zero-based sensor index, got '" + expression + "'");
  }
  const std::string index_text = value.substr(open + 1, close - open - 1);
  if(!std::all_of(index_text.begin(), index_text.end(), [](unsigned char c) { return std::isdigit(c); }))
  {
    throw std::invalid_argument(context + " has an invalid sensor index in '" + expression + "'");
  }
  return {value.substr(0, open), static_cast<std::size_t>(std::stoull(index_text))};
}

std::string parse_actuator_source(const std::string & expression, const std::string & context)
{
  constexpr char prefix[] = "mujoco_actuator:";
  if(expression.rfind(prefix, 0) != 0 || expression.size() == sizeof(prefix) - 1)
  {
    throw std::invalid_argument(
        context + " must use mujoco_actuator:<name>, got '" + expression + "'");
  }
  return expression.substr(sizeof(prefix) - 1);
}

enum class ScalarStateSource
{
  SENSOR,
  ACTUATOR_CONTROL
};

struct ScalarState
{
  std::string component;
  std::string interface;
  ScalarStateSource source = ScalarStateSource::SENSOR;
  MujocoRosUtils::MujocoSensorBinding sensor;
  MujocoRosUtils::MujocoActuatorBinding actuator;
  std::size_t sensor_index = 0;
  double scale = 1.0;
  double offset = 0.0;
  double value = 0.0;
  double initial_value = 0.0;

  void update(const mjData * data)
  {
    const double raw = source == ScalarStateSource::SENSOR
                         ? sensor.read(data, sensor_index)
                         : actuator.read_control(data);
    value = raw * scale + offset;
  }
};

struct ScalarCommand
{
  std::string component;
  std::string interface;
  MujocoRosUtils::MujocoActuatorBinding actuator;
  double value = 0.0;
  double initial_value = 0.0;
  double minimum = std::numeric_limits<double>::lowest();
  double maximum = std::numeric_limits<double>::max();
};

double interface_initial_value(
    const hardware_interface::InterfaceInfo & interface, const std::string & component)
{
  return interface.initial_value.empty()
           ? 0.0
           : parse_number(
               interface.initial_value,
               "initial value for '" + component + "/" + interface.name + "'");
}

void validate_double_interface(
    const hardware_interface::InterfaceInfo & interface, const std::string & component)
{
  if(!MujocoRosUtils::ros2_control_compat::is_scalar_double(interface))
  {
    throw std::invalid_argument(
        "Interface '" + component + "/" + interface.name
        + "' must use scalar double data");
  }
}

} // namespace

struct Joint
{
  enum class CommandMode
  {
    NONE,
    POSITION,
    VELOCITY,
    EFFORT
  };

  std::size_t id;   // id of the joint in the Mujoco model
  std::string name; // Name of the joint

  // States
  double position = 0.0;
  double initial_position = 0.0;
  double velocity = 0.0;
  double initial_velocity = 0.0;
  double effort = 0.0;
  double initial_effort = 0.0;

  // Commands
  double position_cmd = 0.0;
  double velocity_cmd = 0.0;
  double effort_cmd = 0.0;

  // Command limits
  double min_position_cmd = -std::numeric_limits<double>::max();
  double max_position_cmd = std::numeric_limits<double>::max();
  double min_velocity_cmd = -std::numeric_limits<double>::max();
  double max_velocity_cmd = std::numeric_limits<double>::max();
  double min_effort_cmd = -std::numeric_limits<double>::max();
  double max_effort_cmd = std::numeric_limits<double>::max();

  // PID controllers
  control_toolbox::Pid position_pid;
  control_toolbox::Pid velocity_pid;
  bool is_pid_enabled = false;
  bool position_pid_enabled = false;
  bool velocity_pid_enabled = false;

  // Joint limits
  joint_limits::JointLimits joint_limits;

  // Actuator mapping (support multiple actuators per joint)
  std::size_t position_actuator_id = static_cast<std::size_t>(-1);
  std::size_t velocity_actuator_id = static_cast<std::size_t>(-1);
  std::size_t effort_actuator_id   = static_cast<std::size_t>(-1);

  // Which command interfaces are exported for this joint
  bool has_position_cmd = false;
  bool has_velocity_cmd = false;
  bool has_effort_cmd   = false;
  CommandMode active_command_mode = CommandMode::NONE;

  // Mimic support
  bool        is_mimic   = false;
  double      multiplier = 1.0;
  std::size_t mimicked_joint_index = INVALID_INDEX;
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

  /// \brief Scalar sensor and GPIO state storage. deque keeps exported pointers stable.
  std::deque<ScalarState> scalar_states_;

  /// \brief Scalar GPIO command storage. deque keeps exported pointers stable.
  std::deque<ScalarCommand> scalar_commands_;

  /// \brief Actuators with one command owner across joints and GPIOs.
  std::set<int> commanded_actuators_;

  /// \brief Validated command modes waiting for perform_command_mode_switch().
  std::optional<std::vector<Joint::CommandMode>> pending_command_modes_;

  /// \brief Lifecycle state is enforced after the first lifecycle callback.
  bool lifecycle_state_known_{false};
  bool active_{false};

  bool diagnostics_enabled_{false};
  std::atomic<std::uint64_t> read_count_{0};
  std::atomic<std::uint64_t> write_count_{0};
  std::atomic<std::uint64_t> reset_count_{0};
  std::atomic<std::uint64_t> invalid_command_count_{0};
  std::atomic<std::uint64_t> missed_period_count_{0};
  std::atomic<std::uint64_t> last_read_duration_ns_{0};
  std::atomic<std::uint64_t> max_read_duration_ns_{0};
  std::atomic<std::uint64_t> last_write_duration_ns_{0};
  std::atomic<std::uint64_t> max_write_duration_ns_{0};

  /// \brief Joint command publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_publisher_;
};

MujocoSystem::MujocoSystem() = default;
MujocoSystem::~MujocoSystem() = default;

bool MujocoSystem::initialize(rclcpp::Node::SharedPtr node, const mjModel *m, mjData *d,
                              const hardware_interface::HardwareInfo &hardware_info)
{
  if (!node || !m || !d)
  {
    if(node)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize MujocoSystem: null pointer provided.");
    }
    return false;
  }
  node_ = node;
  impl_ = std::make_unique<MujocoSystemPrivate>(m, d);
  try
  {
    impl_->diagnostics_enabled_ = parse_optional_bool(
        hardware_info.hardware_parameters, "diagnostics_enabled", false, hardware_info.name);
  }
  catch(const std::exception & e)
  {
    RCLCPP_ERROR(
        node_->get_logger(), "Failed to initialize MujocoSystem for '%s': %s",
        hardware_info.name.c_str(), e.what());
    return false;
  }

  impl_->joint_cmd_publisher_
    = node_->create_publisher<sensor_msgs::msg::JointState>("joint_commands", rclcpp::QoS(10));

  RCLCPP_INFO(node_->get_logger(), "Initializing MujocoSystem '%s'\n", hardware_info.name.c_str());

  try
  {
    register_joints(hardware_info, m);
    register_sensors(hardware_info);
    register_gpios(hardware_info);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MujocoSystem for '%s': %s",
                 hardware_info.name.c_str(), e.what());
    return false;
  }

  return true;
}

void MujocoSystem::reset(void)
{
  ++impl_->reset_count_;
  std::stringstream ss;
  for (auto &joint : impl_->joints_)
  {
    const char *joint_name_char = mj_id2name(impl_->model_, mjOBJ_JOINT, joint.id);
    std::string joint_name      = (joint_name_char && strlen(joint_name_char) > 0)
                                    ? joint_name_char
                                    : "unknown_joint_" + std::to_string(joint.id);

    ss << "Joint '" << joint_name << "' reset to pos=" << joint.initial_position
       << " vel=" << joint.initial_velocity << "\n";

    // ── mirror variables (also back-fill the command interfaces, which point here) ──
    joint.position     = joint.initial_position;
    joint.velocity     = joint.initial_velocity;
    joint.effort       = joint.initial_effort;
    joint.position_cmd = joint.initial_position;
    joint.velocity_cmd = joint.initial_velocity;
    joint.effort_cmd   = joint.initial_effort;

    // ── MuJoCo physics state ──────────────────────────────────────────────────
    const int qpos_idx                      = impl_->model_->jnt_qposadr[joint.id];
    const int dof_idx                       = impl_->model_->jnt_dofadr[joint.id];
    impl_->data_->qpos[qpos_idx]            = joint.initial_position;
    impl_->data_->qvel[dof_idx]             = joint.initial_velocity;
    // ── MuJoCo actuator commands ─────────────────────────────────────────────
    if (joint.has_position_cmd && joint.position_actuator_id != static_cast<std::size_t>(-1))
      impl_->data_->ctrl[joint.position_actuator_id] =
          joint.position_pid_enabled ? 0.0 : joint.initial_position;
    if (joint.has_velocity_cmd && joint.velocity_actuator_id != static_cast<std::size_t>(-1))
      impl_->data_->ctrl[joint.velocity_actuator_id] =
          joint.velocity_pid_enabled ? 0.0 : joint.initial_velocity;
    if (joint.has_effort_cmd && joint.effort_actuator_id != static_cast<std::size_t>(-1))
      impl_->data_->ctrl[joint.effort_actuator_id] = joint.initial_effort;

    // ── PID state (clears integral windup and error history) ─────────────────
    if (joint.is_pid_enabled)
    {
      joint.position_pid.reset();
      joint.velocity_pid.reset();
    }
  }

  for(auto & state : impl_->scalar_states_)
  {
    state.value = state.initial_value;
  }
  for(auto & command : impl_->scalar_commands_)
  {
    command.value = command.initial_value;
    command.actuator.write(
        impl_->data_, command.initial_value, command.minimum, command.maximum);
  }

  if (!ss.str().empty())
    RCLCPP_DEBUG(node_->get_logger(), "MujocoSystem reset:\n%s", ss.str().c_str());
  else
    RCLCPP_WARN(node_->get_logger(), "No joints to reset");

  RCLCPP_DEBUG(node_->get_logger(), "MujocoSystem reset complete");
}

void MujocoSystem::register_joints(const hardware_interface::HardwareInfo &hardware_info,
                                   const mjModel                          *m)
{
  RCLCPP_INFO(node_->get_logger(), "[register_joints] resizing joints_ to %zu (m=%p d=%p njnt=%ld nq=%ld)",
              hardware_info.joints.size(), (void*)m, (void*)impl_->data_, (long)m->njnt, (long)m->nq);
  impl_->joints_.resize(hardware_info.joints.size());
  RCLCPP_INFO(node_->get_logger(), "[register_joints] joints_ resized OK");

  // Try to load URDF model for joint limits
  urdf::Model urdf_model;
  bool has_urdf = false;
  if (!hardware_info.original_xml.empty())
  {
    if (urdf_model.initString(hardware_info.original_xml))
    {
      has_urdf = true;
      RCLCPP_INFO(node_->get_logger(), "Successfully loaded URDF model for joint limits");
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse URDF from hardware_info, joint limits will not be available");
    }
  }

  // Register joints from the URDF information
  for (unsigned int i = 0; i < hardware_info.joints.size(); ++i)
  {
    auto &joint_info = hardware_info.joints[i];

    Joint &last_joint = impl_->joints_[i];

    // Find joint name in the model
    int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_info.name.c_str());
    if (joint_id < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint '%s' declared in <ros2_control> tag is NOT found in Mujoco model", joint_info.name.c_str());
      throw std::runtime_error("Joint '" + joint_info.name + "' not found in Mujoco model");
    }

    last_joint.name = joint_info.name;
    last_joint.id   = joint_id;

    auto mimicked_joint_it = joint_info.parameters.find("mimic");

    if (mimicked_joint_it != joint_info.parameters.end())
    {
      const auto target_it = std::find_if(
          hardware_info.joints.begin(), hardware_info.joints.end(),
          [&mimicked_joint_it](const hardware_interface::ComponentInfo & candidate)
          {
            return candidate.name == mimicked_joint_it->second;
          });
      if(target_it == hardware_info.joints.end())
      {
        throw std::invalid_argument(
            "Mimic joint '" + joint_info.name + "' references unknown ros2_control joint '"
            + mimicked_joint_it->second + "'");
      }
      last_joint.is_mimic = true;
      last_joint.mimicked_joint_index =
          static_cast<std::size_t>(std::distance(hardware_info.joints.begin(), target_it));
      if(last_joint.mimicked_joint_index == i)
      {
        throw std::invalid_argument("Joint '" + joint_info.name + "' cannot mimic itself");
      }
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

    // Defer actuator mapping until after command interfaces are known

    // impl_->joints_[i] = new_joint;
    // Joint &last_joint = impl_->joints_[i];

    // Get joint limits from URDF if available
    if (has_urdf)
    {
      auto urdf_joint = urdf_model.getJoint(last_joint.name);
      if (urdf_joint)
      {
        get_joint_limits(urdf_joint, last_joint.joint_limits);
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Joint '%s' not found in URDF model", 
                    last_joint.name.c_str());
      }
    }

    if (last_joint.is_mimic)
    {
      RCLCPP_INFO(node_->get_logger(), "Registered mimic joint '%s' id %d\n",
                  joint_info.name.c_str(), joint_id);
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
        }
      }
    }

    auto get_min_value = [this](const hardware_interface::InterfaceInfo &info) {
      if (!info.min.empty())
      {
        try
        {
          return std::stod(info.min);
        }
        catch (std::invalid_argument &e)
        {
          throw std::runtime_error("Invalid min value '" + info.min + "': " + e.what());
        }
      }
      return std::numeric_limits<double>::lowest();
    };

    auto get_max_value = [this](const hardware_interface::InterfaceInfo &info) {
      if (!info.max.empty())
      {
        try
        {
          return std::stod(info.max);
        }
        catch (std::invalid_argument &e)
        {
          throw std::runtime_error("Invalid max value '" + info.max + "': " + e.what());
        }
      }
      return std::numeric_limits<double>::max();
    };

    // * Configuring command interfaces internal variables
    for (const auto &cmd_if : joint_info.command_interfaces)
    {
      RCLCPP_INFO(node_->get_logger(), "Registering Command Interface '%s' for joint '%s'",
                  cmd_if.name.c_str(), joint_info.name.c_str());
      try {
        if (cmd_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
        {
          impl_->command_interfaces_.emplace_back(joint_info.name, cmd_if.name,
                                                  &last_joint.position_cmd);
          last_joint.has_position_cmd = true;
          if (!std::isnan(initial_position))
          {
            last_joint.position_cmd = initial_position;
          }
          last_joint.min_position_cmd = get_min_value(cmd_if);
          last_joint.max_position_cmd = get_max_value(cmd_if);
        }

        if (cmd_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
        {
          impl_->command_interfaces_.emplace_back(joint_info.name, cmd_if.name,
                                                  &last_joint.velocity_cmd);
          last_joint.has_velocity_cmd = true;
          if (!std::isnan(initial_velocity))
          {
            last_joint.velocity_cmd = initial_velocity;
          }
          last_joint.min_velocity_cmd = get_min_value(cmd_if);
          last_joint.max_velocity_cmd = get_max_value(cmd_if);
        }

        if (cmd_if.name.find(hardware_interface::HW_IF_EFFORT) != std::string::npos)
        {
          impl_->command_interfaces_.emplace_back(joint_info.name, cmd_if.name,
                                                  &last_joint.effort_cmd);
          last_joint.has_effort_cmd = true;
          if (!std::isnan(initial_effort))
          {
            last_joint.effort_cmd = initial_effort;
          }
          last_joint.min_effort_cmd = get_min_value(cmd_if);
          last_joint.max_effort_cmd = get_max_value(cmd_if);
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error parsing command interface limits for joint '%s': %s",
                     joint_info.name.c_str(), e.what());
      }

      // Check if PID control is enabled
      if (cmd_if.name.find("_pid") != std::string::npos)
      {
        last_joint.is_pid_enabled = true;
        last_joint.position_pid_enabled |=
            cmd_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos;
        last_joint.velocity_pid_enabled |=
            cmd_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos;
      }
    }

    // Get PID gains if PID is enabled
    if (last_joint.is_pid_enabled)
    {
      last_joint.position_pid = get_pid_gains(joint_info, hardware_interface::HW_IF_POSITION);
      last_joint.velocity_pid = get_pid_gains(joint_info, hardware_interface::HW_IF_VELOCITY);
    }

    // Map MuJoCo actuators to this joint's command interfaces.
    {
      const auto resolve_explicit_actuator =
          [this, m, joint_id, &joint_info](
              const char *parameter, bool required) -> std::optional<int>
      {
        const auto parameter_it = joint_info.parameters.find(parameter);
        if(parameter_it == joint_info.parameters.end() || parameter_it->second.empty())
        {
          if(required)
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "Joint '%s' has no '%s' parameter; using legacy actuator inference",
                joint_info.name.c_str(), parameter);
          }
          return std::nullopt;
        }

        const int actuator_id =
            mj_name2id(m, mjOBJ_ACTUATOR, parameter_it->second.c_str());
        if(actuator_id < 0)
        {
          throw std::invalid_argument(
              "Joint '" + joint_info.name + "' maps '" + parameter + "' to unknown actuator '"
              + parameter_it->second + "'");
        }
        if(m->actuator_trntype[actuator_id] != mjTRN_JOINT
           || m->actuator_trnid[2 * actuator_id] != joint_id)
        {
          throw std::invalid_argument(
              "Actuator '" + parameter_it->second + "' mapped by joint '" + joint_info.name
              + "' does not transmit to that joint");
        }
        return actuator_id;
      };

      const auto position_id =
          resolve_explicit_actuator("position_actuator", last_joint.has_position_cmd);
      const auto velocity_id =
          resolve_explicit_actuator("velocity_actuator", last_joint.has_velocity_cmd);
      const auto effort_id =
          resolve_explicit_actuator("effort_actuator", last_joint.has_effort_cmd);

      if(position_id)
      {
        last_joint.position_actuator_id = static_cast<std::size_t>(*position_id);
      }
      if(velocity_id)
      {
        last_joint.velocity_actuator_id = static_cast<std::size_t>(*velocity_id);
      }
      if(effort_id)
      {
        last_joint.effort_actuator_id = static_cast<std::size_t>(*effort_id);
      }

      for(int idx = 0; idx < static_cast<int>(m->nu); ++idx)
      {
        if(m->actuator_trntype[idx] != mjTRN_JOINT
           || m->actuator_trnid[2 * idx] != joint_id)
        {
          continue;
        }

        const char *actuator_name = mj_id2name(m, mjOBJ_ACTUATOR, idx);
        const std::string name = actuator_name ? actuator_name : "";
        bool looks_position = name.find("position") != std::string::npos;
        bool looks_velocity = name.find("velocity") != std::string::npos;
        if(!looks_velocity)
        {
          looks_position |=
              (m->actuator_dyntype[idx] == mjDYN_NONE
               || m->actuator_dyntype[idx] == mjDYN_FILTEREXACT)
              && m->actuator_gaintype[idx] == mjGAIN_FIXED
              && m->actuator_biastype[idx] == mjBIAS_AFFINE;
        }
        if(!looks_position)
        {
          looks_velocity |=
              m->actuator_dyntype[idx] == mjDYN_NONE
              && m->actuator_gaintype[idx] == mjGAIN_FIXED
              && m->actuator_biastype[idx] == mjBIAS_AFFINE;
        }

        if(last_joint.has_position_cmd && !position_id && looks_position
           && last_joint.position_actuator_id == static_cast<std::size_t>(-1))
        {
          last_joint.position_actuator_id = static_cast<std::size_t>(idx);
          continue;
        }
        if(last_joint.has_velocity_cmd && !velocity_id && looks_velocity
           && last_joint.velocity_actuator_id == static_cast<std::size_t>(-1))
        {
          last_joint.velocity_actuator_id = static_cast<std::size_t>(idx);
          continue;
        }
        if(last_joint.has_effort_cmd && !effort_id && !looks_position && !looks_velocity
           && last_joint.effort_actuator_id == static_cast<std::size_t>(-1))
        {
          last_joint.effort_actuator_id = static_cast<std::size_t>(idx);
        }
      }

      const auto register_mapping =
          [this, &last_joint](
              bool has_command, std::size_t actuator_id, const char *interface,
              double initial_value)
      {
        if(!has_command)
        {
          return;
        }
        if(actuator_id == static_cast<std::size_t>(-1))
        {
          throw std::invalid_argument(
              "Joint '" + last_joint.name + "' has a '" + interface
              + "' command interface but no matching MuJoCo actuator");
        }
        if(!impl_->commanded_actuators_.insert(static_cast<int>(actuator_id)).second)
        {
          const char *actuator_name =
              mj_id2name(impl_->model_, mjOBJ_ACTUATOR, static_cast<int>(actuator_id));
          throw std::invalid_argument(
              "MuJoCo actuator '" + std::string(actuator_name ? actuator_name : "unnamed")
              + "' has more than one command owner");
        }
        impl_->data_->ctrl[actuator_id] = initial_value;
      };

      register_mapping(
          last_joint.has_position_cmd, last_joint.position_actuator_id, "position",
          last_joint.position_pid_enabled ? 0.0 : last_joint.position_cmd);
      register_mapping(
          last_joint.has_velocity_cmd, last_joint.velocity_actuator_id, "velocity",
          last_joint.velocity_pid_enabled ? 0.0 : last_joint.velocity_cmd);
      register_mapping(
          last_joint.has_effort_cmd, last_joint.effort_actuator_id, "effort",
          last_joint.effort_cmd);

      if(last_joint.has_position_cmd)
      {
        last_joint.active_command_mode = Joint::CommandMode::POSITION;
      }
      else if(last_joint.has_velocity_cmd)
      {
        last_joint.active_command_mode = Joint::CommandMode::VELOCITY;
      }
      else if(last_joint.has_effort_cmd)
      {
        last_joint.active_command_mode = Joint::CommandMode::EFFORT;
      }
    }

    RCLCPP_INFO(node_->get_logger(), "Registered joint '%s' id %d\n", joint_info.name.c_str(),
                joint_id);
  }
}

void MujocoSystem::register_sensors(const hardware_interface::HardwareInfo & hardware_info)
{
  struct ProfileEntry
  {
    const char * interface;
    const char * parameter;
    std::size_t index;
    int dimension;
    int sensor_type;
  };

  const std::unordered_map<std::string, std::vector<ProfileEntry>> profiles = {
      {"imu",
       {{"orientation.x", "orientation_sensor", 1, 4, mjSENS_FRAMEQUAT},
        {"orientation.y", "orientation_sensor", 2, 4, mjSENS_FRAMEQUAT},
        {"orientation.z", "orientation_sensor", 3, 4, mjSENS_FRAMEQUAT},
        {"orientation.w", "orientation_sensor", 0, 4, mjSENS_FRAMEQUAT},
        {"angular_velocity.x", "angular_velocity_sensor", 0, 3, mjSENS_GYRO},
        {"angular_velocity.y", "angular_velocity_sensor", 1, 3, mjSENS_GYRO},
        {"angular_velocity.z", "angular_velocity_sensor", 2, 3, mjSENS_GYRO},
        {"linear_acceleration.x", "linear_acceleration_sensor", 0, 3, mjSENS_ACCELEROMETER},
        {"linear_acceleration.y", "linear_acceleration_sensor", 1, 3, mjSENS_ACCELEROMETER},
        {"linear_acceleration.z", "linear_acceleration_sensor", 2, 3, mjSENS_ACCELEROMETER}}},
      {"force_torque",
       {{"force.x", "force_sensor", 0, 3, mjSENS_FORCE},
        {"force.y", "force_sensor", 1, 3, mjSENS_FORCE},
        {"force.z", "force_sensor", 2, 3, mjSENS_FORCE},
        {"torque.x", "torque_sensor", 0, 3, mjSENS_TORQUE},
        {"torque.y", "torque_sensor", 1, 3, mjSENS_TORQUE},
        {"torque.z", "torque_sensor", 2, 3, mjSENS_TORQUE}}},
      {"range", {{"range", "range_sensor", 0, 1, -1}}},
      {"pose",
       {{"position.x", "position_sensor", 0, 3, mjSENS_FRAMEPOS},
        {"position.y", "position_sensor", 1, 3, mjSENS_FRAMEPOS},
        {"position.z", "position_sensor", 2, 3, mjSENS_FRAMEPOS},
        {"orientation.x", "orientation_sensor", 1, 4, mjSENS_FRAMEQUAT},
        {"orientation.y", "orientation_sensor", 2, 4, mjSENS_FRAMEQUAT},
        {"orientation.z", "orientation_sensor", 3, 4, mjSENS_FRAMEQUAT},
        {"orientation.w", "orientation_sensor", 0, 4, mjSENS_FRAMEQUAT}}}};

  for(const auto & component : hardware_info.sensors)
  {
    const std::string profile = required_parameter(component, "profile");
    std::unordered_map<std::string, SensorSource> mappings;

    if(profile == "generic")
    {
      for(const auto & interface : component.state_interfaces)
      {
        const std::string key = "state." + interface.name;
        mappings.emplace(
            interface.name,
            parse_sensor_source(required_parameter(component, key),
                                "Sensor interface '" + component.name + "/" + interface.name + "'"));
      }
    }
    else
    {
      const auto profile_it = profiles.find(profile);
      if(profile_it == profiles.end())
      {
        throw std::invalid_argument(
            "Sensor '" + component.name + "' has unsupported profile '" + profile + "'");
      }

      std::unordered_set<std::string> configured_interfaces;
      for(const auto & interface : component.state_interfaces)
      {
        if(!configured_interfaces.insert(interface.name).second)
        {
          throw std::invalid_argument(
              "Sensor '" + component.name + "' declares duplicate interface '" + interface.name + "'");
        }
      }
      if(configured_interfaces.size() != profile_it->second.size())
      {
        throw std::invalid_argument(
            "Sensor '" + component.name + "' profile '" + profile
            + "' requires exactly " + std::to_string(profile_it->second.size())
            + " state interfaces");
      }

      std::unordered_map<std::string, MujocoRosUtils::MujocoSensorBinding> sources;
      for(const auto & entry : profile_it->second)
      {
        if(configured_interfaces.count(entry.interface) == 0)
        {
          throw std::invalid_argument(
              "Sensor '" + component.name + "' profile '" + profile
              + "' requires state interface '" + entry.interface + "'");
        }
        auto source_it = sources.find(entry.parameter);
        if(source_it == sources.end())
        {
          const std::string sensor_name = required_parameter(component, entry.parameter);
          source_it = sources.emplace(
              entry.parameter,
              MujocoRosUtils::MujocoSensorBinding(impl_->model_, sensor_name)).first;
        }
        if(source_it->second.dimension() != entry.dimension)
        {
          throw std::invalid_argument(
              "MuJoCo sensor '" + source_it->second.name() + "' for '" + component.name
              + "' must have dimension " + std::to_string(entry.dimension));
        }
        if(entry.sensor_type >= 0 && source_it->second.type() != entry.sensor_type)
        {
          throw std::invalid_argument(
              "MuJoCo sensor '" + source_it->second.name() + "' has the wrong type for '"
              + component.name + "/" + entry.interface + "'");
        }
        mappings.emplace(
            entry.interface, SensorSource{source_it->second.name(), entry.index});
      }
    }

    std::unordered_set<std::string> destinations;
    for(const auto & interface : component.state_interfaces)
    {
      validate_double_interface(interface, component.name);
      if(!destinations.insert(interface.name).second)
      {
        throw std::invalid_argument(
            "Sensor '" + component.name + "' declares duplicate interface '" + interface.name + "'");
      }
      const auto mapping_it = mappings.find(interface.name);
      if(mapping_it == mappings.end())
      {
        throw std::invalid_argument(
            "Sensor interface '" + component.name + "/" + interface.name + "' has no mapping");
      }

      ScalarState state;
      state.component = component.name;
      state.interface = interface.name;
      state.sensor = MujocoRosUtils::MujocoSensorBinding(impl_->model_, mapping_it->second.name);
      state.sensor_index = mapping_it->second.index;
      if(state.sensor_index >= static_cast<std::size_t>(state.sensor.dimension()))
      {
        throw std::invalid_argument(
            "Sensor interface '" + component.name + "/" + interface.name
            + "' references element " + std::to_string(state.sensor_index)
            + " of dimension " + std::to_string(state.sensor.dimension()));
      }
      state.scale = parse_optional_number(
          component.parameters, "scale." + interface.name, 1.0, component.name);
      state.offset = parse_optional_number(
          component.parameters, "offset." + interface.name, 0.0, component.name);
      state.initial_value = interface_initial_value(interface, component.name);
      state.value = state.initial_value;
      impl_->scalar_states_.push_back(std::move(state));
      auto & stored = impl_->scalar_states_.back();
      impl_->state_interfaces_.emplace_back(
          stored.component, stored.interface, &stored.value);
    }
  }
}

void MujocoSystem::register_gpios(const hardware_interface::HardwareInfo & hardware_info)
{
  for(const auto & component : hardware_info.gpios)
  {
    std::unordered_set<std::string> state_destinations;
    for(const auto & interface : component.state_interfaces)
    {
      validate_double_interface(interface, component.name);
      if(!state_destinations.insert(interface.name).second)
      {
        throw std::invalid_argument(
            "GPIO '" + component.name + "' declares duplicate state interface '" + interface.name + "'");
      }

      const std::string key = "state." + interface.name;
      const std::string expression = required_parameter(component, key);
      ScalarState state;
      state.component = component.name;
      state.interface = interface.name;
      if(expression.rfind("mujoco_sensor:", 0) == 0)
      {
        const SensorSource source = parse_sensor_source(
            expression, "GPIO state interface '" + component.name + "/" + interface.name + "'");
        state.source = ScalarStateSource::SENSOR;
        state.sensor = MujocoRosUtils::MujocoSensorBinding(impl_->model_, source.name);
        state.sensor_index = source.index;
        if(state.sensor_index >= static_cast<std::size_t>(state.sensor.dimension()))
        {
          throw std::invalid_argument(
              "GPIO state interface '" + component.name + "/" + interface.name
              + "' references element " + std::to_string(state.sensor_index)
              + " of dimension " + std::to_string(state.sensor.dimension()));
        }
      }
      else
      {
        state.source = ScalarStateSource::ACTUATOR_CONTROL;
        state.actuator = MujocoRosUtils::MujocoActuatorBinding(
            impl_->model_,
            parse_actuator_source(
                expression,
                "GPIO state interface '" + component.name + "/" + interface.name + "'"));
      }
      state.scale = parse_optional_number(
          component.parameters, "scale." + interface.name, 1.0, component.name);
      state.offset = parse_optional_number(
          component.parameters, "offset." + interface.name, 0.0, component.name);
      state.initial_value = interface_initial_value(interface, component.name);
      state.value = state.initial_value;
      impl_->scalar_states_.push_back(std::move(state));
      auto & stored = impl_->scalar_states_.back();
      impl_->state_interfaces_.emplace_back(
          stored.component, stored.interface, &stored.value);
    }

    std::unordered_set<std::string> command_destinations;
    for(const auto & interface : component.command_interfaces)
    {
      validate_double_interface(interface, component.name);
      if(!command_destinations.insert(interface.name).second)
      {
        throw std::invalid_argument(
            "GPIO '" + component.name + "' declares duplicate command interface '"
            + interface.name + "'");
      }

      ScalarCommand command;
      command.component = component.name;
      command.interface = interface.name;
      command.actuator = MujocoRosUtils::MujocoActuatorBinding(
          impl_->model_,
          parse_actuator_source(
              required_parameter(component, "command." + interface.name),
              "GPIO command interface '" + component.name + "/" + interface.name + "'"));
      if(!impl_->commanded_actuators_.insert(command.actuator.id()).second)
      {
        throw std::invalid_argument(
            "MuJoCo actuator '" + command.actuator.name()
            + "' has more than one command owner");
      }
      command.minimum = interface.min.empty()
                          ? std::numeric_limits<double>::lowest()
                          : parse_number(
                              interface.min,
                              "minimum for '" + component.name + "/" + interface.name + "'");
      command.maximum = interface.max.empty()
                          ? std::numeric_limits<double>::max()
                          : parse_number(
                              interface.max,
                              "maximum for '" + component.name + "/" + interface.name + "'");
      if(command.minimum > command.maximum)
      {
        throw std::invalid_argument(
            "GPIO command interface '" + component.name + "/" + interface.name
            + "' has minimum greater than maximum");
      }
      command.initial_value = interface_initial_value(interface, component.name);
      const double validated_initial = command.actuator.clamp(
          command.initial_value, command.minimum, command.maximum);
      if(validated_initial != command.initial_value)
      {
        throw std::invalid_argument(
            "Initial value for GPIO command interface '" + component.name + "/"
            + interface.name + "' is outside its command range");
      }
      command.value = command.initial_value;
      impl_->scalar_commands_.push_back(std::move(command));
      auto & stored = impl_->scalar_commands_.back();
      impl_->command_interfaces_.emplace_back(
          stored.component, stored.interface, &stored.value);
    }
  }
}

CallbackReturn MujocoSystem::on_activate(const State &previous_state)
{
  impl_->lifecycle_state_known_ = true;
  impl_->active_ = true;
  RCLCPP_INFO(node_->get_logger(), "Activating 'MujocoSystem' from previous state: %s\n",
              previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_deactivate(const State &previous_state)
{
  impl_->lifecycle_state_known_ = true;
  impl_->active_ = false;
  RCLCPP_INFO(node_->get_logger(), "Deactivating 'MujocoSystem' from previous state: %s\n",
              previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

#if MUJOCO_ROS_UTILS_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS
CallbackReturn MujocoSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
#else
CallbackReturn MujocoSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if(hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
#endif

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

namespace
{

std::optional<Joint::CommandMode> command_mode_for_interface(const std::string &interface)
{
  if(interface == hardware_interface::HW_IF_POSITION || interface == "position_pid")
  {
    return Joint::CommandMode::POSITION;
  }
  if(interface == hardware_interface::HW_IF_VELOCITY || interface == "velocity_pid")
  {
    return Joint::CommandMode::VELOCITY;
  }
  if(interface == hardware_interface::HW_IF_EFFORT)
  {
    return Joint::CommandMode::EFFORT;
  }
  return std::nullopt;
}

bool joint_supports_mode(const Joint &joint, Joint::CommandMode mode)
{
  switch(mode)
  {
    case Joint::CommandMode::POSITION:
      return joint.has_position_cmd;
    case Joint::CommandMode::VELOCITY:
      return joint.has_velocity_cmd;
    case Joint::CommandMode::EFFORT:
      return joint.has_effort_cmd;
    case Joint::CommandMode::NONE:
      return true;
  }
  return false;
}

} // namespace

return_type MujocoSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
{
  std::vector<Joint::CommandMode> proposed_modes;
  proposed_modes.reserve(impl_->joints_.size());
  for(const auto &joint : impl_->joints_)
  {
    proposed_modes.push_back(joint.active_command_mode);
  }

  const auto apply_interfaces =
      [this, &proposed_modes](
          const std::vector<std::string> &interfaces, bool starting) -> bool
  {
    for(const auto &key : interfaces)
    {
      const std::size_t separator = key.rfind('/');
      if(separator == std::string::npos)
      {
        continue;
      }
      const std::string joint_name = key.substr(0, separator);
      const auto joint_it = std::find_if(
          impl_->joints_.begin(), impl_->joints_.end(),
          [&joint_name](const Joint &joint) { return joint.name == joint_name; });
      if(joint_it == impl_->joints_.end())
      {
        continue;
      }

      const auto requested_mode = command_mode_for_interface(key.substr(separator + 1));
      if(!requested_mode || !joint_supports_mode(*joint_it, *requested_mode))
      {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported command interface '%s'", key.c_str());
        return false;
      }

      const std::size_t joint_index =
          static_cast<std::size_t>(std::distance(impl_->joints_.begin(), joint_it));
      auto &proposed_mode = proposed_modes[joint_index];
      if(starting)
      {
        if(proposed_mode != Joint::CommandMode::NONE && proposed_mode != *requested_mode)
        {
          RCLCPP_ERROR(
              node_->get_logger(), "Joint '%s' cannot activate multiple command modes",
              joint_name.c_str());
          return false;
        }
        proposed_mode = *requested_mode;
      }
      else if(proposed_mode == *requested_mode)
      {
        proposed_mode = Joint::CommandMode::NONE;
      }
    }
    return true;
  };

  if(!apply_interfaces(stop_interfaces, false) || !apply_interfaces(start_interfaces, true))
  {
    impl_->pending_command_modes_.reset();
    return return_type::ERROR;
  }

  impl_->pending_command_modes_ = std::move(proposed_modes);
  return return_type::OK;
}

return_type MujocoSystem::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
{
  if(!impl_->pending_command_modes_
     && prepare_command_mode_switch(start_interfaces, stop_interfaces) != return_type::OK)
  {
    return return_type::ERROR;
  }

  for(std::size_t index = 0; index < impl_->joints_.size(); ++index)
  {
    auto &joint = impl_->joints_[index];
    const auto next_mode = (*impl_->pending_command_modes_)[index];
    if(next_mode == joint.active_command_mode)
    {
      continue;
    }
    joint.active_command_mode = next_mode;
    switch(next_mode)
    {
      case Joint::CommandMode::POSITION:
        joint.position_cmd = joint.position;
        break;
      case Joint::CommandMode::VELOCITY:
        joint.velocity_cmd = joint.velocity;
        break;
      case Joint::CommandMode::EFFORT:
        joint.effort_cmd = 0.0;
        break;
      case Joint::CommandMode::NONE:
        break;
    }
    joint.position_pid.reset();
    joint.velocity_pid.reset();
  }
  impl_->pending_command_modes_.reset();
  return return_type::OK;
}

return_type MujocoSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  const auto started = impl_->diagnostics_enabled_
                         ? std::chrono::steady_clock::now()
                         : std::chrono::steady_clock::time_point{};
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
    joint.effort              = impl_->data_->qfrc_actuator[joint_data_id_in_qvel];
  }
  for(auto & state : impl_->scalar_states_)
  {
    state.update(impl_->data_);
  }
  ++impl_->read_count_;
  if(impl_->diagnostics_enabled_)
  {
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started);
    const auto duration_ns = static_cast<std::uint64_t>(duration.count());
    impl_->last_read_duration_ns_ = duration_ns;
    auto maximum = impl_->max_read_duration_ns_.load();
    while(maximum < duration_ns
          && !impl_->max_read_duration_ns_.compare_exchange_weak(maximum, duration_ns))
    {
    }
    if(period.nanoseconds() > 0
       && duration_ns > static_cast<std::uint64_t>(period.nanoseconds()))
    {
      ++impl_->missed_period_count_;
    }
  }
  return return_type::OK;
}

return_type MujocoSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  const auto started = impl_->diagnostics_enabled_
                         ? std::chrono::steady_clock::now()
                         : std::chrono::steady_clock::time_point{};
  const auto finish_diagnostics = [this, &period, started]()
  {
    ++impl_->write_count_;
    if(!impl_->diagnostics_enabled_)
    {
      return;
    }
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - started);
    const auto duration_ns = static_cast<std::uint64_t>(duration.count());
    impl_->last_write_duration_ns_ = duration_ns;
    auto maximum = impl_->max_write_duration_ns_.load();
    while(maximum < duration_ns
          && !impl_->max_write_duration_ns_.compare_exchange_weak(maximum, duration_ns))
    {
    }
    if(period.nanoseconds() > 0
       && duration_ns > static_cast<std::uint64_t>(period.nanoseconds()))
    {
      ++impl_->missed_period_count_;
    }
  };

  // * If sim is reset
  if (period.seconds() <= 0)
  {
    reset();
    finish_diagnostics();
    return return_type::OK;
  }

  if(impl_->lifecycle_state_known_ && !impl_->active_)
  {
    finish_diagnostics();
    return return_type::OK;
  }

  const auto valid_command = [this](double value, const std::string & name)
  {
    if(std::isfinite(value))
    {
      return true;
    }
    ++impl_->invalid_command_count_;
    RCLCPP_ERROR(node_->get_logger(), "Command '%s' is not finite", name.c_str());
    return false;
  };
  for(const auto & command : impl_->scalar_commands_)
  {
    if(!valid_command(command.value, command.component + "/" + command.interface))
    {
      finish_diagnostics();
      return return_type::ERROR;
    }
  }
  for(const auto & joint : impl_->joints_)
  {
    const bool valid =
        joint.active_command_mode == Joint::CommandMode::NONE
        || (joint.active_command_mode == Joint::CommandMode::POSITION
            && valid_command(joint.position_cmd, joint.name + "/position"))
        || (joint.active_command_mode == Joint::CommandMode::VELOCITY
            && valid_command(joint.velocity_cmd, joint.name + "/velocity"))
        || (joint.active_command_mode == Joint::CommandMode::EFFORT
            && valid_command(joint.effort_cmd, joint.name + "/effort"));
    if(!valid)
    {
      finish_diagnostics();
      return return_type::ERROR;
    }
  }

  // * Update mimic joints command
  for (unsigned int i = 0; i < impl_->joints_.size(); ++i)
  {
    auto &joint = impl_->joints_[i];
    if (joint.is_mimic)
    {
      joint.position_cmd =
          joint.multiplier * impl_->joints_[joint.mimicked_joint_index].position_cmd;
      joint.velocity_cmd =
          joint.multiplier * impl_->joints_[joint.mimicked_joint_index].velocity_cmd;
      joint.effort_cmd =
          joint.multiplier * impl_->joints_[joint.mimicked_joint_index].effort_cmd;
    }
  }

  try
  {
    for(const auto & command : impl_->scalar_commands_)
    {
      command.actuator.write(
          impl_->data_, command.value, command.minimum, command.maximum);
    }
  }
  catch(const std::exception & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to write GPIO command: %s", e.what());
    ++impl_->invalid_command_count_;
    finish_diagnostics();
    return return_type::ERROR;
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
      continue; // logic of mimic joints is already in the MimicJoint plugin
    }
    else
    {
      int joint_data_id_in_qpos = impl_->model_->jnt_qposadr[joint.id];
      int joint_data_id_in_qvel = impl_->model_->jnt_dofadr[joint.id];

      if (joint.active_command_mode == Joint::CommandMode::POSITION
          && joint.has_position_cmd
          && joint.position_actuator_id != static_cast<std::size_t>(-1))
      {
        ss << "Joint '" << joint.name.c_str() << "' pos_cmd: " << joint.position_cmd << std::endl;
        
        if (joint.position_pid_enabled)
        {
          // Use PID control
          double error = joint.position_cmd - impl_->data_->qpos[joint_data_id_in_qpos];
          double effort = joint.position_pid.computeCommand(error, period.nanoseconds());
          
          // Apply effort limits
          double min_eff = joint.joint_limits.has_effort_limits ? -joint.joint_limits.max_effort : std::numeric_limits<double>::lowest();
          min_eff = std::max(min_eff, joint.min_effort_cmd);
          double max_eff = joint.joint_limits.has_effort_limits ? joint.joint_limits.max_effort : std::numeric_limits<double>::max();
          max_eff = std::min(max_eff, joint.max_effort_cmd);
          
          impl_->data_->ctrl[joint.position_actuator_id] = clamp(effort, min_eff, max_eff);
        }
        else
        {
          // Direct position control
          double minimum = joint.min_position_cmd;
          double maximum = joint.max_position_cmd;
          if(joint.joint_limits.has_position_limits)
          {
            minimum = std::max(minimum, joint.joint_limits.min_position);
            maximum = std::min(maximum, joint.joint_limits.max_position);
          }
          impl_->data_->ctrl[joint.position_actuator_id] =
              clamp(joint.position_cmd, minimum, maximum);
        }
      }
      if (joint.active_command_mode == Joint::CommandMode::VELOCITY
          && joint.has_velocity_cmd
          && joint.velocity_actuator_id != static_cast<std::size_t>(-1))
      {
        if (joint.velocity_pid_enabled)
        {
          // Use PID control
          double error = joint.velocity_cmd - impl_->data_->qvel[joint_data_id_in_qvel];
          double effort = joint.velocity_pid.computeCommand(error, period.nanoseconds());
          
          // Apply effort limits
          double min_eff = joint.joint_limits.has_effort_limits ? -joint.joint_limits.max_effort : std::numeric_limits<double>::lowest();
          min_eff = std::max(min_eff, joint.min_effort_cmd);
          double max_eff = joint.joint_limits.has_effort_limits ? joint.joint_limits.max_effort : std::numeric_limits<double>::max();
          max_eff = std::min(max_eff, joint.max_effort_cmd);
          
          impl_->data_->ctrl[joint.velocity_actuator_id] = clamp(effort, min_eff, max_eff);
        }
        else
        {
          // Direct velocity control
          double minimum = joint.min_velocity_cmd;
          double maximum = joint.max_velocity_cmd;
          if(joint.joint_limits.has_velocity_limits)
          {
            minimum = std::max(minimum, -joint.joint_limits.max_velocity);
            maximum = std::min(maximum, joint.joint_limits.max_velocity);
          }
          impl_->data_->ctrl[joint.velocity_actuator_id] =
              clamp(joint.velocity_cmd, minimum, maximum);
        }
      }
      if (joint.active_command_mode == Joint::CommandMode::EFFORT
          && joint.has_effort_cmd
          && joint.effort_actuator_id != static_cast<std::size_t>(-1))
      {
        // Apply effort limits
        double min_eff = joint.joint_limits.has_effort_limits ? -joint.joint_limits.max_effort : std::numeric_limits<double>::lowest();
        min_eff = std::max(min_eff, joint.min_effort_cmd);
        double max_eff = joint.joint_limits.has_effort_limits ? joint.joint_limits.max_effort : std::numeric_limits<double>::max();
        max_eff = std::min(max_eff, joint.max_effort_cmd);
        
        impl_->data_->ctrl[joint.effort_actuator_id] = clamp(joint.effort_cmd, min_eff, max_eff);
      }
    }
  }

  if (impl_->joint_cmd_publisher_)
  {
    joint_state_msg.header.stamp = time;
    impl_->joint_cmd_publisher_->publish(joint_state_msg);
  }
  finish_diagnostics();
  return return_type::OK;
}

MujocoSystemDiagnostics MujocoSystem::diagnostics() const
{
  MujocoSystemDiagnostics result;
  if(!impl_)
  {
    return result;
  }
  result.enabled = impl_->diagnostics_enabled_;
  result.read_count = impl_->read_count_.load();
  result.write_count = impl_->write_count_.load();
  result.reset_count = impl_->reset_count_.load();
  result.invalid_command_count = impl_->invalid_command_count_.load();
  result.missed_period_count = impl_->missed_period_count_.load();
  result.last_read_duration_ns = impl_->last_read_duration_ns_.load();
  result.max_read_duration_ns = impl_->max_read_duration_ns_.load();
  result.last_write_duration_ns = impl_->last_write_duration_ns_.load();
  result.max_write_duration_ns = impl_->max_write_duration_ns_.load();
  return result;
}

void MujocoSystem::get_joint_limits(urdf::JointConstSharedPtr urdf_joint, 
                                     joint_limits::JointLimits& joint_limits)
{
  if (urdf_joint && urdf_joint->limits)
  {
    joint_limits.has_position_limits = true;
    joint_limits.min_position = urdf_joint->limits->lower;
    joint_limits.max_position = urdf_joint->limits->upper;
    joint_limits.has_velocity_limits = true;
    joint_limits.max_velocity = urdf_joint->limits->velocity;
    joint_limits.has_effort_limits = true;
    joint_limits.max_effort = urdf_joint->limits->effort;
  }
}

control_toolbox::Pid MujocoSystem::get_pid_gains(const hardware_interface::ComponentInfo& joint_info, 
                                                   std::string command_interface)
{
  double kp = 0.0, ki = 0.0, kd = 0.0;
  
  // Get joint limits to use as default i_max/i_min
  auto joint_it = std::find_if(impl_->joints_.begin(), impl_->joints_.end(),
                                [&joint_info](const Joint& j) { return j.name == joint_info.name; });
  
  double i_max = std::numeric_limits<double>::max();
  double i_min = std::numeric_limits<double>::lowest();
  
  if (joint_it != impl_->joints_.end() && joint_it->joint_limits.has_effort_limits)
  {
    i_max = joint_it->joint_limits.max_effort;
    i_min = -joint_it->joint_limits.max_effort;
  }
  
  std::string key;
  key = command_interface + std::string(PARAM_KP);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    try
    {
      kp = std::stod(joint_info.parameters.at(key));
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse %s for joint %s: %s", 
                  key.c_str(), joint_info.name.c_str(), e.what());
    }
  }

  key = command_interface + std::string(PARAM_KI);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    try
    {
      ki = std::stod(joint_info.parameters.at(key));
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse %s for joint %s: %s", 
                  key.c_str(), joint_info.name.c_str(), e.what());
    }
  }

  key = command_interface + std::string(PARAM_KD);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    try
    {
      kd = std::stod(joint_info.parameters.at(key));
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse %s for joint %s: %s", 
                  key.c_str(), joint_info.name.c_str(), e.what());
    }
  }

  bool enable_anti_windup = false;
  key = command_interface + std::string(PARAM_I_MAX);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    try
    {
      i_max = std::stod(joint_info.parameters.at(key));
      enable_anti_windup = true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse %s for joint %s: %s", 
                  key.c_str(), joint_info.name.c_str(), e.what());
    }
  }

  key = command_interface + std::string(PARAM_I_MIN);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    try
    {
      i_min = std::stod(joint_info.parameters.at(key));
      enable_anti_windup = true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse %s for joint %s: %s", 
                  key.c_str(), joint_info.name.c_str(), e.what());
    }
  }

  return control_toolbox::Pid(kp, ki, kd, i_max, i_min, enable_anti_windup);
}

} // namespace mujoco_ros2_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem,
                       mujoco_ros2_control::MujocoSystemInterface)
