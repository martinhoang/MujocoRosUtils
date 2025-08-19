#include "ActuatorCommand.h"
#include "mujoco_utils.hpp"

#include <mujoco/mujoco.h>
#include <iostream>

namespace MujocoRosUtils
{

constexpr char ATTR_NODE_NAME[] = "node_name";
constexpr char ATTR_TOPIC_NAME[] = "topic_name";
constexpr char ATTR_PUBLISH_RATE[] = "publish_rate";

void ActuatorCommand::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  print_info("Registering ActuatorCommand plugin\n");

  plugin.name = "MujocoRosUtils::ActuatorCommand";
  // Allow plugins to be placed on either the body element or the actuator element
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  std::vector<const char *> attributes = {ATTR_NODE_NAME, ATTR_TOPIC_NAME, ATTR_PUBLISH_RATE};

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  plugin.needstage = 0;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    int nplugin = m->nplugin;
    auto plugin_instance = ActuatorCommand::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    print_debug("[ActuatorCommand.init] plugin created with id: %d / Total: %d plugins\n", plugin_id, nplugin);
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance.release());
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<ActuatorCommand *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class ActuatorCommand *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class ActuatorCommand *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);

  print_confirm("Successfully registered 'ActuatorCommand' plugin\n");
}

std::unique_ptr<ActuatorCommand> ActuatorCommand::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // topic_name
  const char * topic_name_str = mj_getPluginConfig(m, plugin_id, ATTR_TOPIC_NAME);
  std::string topic_name = topic_name_str ? std::string(topic_name_str) : "";

  std::vector<int> actuator_ids;

  ///@note: 1 plugin can share multiple actuators, so get all actuators that are associated with this plugin_id
  for(int i = 0; i < m->nu; ++i)
  {
    if(m->actuator_plugin[i] == plugin_id)
    {
      actuator_ids.push_back(i);
    }
  }

  if(actuator_ids.empty())
  {
    mju_error("[ActuatorCommand] No plugin definition in <actuator> found for plugin ID %d. Check your plugin "
              "definition again.\n",
              plugin_id);
  }

  print_debug("Number of actuators in the model: %d\n", m->nu);

  std::vector<int> active_actuator_ids;
  std::vector<const char *> joint_names;

  //* For each actuator IDs associated with this plugin_id,
  //* find their corresponding joint and check if they are actively controlled
  //* by an existing position PID controller. If so, add the position PID actuator
  //* to the active_actuator_ids vector.
  for(const auto & actuator_id : actuator_ids)
  {
    print_debug("Creating 'ActuatorCommand' actuator id %d with plugin id %d\n", actuator_id, plugin_id);

    // Get the actuator name for debugging purposes
    const char * actuator_name_char = mj_id2name(m, mjOBJ_ACTUATOR, actuator_id);
    if(actuator_name_char && strlen(actuator_name_char) > 0)
    {
      print_debug("Actuator %d: %s\n", actuator_id, actuator_name_char);
    }
    else
    {
      print_warning("Actuator %d does not have a name, skipping.\n", actuator_id);
    }

    //* Get the joint name associated with this actuator
    ///@note: transmission id, trnid is nu x 2, i.e. two times number of actuator, so need to indexing with step of 2
    int joint_id = m->actuator_trnid[2 * actuator_id];
    // Get the joint name
    const char * joint_name_at_actuator = mj_id2name(m, mjOBJ_JOINT, joint_id);
    if(joint_name_at_actuator && strlen(joint_name_at_actuator) > 0)
    {
      print_debug("Joint name '%s' is associated with this actuator %d.\n", joint_name_at_actuator, actuator_id);
    }
    else
    {
      print_warning("Actuator id %d does not have a valid joint name, skipping.\n", actuator_id);
      // skip this actuator
      continue;
    }

    //* Get the "position" PID actuator corresponding with this actuator-plugin-joint pair
    // since I dont know how to make a PID controller yet
    // I latched on to the logics of the existing built-in position PID controller
    // Trick: check if there is a "position" actuator for that joint already
    // by checking if gain_type is mjGAIN_FIXED and bias_type is mjBIAS_AFFINE
    bool is_found = false;
    for(int idx = 0; idx < m->nu; ++idx)
    {
      // If the actuator is associated with this joint
      if(m->actuator_trnid[2 * idx] == joint_id)
      {
        print_info("Actuator id %d is associated with joint '%s'\n", idx, joint_name_at_actuator);
        int gain_type = m->actuator_gaintype[idx];
        int bias_type = m->actuator_biastype[idx];

        if(gain_type == mjGAIN_FIXED && bias_type == mjBIAS_AFFINE)
        {
          active_actuator_ids.push_back(idx);
          is_found = true;
          break; // No need to check further, we found the actuator
        }
      }
    }

    if(!is_found)
    {
      print_warning("No actively-controlled actuator found for joint '%s' with id %d, skipping. Did you forget to add "
                    "a position PID controller for this joint, e.g. <actuator><position></position></actuator>?\n",
                    joint_name_at_actuator, joint_id);
      continue; // Skip to the next actuator
    }

    print_info("Found active actuator id %d for joint '%s' with plugin id %d\n", active_actuator_ids.back(),
               joint_name_at_actuator, plugin_id);
  }

  if(active_actuator_ids.empty())
  {
    mju_error("[ActuatorCommand] No actuator/joint found for this plugin id %d. Check your plugin definition again.\n",
              plugin_id);
  }

  const char * publish_rate_char = mj_getPluginConfig(m, 0, ATTR_PUBLISH_RATE);
  double publish_rate = 100.0;
  if(publish_rate_char && strlen(publish_rate_char) > 0)
  {
    publish_rate = std::stod(publish_rate_char);
  }
  else
  {
    print_warning("No publish_rate specified in plugin config, using default value of %d.\n", publish_rate);
  }

  return std::unique_ptr<ActuatorCommand>(
      new ActuatorCommand(m, d, std::move(active_actuator_ids), topic_name, publish_rate));
}

ActuatorCommand::ActuatorCommand(const mjModel * m,
                                 mjData * d,
                                 std::vector<int> actuator_ids,
                                 std::string topic_name,
                                 double publish_rate)
: model_(m), data_(d), publish_rate_(publish_rate)
{
  actuators_ = std::move(actuator_ids);

  const char * node_name_char = mj_getPluginConfig(m, 0, ATTR_NODE_NAME);
  std::string node_name = node_name_char ? std::string(node_name_char) : "";

  if(node_name.empty())
  {
    node_name = "actuator_command_plugin";
  }

  if(!nh_)
  {
    print_debug("First time creating ActuatorCommand plugin, initializing ROS node.\n");

    int argc = 0;
    char ** argv = nullptr;

    if(!rclcpp::ok())
    { 
      rclcpp::init(argc, argv);
    }

    rclcpp::NodeOptions node_options;

    nh_ = rclcpp::Node::make_shared(node_name, node_options);

    if(topic_name.empty())
    {
      topic_name = node_name + "/command";
    }
    joint_cmd_array_sub_ = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_name + "_array", 1,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        { jointCommandCallback(active_joint_names_, msg->data); });

    // Subscriber for JointTrajectory messages
    joint_trajectory_sub_ = nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        topic_name + "_trajectory", 1,
        [&](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
        {
          if(msg->points.empty())
          {
            print_warning("[ActuatorCommand] Received empty JointTrajectory message, ignoring.\n");
            return;
          }
          jointCommandCallback(msg->joint_names, msg->points[0].positions);
        });

    joint_cmd_joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
        topic_name, 1,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) { jointCommandCallback(msg->name, msg->position); });

    // Publisher for JointState messages
    joint_state_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(node_name + "/joint_states", 1);

    // Use a dedicated queue so as not to call callbacks of other modules
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(nh_);
  }
  else
  {
    print_debug("Reusing existing ActuatorCommand plugin ROS node.\n");
  }

  int number_of_actuators = static_cast<int>(actuators_.size());
  ctrl_.resize(number_of_actuators, std::numeric_limits<mjtNum>::quiet_NaN());
  print_debug("Number of actuators: %d\n", number_of_actuators);
  for(const auto & actuator_id : actuators_)
  {
    const char * joint_name_char = mj_id2name(m, mjOBJ_JOINT, m->actuator_trnid[2 * actuator_id]);
    std::string joint_name = joint_name_char ? std::string(joint_name_char) : "UNKNOWN";
    print_debug("- Actuator ID: %d\tJoint:%s\n", actuator_id, joint_name.c_str());
    active_joint_names_.push_back(joint_name);
  }
}

ActuatorCommand::~ActuatorCommand()
{
  if(nh_)
  {
    print_confirm("Shutting down ActuatorCommand plugin ROS node...\n");
    rclcpp::shutdown();
    joint_cmd_array_sub_.reset();
    joint_trajectory_sub_.reset();
    joint_cmd_joint_state_sub_.reset();
    joint_state_pub_.reset();
    nh_.reset();
    executor_.reset();
    actuators_.clear();
    ctrl_.clear();
  }
  else
  {
    print_confirm("ActuatorCommand plugin ROS node was not initialized, nothing to shut down.\n");
  }
}

void ActuatorCommand::reset(const mjModel *, // m
                            int // plugin_id
)
{
}

void ActuatorCommand::compute(const mjModel *, // m
                              mjData * d,
                              int // plugin_id
)
{
  if(!rclcpp::ok())
  {
    mju_error("[ActuatorCommand] rclcpp is not ok, cannot compute actuator command.");
  }

  // print_debug("[ActuatorCommand.compute] Executing compute for plugin ID: %d\n", plugin_id);

  // Call ROS callback
  executor_->spin_once(std::chrono::seconds(0));

  // Set actuator command
  for(size_t i = 0; i < actuators_.size(); ++i)
  {
    if(!std::isnan(ctrl_[i]))
    {
      d->ctrl[actuators_[i]] = ctrl_[i];
    }
  }

  if(joint_state_pub_)
  {
    auto time_now = nh_->get_clock()->now();

    // TODO: the publish rate is rate-limited by the render loop rate as this function is called there.
    if((time_now - last_joint_state_publish_time_).seconds() >= (1.0 / publish_rate_))
    {
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = time_now;
      joint_state_msg.name = active_joint_names_;
      joint_state_msg.position.resize(active_joint_names_.size());

      for(size_t i = 0; i < actuators_.size(); ++i)
      {
        int joint_id = model_->actuator_trnid[2 * actuators_[i]];
        int qpos_id = model_->jnt_qposadr[joint_id];
        joint_state_msg.position[i] = std::isnan(data_->qpos[qpos_id]) ? 0.0 : data_->qpos[qpos_id];
      }

      joint_state_pub_->publish(joint_state_msg);
      last_joint_state_publish_time_ = time_now;
    }
  }
}

void ActuatorCommand::jointCommandCallback(std::vector<std::string> & names, std::vector<double> & positions)
{
  if(names.empty() || positions.empty())
  {
    print_warning("[ActuatorCommand] Received empty JointCommand message, ignoring.\n");
    return;
  }

  if(names.size() != positions.size())
  {
    print_warning("[ActuatorCommand] Joint names and positions size mismatch: %zu vs %zu, ignoring.\n", names.size(),
                  positions.size());
    return;
  }

  // Update control based on the first point in the trajectory
  bool is_valid = true;
  std::vector<mjtNum> new_ctrl(ctrl_.size(), std::numeric_limits<mjtNum>::quiet_NaN());

  for(size_t i = 0; i < names.size(); ++i)
  {
    const auto & joint_name = names[i];
    auto it = std::find(active_joint_names_.begin(), active_joint_names_.end(), joint_name);
    if(it != active_joint_names_.end())
    {
      size_t index = std::distance(active_joint_names_.begin(), it);
      new_ctrl[index] = positions[i];
    }
    else
    {
      is_valid = false;
      std::stringstream ss;
      for(const auto & active_joint : active_joint_names_)
      {
        ss << "- " << active_joint << "\n";
      }
      print_warning(
          "[ActuatorCommand] Joint '%s' not found in active joints, skipping.\nOnly these joints are allowed:\n%s",
          joint_name.c_str(), ss.str().c_str());
      break;
    }
  }

  if(is_valid)
  {
    print_debug("[ActuatorCommand] Received JointTrajectory command\n");
    ctrl_ = std::move(new_ctrl);
  }
}

} // namespace MujocoRosUtils
