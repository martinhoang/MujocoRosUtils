#include "ActuatorCommand.h"

#include <mujoco/mujoco.h>

#include "mujoco_utils.hpp"

#include <iostream>

namespace MujocoRosUtils
{

constexpr char ATTR_JOINT[] = "joint";

void ActuatorCommand::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  print_info("Registering ActuatorCommand plugin\n");

  plugin.name = "MujocoRosUtils::ActuatorCommand";
  // Allow plugins to be placed on either the body element or the actuator element
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  std::vector<const char *> attributes = {ATTR_JOINT};

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    int nplugin = m->nplugin;
    // print_info("[ActuatorCommand.init] creating plugin instance with id: %d / %d\n", plugin_id, nplugin);
    auto plugin_instance = ActuatorCommand::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    print_info("[ActuatorCommand.init] plugin created with id: %d / Total: %d plugins\n", plugin_id, nplugin);
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

  print_info("ActuatorCommand plugin registered\n");
}

std::unique_ptr<ActuatorCommand> ActuatorCommand::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // topic_name
  const char * topic_name_str = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = topic_name_str ? std::string(topic_name_str) : "";

  int actuator_id = -1;

  for(int i = 0; i < m->nu; ++i)
  {
    if(m->actuator_plugin[i] == plugin_id)
    {
      actuator_id = i;
      break;
    }
  }

  print_debug("Creating 'ActuatorCommand' actuator id %d with plugin id %d\n", actuator_id, plugin_id);

  print_debug("Number of actuators in the model: %d\n", m->nu);

  const char * actuator_name_char = mj_id2name(m, mjOBJ_ACTUATOR, actuator_id);
  if(actuator_name_char && strlen(actuator_name_char) > 0)
  {
    print_debug("Actuator %d: %s\n", actuator_id, actuator_name_char);
  }
  else
  {
    print_warning("Actuator %d does not have a name, skipping.\n", actuator_id);
  }

  // transmission id, trnid is nu x 2, i.e. two times number of actuator, so need to indexing with step of 2
  int joint_id = m->actuator_trnid[2 * actuator_id];
  const char * joint_name_at_actuator = mj_id2name(m, mjOBJ_JOINT, joint_id);
  if(joint_name_at_actuator && strlen(joint_name_at_actuator) > 0)
  {
    print_debug("Joint name '%s' is associated with this actuator %d.\n", joint_name_at_actuator, actuator_id);
  }
  else
  {
    print_warning("Actuator id %d does not have a valid joint name, skipping.\n", actuator_id);
  }

  // Trick: check if there is a "position" actuator for that joint already
  // by checking if gain_type is mjGAIN_FIXED and bias_type is mjBIAS_AFFINE
  int active_actuator_id = -1;
  for(int idx = 0; idx < m->nu; ++idx)
  {
    if(m->actuator_trnid[2 * idx] == joint_id)
    {
      print_info("Actuator id %d is associated with joint '%s'\n", idx, joint_name_at_actuator);
      int gain_type = m->actuator_gaintype[idx];
      int bias_type = m->actuator_biastype[idx];

      if(gain_type == mjGAIN_FIXED && bias_type == mjBIAS_AFFINE)
      {
        active_actuator_id = idx;
      }
    }
  }
  if(active_actuator_id < 0)
  {
    mju_error("[ActuatorCommand] No active actuator found for joint '%s' with id %d", joint_name_at_actuator, joint_id);
  }
  else
  {
    print_confirm("Found active actuator id %d associated with 'ActuatorCommand' plugin id %d and joint '%s'\n",
               active_actuator_id, plugin_id, joint_name_at_actuator);
  }

  return std::unique_ptr<ActuatorCommand>(new ActuatorCommand(m, d, active_actuator_id, topic_name));
}

ActuatorCommand::ActuatorCommand(const mjModel * m,
                                 mjData *, // d
                                 int actuator_id,
                                 std::string topic_name)
: actuator_id_(actuator_id)
{
  actuators_.push_back(actuator_id);

  const char * node_name_char = mj_getPluginConfig(m, 0, "node_name");
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
    sub_ = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_name, 1, std::bind(&ActuatorCommand::callback, this, std::placeholders::_1));
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
  print_debug("Number of controls: %zu\n", ctrl_.size());
}

ActuatorCommand::~ActuatorCommand()
{
  if(nh_)
  {
    print_confirm("Shutting down ActuatorCommand plugin ROS node...\n");
    rclcpp::shutdown();
    sub_.reset();
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
                              int plugin_id)
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
}

void ActuatorCommand::callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if(msg->data.size() == ctrl_.size())
  {
    print_debug("Received actuator command with size %zu for %zu num of actuators\n", msg->data.size(),
                actuators_.size());
    ctrl_ = msg->data;
  }
  else
  {
    mju_warning("[ActuatorCommand] Received command size (%zu) does not match actuator count (%zu).", msg->data.size(),
              ctrl_.size());
  }
}

} // namespace MujocoRosUtils
