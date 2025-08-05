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
    print_info("[ActuatorCommand.init] plugin created with id: %d / Total: %d\n", plugin_id, nplugin);
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
  // actuator_name
  const char * joint_name_char = mj_getPluginConfig(m, plugin_id, "joint");

  if(!joint_name_char)
  {
    mju_error("[ActuatorCommand] plugin id %d: `joint` is missing.", plugin_id);
    return nullptr;
  }

  if(strlen(joint_name_char) == 0)
  {
    mju_error("[ActuatorCommand] plugin id %d: `joint` is missing.", plugin_id);
    return nullptr;
  }

  int joint_id = 0;
  for(; joint_id < m->njnt; joint_id++)
  {
    const char * name = mj_id2name(m, mjOBJ_JOINT, joint_id);
    if(name && strcmp(joint_name_char, name) == 0)
    {
      break;
    }
  }

  if(joint_id == m->njnt)
  {
    mju_error("[ActuatorCommand] Joint '%s' not found.", joint_name_char);
    return nullptr;
  }

  // Check to see if there exists an (position) actuator of a joint whose name matches that of the joint definition
  int actuator_id = 0;
  const char * actuator_name_char{NULL};

  for(; actuator_id < m->nu; actuator_id++)
  {
    actuator_name_char = mj_id2name(m, mjOBJ_ACTUATOR, actuator_id);
    if(actuator_name_char && strcmp(joint_name_char, actuator_name_char) == 0)
    {
      break;
    }
  }

  if(actuator_id == m->nu)
  {
    mju_error("[ActuatorCommand] Actuator for joint '%s' not found.", joint_name_char);
    return nullptr;
  }

  // topic_name
  std::string topic_name = mj_getPluginConfig(m, plugin_id, "topic_name");
  if(topic_name.empty())
  {
    topic_name = std::string(actuator_name_char) + "/command";
  }

  return std::unique_ptr<ActuatorCommand>(new ActuatorCommand(m, d, actuator_id, topic_name));
}

ActuatorCommand::ActuatorCommand(const mjModel * m,
                                 mjData *, // d
                                 int actuator_id,
                                 std::string topic_name)
: actuator_id_(actuator_id)
{
  const char * name = mj_id2name(m, mjOBJ_ACTUATOR, actuator_id);
  std::string actuator_name = name ? std::string(name) : "";
  if(topic_name.empty())
  {
    topic_name = "mujoco/" + actuator_name;
  }

  int argc = 0;
  char ** argv = nullptr;
  if(!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;

  nh_ = rclcpp::Node::make_shared(actuator_name + "_actuator_command_plugin", node_options);
  sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
      topic_name, 1, std::bind(&ActuatorCommand::callback, this, std::placeholders::_1));
  // // Use a dedicated queue so as not to call callbacks of other modules
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);
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
    return;
  }
  // Call ROS callback
  executor_->spin_once(std::chrono::seconds(0));

  // Set actuator command
  if(!std::isnan(ctrl_))
  {
    d->ctrl[actuator_id_] = ctrl_;
    ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();
  }
}

void ActuatorCommand::callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  ctrl_ = msg->data;
}

} // namespace MujocoRosUtils
