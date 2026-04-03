#include "TopicControl.h"
#include "mujoco_utils.hpp"

#include <mujoco/mujoco.h>
#include <iostream>
#include <limits>

namespace MujocoRosUtils
{

constexpr char ATTR_NODE_NAME[] = "node_name";
constexpr char ATTR_TOPIC_NAME[] = "topic_name";

void TopicControl::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  print_info("Registering TopicControl plugin\n");

  plugin.name = "MujocoRosUtils::TopicControl";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  std::vector<const char *> attributes = {ATTR_NODE_NAME, ATTR_TOPIC_NAME};

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  plugin.nstate = +[](const mjModel *, int) { return 0; };
  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };
  plugin.needstage = 0;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto plugin_instance = TopicControl::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance.release());
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<TopicControl *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class TopicControl *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int)
  {
    auto * plugin_instance = reinterpret_cast<class TopicControl *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

std::unique_ptr<TopicControl> TopicControl::Create(const mjModel * m, mjData * d, int plugin_id)
{
  const char * topic_name_str = mj_getPluginConfig(m, plugin_id, ATTR_TOPIC_NAME);
  std::string topic_name = topic_name_str ? std::string(topic_name_str) : "";

  std::vector<int> actuator_ids;
  for(int i = 0; i < m->nu; ++i)
  {
    if(m->actuator_plugin[i] == plugin_id)
    {
      actuator_ids.push_back(i);
    }
  }

  if(actuator_ids.empty())
  {
    mju_error("[TopicControl] No actuator found for plugin ID %d\n", plugin_id);
    return nullptr;
  }

  return std::unique_ptr<TopicControl>(new TopicControl(m, d, std::move(actuator_ids), topic_name));
}

TopicControl::TopicControl(const mjModel * m, mjData * d, std::vector<int> actuator_ids, std::string topic_name)
: model_(m), data_(d), actuators_(std::move(actuator_ids))
{
  ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();

  const char * node_name_char = mj_getPluginConfig(m, 0, ATTR_NODE_NAME);
  std::string node_name = node_name_char ? std::string(node_name_char) : "topic_control_plugin";

  if(!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  nh_ = rclcpp::Node::make_shared(node_name);

  if(topic_name.empty())
  {
    topic_name = "command";
  }

  sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
    topic_name, 1,
    std::bind(&TopicControl::commandCallback, this, std::placeholders::_1)
  );

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);

  print_info("[TopicControl] Initialized for %zu actuators on topic '%s'\n", actuators_.size(), topic_name.c_str());
}

TopicControl::~TopicControl()
{
  if(executor_ && nh_)
  {
    executor_->remove_node(nh_);
  }
}

void TopicControl::reset(const mjModel *, int)
{
  ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();
}

void TopicControl::compute(const mjModel *, mjData * d, int)
{
  if(executor_)
  {
    executor_->spin_once(std::chrono::seconds(0));
  }

  if(!std::isnan(ctrl_))
  {
    for(int id : actuators_)
    {
      d->ctrl[id] = ctrl_;
    }
  }
}

void TopicControl::commandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  ctrl_ = msg->data;
}

} // namespace MujocoRosUtils
