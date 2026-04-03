#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include <string>
#include <vector>
#include <memory>

namespace MujocoRosUtils
{

class TopicControl
{
public:
  static void RegisterPlugin();
  static std::unique_ptr<TopicControl> Create(const mjModel * m, mjData * d, int plugin_id);

  ~TopicControl();
  void reset(const mjModel * m, int plugin_id);
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  TopicControl(const mjModel * m, mjData * d, std::vector<int> actuator_ids, std::string topic_name);

  void commandCallback(const std_msgs::msg::Float64::SharedPtr msg);

  int actuator_id_ = -1;
  std::vector<int> actuators_;
  const mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  mjtNum ctrl_ = 0.0;
  bool command_received_ = false;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

} // namespace MujocoRosUtils
