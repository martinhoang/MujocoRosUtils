#pragma once

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco_system.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace MujocoRosUtils
{

class Ros2Control
{
public:
  static void                         RegisterPlugin();
  static std::unique_ptr<Ros2Control> Create(const mjModel *m, mjData *d, int plugin_id);

public:
  Ros2Control(Ros2Control &&) = default;
  Ros2Control(const mjModel *model, mjData *data,
              std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system);

  void reset(const mjModel *m, int plugin_id);
  void compute(const mjModel *m, mjData *d, int plugin_id);

protected:
  rclcpp::Node::SharedPtr node_  = nullptr;
  const mjModel          *model_ = nullptr;
  mjData                 *data_  = nullptr;
  static inline std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>>
                                                              mujoco_system_loader_ = nullptr;
  std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system_        = nullptr;
  std::unique_ptr<controller_manager::ControllerManager>      controller_manager_   = nullptr;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>   executor_             = nullptr;
};

} // namespace MujocoRosUtils