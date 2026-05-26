#pragma once

#include <controller_manager/controller_manager.hpp>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco_system.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace MujocoRosUtils
{

class Ros2Control
{
public:
  static void                         RegisterPlugin();
  static std::unique_ptr<Ros2Control> Create(const mjModel *m, mjData *d, int plugin_id);

public:
  Ros2Control(Ros2Control &&) = default;
  Ros2Control(const mjModel *model, mjData *data, std::string &config_file_path,
              std::string node_namespace   = "",
              std::string robot_param_node = "robot_state_publisher");
  ~Ros2Control();

  void reset(const mjModel *m, int plugin_id);
  void compute(const mjModel *m, mjData *d, int plugin_id);

protected:
  bool                    initialize();
  rclcpp::Node::SharedPtr node_  = nullptr;
  const mjModel          *model_ = nullptr;
  mjData                 *data_  = nullptr;
  static inline std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>>
                                                            mujoco_system_loader_ = nullptr;
  std::shared_ptr<controller_manager::ControllerManager>    controller_manager_   = nullptr;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_             = nullptr;
  std::thread                                               executor_thread_;
  bool                                                      stop_executor_thread_ = true;

  double       update_rate_    = 100.0;
  double       control_period_ = 1.0 / update_rate_;
  rclcpp::Time last_update_    = rclcpp::Time{(uint64_t)0, RCL_ROS_TIME};

  // Initialization state management
  bool        initialized_ = false;
  std::string config_file_path_;
  std::string node_namespace_;
  std::string robot_param_node_;

  /// Wall-clock time of the last initialize() attempt (avoids blocking the sim
  /// thread on every step while the robot_state_publisher is not yet available).
  std::chrono::steady_clock::time_point last_init_attempt_{};
  static constexpr double INIT_RETRY_INTERVAL_S = 5.0;
  /// How many times initialize() has been retried (for log-spam suppression).
  int init_retry_count_ = 0;

  // Simulation reset — set by service callback, applied in compute() to avoid mid-step races
  std::atomic<bool> reset_requested_{false};
  // Set whenever a reset is detected (service or viewer Backspace).
  // Forces write() with period=0 on the next tick so MujocoSystem::reset() runs
  // and d->ctrl is held at initial values before any stale controller commands land.
  std::atomic<bool>                                  hardware_reset_pending_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  static inline std::atomic<int> ros_control_instances_{0};
  // Serialises the rcl_context global_arguments read-modify-write across all
  // plugin instances in the same process (MuJoCo loads every plugin concurrently).
  static inline std::mutex rcl_global_args_mutex_;
};

} // namespace MujocoRosUtils