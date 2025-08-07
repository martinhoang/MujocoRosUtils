#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <limits>
#include <string>

namespace MujocoRosUtils
{

  //* USAGE */
  // <mujoco>
  //
  // <extension>
  //   <plugin plugin="MujocoRosUtils::ActuatorCommand">
  //     <instance name="ros2_control" />
  //   </plugin>
  //   ... other plugins ...
  // </extension>
  //
  // <worldbody>
  // ...
  // </worldbody>
  //
  // <actuator>
  //   <position name="target_joint" joint="target_joint" kp="100" ctrlrange="-3.14159 3.14159" forcelimited="true" forcerange="-1000 1000" />
  //   <plugin plugin="MujocoRosUtils::ActuatorCommand" joint="target_joint" instance="ros2_control"/>
  // </actuator>
  // ... other actuators ...
  // </mujoco>

/** \brief Plugin to send a command to an actuator via ROS topic. */
class ActuatorCommand
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static std::unique_ptr<ActuatorCommand> Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ActuatorCommand(ActuatorCommand &&) = default;
  ~ActuatorCommand();

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param actuator_id actuator ID
      \param topic_name topic name
  */
  ActuatorCommand(const mjModel * m, mjData * d, std::vector<int> actuator_ids, std::string topic_name);

  /** \brief Constructor.
      \param msg command message
  */
  void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /** \brief Callback for joint trajectory commands.
      \param msg command message
  */
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

protected:
  //! Actuator ID
  int actuator_id_ = -1;
  std::vector<int> actuators_;

  //! Actuator command (NaN for no command)
  std::vector<mjtNum> ctrl_;

  //! Joint names asssociated with the actuators
  std::vector<std::string> active_joint_names_;

  //! ROS variables
  //! @{
  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;
  //! @}
};

} // namespace MujocoRosUtils
