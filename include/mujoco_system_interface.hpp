#pragma once

#include <hardware_interface/system_interface.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{

/**
 * @brief This class is the base public interface for MujocoSystem
 *
 */
class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Initialize the MujocoSystemInterface
   *
   * @param node  pointer to the ROS 2 node
   * @param m constant pointer to the Mujoco model
   * @param d pointer to the Mujoco data
   * @param info hardware interface information
   * @return true if initialization was successful
   * @return false if initialization failed
   */
  virtual bool initialize(rclcpp::Node::SharedPtr node, const mjModel *m, mjData *d,
                          const hardware_interface::HardwareInfo &info)
    = 0;

protected:
  rclcpp::Node::SharedPtr node_;
};

} // namespace mujoco_ros2_control