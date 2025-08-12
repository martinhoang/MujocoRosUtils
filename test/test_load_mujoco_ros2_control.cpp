#include "mujoco_system_interface.hpp"
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_load_mujoco_ros2_control_node");

  std::unique_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>> class_loader;

  std::string base_class = "mujoco_ros2_control::MujocoSystemInterface";
  try
  {
    class_loader
      = std::make_unique<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>>(
        "mujoco_ros_utils", /* Package where this plugin is located */
        base_class);
  }
  catch (const pluginlib::ClassLoaderException &ex)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to create class loader: %s", ex.what());
    return 1;
  }

  std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system;

  try
  {
    mujoco_system = class_loader->createSharedInstance("mujoco_ros2_control/MujocoSystem");
  }
  catch (pluginlib::PluginlibException &ex)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to create 'MujocoSystem' instance: %s", ex.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}