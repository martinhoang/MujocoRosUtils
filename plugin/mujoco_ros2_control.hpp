#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace MujocoRosUtils
{

class MujocoRos2Control : public hardware_interface::SystemInterface
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static MujocoRos2Control * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Destructor. */
  ~MujocoRos2Control();

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

  /** \brief hardware_interface::SystemInterface overrides. */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param joint_names list of joint names
      \param node_name node name
  */
  MujocoRos2Control(const mjModel * m,
                    mjData * d,
                    const std::vector<std::string> & joint_names,
                    const std::string & node_name);

  /** \brief Initialize ROS components. */
  void init_ros();

protected:
  //! Mujoco data
  //! @{
  const mjModel * m_ = nullptr;
  mjData * d_ = nullptr;
  //! @}

  //! ROS variables
  //! @{
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<std::thread> ros_spin_thread_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  //! @}

  //! Joint states and commands
  //! @{
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_pos_;
  std::vector<double> hw_states_vel_;
  std::vector<int> qpos_adr_;
  std::vector<int> qvel_adr_;
  std::vector<int> ctrl_adr_;
  //! @}

  //! Control settings
  bool is_running_ = false;

  //! Previous time for update loop
  rclcpp::Time last_update_time_;
};

} // namespace MujocoRosUtils
