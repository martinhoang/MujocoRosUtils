#pragma once

#include "SimDataRegistry.hpp"
#include "SimRecorder.hpp"
#include "mujoco_ros_utils/srv/start_recording.hpp"
#include "mujoco_ros_utils/srv/stop_recording.hpp"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace MujocoRosUtils
{

/**
 * SimDataAggregator — MuJoCo passive plugin
 *
 * Gathers simulation data from within the same process (no ROS 2 DDS hop) and
 * publishes a SimSnapshot into the SimDataRegistry every simulation step.
 *
 * What it collects
 * ----------------
 *   cameras    : Latest BGR frames from any ImagePublisher instances whose
 *                `namespace` attribute matches one of the names in the
 *                `camera_namespaces` plugin config list.
 *   joints     : Position, velocity, effort (qfrc_actuator) and actuator ctrl
 *                values for every joint listed in `joint_names`, read directly
 *                from mjData (zero copies, no networking).
 *
 * Plugin attributes
 * -----------------
 *   instance_name      (required) Key under which the SimSnapshot is stored in
 *                                 SimDataRegistry.  Use this name to call
 *                                 SimDataRegistry::instance().getSnapshot().
 *   camera_namespaces  (optional) Comma-separated list of ImagePublisher
 *                                 namespace values to aggregate, e.g.
 *                                 "wrist_cam,base_cam".
 *   joint_names        (optional) Comma-separated list of joint names whose
 *                                 state and commands to aggregate, e.g.
 *                                 "shoulder_joint,elbow_joint,wrist_joint".
 *
 * Example XML
 * -----------
 *   <plugin name="sim_data" plugin="MujocoRosUtils::SimDataAggregator">
 *     <config key="instance_name"     value="my_aggregator"/>
 *     <config key="camera_namespaces" value="wrist_cam,base_cam"/>
 *     <config key="joint_names"       value="joint1,joint2,joint3"/>
 *   </plugin>
 *
 * Accessing the data
 * ------------------
 *   SimSnapshot snap;
 *   if (SimDataRegistry::instance().getSnapshot("my_aggregator", snap)) {
 *     // snap.cameras["wrist_cam"].data   — BGR pixels
 *     // snap.joints["joint1"].position   — rad or m
 *     // snap.joints["joint1"].actuator_ctrls["joint1_pos"] — ctrl value
 *   }
 */
class SimDataAggregator
{
public:
  /** \brief Register the plugin with MuJoCo. */
  static void RegisterPlugin();

  /** \brief Create an instance (called by MuJoCo during model load). */
  static SimDataAggregator * Create(const mjModel * m, mjData * d, int plugin_id);

  SimDataAggregator(SimDataAggregator &&) = default;
  ~SimDataAggregator();

  /** \brief Called by MuJoCo on simulation reset. */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Called by MuJoCo every simulation step. */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  SimDataAggregator(std::string instance_name,
                    std::vector<std::string> camera_namespaces,
                    std::vector<std::string> joint_names);

  std::string              instance_name_;
  std::vector<std::string> camera_namespaces_;
  std::vector<std::string> joint_names_;

  // ── ROS 2 node + executor (hosts the recording services) ──────────────────
  rclcpp::Node::SharedPtr                                    node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread                                                executor_thread_;
  std::atomic<bool>                                          stop_executor_{false};

  using StartSrv  = mujoco_ros_utils::srv::StartRecording;
  using StopSrv   = mujoco_ros_utils::srv::StopRecording;
  using TriggerSrv = std_srvs::srv::Trigger;
  rclcpp::Service<StartSrv>::SharedPtr   start_srv_;
  rclcpp::Service<StopSrv>::SharedPtr    stop_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr cancel_srv_;  ///< stop + discard (failure/cancel)

  // ── Recorder ──────────────────────────────────────────────────────────────
  std::unique_ptr<SimRecorder> recorder_;

  // Per-camera last-seen seq: avoids redundant 900KB BGR copies in compute()
  std::unordered_map<std::string, uint64_t> last_camera_snap_seq_;

  // Per-key throttle for repeated warnings (keys are joint names or warning tags)
  static constexpr double WARN_THROTTLE_S = 10.0;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> warn_throttle_;
};

}  // namespace MujocoRosUtils
