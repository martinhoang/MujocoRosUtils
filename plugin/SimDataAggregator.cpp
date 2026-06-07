#include "SimDataAggregator.h"
#include "RosContextManager.hpp"
#include "SimDataRegistry.hpp"
#include "mujoco_utils.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace MujocoRosUtils
{

// ── plugin attribute keys ──────────────────────────────────────────────────────
constexpr char ATTR_INSTANCE_NAME[]      = "instance_name";
constexpr char ATTR_CAMERA_NAMESPACES[]  = "camera_namespaces";
constexpr char ATTR_JOINT_NAMES[]        = "joint_names";

// ── helpers ────────────────────────────────────────────────────────────────────

/// Split a comma-separated string into a trimmed vector of non-empty tokens.
static std::vector<std::string> splitComma(const std::string & s)
{
  std::vector<std::string> result;
  std::istringstream       ss(s);
  std::string              token;
  while (std::getline(ss, token, ','))
  {
    // trim whitespace
    token.erase(token.begin(), std::find_if(token.begin(), token.end(), [](unsigned char c) {
                  return !std::isspace(c);
                }));
    token.erase(std::find_if(token.rbegin(), token.rend(),
                             [](unsigned char c) { return !std::isspace(c); })
                  .base(),
                token.end());
    if (!token.empty())
      result.push_back(token);
  }
  return result;
}

// ── RegisterPlugin ─────────────────────────────────────────────────────────────

void SimDataAggregator::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::SimDataAggregator";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char * attributes[] = {ATTR_INSTANCE_NAME, ATTR_CAMERA_NAMESPACES, ATTR_JOINT_NAMES};
  plugin.nattribute          = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes          = attributes;

  plugin.nstate = +[](const mjModel *, int) { return 0; };

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id) {
    auto * inst = SimDataAggregator::Create(m, d, plugin_id);
    if (!inst)
      return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(inst);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id) {
    delete reinterpret_cast<SimDataAggregator *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, void * plugin_data, int plugin_id) {
    reinterpret_cast<SimDataAggregator *>(plugin_data)->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int) {
    reinterpret_cast<SimDataAggregator *>(d->plugin_data[plugin_id])->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
  print_confirm("Successfully registered 'MujocoRosUtils::SimDataAggregator' plugin\n");
}

// ── Create ─────────────────────────────────────────────────────────────────────

SimDataAggregator * SimDataAggregator::Create(const mjModel * m, mjData *, int plugin_id)
{
  // instance_name (required)
  const char * name_c = mj_getPluginConfig(m, plugin_id, ATTR_INSTANCE_NAME);
  if (!name_c || strlen(name_c) == 0)
  {
    mju_error("[SimDataAggregator] `instance_name` attribute is required.");
    return nullptr;
  }
  std::string instance_name(name_c);

  // camera_namespaces (optional)
  std::vector<std::string> camera_namespaces;
  const char * cams_c = mj_getPluginConfig(m, plugin_id, ATTR_CAMERA_NAMESPACES);
  if (cams_c && strlen(cams_c) > 0)
    camera_namespaces = splitComma(cams_c);

  // joint_names (optional)
  std::vector<std::string> joint_names;
  const char * joints_c = mj_getPluginConfig(m, plugin_id, ATTR_JOINT_NAMES);
  if (joints_c && strlen(joints_c) > 0)
    joint_names = splitComma(joints_c);

  print_confirm("[SimDataAggregator] Creating instance '%s' (%zu cameras, %zu joints)\n",
                instance_name.c_str(), camera_namespaces.size(), joint_names.size());

  return new SimDataAggregator(std::move(instance_name),
                               std::move(camera_namespaces),
                               std::move(joint_names));
}

// ── Constructor ────────────────────────────────────────────────────────────────

SimDataAggregator::SimDataAggregator(std::string              instance_name,
                                     std::vector<std::string> camera_namespaces,
                                     std::vector<std::string> joint_names)
    : instance_name_(std::move(instance_name))
    , camera_namespaces_(std::move(camera_namespaces))
    , joint_names_(std::move(joint_names))
{
  // Tell the registry which camera keys we need so ImagePublisher knows to push frames.
  auto & reg = SimDataRegistry::instance();
  for (const auto & ns : camera_namespaces_)
    reg.registerCameraConsumer(ns);

  // ── ROS 2 node + services ──────────────────────────────────────────────────
  {
    int    argc = 0;
    char **argv = nullptr;
    ros_context_lease_.acquire(argc, argv);
  }

  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"use_sim_time", true}});
  opts.automatically_declare_parameters_from_overrides(true);
  node_ = rclcpp::Node::make_shared("sim_data_aggregator_" + instance_name_, opts);

  // ── start_recording service ────────────────────────────────────────────────
  recorder_ = std::make_unique<SimRecorder>(instance_name_, camera_namespaces_, joint_names_);

  start_srv_ = node_->create_service<StartSrv>(
    "~/start_recording",
    [this](const StartSrv::Request::SharedPtr  req,
           StartSrv::Response::SharedPtr        res) {
      if (recorder_->isRecording())
      {
        res->success = false;
        res->message = "Already recording.  Call stop_recording first.";
        return;
      }

      SimRecorder::Format fmt = SimRecorder::Format::HDF5;  // default
      if (req->format == "mcap")
        fmt = SimRecorder::Format::MCAP;
      else if (req->format == "lerobot")
        fmt = SimRecorder::Format::LeRobot;
      else if (req->format == "both")
        fmt = SimRecorder::Format::Both;
      else if (req->format == "hdf5_and_mcap")
        fmt = SimRecorder::Format::HDF5AndMCAP;
      else if (req->format == "hdf5_and_lerobot")
        fmt = SimRecorder::Format::HDF5AndLeRobot;
      else if (!req->format.empty() && req->format != "hdf5")
      {
        res->success = false;
        res->message = "Unknown format '" + req->format +
                       "'. Use 'hdf5' (default), 'mcap', 'lerobot', 'both', "
                       "'hdf5_and_mcap', or 'hdf5_and_lerobot'.";
        return;
      }

      const bool ok = recorder_->start(req->output_dir, fmt,
                                        req->episode_index,
                                        req->task_description);
      res->success              = ok;
      res->message              = ok ? "Recording started." : "Failed to start recording.";
      res->actual_path          = req->output_dir;
      res->actual_episode_index = ok ? recorder_->episodeIdx() : -1;
    });

  // ── stop_recording service ─────────────────────────────────────────────────
  stop_srv_ = node_->create_service<StopSrv>(
    "~/stop_recording",
    [this](const StopSrv::Request::SharedPtr  req,
           StopSrv::Response::SharedPtr        res) {
      if (!recorder_->isRecording())
      {
        res->success = false;
        res->message = "Not recording.";
        return;
      }

      int    frames;
      double dur;
      const bool ok = recorder_->stop(frames, dur, req->discard);
      res->success          = ok;
      res->frames_recorded  = frames;
      res->duration_seconds = dur;
      res->message          = ok ? "Recording saved." : "Recording stopped with errors (see log).";
    });

  // ── cancel_recording service (stop + discard — use for failure/cancel) ─────
  cancel_srv_ = node_->create_service<TriggerSrv>(
    "~/cancel_recording",
    [this](const TriggerSrv::Request::SharedPtr,
           TriggerSrv::Response::SharedPtr res) {
      if (!recorder_->isRecording())
      {
        res->success = false;
        res->message = "Not recording.";
        return;
      }
      int    frames;
      double dur;
      recorder_->stop(frames, dur, /*discard=*/true);
      res->success = true;
      res->message = "Recording cancelled and data discarded.";
    });

  // ── Spin executor in background thread ────────────────────────────────────
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  stop_executor_.store(false);
  executor_thread_ = std::thread([this]() {
    while (!stop_executor_.load())
      executor_->spin_some(std::chrono::milliseconds(10));
  });

  RCLCPP_INFO(node_->get_logger(),
              "SimDataAggregator '%s' ready.  "
              "Services: ~/start_recording, ~/stop_recording, ~/cancel_recording",
              instance_name_.c_str());
}

// ── Destructor ─────────────────────────────────────────────────────────────────

SimDataAggregator::~SimDataAggregator()
{
  // Stop recording if active.
  if (recorder_ && recorder_->isRecording())
  {
    try
    {
      int    f;
      double d;
      recorder_->stop(f, d, /*discard=*/false);
    }
    catch(const std::exception & e)
    {
      if(node_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to stop recorder during shutdown: %s", e.what());
      }
    }
    catch(...)
    {
      if(node_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to stop recorder during shutdown");
      }
    }
  }

  // Shut down executor.
  stop_executor_.store(true);
  if (executor_thread_.joinable())
    executor_thread_.join();

  // Release camera consumer registrations.
  auto & reg = SimDataRegistry::instance();
  for (const auto & ns : camera_namespaces_)
    reg.unregisterCameraConsumer(ns);

}

// ── reset ──────────────────────────────────────────────────────────────────────

void SimDataAggregator::reset(const mjModel *, int)
{}

// ── compute ────────────────────────────────────────────────────────────────────

void SimDataAggregator::compute(const mjModel * m, mjData * d, int)
{
  SimSnapshot snap;
  snap.sim_time = d->time;

  // ── Camera frames ─────────────────────────────────────────────────────────
  // Only copy full BGR data when a new frame has been published (seq changed).
  // ImagePublisher runs at ~30 Hz inside a 2000 Hz sim loop, so the cache holds
  // the same frame for ~66 consecutive steps.  Skipping the 900 KB memcpy on
  // those steps drops camera-copy bandwidth from ~1.8 GB/s to ~27 MB/s.
  auto & registry = SimDataRegistry::instance();
  for (const auto & ns : camera_namespaces_)
  {
    const uint64_t cur_seq = registry.getCameraFrameSeq(ns);
    if (cur_seq == last_camera_snap_seq_[ns])
      continue;  // same frame as last step — skip expensive copy
    last_camera_snap_seq_[ns] = cur_seq;

    CameraFrame frame;
    if (registry.getCameraFrame(ns, frame))
      snap.cameras[ns] = std::move(frame);
  }

  // ── Joint data ────────────────────────────────────────────────────────────
  const auto now = std::chrono::steady_clock::now();
  for (const auto & jname : joint_names_)
  {
    int jid = mj_name2id(m, mjOBJ_JOINT, jname.c_str());
    if (jid < 0)
    {
      auto & last = warn_throttle_[jname];
      if (std::chrono::duration<double>(now - last).count() >= WARN_THROTTLE_S)
      {
        print_warning("[SimDataAggregator] Joint '%s' not found in model, skipping.\n",
                      jname.c_str());
        last = now;
      }
      continue;
    }

    // Only handle 1-DOF joints (hinge / slide)
    const int jtype = m->jnt_type[jid];
    if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE)
    {
      const std::string wkey = jname + ":multidof";
      auto & last = warn_throttle_[wkey];
      if (std::chrono::duration<double>(now - last).count() >= WARN_THROTTLE_S)
      {
        print_warning("[SimDataAggregator] Joint '%s' is not a hinge or slide joint — "
                      "multi-DOF joints are not supported, skipping.\n",
                      jname.c_str());
        last = now;
      }
      continue;
    }

    JointSnapshot js;
    const int     qadr = m->jnt_qposadr[jid];
    const int     dadr = m->jnt_dofadr[jid];
    js.position        = d->qpos[qadr];
    js.velocity        = d->qvel[dadr];
    js.effort          = d->qfrc_actuator[dadr];

    // Collect ctrl from every actuator whose transmission targets this joint
    for (int aid = 0; aid < m->nu; ++aid)
    {
      if (m->actuator_trntype[aid] == mjTRN_JOINT && m->actuator_trnid[aid * 2] == jid)
      {
        const char * aname = mj_id2name(m, mjOBJ_ACTUATOR, aid);
        std::string  key   = aname ? std::string(aname) : ("actuator_" + std::to_string(aid));
        js.actuator_ctrls[key] = d->ctrl[aid];
      }
    }

    snap.joints[jname] = std::move(js);
  }

  snap.valid = true;

  // Feed recorder directly (before move) — no-op when not recording.
  if (recorder_ && recorder_->isRecording())
    recorder_->addFrame(snap);

  registry.updateSnapshot(instance_name_, std::move(snap));
}

}  // namespace MujocoRosUtils
