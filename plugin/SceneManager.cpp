#include "SceneManager.h"
#include "mujoco_utils.hpp"
#include "../simulate/between_step_hook.h"

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#include <cctype>
#include <cmath>
#include <execinfo.h>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <unistd.h>
#include <vector>

// Wraps every std::regex construction: prints the pattern and any error + backtrace.
#define MAKE_REGEX(var, pat_expr) \
  std::regex var; \
  do { \
    const std::string _pat_str_ = (pat_expr); \
    try { var = std::regex(_pat_str_); } \
    catch (const std::regex_error & _e_) { \
      fprintf(stderr, "[SceneManager][regex] %s:%d '" #var "' ERROR on pattern: %s\n  what(): %s\n", \
              __FILE__, __LINE__, _pat_str_.c_str(), _e_.what()); \
      void * _bt_[64]; int _nb_ = backtrace(_bt_, 64); \
      backtrace_symbols_fd(_bt_, _nb_, fileno(stderr)); \
      throw; \
    } \
  } while (0)

namespace fs = std::filesystem;

namespace MujocoRosUtils
{

// ── Static shared-state registry ──────────────────────────────────────────
std::mutex                                                      SceneManager::s_registry_mutex_;
std::unordered_map<std::string, std::shared_ptr<SceneManager::SharedState>>
                                                                SceneManager::s_registry_;

// ── RegisterPlugin ─────────────────────────────────────────────────────────

void SceneManager::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name            = "MujocoRosUtils::SceneManager";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  static const char * attributes[] = {"node_name", "namespace"};
  plugin.nattribute          = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes          = attributes;

  plugin.nstate     = +[](const mjModel *, int) { return 0; };
  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };
  plugin.needstage   = mjSTAGE_POS;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id) -> int {
    auto * instance = SceneManager::Create(m, d, plugin_id);
    if (!instance)
      return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id) {
    delete reinterpret_cast<SceneManager *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * /*m*/, double * /*plugin_state*/,
                    void * /*plugin_data*/, int /*plugin_id*/) {
    // SceneManager has no plugin state array — nothing to reset.
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int) {
    reinterpret_cast<SceneManager *>(d->plugin_data[plugin_id])->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

// ── Create ─────────────────────────────────────────────────────────────────

SceneManager * SceneManager::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // Read plugin attributes
  auto getAttr = [&](const char * name) -> std::string {
    const char * v = mj_getPluginConfig(m, plugin_id, name);
    return (v && v[0]) ? std::string(v) : std::string();
  };

  const std::string node_name = getAttr("node_name").empty()
                                  ? "scene_manager"
                                  : getAttr("node_name");
  const std::string node_ns   = getAttr("namespace");

  try
  {
    return new SceneManager(node_name, node_ns);
  }
  catch (const std::exception & e)
  {
    mju_error("[SceneManager] Create() failed: %s", e.what());
    return nullptr;
  }
}

// ── SharedState::initialize ──────────────────────────────────────────────────

void SceneManager::SharedState::initialize(const std::string & node_name,
                                           const std::string & node_namespace)
{
  using SpawnSrv           = mujoco_ros_utils::srv::SpawnEntity;
  using DeleteSrv         = mujoco_ros_utils::srv::DeleteEntity;
  using ListSrv            = mujoco_ros_utils::srv::ListEntities;
  using SetBodyPoseSrv     = mujoco_ros_utils::srv::SetBodyPose;
  using SetGeomPropsSrv    = mujoco_ros_utils::srv::SetGeomProperties;
  using SetEqActiveSrv     = mujoco_ros_utils::srv::SetEqualityActive;
  using SetJointPositionsSrv = mujoco_ros_utils::srv::SetJointPositions;
  using ApplyBodyWrenchSrv   = mujoco_ros_utils::srv::ApplyBodyWrench;
  using GetBodyPoseSrv     = mujoco_ros_utils::srv::GetBodyPose;
  using GetGeomPropsSrv    = mujoco_ros_utils::srv::GetGeomProperties;
  using GetJointStateSrv   = mujoco_ros_utils::srv::GetJointState;
  using GetModelInfoSrv    = mujoco_ros_utils::srv::GetModelInfo;

  // Create an isolated ROS2 context so that rclcpp::shutdown() called by
  // RosControl (when ros_control_instances_ drops to 0 during mj_recompile)
  // does NOT kill our executor.  The custom context uses the same domain ID
  // as the default context (inherited from ROS_DOMAIN_ID env var), so our
  // node is still discoverable and callable from the CLI and other nodes.
  rclcpp::InitOptions init_opts;
  init_opts.shutdown_on_signal = false;  // don't react to SIGINT/SIGTERM
  rclcpp_context = std::make_shared<rclcpp::Context>();
  rclcpp_context->init(0, nullptr, init_opts);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.context(rclcpp_context);
  node = rclcpp::Node::make_shared(node_name, node_namespace, opts);

  // Capture a raw pointer to *this.  SharedState is owned by the static
  // registry (strong shared_ptr) so it lives for the process lifetime once
  // created — raw pointer capture in these lambdas is safe.
  SharedState * ss = this;

  // Helper: poll op->state until done or timeout.  Avoids std::future which
  // triggers a pthread priority-inheritance assertion when set_value() is
  // called from the RT-scheduled simulation thread.
  auto wait_op = [](const std::shared_ptr<PendingOp> & op, int timeout_s) -> bool {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    while (op->state.load(std::memory_order_acquire) == 0 &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return op->state.load(std::memory_order_acquire) != 0;
  };

  // ── reset_simulation ─────────────────────────────────────────────────────
  reset_simulation_srv = node->create_service<std_srvs::srv::Trigger>(
    "~/reset_simulation",
    [ss, wait_op](const std_srvs::srv::Trigger::Request::SharedPtr  /*req*/,
         std_srvs::srv::Trigger::Response::SharedPtr         res) {
      auto op  = std::make_shared<PendingOp>();
      op->type = OpType::RESET_SIMULATION;
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) {
        res->success = (op->state.load() == 1);
        res->message = op->result_msg;
      } else {
        res->success = false;
        res->message = "Timeout: reset_simulation within 5 s.";
      }
    });

  spawn_srv = node->create_service<SpawnSrv>(
    "~/spawn_entity",
    [ss, wait_op](const SpawnSrv::Request::SharedPtr  req,
         SpawnSrv::Response::SharedPtr        res) {
      if (req->name.empty())
      {
        res->success = false;
        res->message = "Entity name must not be empty.";
        return;
      }
      {
        std::lock_guard<std::mutex> lk(ss->pending_mutex);
        if (ss->spawned_entities.count(req->name))
        {
          res->success = false;
          res->message = "Entity '" + req->name + "' already exists — despawn first.";
          return;
        }
      }

      std::string xml;
      std::string source_directory;
      if (!req->xml_content.empty())
        xml = req->xml_content;
      else if (!req->xml_path.empty())
      {
        const fs::path xml_path = fs::absolute(req->xml_path);
        std::ifstream f(xml_path);
        if (!f) { res->success = false; res->message = "Cannot open xml_path: " + req->xml_path; return; }
        xml = std::string((std::istreambuf_iterator<char>(f)), {});
        source_directory = xml_path.parent_path().string();
      }
      else
      {
        res->success = false;
        res->message = "Either xml_content or xml_path must be provided.";
        return;
      }

      auto op        = std::make_shared<PendingOp>();
      op->type       = OpType::SPAWN;
      op->name       = req->name;
      op->xml        = xml;
      op->source_directory = source_directory;
      op->attach_to  = req->attach_to.empty() || req->attach_to == "worldbody"
                         ? "world" : req->attach_to;
      op->pos[0]     = req->pos_x; op->pos[1] = req->pos_y; op->pos[2] = req->pos_z;
      op->quat[0]    = (req->rot_qw != 0.0 || req->rot_qx != 0.0 ||
                        req->rot_qy != 0.0 || req->rot_qz != 0.0) ? req->rot_qw : 1.0;
      op->quat[1]    = req->rot_qx; op->quat[2] = req->rot_qy; op->quat[3] = req->rot_qz;
      op->with_freejoint = req->with_freejoint;

      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      auto fut = wait_op(op, 15);
      if (fut)
      {
        bool ok = (op->state.load() == 1);
        res->success = ok; res->message = op->result_msg;
        if (ok) {
          std::lock_guard<std::mutex> lk(ss->pending_mutex);
          auto it = ss->spawned_entities.find(req->name);
          if (it != ss->spawned_entities.end())
            res->spawned_body_name = it->second.spawned_body;
        }
      }
      else
      {
        res->success = false;
        res->message = "Timeout: sim thread did not apply spawn within 15 s.";
      }
    });

  // ── despawn_entity ───────────────────────────────────────────────────────
  despawn_srv = node->create_service<DeleteSrv>(
    "~/despawn_entity",
    [ss, wait_op](const DeleteSrv::Request::SharedPtr  req,
         DeleteSrv::Response::SharedPtr        res) {
      {
        std::lock_guard<std::mutex> lk(ss->pending_mutex);
        if (!ss->spawned_entities.count(req->name))
        {
          res->success = false;
          res->message = "Entity '" + req->name + "' not found.";
          return;
        }
      }
      auto op  = std::make_shared<PendingOp>();
      op->type = OpType::DESPAWN;
      op->name = req->name;

      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 15)) {
        res->success = (op->state.load() == 1); res->message = op->result_msg;
      } else {
        res->success = false;
        res->message = "Timeout: sim thread did not apply despawn within 15 s.";
      }
    });

  // ── list_entities ────────────────────────────────────────────────────────
  list_srv = node->create_service<ListSrv>(
    "~/list_entities",
    [ss](const ListSrv::Request::SharedPtr  /*req*/,
         ListSrv::Response::SharedPtr        res) {
      std::lock_guard<std::mutex> lk(ss->pending_mutex);
      for (const auto & [name, ent] : ss->spawned_entities) {
        res->names.push_back(name);
        res->body_names.push_back(ent.spawned_body);
        res->attach_points.push_back(ent.attach_to);
      }
    });

  // ── set_body_pose ────────────────────────────────────────────────────────
  set_body_pose_srv = node->create_service<SetBodyPoseSrv>(
    "~/set_body_pose",
    [ss, wait_op](const SetBodyPoseSrv::Request::SharedPtr  req,
         SetBodyPoseSrv::Response::SharedPtr        res) {
      if (req->body_name.empty()) { res->success = false; res->message = "body_name must not be empty."; return; }
      auto op      = std::make_shared<PendingOp>();
      op->type     = OpType::SET_BODY_POSE; op->name = req->body_name;
      op->pos[0]   = req->pos_x; op->pos[1] = req->pos_y; op->pos[2] = req->pos_z;
      op->quat[0]  = req->rot_qw; op->quat[1] = req->rot_qx;
      op->quat[2]  = req->rot_qy; op->quat[3] = req->rot_qz;
      op->relative = req->relative;
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) { res->success = (op->state.load() == 1); res->message = op->result_msg; }
      else { res->success = false; res->message = "Timeout: set_body_pose within 5 s."; }
    });

  // ── set_geom_properties ──────────────────────────────────────────────────
  set_geom_props_srv = node->create_service<SetGeomPropsSrv>(
    "~/set_geom_properties",
    [ss, wait_op](const SetGeomPropsSrv::Request::SharedPtr  req,
         SetGeomPropsSrv::Response::SharedPtr        res) {
      if (req->geom_name.empty()) { res->success = false; res->message = "geom_name must not be empty."; return; }
      auto op          = std::make_shared<PendingOp>();
      op->type         = OpType::SET_GEOM_PROPERTIES; op->name = req->geom_name;
      op->geom_size[0] = req->size_x; op->geom_size[1] = req->size_y; op->geom_size[2] = req->size_z;
      op->geom_rgba[0] = req->rgba_r; op->geom_rgba[1] = req->rgba_g;
      op->geom_rgba[2] = req->rgba_b; op->geom_rgba[3] = req->rgba_a;
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) { res->success = (op->state.load() == 1); res->message = op->result_msg; }
      else { res->success = false; res->message = "Timeout: set_geom_properties within 5 s."; }
    });

  // ── set_equality_active ──────────────────────────────────────────────────
  set_equality_active_srv = node->create_service<SetEqActiveSrv>(
    "~/set_equality_active",
    [ss, wait_op](const SetEqActiveSrv::Request::SharedPtr  req,
         SetEqActiveSrv::Response::SharedPtr        res) {
      if (req->constraint_name.empty()) { res->success = false; res->message = "constraint_name must not be empty."; return; }
      auto op               = std::make_shared<PendingOp>();
      op->type              = OpType::SET_EQUALITY_ACTIVE;
      op->name              = req->constraint_name;
      op->eq_active         = req->active;
      op->use_current_pose  = req->use_current_pose;
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) { res->success = (op->state.load() == 1); res->message = op->result_msg; }
      else { res->success = false; res->message = "Timeout: set_equality_active within 5 s."; }
    });

  // ── set_joint_positions ──────────────────────────────────────────────────
  set_joint_positions_srv = node->create_service<SetJointPositionsSrv>(
    "~/set_joint_positions",
    [ss, wait_op](const SetJointPositionsSrv::Request::SharedPtr  req,
         SetJointPositionsSrv::Response::SharedPtr        res) {
      if (req->joint_names.empty())
      { res->success = false; res->message = "joint_names must not be empty."; return; }
      if (req->joint_names.size() != req->positions.size())
      { res->success = false; res->message = "joint_names and positions must have the same length."; return; }
      auto op = std::make_shared<PendingOp>();
      op->type           = OpType::SET_JOINT_POSITIONS;
      op->joint_names    = req->joint_names;
      op->joint_positions = req->positions;
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) { res->success = (op->state.load() == 1); res->message = op->result_msg; }
      else { res->success = false; res->message = "Timeout: set_joint_positions within 5 s."; }
    });

  // ── apply_body_wrench ────────────────────────────────────────────────────
  apply_body_wrench_srv = node->create_service<ApplyBodyWrenchSrv>(
    "~/apply_body_wrench",
    [ss, wait_op](const ApplyBodyWrenchSrv::Request::SharedPtr  req,
         ApplyBodyWrenchSrv::Response::SharedPtr        res) {
      if (req->body_name.empty())
      { res->success = false; res->message = "body_name must not be empty."; return; }
      auto op = std::make_shared<PendingOp>();
      op->type            = OpType::APPLY_BODY_WRENCH;
      op->name            = req->body_name;
      op->reference_frame = req->reference_frame.empty() ? "body" : req->reference_frame;
      op->force[0] = req->force_x;  op->force[1] = req->force_y;  op->force[2] = req->force_z;
      op->torque[0] = req->torque_x; op->torque[1] = req->torque_y; op->torque[2] = req->torque_z;
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) { res->success = (op->state.load() == 1); res->message = op->result_msg; }
      else { res->success = false; res->message = "Timeout: apply_body_wrench within 5 s."; }
    });

  // ── get_body_pose ────────────────────────────────────────────────────────
  get_body_pose_srv = node->create_service<GetBodyPoseSrv>(
    "~/get_body_pose",
    [ss, wait_op](const GetBodyPoseSrv::Request::SharedPtr  req,
         GetBodyPoseSrv::Response::SharedPtr        res) {
      if (req->body_name.empty()) { res->success = false; res->message = "body_name must not be empty."; return; }
      auto op            = std::make_shared<PendingOp>();
      op->type           = OpType::GET_BODY_POSE; op->name = req->body_name;
      op->body_pose_data = std::make_shared<BodyPoseData>();
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) {
        res->success = (op->state.load() == 1); res->message = op->result_msg;
        if (res->success) {
          const auto & r = *op->body_pose_data;
          res->pos_x = r.pos[0]; res->pos_y = r.pos[1]; res->pos_z = r.pos[2];
          res->rot_qw = r.quat[0]; res->rot_qx = r.quat[1];
          res->rot_qy = r.quat[2]; res->rot_qz = r.quat[3];
          res->angular_vel_x = r.angular_vel[0]; res->angular_vel_y = r.angular_vel[1]; res->angular_vel_z = r.angular_vel[2];
          res->linear_vel_x  = r.linear_vel[0];  res->linear_vel_y  = r.linear_vel[1];  res->linear_vel_z  = r.linear_vel[2];
          res->parent_body = r.parent_body; res->is_dynamic = r.is_dynamic;
        }
      } else { res->success = false; res->message = "Timeout: get_body_pose within 5 s."; }
    });

  // ── get_geom_properties ──────────────────────────────────────────────────
  get_geom_props_srv = node->create_service<GetGeomPropsSrv>(
    "~/get_geom_properties",
    [ss, wait_op](const GetGeomPropsSrv::Request::SharedPtr  req,
         GetGeomPropsSrv::Response::SharedPtr        res) {
      if (req->geom_name.empty()) { res->success = false; res->message = "geom_name must not be empty."; return; }
      auto op             = std::make_shared<PendingOp>();
      op->type            = OpType::GET_GEOM_PROPERTIES; op->name = req->geom_name;
      op->geom_props_data = std::make_shared<GeomPropsData>();
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) {
        res->success = (op->state.load() == 1); res->message = op->result_msg;
        if (res->success) {
          const auto & r = *op->geom_props_data;
          res->size_x = r.size[0]; res->size_y = r.size[1]; res->size_z = r.size[2];
          res->rgba_r = r.rgba[0]; res->rgba_g = r.rgba[1];
          res->rgba_b = r.rgba[2]; res->rgba_a = r.rgba[3];
          res->geom_type = geomTypeToString(r.type); res->parent_body = r.parent_body;
        }
      } else { res->success = false; res->message = "Timeout: get_geom_properties within 5 s."; }
    });

  // ── get_joint_state ──────────────────────────────────────────────────────
  get_joint_state_srv = node->create_service<GetJointStateSrv>(
    "~/get_joint_state",
    [ss, wait_op](const GetJointStateSrv::Request::SharedPtr  req,
         GetJointStateSrv::Response::SharedPtr        res) {
      auto op             = std::make_shared<PendingOp>();
      op->type            = OpType::GET_JOINT_STATE;
      op->joint_names     = req->joint_names;   // empty = query all
      op->joint_state_data = std::make_shared<JointStateData>();
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) {
        res->success = (op->state.load() == 1); res->message = op->result_msg;
        if (res->success) {
          const auto & r = *op->joint_state_data;
          res->joint_names_out = r.joint_names;
          res->pos_starts  = r.pos_starts;
          res->pos_lengths = r.pos_lengths;
          res->vel_starts  = r.vel_starts;
          res->vel_lengths = r.vel_lengths;
          res->positions   = r.positions;
          res->velocities  = r.velocities;
          res->efforts     = r.efforts;
        }
      } else { res->success = false; res->message = "Timeout: get_joint_state within 5 s."; }
    });

  // ── get_model_info ───────────────────────────────────────────────────────
  get_model_info_srv = node->create_service<GetModelInfoSrv>(
    "~/get_model_info",
    [ss, wait_op](const GetModelInfoSrv::Request::SharedPtr  req,
         GetModelInfoSrv::Response::SharedPtr        res) {
      auto op             = std::make_shared<PendingOp>();
      op->type            = OpType::GET_MODEL_INFO; op->name = req->filter_prefix;
      op->model_info_data = std::make_shared<ModelInfoData>();
      { std::lock_guard<std::mutex> lk(ss->pending_mutex); ss->pending_ops.push(op); }
      ss->ops_pending.store(true);
      if (wait_op(op, 5)) {
        res->success = (op->state.load() == 1); res->message = op->result_msg;
        if (res->success) {
          const auto & r = *op->model_info_data;
          res->body_names = r.bodies; res->geom_names = r.geoms;
          res->joint_names = r.joints; res->actuator_names = r.actuators;
          res->nbody = r.nbody; res->ngeom = r.ngeom; res->njnt = r.njnt; res->nu = r.nu;
        }
      } else { res->success = false; res->message = "Timeout: get_model_info within 5 s."; }
    });

  // ── executor ─────────────────────────────────────────────────────────────
  // Use the same custom context so the executor is not affected by
  // rclcpp::shutdown() on the default context.
  rclcpp::ExecutorOptions exec_opts;
  exec_opts.context = rclcpp_context;
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exec_opts);
  executor->add_node(node);
  executor_thread = std::thread([this]() { executor->spin(); });
}

SceneManager::SharedState::~SharedState()
{
  if (executor)
  {
    stop_executor.store(true);
    executor->cancel();
    if (executor_thread.joinable())
      executor_thread.join();
  }
  if (rclcpp_context && rclcpp_context->is_valid())
    rclcpp_context->shutdown("SceneManager destroyed");
}

// ── Constructor / Destructor ────────────────────────────────────────────────

SceneManager::SceneManager(std::string node_name, std::string node_namespace)
{
  // Build the registry key.
  node_key_ = (node_namespace.empty() ? "" : node_namespace + "/") + node_name;

  std::lock_guard<std::mutex> lk(s_registry_mutex_);
  auto it = s_registry_.find(node_key_);
  if (it == s_registry_.end())
  {
    // First SceneManager for this key: create and initialise SharedState.
    // SharedState::initialize() creates its own rclcpp::Context so it is
    // isolated from the default context's lifecycle (e.g. rclcpp::shutdown()
    // called by RosControl during mj_recompile does not kill our executor).
    auto ss = std::make_shared<SharedState>();
    ss->initialize(node_name, node_namespace);
    s_registry_[node_key_] = ss;
    shared_ = ss;
  }
  else
  {
    // Double-init or post-recompile re-init: reuse existing SharedState.
    shared_ = it->second;
  }

  print_confirm("[SceneManager] Plugin initialised — node: %s/%s\n",
                node_namespace.c_str(), node_name.c_str());
}

SceneManager::~SceneManager()
{
  // SharedState is owned by the static registry — do NOT cancel the executor
  // here.  It must keep running across mj_recompile re-initializations.
}

// ── reset / compute ─────────────────────────────────────────────────────────

void SceneManager::reset(const mjModel * /*m*/, int /*plugin_id*/)
{
  // Clear any pending operations so stale weld activations from before a NaN
  // crash cannot re-fire with wrong eq_data after sim reset → prevents reset loop.
  if (shared_) {
    std::lock_guard<std::mutex> lk(shared_->pending_mutex);
    while (!shared_->pending_ops.empty()) shared_->pending_ops.pop();
    shared_->ops_pending.store(false);
  }
}

void SceneManager::compute(const mjModel * m, mjData * d, int /*plugin_id*/)
{
  // Detect sim reset: time jumped backwards (e.g. NaN recovery, reset_after_run).
  // Deactivate all equality constraints so a stale active weld doesn't produce
  // a rank-deficient Hessian on the first step after reset.
  const double cur_time = d->time;
  if (shared_->last_sim_time >= 0.0 && cur_time < shared_->last_sim_time) {
    for (int i = 0; i < m->neq; i++) {
      d->eq_active[i] = 0;
    }
    RCLCPP_INFO(shared_->node->get_logger(),
      "[SceneManager] Reset detected (%.3f → %.3f) — all equality constraints deactivated",
      shared_->last_sim_time, cur_time);
  }
  shared_->last_sim_time = cur_time;

  if (!shared_->ops_pending.load())
    return;

  // Drain the queue.  Non-recompile ops are applied directly; SPAWN/DESPAWN
  // register a between-step hook (see applyPendingOps for details).
  applyPendingOps(const_cast<mjModel *>(m), d);
}

// ── applyPendingOps ─────────────────────────────────────────────────────────

void SceneManager::applyPendingOps(mjModel * m, mjData * d)
{
  // Save SharedState on the stack.  For SPAWN/DESPAWN we register a
  // between-step hook that calls mj_recompile BETWEEN physics steps.
  // mj_recompile triggers plugin.destroy (delete this) + plugin.init (new
  // instance), so we must not touch any member via `this` after that point.
  auto shared = shared_;

  std::queue<std::shared_ptr<PendingOp>> to_process;
  {
    std::lock_guard<std::mutex> lk(shared->pending_mutex);
    std::swap(to_process, shared->pending_ops);
    shared->ops_pending.store(false);
  }

  while (!to_process.empty())
  {
    auto op = to_process.front();
    to_process.pop();

    // ── SPAWN / DESPAWN: requires mj_recompile ───────────────────────────
    // We CANNOT call mj_recompile from within mj_step: it resets d->arena
    // and corrupts the in-progress step.  Instead we:
    //   1. Prepare the mjSpec in compute() (safe — reads model structure only)
    //   2. Register a between-step hook that the custom simulate binary calls
    //      between physics steps (while sim.mtx is held, no concurrent step)
    //   3. Put any remaining recompile ops back in the queue so the next step
    //      picks them up after the hook fires.
    if (op->type == OpType::RESET_SIMULATION)
    {
      // Like SPAWN/DESPAWN, mj_resetData must fire between steps.
      auto op_captured = op;
      auto shared_cap  = shared;
      mujoco_ros_utils::RegisterBetweenStepHook(
        [op_captured, shared_cap](mjModel * bm, mjData * bd) {
          mj_resetData(bm, bd);
          op_captured->state.store(1, std::memory_order_release);
          print_confirm("[SceneManager] Simulation data reset (mj_resetData).\n");
        });
      // Put remaining ops back so next compute() picks them up
      if (!to_process.empty())
      {
        std::lock_guard<std::mutex> lk(shared->pending_mutex);
        std::queue<std::shared_ptr<PendingOp>> merged;
        while (!to_process.empty()) {
          merged.push(to_process.front()); to_process.pop(); }
        while (!shared->pending_ops.empty()) {
          merged.push(shared->pending_ops.front()); shared->pending_ops.pop(); }
        shared->pending_ops = std::move(merged);
        shared->ops_pending.store(true);
      }
      return;
    }

    if (op->type == OpType::SPAWN || op->type == OpType::DESPAWN)
    {
      std::string spawned_body;
      std::string new_xml;   // merged (spawn) or trimmed (despawn) XML — stored in SharedState after recompile
      std::string prep_err;
      mjSpec * spec = nullptr;

      if (op->type == OpType::SPAWN)
        spec = prepareSpawnSpec(m, *op, shared, spawned_body, new_xml, prep_err);
      else
        spec = prepareDespawnSpec(m, shared, *op, new_xml, prep_err);

      if (!spec)
      {
        op->result_msg = prep_err;
        op->state.store(2, std::memory_order_release);
      }
      else
      {
        // Capture everything by value; `this` is NOT captured (may be freed by mj_recompile).
        const bool   is_spawn   = (op->type == OpType::SPAWN);
        const std::string s_body      = spawned_body;
        const std::string s_attach_to = op->attach_to;
        const std::string s_name      = op->name;
        const std::string s_new_xml   = new_xml;
        auto op_captured   = op;
        auto shared_cap    = shared;

        mujoco_ros_utils::RegisterBetweenStepHook(
          [spec, is_spawn, s_body, s_attach_to, s_name, s_new_xml, op_captured, shared_cap]
          (mjModel * bm, mjData * bd) mutable
          {
            const int rc = mj_recompile(spec, nullptr, bm, bd);
            if (rc != 0)
            {
              const char * err = mjs_getError(spec);
              op_captured->result_msg = std::string("mj_recompile failed: ")
                                        + (err ? err : "(no details)");
              op_captured->state.store(2, std::memory_order_release);
            }
            else
            {
              std::lock_guard<std::mutex> lk(shared_cap->pending_mutex);
              // Update running XML so future spawn/despawn ops see the current model.
              if (!s_new_xml.empty())
                shared_cap->current_model_xml = s_new_xml;

              if (is_spawn)
              {
                shared_cap->spawned_entities[s_name] = {s_name, s_body, s_attach_to};
                op_captured->result_msg = "Spawned: " + s_body;
                print_confirm("[SceneManager] Spawned entity '%s' → body '%s'\n",
                              s_name.c_str(), s_body.c_str());
              }
              else
              {
                shared_cap->spawned_entities.erase(s_name);
                op_captured->result_msg = "Despawned: " + s_name;
                print_confirm("[SceneManager] Despawned entity '%s'\n", s_name.c_str());
              }
              op_captured->state.store(1, std::memory_order_release);
            }
            mj_deleteSpec(spec);
          });

        // Put remaining recompile ops back and set ops_pending so the next
        // compute() call (after the hook fires) picks them up.
        if (!to_process.empty())
        {
          std::lock_guard<std::mutex> lk(shared->pending_mutex);
          // Prepend remaining ops back to front of queue
          std::queue<std::shared_ptr<PendingOp>> merged;
          while (!to_process.empty()) {
            merged.push(to_process.front());
            to_process.pop();
          }
          // Merge with any newly-arriving ops
          while (!shared->pending_ops.empty()) {
            merged.push(shared->pending_ops.front());
            shared->pending_ops.pop();
          }
          shared->pending_ops = std::move(merged);
          shared->ops_pending.store(true);
        }
        return;  // Hook registered; exit compute() — DO NOT signal op here
      }
      continue;
    }

    // ── In-step ops (no recompile needed) ───────────────────────────────
    std::string err;
    bool ok = false;

    if (op->type == OpType::SET_BODY_POSE)
      ok = applySetBodyPose(m, d, *op, err);
    else if (op->type == OpType::SET_GEOM_PROPERTIES)
      ok = applySetGeomProperties(m, d, *op, err);
    else if (op->type == OpType::SET_EQUALITY_ACTIVE)
      ok = applySetEqualityActive(m, d, *op, err);
    else if (op->type == OpType::SET_JOINT_POSITIONS)
      ok = applySetJointPositions(m, d, *op, err);
    else if (op->type == OpType::APPLY_BODY_WRENCH)
      ok = applyBodyWrench(m, d, *op, err);
    else if (op->type == OpType::GET_BODY_POSE)
      ok = applyGetBodyPose(m, d, *op, err);
    else if (op->type == OpType::GET_GEOM_PROPERTIES)
      ok = applyGetGeomProperties(m, *op, err);
    else if (op->type == OpType::GET_JOINT_STATE)
      ok = applyGetJointState(m, d, *op, err);
    else if (op->type == OpType::GET_MODEL_INFO)
      ok = applyGetModelInfo(m, *op);

    op->result_msg = ok
      ? (op->type == OpType::SET_BODY_POSE      ? "Body pose updated: " + op->name
         : op->type == OpType::SET_GEOM_PROPERTIES ? "Geom properties updated: " + op->name
         : op->type == OpType::SET_EQUALITY_ACTIVE ? (std::string("Equality ") + (op->eq_active ? "activated" : "deactivated") + ": " + op->name)
         : op->type == OpType::SET_JOINT_POSITIONS ? "Joint positions updated"
         : op->type == OpType::APPLY_BODY_WRENCH   ? "Wrench applied: " + op->name
         : "ok")
      : err;
    op->state.store(ok ? 1 : 2, std::memory_order_release);
  }
}

// ── prepareSpawnSpec ─────────────────────────────────────────────────────────
// Builds and returns a ready-to-recompile mjSpec for spawn.  The actual
// mj_recompile call happens in the between-step hook registered by applyPendingOps.
//
// Strategy: serialize current model -> XML, inject the child body XML, then
// reparse the merged document. The launched world must already contain any
// global assets referenced by the spawned body.

namespace {

std::string extractWorldbodyContent(const std::string & xml)
{
  static const std::string OPEN = "<worldbody>";
  static const std::string CLOSE = "</worldbody>";
  const size_t start = xml.find(OPEN);
  const size_t end = xml.rfind(CLOSE);
  if (start == std::string::npos || end == std::string::npos || end < start)
    return "";
  return xml.substr(start + OPEN.size(), end - start - OPEN.size());
}

std::vector<std::string> extractSectionContents(
  const std::string & xml, const std::string & section)
{
  const std::string open = "<" + section;
  const std::string close = "</" + section + ">";
  std::vector<std::string> contents;
  size_t search = 0;

  auto find_open = [&](size_t from) {
    size_t position = from;
    while ((position = xml.find(open, position)) != std::string::npos)
    {
      const size_t after_name = position + open.size();
      if (after_name < xml.size() &&
          (xml[after_name] == '>' ||
           std::isspace(static_cast<unsigned char>(xml[after_name]))))
        return position;
      position = after_name;
    }
    return std::string::npos;
  };

  while (search < xml.size())
  {
    const size_t opening = find_open(search);
    if (opening == std::string::npos)
      break;

    const size_t opening_end = xml.find('>', opening + open.size());
    if (opening_end == std::string::npos)
      return {};
    const size_t content_start = opening_end + 1;
    int depth = 1;
    size_t scan = content_start;
    while (depth > 0)
    {
      const size_t next_open = find_open(scan);
      const size_t next_close = xml.find(close, scan);
      if (next_close == std::string::npos)
        return {};

      if (next_open != std::string::npos && next_open < next_close)
      {
        const size_t nested_end = xml.find('>', next_open + open.size());
        if (nested_end == std::string::npos)
          return {};
        ++depth;
        scan = nested_end + 1;
      }
      else
      {
        --depth;
        if (depth == 0)
        {
          contents.push_back(
            xml.substr(content_start, next_close - content_start));
          search = next_close + close.size();
        }
        scan = next_close + close.size();
      }
    }
  }
  return contents;
}

std::string joinSectionContents(
  const std::string & xml, const std::string & section)
{
  std::string result;
  for (const auto & content : extractSectionContents(xml, section))
  {
    if (!result.empty())
      result += "\n";
    result += content;
  }
  return result;
}

std::string extractFirstBodyElement(const std::string & xml)
{
  const size_t body_start = xml.find("<body");
  if (body_start == std::string::npos)
    return "";
  const size_t opening_end = xml.find('>', body_start);
  if (opening_end == std::string::npos)
    return "";
  if (opening_end > body_start && xml[opening_end - 1] == '/')
    return xml.substr(body_start, opening_end - body_start + 1);

  int depth = 1;
  size_t scan = opening_end + 1;
  while (depth > 0)
  {
    const size_t next_open = xml.find("<body", scan);
    const size_t next_close = xml.find("</body>", scan);
    if (next_close == std::string::npos)
      return "";
    if (next_open != std::string::npos && next_open < next_close)
    {
      const size_t next_open_end = xml.find('>', next_open);
      if (next_open_end == std::string::npos)
        return "";
      if (xml[next_open_end - 1] != '/')
        ++depth;
      scan = next_open_end + 1;
    }
    else
    {
      --depth;
      scan = next_close + 7;
    }
  }
  return xml.substr(body_start, scan - body_start);
}

std::string extractElementName(const std::string & element)
{
  const size_t opening_end = element.find('>');
  if (opening_end == std::string::npos)
    return "";
  const std::string opening_tag = element.substr(0, opening_end + 1);
  MAKE_REGEX(name_pattern, std::string(R"(\sname\s*=\s*["']([^"']+)["'])"));
  std::smatch match;
  return std::regex_search(opening_tag, match, name_pattern)
           ? match.str(1)
           : "";
}

std::string applyNamePrefix(const std::string & xml, const std::string & prefix)
{
  std::string result;
  size_t pos = 0;
  while (pos < xml.size())
  {
    const size_t attr = xml.find("name=", pos);
    if (attr == std::string::npos) {
      result += xml.substr(pos);
      break;
    }
    result += xml.substr(pos, attr + 5 - pos);
    pos = attr + 5;
    if (pos >= xml.size())
      break;
    const char quote = xml[pos];
    if (quote != '"' && quote != '\'') {
      result += quote;
      ++pos;
      continue;
    }
    result += quote;
    result += prefix;
    const size_t value_start = ++pos;
    const size_t value_end = xml.find(quote, value_start);
    if (value_end == std::string::npos) {
      result += xml.substr(value_start);
      break;
    }
    result += xml.substr(value_start, value_end - value_start);
    result += quote;
    pos = value_end + 1;
  }
  return result;
}

std::string applyReferencePrefix(
  const std::string & xml, const std::string & prefix)
{
  static const std::vector<std::string> attributes = {
    "body", "body1", "body2", "geom", "geom1", "geom2", "joint",
    "joint1", "joint2", "material", "mesh", "site", "target",
    "tendon", "texture", "objname", "class", "childclass"
  };

  std::string result = xml;
  for (const auto & attribute : attributes)
  {
    const std::string _pat_attr =
      "(\\s" + attribute + "\\s*=\\s*[\"'])([^\"']+)([\"'])";
    MAKE_REGEX(pattern, _pat_attr);
    std::string replaced;
    size_t previous = 0;
    for (std::sregex_iterator it(result.begin(), result.end(), pattern), end;
         it != end; ++it)
    {
      const auto & match = *it;
      replaced.append(result, previous, static_cast<size_t>(match.position()) - previous);
      replaced += match.str(1);
      if (match.str(2).rfind(prefix, 0) != 0)
        replaced += prefix;
      replaced += match.str(2);
      replaced += match.str(3);
      previous = static_cast<size_t>(match.position() + match.length());
    }
    if (previous != 0)
    {
      replaced.append(result, previous, std::string::npos);
      result = std::move(replaced);
    }
  }
  return result;
}

std::string applyPluginConfigReferencePrefix(
  const std::string & xml, const std::string & prefix)
{
  static const std::vector<std::string> reference_keys = {
    "depth_camera_name", "color_camera_name", "camera_name"
  };

  std::string result = xml;
  for (const auto & key : reference_keys)
  {
    const std::string _pat_key =
      "(<config\\s+key\\s*=\\s*[\"'])" + key +
      "([\"']\\s+value\\s*=\\s*[\"'])([^\"']+)([\"'])";
    MAKE_REGEX(pattern, _pat_key);
    std::string replaced;
    size_t previous = 0;
    for (std::sregex_iterator it(result.begin(), result.end(), pattern), end;
         it != end; ++it)
    {
      const auto & match = *it;
      replaced.append(
        result, previous, static_cast<size_t>(match.position()) - previous);
      replaced += match.str(1) + key + match.str(2);
      if (match.str(3).rfind(prefix, 0) != 0)
        replaced += prefix;
      replaced += match.str(3) + match.str(4);
      previous = static_cast<size_t>(match.position() + match.length());
    }
    if (previous != 0)
    {
      replaced.append(result, previous, std::string::npos);
      result = std::move(replaced);
    }
  }
  return result;
}

std::string resolveResourcePaths(
  const std::string & xml, const std::string & source_directory)
{
  if (source_directory.empty())
    return xml;

  const fs::path source_dir(source_directory);
  MAKE_REGEX(file_pattern, std::string(R"((\sfile\s*=\s*["'])([^"']+)(["']))"));
  std::string result;
  size_t previous = 0;
  for (std::sregex_iterator it(xml.begin(), xml.end(), file_pattern), end;
       it != end; ++it)
  {
    const auto & match = *it;
    result.append(
      xml, previous, static_cast<size_t>(match.position()) - previous);
    fs::path resource(match.str(2));
    if (resource.is_relative())
    {
      const fs::path direct = source_dir / resource;
      const fs::path asset = source_dir / "assets" / resource;
      resource = fs::exists(direct) ? direct : asset;
    }
    result += match.str(1) + resource.lexically_normal().string() + match.str(3);
    previous = static_cast<size_t>(match.position() + match.length());
  }
  if (previous == 0)
    return xml;
  result.append(xml, previous, std::string::npos);
  return result;
}

std::string mergeMissingPluginDeclarations(
  const std::string & main_xml, const std::string & child_extension)
{
  MAKE_REGEX(plugin_pattern, std::string(
    R"(<plugin\s+plugin\s*=\s*["']([^"']+)["'][^>]*(?:/>|>[\s\S]*?</plugin>))"));
  std::string result;
  for (std::sregex_iterator it(
         child_extension.begin(), child_extension.end(), plugin_pattern), end;
       it != end; ++it)
  {
    const std::string plugin_name = (*it).str(1);
    const std::string _pat_existing =
      "plugin\\s*=\\s*[\"']" + plugin_name + "[\"']";
    MAKE_REGEX(existing, _pat_existing);
    if (!std::regex_search(main_xml, existing))
      result += (*it).str(0) + "\n";
  }
  return result;
}

bool mergeSection(
  std::string & main_xml, const std::string & section,
  const std::string & content)
{
  if (content.empty())
    return true;

  const std::string close = "</" + section + ">";
  size_t insertion = main_xml.rfind(close);
  if (insertion != std::string::npos)
  {
    main_xml.insert(insertion, "\n" + content + "\n");
    return true;
  }

  insertion = main_xml.rfind("</mujoco>");
  if (insertion == std::string::npos)
    return false;
  main_xml.insert(
    insertion,
    "\n<" + section + ">\n" + content + "\n</" + section + ">\n");
  return true;
}

std::string setFirstBodyPose(const std::string & xml,
                             const double pos[3], const double quat[4])
{
  const size_t start = xml.find("<body");
  const size_t end = start == std::string::npos ? std::string::npos
                                                 : xml.find('>', start);
  if (end == std::string::npos)
    return xml;

  auto format = [](double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.6g", value);
    return std::string(buffer);
  };
  std::string tag = xml.substr(start, end - start);
  MAKE_REGEX(_re_pos,  std::string(R"(\s+pos\s*=\s*["'][^"']*["'])"));
  MAKE_REGEX(_re_quat, std::string(R"(\s+quat\s*=\s*["'][^"']*["'])"));
  tag = std::regex_replace(tag, _re_pos,  "");
  tag = std::regex_replace(tag, _re_quat, "");
  tag += " pos=\"" + format(pos[0]) + " " + format(pos[1]) + " "
         + format(pos[2]) + "\"";
  tag += " quat=\"" + format(quat[0]) + " " + format(quat[1]) + " "
         + format(quat[2]) + " " + format(quat[3]) + "\"";
  return xml.substr(0, start) + tag + xml.substr(end);
}

std::string injectFreeJoint(const std::string & xml)
{
  const size_t body = xml.find("<body");
  const size_t end = body == std::string::npos ? std::string::npos
                                                : xml.find('>', body);
  if (end == std::string::npos)
    return xml;
  return xml.substr(0, end + 1) + "<freejoint/>" + xml.substr(end + 1);
}

}  // namespace

mjSpec * SceneManager::prepareSpawnSpec(mjModel * m, PendingOp & op,
                                         const std::shared_ptr<SharedState> & shared,
                                         std::string & out_body,
                                         std::string & out_merged_xml,
                                         std::string & err_msg)
{
  // Convert URDF → MJCF if needed.
  std::string child_xml = maybeConvertUrdf(op.xml, err_msg);
  if (child_xml.empty())
  {
    err_msg = "URDF/MJCF conversion failed: " + err_msg;
    return nullptr;
  }
  child_xml = resolveResourcePaths(
    wrapMjcf(child_xml), op.source_directory);

  std::string main_xml;
  {
    std::lock_guard<std::mutex> lk(shared->pending_mutex);
    main_xml = shared->current_model_xml;
  }
  if (main_xml.empty())
  {
    char tmpfile[] = "/tmp/mujoco_scene_XXXXXX.xml";
    int fd = mkstemps(tmpfile, 4);
    if (fd < 0) { err_msg = "mkstemps failed"; return nullptr; }
    close(fd);

    char save_err[512] = {};
    if (mj_saveLastXML(tmpfile, m, save_err, sizeof(save_err)) < 0)
    {
      ::unlink(tmpfile);
      err_msg = std::string("mj_saveLastXML failed: ") + save_err;
      return nullptr;
    }

    std::ifstream ifs(tmpfile);
    if (!ifs.is_open()) {
      ::unlink(tmpfile);
      err_msg = "cannot open saved XML temp file";
      return nullptr;
    }
    main_xml.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    ifs.close();
    ::unlink(tmpfile);
  }

  const bool attach_to_world =
    op.attach_to.empty() || op.attach_to == "world";
  if (!attach_to_world &&
      mj_name2id(m, mjOBJ_BODY, op.attach_to.c_str()) < 0)
  {
    err_msg = "attach_to body '" + op.attach_to + "' not found in model";
    return nullptr;
  }

  const std::string prefix = op.name + "/";
  const std::string asset_xml = applyReferencePrefix(
    applyNamePrefix(joinSectionContents(child_xml, "asset"), prefix),
    prefix);
  std::string body_xml =
    extractFirstBodyElement(extractWorldbodyContent(child_xml));
  if (body_xml.empty())
  {
    err_msg = "child XML has no worldbody content";
    return nullptr;
  }
  const std::string first_child_name = extractElementName(body_xml);
  if (first_child_name.empty())
  {
    err_msg = "child XML has no named top-level body";
    return nullptr;
  }
  body_xml = applyReferencePrefix(applyNamePrefix(body_xml, prefix), prefix);
  body_xml = setFirstBodyPose(body_xml, op.pos, op.quat);
  if (op.with_freejoint)
    body_xml = injectFreeJoint(body_xml);

  size_t insertion = std::string::npos;
  if (attach_to_world)
  {
    insertion = main_xml.rfind("</worldbody>");
  }
  else
  {
    insertion = main_xml.find("</" + op.attach_to + ">");
    if (insertion == std::string::npos)
      insertion = main_xml.rfind("</worldbody>");
  }
  if (insertion == std::string::npos)
  {
    err_msg = "current model XML has no worldbody";
    return nullptr;
  }
  main_xml.insert(insertion, "\n" + body_xml + "\n");

  if (!mergeSection(main_xml, "asset", asset_xml))
  {
    err_msg = "current model XML has no mujoco root";
    return nullptr;
  }

  const std::string default_xml = applyReferencePrefix(
    joinSectionContents(child_xml, "default"), prefix);
  if (!mergeSection(main_xml, "default", default_xml))
  {
    err_msg = "failed to merge child default section";
    return nullptr;
  }

  const std::string extension_xml = mergeMissingPluginDeclarations(
    joinSectionContents(main_xml, "extension"),
    joinSectionContents(child_xml, "extension"));
  if (!mergeSection(main_xml, "extension", extension_xml))
  {
    err_msg = "failed to merge child extension section";
    return nullptr;
  }

  for (const std::string section :
       {"actuator", "contact", "equality", "sensor", "tendon"})
  {
    std::string content = joinSectionContents(child_xml, section);
    content = applyNamePrefix(content, prefix);
    content = applyReferencePrefix(content, prefix);
    content = applyPluginConfigReferencePrefix(content, prefix);
    if (!mergeSection(main_xml, section, content))
    {
      err_msg = "failed to merge child " + section + " section";
      return nullptr;
    }
  }

  char merged_err[512] = {};
  mjSpec * spec = mj_parseXMLString(main_xml.c_str(), nullptr,
                                    merged_err, sizeof(merged_err));
  if (!spec)
  {
    err_msg = std::string("merged XML parse error: ") + merged_err;
    return nullptr;
  }

  out_body = prefix + first_child_name;
  out_merged_xml = main_xml;
  return spec;
}

// ── prepareDespawnSpec ───────────────────────────────────────────────────────
// Builds and returns a ready-to-recompile mjSpec for despawn.
// Uses mj_saveLastXML + XML string removal (matching the spawn approach) to
// avoid issues with mj_copyBack not preserving bodies after mj_recompile.

namespace {

// Remove the first body element whose name attribute equals `body_name` from
// an MJCF XML string.  Returns the modified XML, or empty string on failure.
std::string removeBodyFromXml(const std::string & xml, const std::string & body_name)
{
  // Find name="<body_name>" or name='<body_name>' in a <body ... > tag.
  // We search for the body tag, then scan to find the matching </body>.
  std::string result = xml;
  size_t search_pos = 0;

  while (search_pos < result.size())
  {
    size_t body_tag = result.find("<body", search_pos);
    if (body_tag == std::string::npos)
      return result;  // not found (already removed or never existed)

    // Extract the opening tag.
    size_t tag_end = result.find('>', body_tag);
    if (tag_end == std::string::npos)
      break;
    std::string tag = result.substr(body_tag, tag_end - body_tag + 1);

    // Check if this tag has name="body_name" or name='body_name'.
    bool match = false;
    for (char q : {'"', '\''})
    {
      const std::string needle = std::string("name=") + q + body_name + q;
      if (tag.find(needle) != std::string::npos) { match = true; break; }
    }

    if (!match) { search_pos = body_tag + 1; continue; }

    // Found the opening tag.  Now find the MATCHING </body> by tracking depth.
    // Handle self-closing <body ... />.
    if (tag_end > 0 && result[tag_end - 1] == '/')
    {
      result.erase(body_tag, tag_end - body_tag + 1);
      return result;
    }

    int depth = 1;
    size_t scan = tag_end + 1;
    while (depth > 0 && scan < result.size())
    {
      size_t open  = result.find("<body",  scan);
      size_t close = result.find("</body>", scan);
      if (close == std::string::npos) break;
      if (open != std::string::npos && open < close)
      {
        size_t inner_end = result.find('>', open);
        if (inner_end != std::string::npos && result[inner_end - 1] != '/')
          ++depth;
        scan = (inner_end != std::string::npos ? inner_end : open) + 1;
      }
      else
      {
        --depth;
        if (depth == 0)
        {
          size_t end_pos = close + 7;  // len("</body>") == 7
          result.erase(body_tag, end_pos - body_tag);
          return result;
        }
        scan = close + 7;
      }
    }
    break;  // malformed XML
  }

  return result;  // body not found — return unchanged
}

// Remove every child element inside the listed top-level sections whose opening
// tag contains entity_prefix (e.g. "g1/").  Cleans up actuators, sensors,
// tendons, equalities, and contacts that reference the deleted entity's joints.
static std::string removeEntityRefsFromSections(
    const std::string & xml, const std::string & entity_prefix)
{
  static const std::vector<std::string> kSections = {
    "actuator", "sensor", "tendon", "equality", "contact", "asset"
  };
  std::string result = xml;

  for (const auto & section : kSections) {
    const std::string open_sec  = "<" + section;
    const std::string close_sec = "</" + section + ">";
    size_t sec_pos = 0;

    while (true) {
      size_t sec_start = result.find(open_sec, sec_pos);
      if (sec_start == std::string::npos) break;

      size_t sec_tag_end = result.find('>', sec_start);
      if (sec_tag_end == std::string::npos) break;
      if (sec_tag_end > 0 && result[sec_tag_end - 1] == '/') {
        sec_pos = sec_tag_end + 1; continue;  // self-closing section, no children
      }

      size_t sec_end = result.find(close_sec, sec_tag_end);
      if (sec_end == std::string::npos) break;

      size_t pos = sec_tag_end + 1;
      while (pos < sec_end) {
        size_t elem_start = result.find('<', pos);
        if (elem_start == std::string::npos || elem_start >= sec_end) break;

        const char next = result[elem_start + 1];
        if (next == '/' || next == '!') { pos = elem_start + 1; continue; }

        size_t tag_end = result.find('>', elem_start);
        if (tag_end == std::string::npos || tag_end > sec_end) break;

        // Only inspect the opening tag for the entity prefix.
        const std::string tag_content = result.substr(elem_start, tag_end - elem_start + 1);
        if (tag_content.find(entity_prefix) == std::string::npos) {
          pos = tag_end + 1; continue;
        }

        // Determine element extent (self-closing vs. paired tags).
        size_t elem_end;
        if (tag_end > 0 && result[tag_end - 1] == '/') {
          elem_end = tag_end + 1;
        } else {
          size_t name_start = elem_start + 1;
          size_t name_end   = name_start;
          while (name_end < tag_end &&
                 result[name_end] != ' ' && result[name_end] != '>' &&
                 result[name_end] != '\n' && result[name_end] != '\t')
            ++name_end;
          const std::string close_tag = "</" + result.substr(name_start, name_end - name_start) + ">";
          size_t close_pos = result.find(close_tag, tag_end);
          if (close_pos == std::string::npos) { pos = tag_end + 1; continue; }
          elem_end = close_pos + close_tag.size();
        }

        const size_t erased = elem_end - elem_start;
        result.erase(elem_start, erased);
        sec_end -= erased;
        pos = elem_start;
      }
      sec_pos = sec_end;
    }
  }
  return result;
}

} // anonymous namespace (extended)

mjSpec * SceneManager::prepareDespawnSpec(mjModel * m,
                                           const std::shared_ptr<SharedState> & shared,
                                           PendingOp & op,
                                           std::string & out_trimmed_xml,
                                           std::string & err_msg)
{
  SpawnedEntity ent;
  {
    std::lock_guard<std::mutex> lk(shared->pending_mutex);
    auto it = shared->spawned_entities.find(op.name);
    if (it == shared->spawned_entities.end())
    {
      err_msg = "Entity '" + op.name + "' not found";
      return nullptr;
    }
    ent = it->second;
  }

  // Use the running XML snapshot (updated after every spawn recompile).
  // Fall back to mj_saveLastXML only on the very first operation (before any recompile).
  std::string current_xml;
  {
    std::lock_guard<std::mutex> lk(shared->pending_mutex);
    current_xml = shared->current_model_xml;
  }
  if (current_xml.empty())
  {
    char tmpfile[] = "/tmp/mujoco_despawn_XXXXXX.xml";
    int fd = mkstemps(tmpfile, 4);
    if (fd < 0) { err_msg = "mkstemps failed"; return nullptr; }
    close(fd);

    char save_err[512] = {};
    if (mj_saveLastXML(tmpfile, m, save_err, sizeof(save_err)) < 0)
    {
      ::unlink(tmpfile);
      err_msg = std::string("mj_saveLastXML failed: ") + save_err;
      return nullptr;
    }
    std::ifstream ifs(tmpfile);
    if (!ifs.is_open()) { ::unlink(tmpfile); err_msg = "cannot open tmp XML"; return nullptr; }
    current_xml.assign(std::istreambuf_iterator<char>(ifs),
                       std::istreambuf_iterator<char>());
    ifs.close();
    ::unlink(tmpfile);
  }

  // Remove the spawned body from the XML string.
  const std::string & body_name = ent.spawned_body;
  std::string trimmed_xml = removeBodyFromXml(current_xml, body_name);
  if (trimmed_xml == current_xml)
  {
    err_msg = "Cannot find body '" + body_name + "' in current model XML";
    return nullptr;
  }

  // Remove all actuators, sensors, tendons, etc. referencing this entity's joints.
  const std::string entity_prefix = op.name + "/";
  trimmed_xml = removeEntityRefsFromSections(trimmed_xml, entity_prefix);

  // Parse the trimmed XML into a new spec.
  char parse_err[512] = {};
  mjSpec * spec = mj_parseXMLString(trimmed_xml.c_str(), nullptr,
                                    parse_err, sizeof(parse_err));
  if (!spec)
  {
    err_msg = std::string("despawn XML parse error: ") + parse_err;
    return nullptr;
  }

  out_trimmed_xml = trimmed_xml;   // caller stores this in SharedState after recompile
  return spec;
}

// ── applySetBodyPose ─────────────────────────────────────────────────────────

bool SceneManager::applySetBodyPose(mjModel * m, mjData * d, PendingOp & op,
                                     std::string & err_msg)
{
  const int bid = mj_name2id(m, mjOBJ_BODY, op.name.c_str());
  if (bid < 0)
  {
    err_msg = "Body '" + op.name + "' not found in model";
    return false;
  }

  const bool set_pos  = !std::isnan(op.pos[0]) && !std::isnan(op.pos[1]) && !std::isnan(op.pos[2]);
  const bool set_quat = (op.quat[0] != 0.0 || op.quat[1] != 0.0 ||
                         op.quat[2] != 0.0 || op.quat[3] != 0.0);

  // Check if this body has a freejoint as its first joint → dynamic body.
  bool is_free = false;
  int  free_jnt_id   = -1;
  int  free_qposadr  = -1;

  if (m->body_jntnum[bid] > 0)
  {
    const int jnt_start = m->body_jntadr[bid];
    if (m->jnt_type[jnt_start] == mjJNT_FREE)
    {
      is_free       = true;
      free_jnt_id   = jnt_start;
      free_qposadr  = m->jnt_qposadr[jnt_start];
    }
  }

  if (is_free)
  {
    // Dynamic body — write qpos directly.
    if (set_pos)
    {
      if (op.relative)
      {
        d->qpos[free_qposadr + 0] += op.pos[0];
        d->qpos[free_qposadr + 1] += op.pos[1];
        d->qpos[free_qposadr + 2] += op.pos[2];
      }
      else
      {
        d->qpos[free_qposadr + 0] = op.pos[0];
        d->qpos[free_qposadr + 1] = op.pos[1];
        d->qpos[free_qposadr + 2] = op.pos[2];
      }
    }
    if (set_quat)
    {
      // Normalise before writing.
      double norm = std::sqrt(op.quat[0]*op.quat[0] + op.quat[1]*op.quat[1] +
                              op.quat[2]*op.quat[2] + op.quat[3]*op.quat[3]);
      if (norm < 1e-10) { err_msg = "Quaternion norm is zero"; return false; }
      for (int i = 0; i < 4; ++i)
        d->qpos[free_qposadr + 3 + i] = op.quat[i] / norm;
    }
  }
  else
  {
    // Static (welded) body — write model arrays, then propagate kinematics.
    if (set_pos)
    {
      if (op.relative)
      {
        m->body_pos[3*bid + 0] += op.pos[0];
        m->body_pos[3*bid + 1] += op.pos[1];
        m->body_pos[3*bid + 2] += op.pos[2];
      }
      else
      {
        m->body_pos[3*bid + 0] = op.pos[0];
        m->body_pos[3*bid + 1] = op.pos[1];
        m->body_pos[3*bid + 2] = op.pos[2];
      }
    }
    if (set_quat)
    {
      double norm = std::sqrt(op.quat[0]*op.quat[0] + op.quat[1]*op.quat[1] +
                              op.quat[2]*op.quat[2] + op.quat[3]*op.quat[3]);
      if (norm < 1e-10) { err_msg = "Quaternion norm is zero"; return false; }
      for (int i = 0; i < 4; ++i)
        m->body_quat[4*bid + i] = op.quat[i] / norm;
    }
    // Propagate positions to d->xpos / d->xquat so the change is immediately
    // visible even before the next full mj_step.
    mj_kinematics(m, d);
  }

  print_confirm("[SceneManager] Body '%s' pose updated (is_free=%d).\n",
                op.name.c_str(), (int)is_free);
  return true;
}

// ── applySetGeomProperties ───────────────────────────────────────────────────

bool SceneManager::applySetGeomProperties(mjModel * m, mjData * /*d*/, PendingOp & op,
                                           std::string & err_msg)
{
  const int gid = mj_name2id(m, mjOBJ_GEOM, op.name.c_str());
  if (gid < 0)
  {
    err_msg = "Geom '" + op.name + "' not found in model";
    return false;
  }

  // Size: m->geom_size is [ngeom × 3] in row-major order.
  for (int i = 0; i < 3; ++i)
  {
    if (!std::isnan(op.geom_size[i]) && op.geom_size[i] > 0.0)
      m->geom_size[3*gid + i] = op.geom_size[i];
  }

  // RGBA: m->geom_rgba is [ngeom × 4] in row-major order.
  for (int i = 0; i < 4; ++i)
  {
    if (!std::isnan(op.geom_rgba[i]))
    {
      const float v = static_cast<float>(
        std::max(0.0, std::min(1.0, op.geom_rgba[i])));
      m->geom_rgba[4*gid + i] = v;
    }
  }

  print_confirm("[SceneManager] Geom '%s' properties updated.\n", op.name.c_str());
  return true;
}

// ── applySetEqualityActive ───────────────────────────────────────────────────
// Activate or deactivate a named MuJoCo equality constraint (weld, joint, etc.).
// When activating with use_current_pose=true, the current relative pose of the
// two constrained bodies is computed from live simulation state and written into
// m->eq_data so the weld "locks in" the current configuration.
//
// MuJoCo weld eq_data layout (mjNEQDATA = 11 doubles):
//   [0..2]  : relative position of body2 in body1 frame
//   [3..9]  : top two rows of the 3×3 rotation matrix R (body2 relative to body1)
//   [10]    : torquescale (usually 1.0)

bool SceneManager::applySetEqualityActive(mjModel * m, mjData * d, PendingOp & op,
                                           std::string & err_msg)
{
  const int eq_id = mj_name2id(m, mjOBJ_EQUALITY, op.name.c_str());
  if (eq_id < 0)
  {
    err_msg = "Equality constraint '" + op.name + "' not found in model";
    return false;
  }

  if (op.eq_active && op.use_current_pose)
  {
    // Only weld constraints have meaningful body-relative pose data.
    if (m->eq_type[eq_id] != mjEQ_WELD)
    {
      err_msg = "use_current_pose=true is only supported for weld constraints";
      return false;
    }

    const int b1 = m->eq_obj1id[eq_id];
    const int b2 = m->eq_obj2id[eq_id];

    // World-frame positions and orientations (updated by mj_kinematics).
    const mjtNum * p1 = d->xpos  + 3 * b1;
    const mjtNum * p2 = d->xpos  + 3 * b2;
    const mjtNum * q1 = d->xquat + 4 * b1;   // w x y z
    const mjtNum * q2 = d->xquat + 4 * b2;

    // MuJoCo weld eq_data layout (mjNEQDATA = 11):
    //   [0..2]  anchor in body1 frame   (keep as-is, default {0,0,0})
    //   [3..5]  relpos: position of body2 origin in body1 frame
    //   [6..9]  relquat (wxyz): rotation from body2 to body1 (q1_inv * q2)
    //   [10]    torquescale            (keep as-is, default 1.0)

    // Relative position of body2 in body1's frame: R1^T * (p2 - p1)
    mjtNum R1[9], R1T[9];
    mju_quat2Mat(R1, q1);
    mju_transpose(R1T, R1, 3, 3);

    mjtNum dp[3] = { p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2] };
    mjtNum rel_pos[3];
    mju_mulMatVec(rel_pos, R1T, dp, 3, 3);

    // Relative quaternion: q_rel = q1_inv * q2
    mjtNum q1_inv[4] = { q1[0], -q1[1], -q1[2], -q1[3] };
    mjtNum q_rel[4];
    mju_mulQuat(q_rel, q1_inv, q2);

    // Normalize q_rel to guard against floating-point drift
    mju_normalize4(q_rel);

    // Write relpos into [3..5], relquat into [6..9]; leave anchor [0..2] and torquescale [10] intact
    mjtNum * eqdata = m->eq_data + mjNEQDATA * eq_id;
    eqdata[3] = rel_pos[0];
    eqdata[4] = rel_pos[1];
    eqdata[5] = rel_pos[2];
    eqdata[6] = q_rel[0];
    eqdata[7] = q_rel[1];
    eqdata[8] = q_rel[2];
    eqdata[9] = q_rel[3];

    // Match body2's freejoint velocity to body1's so the initial constraint velocity
    // residual is zero — prevents constraint impulse spike on the first solve step.
    // body1 velocity: d->cvel[6*b1 + 0..2] = angular, [3..5] = linear (world frame)
    const int j2 = m->body_jntadr[b2];
    if (j2 >= 0 && m->jnt_type[j2] == mjJNT_FREE)
    {
      const int qvel_adr = m->jnt_dofadr[j2];   // 6 dofs: [ang0..2, lin0..2]
      // body1 cvel in world frame: [ang_x, ang_y, ang_z, lin_x, lin_y, lin_z]
      const mjtNum * cv1 = d->cvel + 6 * b1;
      // angular (world) → freejoint angular dofs 0..2
      d->qvel[qvel_adr + 0] = cv1[0];
      d->qvel[qvel_adr + 1] = cv1[1];
      d->qvel[qvel_adr + 2] = cv1[2];
      // linear (world) → freejoint linear dofs 3..5
      d->qvel[qvel_adr + 3] = cv1[3];
      d->qvel[qvel_adr + 4] = cv1[4];
      d->qvel[qvel_adr + 5] = cv1[5];
      RCLCPP_INFO(shared_->node->get_logger(),
        "[WeldDebug] velocity-matched: ang=(%.3f,%.3f,%.3f) lin=(%.3f,%.3f,%.3f)",
        cv1[0], cv1[1], cv1[2], cv1[3], cv1[4], cv1[5]);
    }

    RCLCPP_INFO(shared_->node->get_logger(),
      "[WeldDebug] b1=%s p1=(%.4f,%.4f,%.4f) q1=(%.4f,%.4f,%.4f,%.4f)",
      m->names + m->name_bodyadr[b1], p1[0], p1[1], p1[2], q1[0], q1[1], q1[2], q1[3]);
    RCLCPP_INFO(shared_->node->get_logger(),
      "[WeldDebug] b2=%s p2=(%.4f,%.4f,%.4f) q2=(%.4f,%.4f,%.4f,%.4f)",
      m->names + m->name_bodyadr[b2], p2[0], p2[1], p2[2], q2[0], q2[1], q2[2], q2[3]);
    RCLCPP_INFO(shared_->node->get_logger(),
      "[WeldDebug] eq_data[3..9]=(%.4f,%.4f,%.4f | %.4f,%.4f,%.4f,%.4f)",
      rel_pos[0], rel_pos[1], rel_pos[2], q_rel[0], q_rel[1], q_rel[2], q_rel[3]);
  }

  // When deactivating a weld, zero body2's freejoint velocity so the released object
  // starts from rest rather than flying away at arm velocity, preventing impulse NaN.
  if (!op.eq_active && m->eq_type[eq_id] == mjEQ_WELD)
  {
    const int b2 = m->eq_obj2id[eq_id];
    const int j2 = m->body_jntadr[b2];
    if (j2 >= 0 && m->jnt_type[j2] == mjJNT_FREE)
    {
      const int qvel_adr = m->jnt_dofadr[j2];
      const mjtNum * cv2 = d->cvel + 6 * b2;
      RCLCPP_INFO(shared_->node->get_logger(),
        "[WeldDebug] deactivate: zeroing body2 vel ang=(%.3f,%.3f,%.3f) lin=(%.3f,%.3f,%.3f)",
        cv2[0], cv2[1], cv2[2], cv2[3], cv2[4], cv2[5]);
      for (int k = 0; k < 6; k++) d->qvel[qvel_adr + k] = 0.0;
    }
  }

  d->eq_active[eq_id] = op.eq_active ? 1 : 0;

  print_confirm("[SceneManager] Equality '%s' %s.\n",
                op.name.c_str(), op.eq_active ? "activated" : "deactivated");
  return true;
}

bool SceneManager::applyGetBodyPose(const mjModel * m, const mjData * d, PendingOp & op,
                                     std::string & err_msg)
{
  const int bid = mj_name2id(m, mjOBJ_BODY, op.name.c_str());
  if (bid < 0)
  {
    err_msg = "Body '" + op.name + "' not found in model";
    return false;
  }

  auto & r    = *op.body_pose_data;
  // World-frame position and orientation (updated after mj_kinematics)
  r.pos[0]  = d->xpos [3*bid + 0];
  r.pos[1]  = d->xpos [3*bid + 1];
  r.pos[2]  = d->xpos [3*bid + 2];
  r.quat[0] = d->xquat[4*bid + 0];   // w
  r.quat[1] = d->xquat[4*bid + 1];   // x
  r.quat[2] = d->xquat[4*bid + 2];   // y
  r.quat[3] = d->xquat[4*bid + 3];   // z

  // World-frame velocity: cvel[0..2] angular, cvel[3..5] linear
  r.angular_vel[0] = d->cvel[6*bid + 0];
  r.angular_vel[1] = d->cvel[6*bid + 1];
  r.angular_vel[2] = d->cvel[6*bid + 2];
  r.linear_vel[0]  = d->cvel[6*bid + 3];
  r.linear_vel[1]  = d->cvel[6*bid + 4];
  r.linear_vel[2]  = d->cvel[6*bid + 5];

  // Parent body name
  const int    parent_id   = m->body_parentid[bid];
  const char * parent_name = mj_id2name(m, mjOBJ_BODY, parent_id);
  r.parent_body = parent_name ? parent_name : "";

  // Dynamic = has a freejoint as its first DOF
  r.is_dynamic = false;
  if (m->body_jntnum[bid] > 0)
  {
    const int jnt = m->body_jntadr[bid];
    r.is_dynamic  = (m->jnt_type[jnt] == mjJNT_FREE);
  }

  return true;
}

// ── applyGetGeomProperties ───────────────────────────────────────────────────

bool SceneManager::applyGetGeomProperties(const mjModel * m, PendingOp & op,
                                           std::string & err_msg)
{
  const int gid = mj_name2id(m, mjOBJ_GEOM, op.name.c_str());
  if (gid < 0)
  {
    err_msg = "Geom '" + op.name + "' not found in model";
    return false;
  }

  auto & r   = *op.geom_props_data;
  r.size[0]  = m->geom_size[3*gid + 0];
  r.size[1]  = m->geom_size[3*gid + 1];
  r.size[2]  = m->geom_size[3*gid + 2];
  r.rgba[0]  = m->geom_rgba[4*gid + 0];
  r.rgba[1]  = m->geom_rgba[4*gid + 1];
  r.rgba[2]  = m->geom_rgba[4*gid + 2];
  r.rgba[3]  = m->geom_rgba[4*gid + 3];
  r.type     = m->geom_type[gid];

  const int    body_id   = m->geom_bodyid[gid];
  const char * body_name = mj_id2name(m, mjOBJ_BODY, body_id);
  r.parent_body = body_name ? body_name : "";

  return true;
}

// ── applyGetModelInfo ────────────────────────────────────────────────────────

bool SceneManager::applyGetModelInfo(const mjModel * m, PendingOp & op)
{
  const std::string & prefix = op.name;   // filter prefix (may be empty)
  auto & r = *op.model_info_data;

  r.nbody = m->nbody;
  r.ngeom = m->ngeom;
  r.njnt  = m->njnt;
  r.nu    = m->nu;

  auto matches = [&](const char * n) -> bool {
    return prefix.empty() || (n && std::string(n).rfind(prefix, 0) == 0);
  };

  for (int i = 0; i < m->nbody; ++i)
  {
    const char * n = mj_id2name(m, mjOBJ_BODY, i);
    if (matches(n)) r.bodies.emplace_back(n ? n : "(unnamed)");
  }
  for (int i = 0; i < m->ngeom; ++i)
  {
    const char * n = mj_id2name(m, mjOBJ_GEOM, i);
    if (matches(n)) r.geoms.emplace_back(n ? n : "(unnamed)");
  }
  for (int i = 0; i < m->njnt; ++i)
  {
    const char * n = mj_id2name(m, mjOBJ_JOINT, i);
    if (matches(n)) r.joints.emplace_back(n ? n : "(unnamed)");
  }
  for (int i = 0; i < m->nu; ++i)
  {
    const char * n = mj_id2name(m, mjOBJ_ACTUATOR, i);
    if (matches(n)) r.actuators.emplace_back(n ? n : "(unnamed)");
  }

  return true;
}

// ── applySetJointPositions ──────────────────────────────────────────────────

bool SceneManager::applySetJointPositions(mjModel * m, mjData * d, PendingOp & op,
                                           std::string & err_msg)
{
  for (size_t i = 0; i < op.joint_names.size(); ++i)
  {
    const int jid = mj_name2id(m, mjOBJ_JOINT, op.joint_names[i].c_str());
    if (jid < 0)
    {
      err_msg = "Joint '" + op.joint_names[i] + "' not found in model";
      return false;
    }

    // Only support 1-DOF joints (hinge, slide) for position teleport.
    // Ball and free joints have multi-DOF qpos — use SetBodyPose instead.
    const int nq = (m->jnt_type[jid] == mjJNT_FREE) ? 7
                 : (m->jnt_type[jid] == mjJNT_BALL) ? 4 : 1;
    if (nq != 1)
    {
      err_msg = "Joint '" + op.joint_names[i] + "' has " + std::to_string(nq)
              + " position coordinates. Use SetBodyPose for multi-DOF joints.";
      return false;
    }

    const int qpos_adr = m->jnt_qposadr[jid];
    d->qpos[qpos_adr] = op.joint_positions[i];
  }

  print_confirm("[SceneManager] %zu joint positions updated.\n", op.joint_names.size());
  return true;
}

// ── applyBodyWrench ─────────────────────────────────────────────────────────

bool SceneManager::applyBodyWrench(mjModel * m, mjData * d, PendingOp & op,
                                    std::string & err_msg)
{
  const int bid = mj_name2id(m, mjOBJ_BODY, op.name.c_str());
  if (bid < 0)
  {
    err_msg = "Body '" + op.name + "' not found in model";
    return false;
  }

  // d->xfrc_applied layout (body-local frame):
  //   [0..2] torque,  [3..5] force
  mjtNum * xfrc = d->xfrc_applied + 6 * bid;

  if (op.reference_frame == "world")
  {
    // Rotate world-frame force/torque into body-local frame.
    // R_body_to_world is stored in d->xmat[9*bid] in column-major order.
    mjtNum R[9], RT[9];
    mju_quat2Mat(R, d->xquat + 4 * bid);
    mju_transpose(RT, R, 3, 3);

    mjtNum force_body[3], torque_body[3];
    mjtNum f_world[3] = {op.force[0],  op.force[1],  op.force[2]};
    mjtNum t_world[3] = {op.torque[0], op.torque[1], op.torque[2]};
    mju_mulMatVec(torque_body, RT, t_world, 3, 3);
    mju_mulMatVec(force_body,  RT, f_world, 3, 3);

    xfrc[0] = torque_body[0]; xfrc[1] = torque_body[1]; xfrc[2] = torque_body[2];
    xfrc[3] = force_body[0];  xfrc[4] = force_body[1];  xfrc[5] = force_body[2];
  }
  else  // "body" frame — write directly
  {
    xfrc[0] = op.torque[0]; xfrc[1] = op.torque[1]; xfrc[2] = op.torque[2];
    xfrc[3] = op.force[0];  xfrc[4] = op.force[1];  xfrc[5] = op.force[2];
  }

  print_confirm("[SceneManager] Wrench applied to body '%s' (frame=%s): "
                "force=(%.2f,%.2f,%.2f) torque=(%.2f,%.2f,%.2f).\n",
                op.name.c_str(), op.reference_frame.c_str(),
                op.force[0], op.force[1], op.force[2],
                op.torque[0], op.torque[1], op.torque[2]);
  return true;
}

// ── applyGetJointState ──────────────────────────────────────────────────────

bool SceneManager::applyGetJointState(const mjModel * m, const mjData * d,
                                       PendingOp & op, std::string & err_msg)
{
  auto & r = *op.joint_state_data;

  // Build a set of names to query, or query all if none specified.
  std::unordered_set<std::string> name_filter;
  bool filter_all = op.joint_names.empty();
  if (!filter_all)
  {
    for (const auto & n : op.joint_names)
      name_filter.insert(n);
  }

  for (int j = 0; j < m->njnt; ++j)
  {
    const char * name = mj_id2name(m, mjOBJ_JOINT, j);
    const std::string jname = name ? name : "(unnamed)";

    if (!filter_all && !name_filter.count(jname))
      continue;

    const int nq = (m->jnt_type[j] == mjJNT_FREE)   ? 7
                 : (m->jnt_type[j] == mjJNT_BALL)   ? 4 : 1;
    const int nv = (m->jnt_type[j] == mjJNT_FREE)   ? 6
                 : (m->jnt_type[j] == mjJNT_BALL)   ? 3 : 1;

    const int qpos_adr = m->jnt_qposadr[j];
    const int dof_adr  = m->jnt_dofadr[j];

    // Position
    int32_t pos_start = static_cast<int32_t>(r.positions.size());
    for (int k = 0; k < nq; ++k)
      r.positions.push_back(d->qpos[qpos_adr + k]);

    // Velocity
    int32_t vel_start = static_cast<int32_t>(r.velocities.size());
    for (int k = 0; k < nv; ++k)
      r.velocities.push_back(d->qvel[dof_adr + k]);

    // Effort: actuator force in joint space from qfrc_actuator
    int32_t eff_start = static_cast<int32_t>(r.efforts.size());
    for (int k = 0; k < nv; ++k)
      r.efforts.push_back(d->qfrc_actuator[dof_adr + k]);

    r.joint_names.push_back(jname);
    r.pos_starts.push_back(pos_start);
    r.pos_lengths.push_back(nq);
    r.vel_starts.push_back(vel_start);
    r.vel_lengths.push_back(nv);
  }

  return true;
}

// ── geomTypeToString ─────────────────────────────────────────────────────────

std::string SceneManager::geomTypeToString(int type)
{
  switch (type)
  {
    case mjGEOM_PLANE:     return "plane";
    case mjGEOM_HFIELD:    return "hfield";
    case mjGEOM_SPHERE:    return "sphere";
    case mjGEOM_CAPSULE:   return "capsule";
    case mjGEOM_ELLIPSOID: return "ellipsoid";
    case mjGEOM_CYLINDER:  return "cylinder";
    case mjGEOM_BOX:       return "box";
    case mjGEOM_MESH:      return "mesh";
    default:               return "unknown(" + std::to_string(type) + ")";
  }
}

// ── Static helpers ────────────────────────────────────────────────────────────

std::string SceneManager::maybeConvertUrdf(const std::string & xml, std::string & err_msg)
{
  // Check if this looks like URDF (starts with <robot).
  auto trimmed = xml;
  const auto first = trimmed.find_first_not_of(" \t\n\r");
  if (first == std::string::npos || trimmed[first] != '<')
    return xml;  // not XML at all — pass through

  const auto tag_start = trimmed.find('<', first);
  if (tag_start == std::string::npos)
    return xml;

  // Check if tag is <robot
  if (trimmed.compare(tag_start + 1, 5, "robot") != 0)
    return xml;  // not URDF — pass through

  // urdf2mjcf writes a model directory containing both XML and converted
  // assets. Use a unique location so concurrent spawns cannot overwrite each
  // other and keep it alive for the lifetime of the running MuJoCo model.
  char temp_template[] = "/tmp/scene_manager_spawn_XXXXXX";
  const char * temp_path = mkdtemp(temp_template);
  if (!temp_path)
  {
    err_msg = "mkdtemp failed for URDF conversion";
    return {};
  }
  const fs::path temp_dir(temp_path);
  const fs::path tmp_urdf = temp_dir / "entity.urdf";
  const fs::path output_dir = temp_dir / "output";
  {
    std::ofstream f(tmp_urdf);
    f << xml;
  }

  const std::string command =
    "urdf2mjcf '" + tmp_urdf.string() + "' -o '" + output_dir.string() +
    "' --no-clock-publisher --log-level ERROR >'" +
    (temp_dir / "urdf2mjcf.log").string() + "' 2>&1";
  int rc = std::system(command.c_str());
  if (rc != 0)
  {
    const std::string fallback =
      "python3 -m urdf2mjcf '" + tmp_urdf.string() + "' -o '" +
      output_dir.string() + "' --no-clock-publisher --log-level ERROR >'" +
      (temp_dir / "urdf2mjcf.log").string() + "' 2>&1";
    rc = std::system(fallback.c_str());
  }
  if (rc != 0)
  {
    err_msg = "urdf2mjcf failed (exit " + std::to_string(rc) +
              "). See " + (temp_dir / "urdf2mjcf.log").string();
    return {};
  }

  const fs::path model_dir = output_dir / "entity";
  const fs::path tmp_mjcf = model_dir / "entity.xml";
  std::ifstream f(tmp_mjcf);
  if (!f)
  {
    err_msg = "urdf2mjcf did not produce output at " + tmp_mjcf.string();
    return {};
  }
  std::string mjcf((std::istreambuf_iterator<char>(f)), {});

  // String-based mjSpec parsing has no source filename from which to resolve
  // resource paths. Make converted asset paths absolute before merging them
  // into the running model.
  MAKE_REGEX(file_pattern, std::string(R"((\sfile\s*=\s*["'])([^"']+)(["']))"));
  std::string resolved;
  size_t previous = 0;
  for (std::sregex_iterator it(mjcf.begin(), mjcf.end(), file_pattern), end;
       it != end; ++it)
  {
    const auto & match = *it;
    resolved.append(mjcf, previous, static_cast<size_t>(match.position()) - previous);
    fs::path resource(match.str(2));
    if (resource.is_relative())
    {
      const fs::path asset_resource = model_dir / "assets" / resource;
      resource = fs::exists(asset_resource)
                   ? asset_resource
                   : model_dir / resource;
    }
    resolved += match.str(1) + resource.string() + match.str(3);
    previous = static_cast<size_t>(match.position() + match.length());
  }
  if (previous != 0)
  {
    resolved.append(mjcf, previous, std::string::npos);
    mjcf = std::move(resolved);
  }
  return mjcf;
}

std::string SceneManager::wrapMjcf(const std::string & xml)
{
  // XML declarations and comments may precede the root element.
  if (xml.find("<mujoco") != std::string::npos)
    return xml;

  // Otherwise, wrap in a minimal <mujoco><worldbody> shell.
  return "<mujoco>\n  <worldbody>\n" + xml + "\n  </worldbody>\n</mujoco>\n";
}

}  // namespace MujocoRosUtils
