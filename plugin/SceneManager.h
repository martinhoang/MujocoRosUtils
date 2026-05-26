#pragma once

#include "mujoco_ros_utils/srv/spawn_entity.hpp"
#include "mujoco_ros_utils/srv/despawn_entity.hpp"
#include "mujoco_ros_utils/srv/list_entities.hpp"
#include "mujoco_ros_utils/srv/set_body_pose.hpp"
#include "mujoco_ros_utils/srv/set_geom_properties.hpp"
#include "mujoco_ros_utils/srv/get_body_pose.hpp"
#include "mujoco_ros_utils/srv/get_geom_properties.hpp"
#include "mujoco_ros_utils/srv/get_model_info.hpp"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace MujocoRosUtils
{

/**
 * SceneManager — MuJoCo passive plugin
 *
 * Provides ROS 2 services to dynamically add and remove robots or free bodies
 * from a running simulation.  Modifications use the MuJoCo spec API:
 *
 *   mj_copyBack  → snapshot live model to spec
 *   mj_parseXMLString → parse new entity MJCF/URDF
 *   mjs_attach   → graft entity onto target body
 *   mjs_delete   → remove entity subtree from spec
 *   mj_recompile → rebuild model in-place, preserving qpos / qvel / ctrl
 *
 * Threading model
 * ---------------
 * Service callbacks (on the ROS executor thread) push `PendingOp` structs into
 * a queue and block on a `std::future` for the result.  The simulation thread
 * (via `compute()`) drains the queue and applies the spec modifications between
 * simulation sub-steps.
 *
 * Double-init and mj_recompile safety
 * ------------------------------------
 * MuJoCo may initialize plugins more than once (double-init at startup, and
 * again after each mj_recompile call from applySpawn/applyDespawn).  All
 * SceneManager instances sharing the same node key point to a single
 * SharedState which owns the ROS2 node, executor, and service handles.
 * SharedState is kept alive by a strong reference in a static registry, so it
 * survives the destruction and re-creation of individual plugin instances.
 *
 * Plugin XML example
 * ------------------
 *   <extension>
 *     <plugin plugin="MujocoRosUtils::SceneManager">
 *       <instance name="scene_manager">
 *         <config key="node_name" value="scene_manager"/>
 *         <config key="namespace" value=""/>
 *       </instance>
 *     </plugin>
 *   </extension>
 */
class SceneManager
{
public:
  static void RegisterPlugin();
  static SceneManager * Create(const mjModel * m, mjData * d, int plugin_id);

  SceneManager(SceneManager &&) = default;
  ~SceneManager();

  void reset(const mjModel * m, int plugin_id);
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  SceneManager(std::string node_name, std::string node_namespace);

private:
  // ── Pending operation ────────────────────────────────────────────────────
  struct SpawnedEntity
  {
    std::string user_name;
    std::string spawned_body;  ///< actual MuJoCo body name after prefix
    std::string attach_to;
  };

  enum class OpType {
    SPAWN, DESPAWN,
    SET_BODY_POSE, SET_GEOM_PROPERTIES,
    GET_BODY_POSE, GET_GEOM_PROPERTIES, GET_MODEL_INFO
  };

  // ── Result structs for GET ops (filled by sim thread) ────────────────────
  struct BodyPoseData
  {
    double pos[3]  = {};
    double quat[4] = {1, 0, 0, 0};   ///< w x y z
    std::string parent_body;
    bool is_dynamic = false;
  };

  struct GeomPropsData
  {
    double size[3] = {};
    float  rgba[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    int    type    = 0;   ///< mjGeomType enum value
    std::string parent_body;
  };

  struct ModelInfoData
  {
    std::vector<std::string> bodies;
    std::vector<std::string> geoms;
    std::vector<std::string> joints;
    std::vector<std::string> actuators;
    int nbody = 0, ngeom = 0, njnt = 0, nu = 0;
  };

  struct PendingOp
  {
    OpType      type;
    std::string name;        ///< user label (SPAWN/DESPAWN) or body/geom name (SET_/GET_*)
    std::string xml;         ///< MJCF string for spawn
    std::string attach_to;   ///< target body name (spawn)
    double      pos[3]   = {0, 0, 0};
    double      quat[4]  = {1, 0, 0, 0};  ///< w x y z
    bool        with_freejoint = false;
    bool        relative       = false;    ///< SET_BODY_POSE: delta from current

    // SET_GEOM_PROPERTIES — NaN means "keep current"
    double geom_size[3] = {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN()};
    double geom_rgba[4] = {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN()};

    // GET op results — allocated by service callback, filled by sim thread
    std::shared_ptr<BodyPoseData>   body_pose_data;
    std::shared_ptr<GeomPropsData>  geom_props_data;
    std::shared_ptr<ModelInfoData>  model_info_data;

    // Result: written by sim thread, polled by service callback.
    // state: 0=pending, 1=success, 2=failed
    std::atomic<int> state{0};
    std::string      result_msg;
  };

  // ── Shared state ─────────────────────────────────────────────────────────
  // Owned by the static registry (strong ref → lives for the process lifetime
  // once created).  All SceneManager instances for the same node key share the
  // same SharedState.  This means the ROS node, executor, and service handles
  // survive mj_recompile (which destroys the current plugin instance and
  // creates a new one).
  struct SharedState
  {
    // ── op queue / entity map ─────────────────────────────────────────────
    std::mutex                               pending_mutex;
    std::queue<std::shared_ptr<PendingOp>>   pending_ops;
    std::atomic<bool>                        ops_pending{false};
    std::unordered_map<std::string, SpawnedEntity> spawned_entities;

    // Running XML snapshot — seeded from mj_saveLastXML on first spawn,
    // then updated after every successful recompile.
    std::string                              current_model_xml;

    // ── ROS2 (created once, never destroyed until process exit) ──────────
    rclcpp::Node::SharedPtr                                     node;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>   executor;
    std::thread                                                 executor_thread;
    std::atomic<bool>                                           stop_executor{false};

    // Service handles — kept alive by this struct
    rclcpp::Service<mujoco_ros_utils::srv::SpawnEntity>::SharedPtr         spawn_srv;
    rclcpp::Service<mujoco_ros_utils::srv::DespawnEntity>::SharedPtr       despawn_srv;
    rclcpp::Service<mujoco_ros_utils::srv::ListEntities>::SharedPtr        list_srv;
    rclcpp::Service<mujoco_ros_utils::srv::SetBodyPose>::SharedPtr         set_body_pose_srv;
    rclcpp::Service<mujoco_ros_utils::srv::SetGeomProperties>::SharedPtr   set_geom_props_srv;
    rclcpp::Service<mujoco_ros_utils::srv::GetBodyPose>::SharedPtr         get_body_pose_srv;
    rclcpp::Service<mujoco_ros_utils::srv::GetGeomProperties>::SharedPtr   get_geom_props_srv;
    rclcpp::Service<mujoco_ros_utils::srv::GetModelInfo>::SharedPtr        get_model_info_srv;

    /// Create the ROS2 node, register all services, and start the executor thread.
    /// Called exactly once per SharedState instance.
    void initialize(const std::string & node_name, const std::string & node_namespace);

    ~SharedState();
  };

  // Registry: strong shared_ptr so SharedState outlives individual plugin instances.
  static std::mutex                                                    s_registry_mutex_;
  static std::unordered_map<std::string, std::shared_ptr<SharedState>> s_registry_;

  std::shared_ptr<SharedState> shared_;   ///< points to the per-key shared state
  std::string                  node_key_; ///< "namespace/node_name"

  // ── Helpers ──────────────────────────────────────────────────────────────
  void applyPendingOps(mjModel * m, mjData * d);

  // Prepare a spawn spec (mj_copyBack + mjs_attach) without recompiling.
  // Returns a heap-allocated mjSpec* (caller must mj_deleteSpec), or nullptr on error.
  static mjSpec * prepareSpawnSpec(mjModel * m, PendingOp & op,
                                   const std::shared_ptr<SharedState> & shared,
                                   std::string & out_body_name,
                                   std::string & out_merged_xml,
                                   std::string & err_msg);

  // Prepare a despawn spec using XML string removal (no mj_copyBack).
  // Returns a heap-allocated mjSpec* (caller must mj_deleteSpec), or nullptr on error.
  // Also writes the trimmed XML to out_trimmed_xml for SharedState update.
  static mjSpec * prepareDespawnSpec(mjModel * m,
                                     const std::shared_ptr<SharedState> & shared,
                                     PendingOp & op,
                                     std::string & out_trimmed_xml,
                                     std::string & err_msg);

  bool applySetBodyPose(mjModel * m, mjData * d, PendingOp & op, std::string & err_msg);
  bool applySetGeomProperties(mjModel * m, mjData * d, PendingOp & op, std::string & err_msg);
  bool applyGetBodyPose(const mjModel * m, const mjData * d, PendingOp & op, std::string & err_msg);
  bool applyGetGeomProperties(const mjModel * m, PendingOp & op, std::string & err_msg);
  bool applyGetModelInfo(const mjModel * m, PendingOp & op);

  /// Convert a URDF/Xacro string to MJCF using urdf2mjcf, if the input looks
  /// like URDF (starts with "<robot").  Returns the XML unchanged otherwise.
  static std::string maybeConvertUrdf(const std::string & xml, std::string & err_msg);

  /// Wrap the given MJCF fragment in a <mujoco><worldbody>…</worldbody></mujoco>
  /// shell if it is not already a full MJCF document.
  static std::string wrapMjcf(const std::string & xml);

  /// Convert a mjGeomType integer to a human-readable string.
  static std::string geomTypeToString(int type);
};

}  // namespace MujocoRosUtils
