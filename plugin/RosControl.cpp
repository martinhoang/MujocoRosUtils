#include "RosControl.hpp"
#include "mujoco_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <mujoco/mujoco.h>

constexpr char ATTR_NODE_NAME[]        = "node_name";
constexpr char ATTR_PUBLISH_RATE[]     = "publish_rate";
constexpr char ATTR_ROBOT_PARAM_NODE[] = "robot_param_node";
constexpr char ATTR_CONFIG_FILE[]      = "config_file";
constexpr char ATTR_PARAMETERS[]       = "parameters";

using namespace std::chrono_literals;

// ResourceManager subclass that overrides load_and_initialize_components() so that
// the ControllerManager's robot_description topic callback loads MuJoCo hardware
// (with access to mjModel*/mjData*) instead of trying to instantiate it via pluginlib
// from the URDF — which would fail and then re-register it as a duplicate.
// This mirrors the pattern used by gz_ros2_control's GZResourceManager.
class MujocoResourceManager : public hardware_interface::ResourceManager
{
public:
  MujocoResourceManager(
    rclcpp::Node::SharedPtr node,
    const mjModel * model,
    mjData * data,
    std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>> loader)
  : hardware_interface::ResourceManager(node->get_clock(), node->get_logger())
  , node_(node), model_(model), data_(data), mujoco_system_loader_(loader)
  {}

  // Called by ControllerManager when it receives the /robot_description topic.
  bool load_and_initialize_components(
    const hardware_interface::ResourceManagerParams & params) override
  {
    components_are_loaded_and_initialized_ = true;

    std::vector<hardware_interface::HardwareInfo> control_hardware_info;
    try
    {
      control_hardware_info =
        hardware_interface::parse_control_resources_from_urdf(params.robot_description);
    }
    catch (const std::runtime_error & ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse control hardware info: %s", ex.what());
      components_are_loaded_and_initialized_ = false;
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Got the control hardware info from URDF with length: %zu",
                control_hardware_info.size());

    for (const auto & hardware_info : control_hardware_info)
    {
      RCLCPP_INFO(node_->get_logger(), "Hardware system: %s",
                  hardware_info.hardware_plugin_name.c_str());

      std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system;
      try
      {
        mujoco_system.reset(
          mujoco_system_loader_->createUnmanagedInstance(hardware_info.hardware_plugin_name));
      }
      catch (pluginlib::PluginlibException & ex)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to create MujocoSystem instance for '%s': %s",
                     hardware_info.name.c_str(), ex.what());
        components_are_loaded_and_initialized_ = false;
        return false;
      }

      if (!mujoco_system->initialize(node_, model_, data_, hardware_info))
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MujocoSystem for '%s'",
                     hardware_info.name.c_str());
        components_are_loaded_and_initialized_ = false;
        return false;
      }
      print_confirm("MujocoSystem initialized successfully for '%s'\n",
                    hardware_info.name.c_str());

      hardware_interface::HardwareComponentParams hw_params;
      hw_params.hardware_info  = hardware_info;
      hw_params.clock          = params.clock;
      hw_params.logger         = params.logger;
      hw_params.node_namespace = params.node_namespace;
      hw_params.executor       = params.executor;
      import_component(std::move(mujoco_system), hw_params);
    }

    return components_are_loaded_and_initialized_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  const mjModel * model_;
  mjData * data_;
  std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>>
    mujoco_system_loader_;
};

namespace MujocoRosUtils
{

void Ros2Control::RegisterPlugin()
{
  // Register the plugin with MuJoCo
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  RCLCPP_INFO(rclcpp::get_logger("RosControl"), "Registering Ros2Control plugin\n");

  plugin.name = "MujocoRosUtils::Ros2Control";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  std::vector<const char *> attributes
    = {ATTR_NODE_NAME, ATTR_PUBLISH_RATE, ATTR_ROBOT_PARAM_NODE, ATTR_CONFIG_FILE, ATTR_PARAMETERS};

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  plugin.nstate = +[](const mjModel *, // m
                      int              // plugin_id
                   ) {
    return 0;
  };

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    auto plugin_instance = Ros2Control::Create(m, d, plugin_id);
    if (!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance.release());
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<Ros2Control *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel *m, double *plugin_state, void *plugin_data, int plugin_id) {
    auto *plugin_instance = reinterpret_cast<Ros2Control *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int capability_bit) {
    auto *plugin_instance = reinterpret_cast<Ros2Control *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
  RCLCPP_INFO(rclcpp::get_logger("RosControl"), "Successfully registered 'Ros2Control' plugin\n");
}

std::unique_ptr<Ros2Control> Ros2Control::Create(const mjModel *m, mjData *d, int plugin_id)
{
  if (!mujoco_system_loader_)
  {
    try
    {
      mujoco_system_loader_.reset(
        new pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>(
          "mujoco_ros_utils", /* Package where this plugin is located */
          "mujoco_ros2_control::MujocoSystemInterface"));
    }
    catch (pluginlib::PluginlibException &ex)
    {
      mju_error("Failed to create hardware interface plugin loader. Error: %s", ex.what());
    }
  }

  // Find the config file
  const char *config_file_path_char     = mj_getPluginConfig(m, plugin_id, ATTR_CONFIG_FILE);
  const char *parameters_file_path_char = mj_getPluginConfig(m, plugin_id, ATTR_PARAMETERS);
  std::string config_file_path;
  if (config_file_path_char && strlen(config_file_path_char) > 0)
  {
    config_file_path = config_file_path_char;
  }
  else if (parameters_file_path_char && strlen(parameters_file_path_char) > 0)
  {
    config_file_path = parameters_file_path_char;
  }
  else
  {
    std::string umanoid_simulation_mujoco = "umanoid_simulation_mujoco";
    auto        umanoid_simulation_mujoco_share_directory
      = ament_index_cpp::get_package_share_directory(umanoid_simulation_mujoco);
    config_file_path = umanoid_simulation_mujoco_share_directory + "/config/ros2_controllers.yaml";
  }

  std::unique_ptr<Ros2Control> ret;
  // Catch any error in the constructor
  try
  {
    ret.reset(new Ros2Control(m, d, config_file_path));
  }
  catch (const std::exception &e)
  {
    mju_error("Failed to create 'Ros2Control' instance. Error: %s", e.what());
  }

  return ret;
}

Ros2Control::Ros2Control(const mjModel *model, mjData *data, std::string &config_file_path)
    : model_(model)
    , data_(data)
    , config_file_path_(config_file_path)
{
  ros_control_instances_++;

  // Attempt initialization, but don't fail if service is unavailable
  try
  {
    initialize();
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(rclcpp::get_logger("Ros2Control"),
                "Initial initialization failed, will retry during compute: %s", e.what());
  }
}

bool Ros2Control::initialize()
{
  if (initialized_ || node_)
  {
    return initialized_;
  }

  if (!node_)
  {
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(rclcpp::get_logger("Ros2Control"), "Passing ros2 config file: %s",
                  config_file_path_.c_str());
      const char *argv[] = {RCL_ROS_ARGS_FLAG, RCL_PARAM_FILE_FLAG, config_file_path_.c_str()};
      int         argc   = sizeof(argv) / sizeof(argv[0]);
      rclcpp::init(argc, argv);
    }

    executor_.reset(new rclcpp::executors::MultiThreadedExecutor());
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)),
    });
    node_ = rclcpp::Node::make_shared("mujoco_ros2_control_node", options);
    executor_->add_node(node_);

    // Get parameters for controller_manager
    auto rcl_context = node_->get_node_base_interface()->get_context()->get_rcl_context();
    std::vector<std::string> arguments;

    if (!config_file_path_.empty())
    {
      arguments.push_back(RCL_ROS_ARGS_FLAG);
      arguments.push_back(RCL_PARAM_FILE_FLAG);
      arguments.push_back(config_file_path_);
    }
    else
    {
      arguments.push_back(RCL_PARAM_FILE_FLAG);
    }

    std::vector<const char *> argv;
    for (const auto &arg : arguments)
    {
      argv.push_back(reinterpret_cast<const char *>(arg.data()));
    }

    rcl_arguments_t rcl_arguments = rcl_get_zero_initialized_arguments();
    rcl_ret_t       rcl_return    = rcl_parse_arguments(static_cast<int>(argv.size()), argv.data(),
                                                        rcl_get_default_allocator(), &rcl_arguments);
    rcl_context->global_arguments = rcl_arguments;

    if (rcl_return != RCL_RET_OK)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error parsing config file at %s:\n%s",
                   config_file_path_.c_str(), rcl_get_error_string().str);
      return false;
    }
    if (rcl_arguments_get_param_files_count(&rcl_arguments) < 1)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse input yaml config file at %s",
                   config_file_path_.c_str());
      return false;
    }

    // MujocoResourceManager overrides load_and_initialize_components() so the
    // ControllerManager's /robot_description topic callback loads hardware with
    // MuJoCo model/data pointers. Hardware is NOT pre-loaded here — the CM
    // triggers it when it subscribes to /robot_description (transient_local QoS
    // means robot_state_publisher's cached message is delivered immediately).
    auto resource_manager = std::make_unique<MujocoResourceManager>(
      node_, model_, data_, mujoco_system_loader_);

    // Loading controller manager
    RCLCPP_INFO(node_->get_logger(), "Loading controller manager\n");
    controller_manager_.reset(new controller_manager::ControllerManager(
      std::move(resource_manager), executor_, "controller_manager", node_->get_namespace()));

    executor_->add_node(controller_manager_);

    if (!controller_manager_->has_parameter("update_rate"))
    {
      mju_error("Missing parameter 'update_rate' in controller manager. "
                "Please set it to a positive integer value.");
    }

    // Getting node update rate
    update_rate_ = controller_manager_->get_parameter("update_rate").as_int();
    RCLCPP_INFO(node_->get_logger(), "Controller manager update rate: %.2f Hz", update_rate_);
    control_period_ = 1.0 / update_rate_;

    controller_manager_->set_parameter(
      rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

    // Spin off the executor thread
    stop_executor_thread_ = false;
    auto spin             = [this]() {
      while (rclcpp::ok() && !stop_executor_thread_)
      {
        executor_->spin_once(std::chrono::milliseconds(100));
      }
    };

    executor_thread_ = std::thread(spin);

    // Mark initialization as complete
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Ros2Control initialization completed successfully");

    // Register /reset_simulation service.
    // Sets an atomic flag; the actual mj_resetData is applied inside compute()
    // so it runs in the simulation thread rather than mid-step from a service callback.
    reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/reset_simulation",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        reset_requested_ = true;
        res->success      = true;
        res->message      = "Simulation reset scheduled — will apply on next compute tick";
        RCLCPP_INFO(node_->get_logger(), "[RosControl] Simulation reset requested");
      });
  }

  return initialized_;
}

Ros2Control::~Ros2Control()
{
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "Destroying Ros2Control plugin\n");
  }
  else
  {
    print_confirm("Destroying Ros2Control plugin (not initialized)\n");
  }

  if (initialized_ && node_)
  {
    stop_executor_thread_ = true;
    try
    {
      executor_->remove_node(node_);
      if (controller_manager_)
      {
        executor_->remove_node(controller_manager_);
      }
    }
    catch (const std::exception &e)
    {
      if (node_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Error while removing nodes from executor: %s This might be normal as the "
                    "node/controller_manager might be cleaned up by now.",
                    e.what());
      }
    }

    if (executor_)
    {
      executor_->cancel();
    }
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
    controller_manager_.reset();
    node_.reset();
    executor_.reset();
  }

  // Always decrement the counter and potentially shut down ROS
  ros_control_instances_--;
  if (ros_control_instances_ == 0 && rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  print_confirm("Ros2Control plugin destroyed successfully\n");
}

void Ros2Control::reset(const mjModel *m, int plugin_id)
{
  if (node_)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Ros2Control plugin reset");
  }
  else
  {
    print_confirm("Ros2Control plugin reset\n");
  }

  last_update_ = rclcpp::Time{(int64_t)0, RCL_ROS_TIME};
  return;
}

void Ros2Control::compute(const mjModel *m, mjData *d, int plugin_id)
{
  // Try to initialize if not yet initialized
  if (!initialized_)
  {
    try
    {
      if (!initialize())
      {
        return;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("Ros2Control"),
                            *rclcpp::Clock::make_shared(RCL_ROS_TIME), 2000,
                            "Initialization still failing: %s", e.what());
      return;
    }
  }

  builtin_interfaces::msg::Time sim_time_now;
  sim_time_now.sec     = static_cast<int32_t>(d->time);
  sim_time_now.nanosec = static_cast<uint32_t>((d->time - sim_time_now.sec) * 1e9);

  // Call ROS callback
  if (rclcpp::ok() && node_ && initialized_)
  {
    // Apply pending simulation reset.
    // The flag is set by the /reset_simulation service callback (executor thread).
    // We apply it here (simulation thread) to keep mjData access single-threaded.
    //
    // IMPORTANT: compute() is called from within mj_step() at the passive-forces stage.
    // mj_resetData() zeros ALL derived state (qM, xpos, xmat, …). If we stop here,
    // the remaining stages of mj_step() (fwdActuation, fwdAcceleration, Euler) run
    // with qM=0 → qacc = M⁻¹·f = NaN → physics diverges → sim stops.
    //
    // The GUI "Backspace" reset works because it calls mj_resetData+mj_forward
    // *between* mj_step() calls (from the render thread, with the sim mutex held).
    //
    // Our fix: after mj_resetData, manually re-run stages 1–6 of mj_step (kinematics
    // through fwdVelocity) to restore kinematic consistency BEFORE the remaining
    // stages continue.  We deliberately skip mj_passive() (we are inside it) and
    // mj_fwdActuation/Acceleration/Constraint (mj_step() will handle those next).
    if (reset_requested_.exchange(false))
    {
      RCLCPP_INFO(node_->get_logger(), "[RosControl] Applying simulation reset");
      mj_resetData(m, d);
      // Restore kinematic consistency so remaining mj_step() stages don't see qM=0.
      mj_kinematics(m, d);       // xpos, xmat, xquat from new qpos
      mj_comPos(m, d);           // qM (joint-space inertia matrix) — CRITICAL
      mj_collision(m, d);        // contact detection at initial pose
      mj_makeConstraint(m, d);   // constraint equations
      mj_projectConstraint(m, d); // constraint projection
      mj_fwdVelocity(m, d);      // velocity-dependent terms (qfrc_bias, cacc)
      last_update_             = rclcpp::Time{(int64_t)0, RCL_ROS_TIME};
      hardware_reset_pending_  = true;
      // Set sim_time_now to post-reset value (d->time = 0) so duration=0 below,
      // causing read/update to be skipped while write(period=0) still fires on this
      // same tick to sync ctrl[] to initial positions.
      sim_time_now.sec     = 0;
      sim_time_now.nanosec = 0;
    }

    rclcpp::Time     now{sim_time_now.sec, sim_time_now.nanosec, RCL_ROS_TIME};

    // Also handle external resets (e.g. viewer Backspace): sim time jumped backwards.
    if (now < last_update_)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[RosControl] Simulation reset detected (%.3f → %.3f s), resetting controller timing",
                  last_update_.seconds(), now.seconds());
      last_update_             = rclcpp::Time{(int64_t)0, RCL_ROS_TIME};
      hardware_reset_pending_  = true;
    }

    rclcpp::Duration duration = now - last_update_;

    if (duration.seconds() > control_period_)
    {
      controller_manager_->read(now, duration);
      controller_manager_->update(now, duration);
      last_update_ = now;
    }

    // If a reset just happened, force period=0 into write() so MujocoSystem::write()
    // syncs all command state back to the post-reset physics state instead of
    // overwriting qpos/qvel with stale pre-reset commanded values.
    rclcpp::Duration write_period = hardware_reset_pending_.exchange(false)
                                      ? rclcpp::Duration{0, 0}
                                      : duration;
    controller_manager_->write(now, write_period);
  }
}

} // namespace MujocoRosUtils