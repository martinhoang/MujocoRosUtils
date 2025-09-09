#include "RosControl.hpp"
#include "mujoco_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <mujoco/mujoco.h>

constexpr char ATTR_NODE_NAME[]        = "node_name";
constexpr char ATTR_PUBLISH_RATE[]     = "publish_rate";
constexpr char ATTR_ROBOT_PARAM_NODE[] = "robot_param_node";
constexpr char ATTR_CONFIG_FILE[]      = "config_file";

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
    = {ATTR_NODE_NAME, ATTR_PUBLISH_RATE, ATTR_ROBOT_PARAM_NODE, ATTR_CONFIG_FILE};

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
  try
  {
    mujoco_system_loader_.reset(
      new pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>(
        "mujoco_ros2_control", /* Package where this plugin is located */
        "mujoco_ros2_control::MujocoSystemInterface"));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    mju_error("Failed to create hardware interface plugin loader. Error: %s", ex.what());
  }

  // Find the config file
  const char *config_file_path_char = mj_getPluginConfig(m, plugin_id, ATTR_CONFIG_FILE);
  std::string config_file_path;
  if (config_file_path_char && strlen(config_file_path_char) > 0)
  {
    config_file_path = config_file_path_char;
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
{
  if (!node_)
  {
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(rclcpp::get_logger("Ros2Control"), "Passing ros2 config file: %s", config_file_path.c_str());
      const char *argv[] = {RCL_ROS_ARGS_FLAG, RCL_PARAM_FILE_FLAG, config_file_path.c_str()};
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

    //* Getting URDF string
    std::string urdf_string;

    // Identify which node to get the robot description from
    const char *robot_param_node_char = mj_getPluginConfig(model_, 0, ATTR_ROBOT_PARAM_NODE);
    std::string robot_param_node
      = robot_param_node_char && strlen(robot_param_node_char) > 0 ? robot_param_node_char : "";
    if (robot_param_node.empty())
    {
      robot_param_node = "robot_state_publisher";
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "Using default robot state publisher: " << robot_param_node << std::endl);
    }

    using namespace std::chrono_literals;
    // Create a parameter client to try to get param from the target node
    auto parameters_client
      = std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_param_node);
    constexpr int max_attempts = 5;
    int           attempts     = 0;
    while (!parameters_client->wait_for_service(0.5s))
    {
      attempts++;
      if (attempts >= max_attempts)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Exceeded max attempts of %d to connect to %s service. Failing.", max_attempts,
                     robot_param_node.c_str());
        throw std::runtime_error("Failed to connect to parameter service from node '"
                                 + robot_param_node + "'");
      }
      if (!rclcpp::ok())
      {
        mju_error("Interrupted while waiting for %s service. Exiting.", robot_param_node.c_str());
      }
      RCLCPP_ERROR(node_->get_logger(), "%s service not available, waiting again...",
                   robot_param_node.c_str());
    }

    std::string param_name = "robot_description";
    RCLCPP_INFO(node_->get_logger(), "Found %s service. Asking for %s", robot_param_node.c_str(),
                param_name.c_str());

    rclcpp::Time start_time = node_->get_clock()->now();
    while (urdf_string.empty() && (node_->get_clock()->now() - start_time).seconds() < 5)
    {
      RCLCPP_INFO(node_->get_logger(), "Waiting for parameter [%s] on the ROS param server.",
                  param_name.c_str());
      try
      {
        auto f = parameters_client->get_parameters({param_name});
        f.wait_for(std::chrono::milliseconds(100));
        executor_->spin_some();
        std::vector<rclcpp::Parameter> values = f.get();
        urdf_string                           = values[0].as_string();
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
      }

      if (!urdf_string.empty())
      {
        break;
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(),
                    "Ros2Control plugin is waiting for model"
                    " URDF in parameter [%s] on the ROS param server.",
                    param_name.c_str());
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100000));

      if (!rclcpp::ok())
      {
        RCLCPP_INFO(node_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                    robot_param_node.c_str());
        return;
      }
    }

    if (urdf_string.empty())
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to get URDF from parameter [%s] on the ROS param server.",
                   param_name.c_str());
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Got the URDF from paramter service");

    // Get parameters for controller_manager
    auto rcl_context = node_->get_node_base_interface()->get_context()->get_rcl_context();
    std::vector<std::string> arguments;

    if (!config_file_path.empty())
    {
      arguments.push_back(RCL_ROS_ARGS_FLAG);
      arguments.push_back(RCL_PARAM_FILE_FLAG);
      arguments.push_back(config_file_path);
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

    rcl_ret_t rcl_return = rcl_parse_arguments(static_cast<int>(argv.size()), argv.data(),
                                               rcl_get_default_allocator(), &rcl_arguments);

    rcl_context->global_arguments = rcl_arguments;

    if (rcl_return != RCL_RET_OK)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error parsing config file at %s:\n%s",
                   config_file_path.c_str(), rcl_get_error_string().str);
      return;
    }
    if (rcl_arguments_get_param_files_count(&rcl_arguments) < 1)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse input yaml config file at %s",
                   config_file_path.c_str());
      return;
    }

    std::vector<hardware_interface::HardwareInfo> control_hardware_info;
    try
    {
      control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    }
    catch (const std::runtime_error &ex)
    {
      mju_warning("Failed to parse control hardware info. Error: %s", ex.what());
    }

    RCLCPP_INFO(node_->get_logger(), "Got the control hardware info from URDF with length: %zu",
                control_hardware_info.size());

    //* Initializing the hardware system

    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();

    try
    {
      resource_manager->load_urdf(urdf_string, false, false);
    }
    catch (...)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error while initializing URDF!");
    }

    for (const auto &hardware_info : control_hardware_info)
    {
      RCLCPP_INFO(node_->get_logger(), "Hardware system: %s",
                  hardware_info.hardware_class_type.c_str());

      std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system;

      // Try to create an instance of the hardware interfaces
      try
      {
        mujoco_system.reset(
          mujoco_system_loader_->createUnmanagedInstance(hardware_info.hardware_class_type));
      }
      catch (pluginlib::PluginlibException &ex)
      {
        mju_error("Failed to create 'MujocoSystem' instance for %s. Error: %s",
                  hardware_info.name.c_str(), ex.what());
        continue;
      }

      // Initializing the MujocoSystem
      if (!mujoco_system->initialize(node_, model_, data_, hardware_info))
      {
        mju_error("Failed to initialize 'MujocoSystem' for %s", hardware_info.name.c_str());
      }
      else
      {
        print_confirm("MujocoSystem initialized successfully for '%s'\n",
                      hardware_info.name.c_str());
      }

      // Load it up to ResourceManager
      resource_manager->import_component(std::move(mujoco_system), hardware_info);

      // Try to activate all components to ACTIVE
      rclcpp_lifecycle::State state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                    hardware_interface::lifecycle_state_names::ACTIVE);
      resource_manager->set_component_state(hardware_info.name, state);
    }

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
  }
}

Ros2Control::~Ros2Control()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying Ros2Control plugin\n");
  if (node_)
  {
    stop_executor_thread_ = true;
    try
    {
      executor_->remove_node(node_);
      executor_->remove_node(controller_manager_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Error while removing nodes from executor: %s...This might be normal as the "
                  "node/controller_manager might be cleaned up by now.",
                  e.what());
    }

    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
    controller_manager_.reset();
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
    print_confirm("Ros2Control plugin destroyed successfully\n");
  }
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
  builtin_interfaces::msg::Time sim_time_now;
  sim_time_now.sec     = static_cast<int32_t>(d->time);
  sim_time_now.nanosec = static_cast<uint32_t>((d->time - sim_time_now.sec) * 1e9);

  // Call ROS callback
  if (rclcpp::ok() && node_)
  {
    rclcpp::Time     now{sim_time_now.sec, sim_time_now.nanosec, RCL_ROS_TIME};
    rclcpp::Duration duration = now - last_update_;

    if (duration.seconds() > control_period_)
    {
      // RCLCPP_INFO(node_->get_logger(), "Controller manager update: %.2f seconds since last
      // update.", duration.seconds());
      controller_manager_->read(now, duration);
      controller_manager_->update(now, duration);
      last_update_ = now;
    }

    // Write the data back to the model
    controller_manager_->write(now, duration);
  }
}

} // namespace MujocoRosUtils