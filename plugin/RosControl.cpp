#include "RosControl.hpp"
#include "mujoco_utils.hpp"

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <mujoco/mujoco.h>

constexpr char ATTR_NODE_NAME[]        = "node_name";
constexpr char ATTR_PUBLISH_RATE[]     = "publish_rate";
constexpr char ATTR_ROBOT_PARAM_NODE[] = "robot_param_node";

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

  std::vector<const char *> attributes = {ATTR_NODE_NAME, ATTR_PUBLISH_RATE};

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
        "mujoco_ros_utils", /* Package where this plugin is located */
        "mujoco_ros2_control::MujocoSystemInterface"));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    mju_error("Failed to create hardware interface plugin loader. Error: %s", ex.what());
  }

  // This is only for testing if mujoco_system can be created
  std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system;
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("RosControl"), "Creating 'MujocoSystem' instance");
    mujoco_system.reset(
      mujoco_system_loader_->createUnmanagedInstance("mujoco_ros2_control/MujocoSystem") /* This refers to 'name' of the plugin if available in the plugin XML */);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    mju_error("Failed to test-create 'MujocoSystem' hardware interface instance. Error: %s",
              ex.what());
  }

  return std::unique_ptr<Ros2Control>(new Ros2Control(m, d));
}

Ros2Control::Ros2Control(const mjModel *model, mjData *data)
    : model_(model)
    , data_(data)
{
  if (!node_)
  {

    if (!rclcpp::ok())
    {
      int    argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    executor_.reset(new rclcpp::executors::MultiThreadedExecutor());
    node_ = rclcpp::Node::make_shared(
      "ros2_control_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
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
    while (!parameters_client->wait_for_service(0.5s))
    {
      if (!rclcpp::ok())
      {
        mju_error("Interrupted while waiting for %s service. Exiting.", robot_param_node.c_str());
      }
      RCLCPP_ERROR(node_->get_logger(), "%s service not available, waiting again...",
                   robot_param_node.c_str());
    }

    std::string  param_name = "robot_description";
    RCLCPP_INFO(node_->get_logger(), "Found %s service. Asking for %s", robot_param_node.c_str(), param_name.c_str());

    rclcpp::Time start_time = node_->get_clock()->now();
    while (urdf_string.empty() && (node_->get_clock()->now() - start_time).seconds() < 5)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "Waiting for parameter [%s] on the ROS param server.",
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
      RCLCPP_INFO(node_->get_logger(), "Hardware Info: %s",
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
        mju_error("Failed to initialize 'MujocoSystem' for %s",
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
  }
}

Ros2Control::~Ros2Control()
{
  if (node_)
  {
    controller_manager_.reset();
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }
}

void Ros2Control::reset(const mjModel *m, int plugin_id)
{
  return;
}

void Ros2Control::compute(const mjModel *m, mjData *d, int plugin_id)
{
  // Call ROS callback
  if (rclcpp::ok() && node_)
  {
    // Spin the executor to process callbacks
    executor_->spin_some();
  }
}

} // namespace MujocoRosUtils