#include "RosControl.hpp"
#include "mujoco_utils.hpp"

#include <hardware_interface/component_parser.hpp>
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

  print_info("Registering Ros2Control plugin\n");

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
  print_confirm("Successfully registered 'Ros2Control' plugin\n");
}

std::unique_ptr<Ros2Control> Ros2Control::Create(const mjModel *m, mjData *d, int plugin_id)
{
  try
  {
    mujoco_system_loader_.reset(
      new pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface>(
        "mujoco_ros_utils", "mujoco_ros2_control::MujocoSystem"));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    mju_error("Failed to create hardware interface plugin loader. Error: %s", ex.what());
  }

  std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system;
  try
  {
    print_info("Creating MujocoSystem instance\n");
    mujoco_system.reset(
      mujoco_system_loader_->createUnmanagedInstance("mujoco_ros2_control::MujocoSystem"));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    mju_error("Failed to create 'MujocoSystem' hardware interface instance. Error: %s", ex.what());
  }

  return std::unique_ptr<Ros2Control>(new Ros2Control(m, d, std::move(mujoco_system)));
}

Ros2Control::Ros2Control(const mjModel *model, mjData *data,
                         std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mujoco_system)
    : model_(model)
    , data_(data)
    , mujoco_system_(std::move(mujoco_system))
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
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("ros2_control_node", options);

    // hardware_interface::HardwareInfo hardware_info;
    // mujoco_system_->initialize(node_, model_, data_, hardware_info);

    // //* Getting URDF string
    // std::string urdf_string;

    // // Method 1: Getting robot description from robot_state_publisher parameter
    // const char *robot_param_node_char = mj_getPluginConfig(model_, 0, ATTR_ROBOT_PARAM_NODE);
    // std::string robot_param_node
    //   = robot_param_node_char && strlen(robot_param_node_char) > 0 ? robot_param_node_char : "";
    // if (robot_param_node.empty())
    // {
    //   print_info("Using default robot state publisher: %s\n", robot_param_node.c_str());
    //   robot_param_node = "robot_state_publisher";
    // }

    // // Method 2: Getting robot description directly from parameter server
    // urdf_string = node_->get_parameter("robot_description").as_string();

    // std::vector<hardware_interface::HardwareInfo> control_hardware_info;
    // try
    // {
    //   control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    // }
    // catch (const std::runtime_error &ex)
    // {
    //   mju_warning("Failed to parse control hardware info. Error: %s", ex.what());
    // }

    // std::unique_ptr<hardware_interface::ResourceManager> resource_manager
    //   = std::make_unique<hardware_interface::ResourceManager>();

    // // Loading controller manager
    // print_info("Loading controller manager\n");
    // controller_manager_.reset(new controller_manager::ControllerManager(
    //   std::move(resource_manager), executor_, "controller_manager", node_->get_namespace()));
  }
}

void Ros2Control::reset(const mjModel *m, int plugin_id)
{
  print_info("Resetting Ros2Control plugin with plugin ID: %d\n", plugin_id);
}

void Ros2Control::compute(const mjModel *m, mjData *d, int plugin_id)
{
  print_info("Computing Ros2Control plugin with plugin ID: %d\n", plugin_id);
}

} // namespace MujocoRosUtils