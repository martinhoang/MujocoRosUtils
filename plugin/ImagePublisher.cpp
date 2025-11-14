#include "ImagePublisher.h"
#include "depth_conversions.hpp"

#include "mujoco_utils.hpp"
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mujoco/mujoco.h>
#include <sensor_msgs/image_encodings.hpp>

namespace MujocoRosUtils
{

constexpr char ATTR_FRAME_ID[]                    = "frame_id";
constexpr char ATTR_NAMESPACE[]                   = "namespace";
constexpr char ATTR_COLOR_TOPIC_NAME[]            = "color_topic_name";
constexpr char ATTR_DEPTH_TOPIC_NAME[]            = "depth_topic_name";
constexpr char ATTR_INFO_TOPIC_NAME[]             = "info_topic_name";
constexpr char ATTR_POINT_CLOUD_TOPIC_NAME[]      = "point_cloud_topic_name";
constexpr char ATTR_ROTATE_POINT_CLOUD[]          = "rotate_point_cloud";
constexpr char ATTR_POINT_CLOUD_ROTATION_PRESET[] = "point_cloud_rotation_preset";
constexpr char ATTR_HEIGHT[]                      = "height";
constexpr char ATTR_WIDTH[]                       = "width";
constexpr char ATTR_PUBLISH_RATE[]                = "publish_rate";
constexpr char ATTR_MAX_RANGE[]                   = "max_range";

void ImagePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::ImagePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char *attributes[] = {ATTR_FRAME_ID,
                              ATTR_NAMESPACE,
                              ATTR_COLOR_TOPIC_NAME,
                              ATTR_DEPTH_TOPIC_NAME,
                              ATTR_INFO_TOPIC_NAME,
                              ATTR_POINT_CLOUD_TOPIC_NAME,
                              ATTR_ROTATE_POINT_CLOUD,
                              ATTR_POINT_CLOUD_ROTATION_PRESET,
                              ATTR_HEIGHT,
                              ATTR_WIDTH,
                              ATTR_PUBLISH_RATE,
                              ATTR_MAX_RANGE};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int              // plugin_id
                   ) {
    return 0;
  };

  plugin.nsensordata = +[](const mjModel *, // m
                           int,             // plugin_id
                           int              // sensor_id
                        ) {
    return 0;
  };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    print_debug("[ImagePublisher::init] Starting initialization for plugin_id=%d\n", plugin_id);
    if (!m || !d)
    {
      mju_error("[ImagePublisher::init] ERROR: Null model or data pointer!");
      return -1;
    }
    auto *plugin_instance = ImagePublisher::Create(m, d, plugin_id);
    if (!plugin_instance)
    {
      mju_error("[ImagePublisher::init] ERROR: Failed to create plugin instance");
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    print_debug("[ImagePublisher::init] Successfully initialized, instance=%p\n", plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    print_debug("[ImagePublisher::destroy] Starting destruction for plugin_id=%d\n", plugin_id);
    if (!d)
    {
      mju_warning("[ImagePublisher::destroy] Null data pointer!");
      return;
    }
    auto *plugin_instance = reinterpret_cast<class ImagePublisher *>(d->plugin_data[plugin_id]);
    if (!plugin_instance)
    {
      mju_warning("[ImagePublisher::destroy] Null plugin instance!");
      return;
    }
    print_debug("[ImagePublisher::destroy] Freeing instance=%p\n", plugin_instance);
    plugin_instance->free();
    delete reinterpret_cast<ImagePublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
    print_debug("[ImagePublisher::destroy] Destruction complete\n");
  };

  plugin.reset = +[](const mjModel *m, double *, // plugin_state
                     void *plugin_data, int plugin_id) {
    print_debug("[ImagePublisher::reset] Called for plugin_id=%d\n", plugin_id);
    if (!plugin_data)
    {
      mju_warning("[ImagePublisher::reset] Null plugin_data!");
      return;
    }
    auto *plugin_instance = reinterpret_cast<class ImagePublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int // capability_bit
                    ) {
    if (!d || !m)
    {
      mju_error("[ImagePublisher::compute] Null model or data pointer!");
      return;
    }
    if (!d->plugin_data[plugin_id])
    {
      mju_error("[ImagePublisher::compute] Null plugin_data for plugin_id=%d", plugin_id);
      return;
    }
    auto *plugin_instance = reinterpret_cast<class ImagePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);

  print_confirm("Successfully registered 'MujocoRosUtils::ImagePublisher' plugin\n");
}

ImagePublisher *ImagePublisher::Create(const mjModel *m, mjData *d, int plugin_id)
{
  print_debug("[ImagePublisher::Create] Starting, plugin_id=%d\n", plugin_id);

  // frame_id
  print_debug("[ImagePublisher::Create] Reading frame_id...\n");
  const char *frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_FRAME_ID);
  std::string frame_id      = "";
  if (frame_id_char && strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
    print_debug("[ImagePublisher::Create] frame_id=%s\n", frame_id.c_str());
  }

  // namespace
  print_debug("[ImagePublisher::Create] Reading namespace...\n");
  const char *namespace_char  = mj_getPluginConfig(m, plugin_id, ATTR_NAMESPACE);
  std::string topic_namespace = "";
  if (namespace_char && strlen(namespace_char) > 0)
  {
    topic_namespace = std::string(namespace_char);
    print_debug("[ImagePublisher::Create] namespace=%s\n", topic_namespace.c_str());
    // Ensure namespace ends with '/' if not empty
    if (!topic_namespace.empty() && topic_namespace.back() != '/')
    {
      topic_namespace += "/";
    }
  }
  else
  {
    print_debug("[ImagePublisher::Create] WARNING: namespace not specified, using empty\n");
    // Don't throw - just use empty namespace
    topic_namespace = "";
  }

  // color_topic_name
  const char *color_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_COLOR_TOPIC_NAME);
  std::string color_topic_name      = "";
  if (color_topic_name_char && strlen(color_topic_name_char) > 0)
  {
    color_topic_name = std::string(color_topic_name_char);
  }

  // depth_topic_name
  const char *depth_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_DEPTH_TOPIC_NAME);
  std::string depth_topic_name      = "";
  if (depth_topic_name_char && strlen(depth_topic_name_char) > 0)
  {
    depth_topic_name = std::string(depth_topic_name_char);
  }

  // info_topic_name
  const char *info_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_INFO_TOPIC_NAME);
  std::string info_topic_name      = "";
  if (info_topic_name_char && strlen(info_topic_name_char) > 0)
  {
    info_topic_name = std::string(info_topic_name_char);
  }

  // point_cloud_topic_name
  const char *point_cloud_topic_name_char
    = mj_getPluginConfig(m, plugin_id, ATTR_POINT_CLOUD_TOPIC_NAME);
  std::string point_cloud_topic_name = "";
  if (point_cloud_topic_name_char && strlen(point_cloud_topic_name_char) > 0)
  {
    point_cloud_topic_name = std::string(point_cloud_topic_name_char);
  }

  // point_cloud rotation enabled
  const char *rotate_point_cloud_char = mj_getPluginConfig(m, plugin_id, ATTR_ROTATE_POINT_CLOUD);
  bool        rotate_point_cloud      = true;
  if (rotate_point_cloud_char && strlen(rotate_point_cloud_char) > 0)
  {
    if (std::string(rotate_point_cloud_char) == "false"
        || std::string(rotate_point_cloud_char) == "0")
    {
      rotate_point_cloud = false;
    }
    else if (std::string(rotate_point_cloud_char) == "true"
             || std::string(rotate_point_cloud_char) == "1")
    {
      rotate_point_cloud = true;
    }
    else
    {
      mju_error("[ImagePublisher] `rotate_point_cloud` must be `true (1)` or `false (0)`.");
      return nullptr;
    }
  }

  // point_cloud_rotation_preset
  const char *point_cloud_rotation_preset_char
    = mj_getPluginConfig(m, plugin_id, ATTR_POINT_CLOUD_ROTATION_PRESET);
  std::string point_cloud_rotation_preset = depthimage_to_pointcloud2::PCL_ROT_PRESET_RDF_TO_FLU;
  if (rotate_point_cloud && point_cloud_rotation_preset_char
      && strlen(point_cloud_rotation_preset_char) > 0)
  {
    point_cloud_rotation_preset = point_cloud_rotation_preset_char;
  }

  // height
  print_debug("[ImagePublisher::Create] Reading height...\n");
  const char *height_char = mj_getPluginConfig(m, plugin_id, ATTR_HEIGHT);
  int         height      = 240;
  if (height_char && strlen(height_char) > 0)
  {
    height = static_cast<int>(strtol(height_char, nullptr, 10));
    print_debug("[ImagePublisher::Create] height=%d\n", height);
  }
  else
  {
    print_debug("[ImagePublisher::Create] height not specified, using default=%d\n", height);
  }
  if (height <= 0)
  {
    mju_error("[ImagePublisher] `height` must be positive.");
    return nullptr;
  }

  // width
  print_debug("[ImagePublisher::Create] Reading width...\n");
  const char *width_char = mj_getPluginConfig(m, plugin_id, ATTR_WIDTH);
  int         width      = 320;
  if (width_char && strlen(width_char) > 0)
  {
    width = static_cast<int>(strtol(width_char, nullptr, 10));
    print_debug("[ImagePublisher::Create] width=%d\n", width);
  }
  else
  {
    print_debug("[ImagePublisher::Create] width not specified, using default=%d\n", width);
  }
  if (width <= 0)
  {
    mju_error("[ImagePublisher] `width` must be positive.");
    return nullptr;
  }

  // publish_rate
  print_debug("[ImagePublisher::Create] Reading publish_rate...\n");
  const char *publish_rate_char = mj_getPluginConfig(m, plugin_id, ATTR_PUBLISH_RATE);
  mjtNum      publish_rate      = 30.0;
  if (publish_rate_char && strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
    print_debug("[ImagePublisher::Create] publish_rate=%f\n", publish_rate);
  }
  else
  {
    print_debug("[ImagePublisher::Create] publish_rate not specified, using default=%f\n",
              publish_rate);
  }
  if (publish_rate <= 0)
  {
    mju_error("[ImagePublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // max_range
  print_debug("[ImagePublisher::Create] Reading max_range...\n");
  const char *max_range_char = mj_getPluginConfig(m, plugin_id, ATTR_MAX_RANGE);
  double      max_range      = 0.0;
  if (max_range_char && strlen(max_range_char) > 0)
  {
    max_range = strtod(max_range_char, nullptr);
    print_debug("[ImagePublisher::Create] max_range=%f\n", max_range);
  }
  else
  {
    print_debug("[ImagePublisher::Create] max_range not specified, using default=%f\n",
              max_range);
  }

  // Set sensor_id
  print_debug("[ImagePublisher::Create] Finding sensor_id, nsensor=%d\n", m->nsensor);
  int sensor_id = 0;
  for (; sensor_id < m->nsensor; sensor_id++)
  {
    if (m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      print_debug("[ImagePublisher::Create] Found sensor at index %d\n", sensor_id);
      break;
    }
  }
  if (sensor_id == m->nsensor)
  {
    mju_error("[ImagePublisher::Create] Plugin not found in sensors!");
    return nullptr;
  }
  if (m->sensor_objtype[sensor_id] != mjOBJ_CAMERA)
  {
    mju_error("[ImagePublisher::Create] Plugin must be attached to a camera!");
    return nullptr;
  }

  print_debug("[ImagePublisher::Create] Creating ImagePublisher instance...\n");
  print_debug("[ImagePublisher::Create] Parameters: sensor_id=%d, width=%d, height=%d, publish_rate=%f\n",
            sensor_id, width, height, publish_rate);

  ImagePublisher *instance = new ImagePublisher(
    m, d, sensor_id, frame_id, topic_namespace, color_topic_name, depth_topic_name, info_topic_name,
    point_cloud_topic_name, rotate_point_cloud, point_cloud_rotation_preset, height, width,
    publish_rate, max_range);

  print_confirm("[ImagePublisher::Create] Instance created successfully at %p\n", (void*)instance);
  return instance;
}

ImagePublisher::ImagePublisher(const mjModel *m,
                               mjData *, // d
                               int sensor_id, const std::string &frame_id,
                               const std::string &topic_namespace, std::string color_topic_name,
                               std::string depth_topic_name, std::string info_topic_name,
                               std::string point_cloud_topic_name, bool rotate_point_cloud,
                               const std::string &point_cloud_rotation_preset, int height,
                               int width, mjtNum publish_rate, double max_range)
    : m_(m)
    , sensor_id_(sensor_id)
    , camera_id_(m->sensor_objid[sensor_id])
    , frame_id_(frame_id)
    , rotate_point_cloud_(rotate_point_cloud)
    , point_cloud_rotation_preset_(point_cloud_rotation_preset)
    , publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
    , viewport_({0, 0, width, height})
{
  std::string camera_name = std::string(mj_id2name(m, mjOBJ_CAMERA, camera_id_));
  if (frame_id_.empty())
  {
    frame_id_ = camera_name;
  }

  // Initialize all topic names with namespace prefix
  color_topic_name
    = topic_namespace + (color_topic_name.empty() ? camera_name + "/color" : color_topic_name);
  depth_topic_name
    = topic_namespace + (depth_topic_name.empty() ? camera_name + "/depth" : depth_topic_name);
  info_topic_name
    = topic_namespace + (info_topic_name.empty() ? camera_name + "/camera_info" : info_topic_name);
  point_cloud_topic_name
    = topic_namespace
      + (point_cloud_topic_name.empty() ? camera_name + "/point_cloud" : point_cloud_topic_name);

  print_debug("[ImagePublisher] Constructor: Starting OpenGL initialization\n");

  // Set GLFW error callback
  glfwSetErrorCallback([](int error, const char *description) {
    mju_warning("[ImagePublisher] GLFW Error %d: %s", error, description);
  });

  // Init OpenGL
  print_debug("[ImagePublisher] Constructor: Calling glfwInit()\n");
  if (!glfwInit())
  {
    mju_error("[ImagePublisher] Could not initialize GLFW.");
  }
  print_debug("[ImagePublisher] Constructor: glfwInit() succeeded\n");

  // Create invisible window, single-buffered
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  print_debug("[ImagePublisher] Constructor: Creating GLFW window %dx%d\n",
            viewport_.width, viewport_.height);
  window_ = glfwCreateWindow(viewport_.width, viewport_.height, "MujocoRosUtils::ImagePublisher",
                             nullptr, nullptr);
  if (!window_)
  {
    mju_error("[ImagePublisher] Could not create GLFW window.");
  }
  print_debug("[ImagePublisher] Constructor: GLFW window created successfully\n");

  // Make context current
  // \todo Is it OK to override the current context of OpenGL?
  print_debug("[ImagePublisher] Constructor: Making GLFW context current\n");
  glfwMakeContextCurrent(window_);
  print_debug("[ImagePublisher] Constructor: GLFW context is current\n");

  // Init default data for visualization structures
  print_debug("[ImagePublisher] Constructor: Initializing MuJoCo visualization structures\n");
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);
  print_debug("[ImagePublisher] Constructor: MuJoCo defaults initialized\n");

  // Create scene and context
  print_debug("[ImagePublisher] Constructor: Creating MuJoCo scene (camera_id=%d)\n", camera_id_);
  mjv_makeScene(m, &scene_, 1000);
  print_debug("[ImagePublisher] Constructor: Creating MuJoCo render context\n");
  mjr_makeContext(m, &context_, mjFONTSCALE_100);
  print_debug("[ImagePublisher] Constructor: MuJoCo scene and context created\n");

  // Need to resize off-screen buffers
  print_debug("[ImagePublisher] Constructor: Resizing offscreen buffers to %dx%d\n",
            viewport_.width, viewport_.height);
  mjr_resizeOffscreen(viewport_.width, viewport_.height, &context_);
  print_debug("[ImagePublisher] Constructor: Offscreen buffers resized\n");

  // Init camera
  camera_.type       = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;
  print_debug("[ImagePublisher] Constructor: Camera initialized (fixed cam id=%d)\n", camera_id_);

  print_debug("[ImagePublisher] Constructor: Setting framebuffer to offscreen\n");
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  if (context_.currentBuffer != mjFB_OFFSCREEN)
  {
    mju_warning(
      "[ImagePublisher] Offscreen rendering not supported, using default/window framebuffer.");
  }
  print_debug("[ImagePublisher] Constructor: Framebuffer set successfully\n");

  // Allocate buffer
  print_debug("[ImagePublisher] Constructor: Allocating image buffers (%dx%d)\n", width, height);
  size_t color_buffer_size = sizeof(unsigned char) * 3 * viewport_.width * viewport_.height;
  size_t depth_buffer_size = sizeof(float) * viewport_.width * viewport_.height;
  print_debug("[ImagePublisher] Constructor: Color buffer size=%zu bytes, Depth buffer size=%zu bytes\n",
            color_buffer_size, depth_buffer_size);

  try
  {
    color_buffer_.reset(new unsigned char[3 * width * height]);
    print_debug("[ImagePublisher] Constructor: color_buffer_ allocated\n");
    depth_buffer_.reset(new float[width * height]);
    print_debug("[ImagePublisher] Constructor: depth_buffer_ allocated\n");
    color_buffer_flipped_.reset(new unsigned char[3 * width * height]);
    print_debug("[ImagePublisher] Constructor: color_buffer_flipped_ allocated\n");
    depth_buffer_flipped_.reset(new float[width * height]);
    print_debug("[ImagePublisher] Constructor: depth_buffer_flipped_ allocated\n");
    color_buffer_back_.reset(new unsigned char[3 * width * height]);
    print_debug("[ImagePublisher] Constructor: color_buffer_back_ allocated\n");
    depth_buffer_back_.reset(new float[width * height]);
    print_debug("[ImagePublisher] Constructor: depth_buffer_back_ allocated\n");
  }
  catch (const std::bad_alloc &e)
  {
    mju_error("[ImagePublisher] Failed to allocate buffers: %s", e.what());
  }

  if (!color_buffer_ || !depth_buffer_ || !color_buffer_flipped_ || !depth_buffer_flipped_
      || !color_buffer_back_ || !depth_buffer_back_)
  {
    mju_error("[ImagePublisher] One or more buffers are null after allocation!");
  }
  print_debug("[ImagePublisher] Constructor: All buffers allocated successfully\n");

  // Init ROS
  print_debug("[ImagePublisher] Constructor: Initializing ROS2\n");
  int    argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok())
  {
    print_debug("[ImagePublisher] Constructor: Calling rclcpp::init()\n");
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;

  node_options.parameter_overrides({
    {"use_sim_time", true}, // Force use simulation time
  });
  node_options.automatically_declare_parameters_from_overrides(true);

  std::string node_name = mj_id2name(m, mjOBJ_SENSOR, sensor_id);
  print_debug("[ImagePublisher] Constructor: Creating ROS2 node '%s'\n", node_name.c_str());

  nh_ = rclcpp::Node::make_shared(node_name, node_options);
  RCLCPP_INFO(nh_->get_logger(), "ROS2 node created");

  auto qos = rclcpp::SensorDataQoS();
  RCLCPP_DEBUG(nh_->get_logger(), "Creating publishers...");
  color_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(color_topic_name, qos);
  depth_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(depth_topic_name, qos);
  info_pub_  = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name, qos);
  point_cloud_pub_
    = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_name, qos);
  RCLCPP_DEBUG(nh_->get_logger(), "Publishers created");

  // If the ros2 params are provided, they will take precedence
  range_max_     = nh_->get_parameter_or("range_max", max_range);
  use_quiet_nan_ = nh_->get_parameter_or("use_quiet_nan", true);

  RCLCPP_INFO(nh_->get_logger(), "ImagePublisher initialized (range_max=%.2f)", range_max_);

  // Mark as initialized before starting the publish thread
  RCLCPP_DEBUG(nh_->get_logger(), "Starting publish thread");
  initialized_    = true;
  publish_thread_ = std::thread(&ImagePublisher::publishThread, this);
  RCLCPP_DEBUG(nh_->get_logger(), "Publish thread started");
}

void ImagePublisher::reset(const mjModel *, // m
                           int              // plugin_id
)
{}

void ImagePublisher::compute(const mjModel *m, mjData *d, int // plugin_id
)
{
  // Safety check: ensure initialization is complete
  if (!initialized_)
  {
    static bool first_warning = true;
    if (first_warning)
    {
      RCLCPP_WARN(nh_->get_logger(), "compute() called before initialization complete, skipping");
      first_warning = false;
    }
    return;
  }

  sim_cnt_++;
  if (sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  static int compute_call_count = 0;
  compute_call_count++;
  if (compute_call_count <= 5 || compute_call_count % 100 == 0)
  {
    RCLCPP_DEBUG(nh_->get_logger(), "compute() call #%d", compute_call_count);
  }

  // Update subscriber counts
  publish_color_ = color_pub_->get_subscription_count() > 0;
  publish_depth_ = depth_pub_->get_subscription_count() > 0;
  publish_info_  = info_pub_->get_subscription_count() > 0;
  publish_cloud_ = point_cloud_pub_->get_subscription_count() > 0;

  // If no one is listening, do nothing
  if (!publish_color_ && !publish_depth_ && !publish_info_ && !publish_cloud_)
  {
    return;
  }

  if (compute_call_count <= 5)
  {
    RCLCPP_DEBUG(nh_->get_logger(), 
                 "Subscribers: color=%d depth=%d info=%d cloud=%d", 
                 publish_color_, publish_depth_, publish_info_, publish_cloud_);
  }

  // Protect OpenGL context operations with mutex to prevent multi-threaded access
  try
  {
    std::lock_guard<std::mutex> gl_lock(gl_context_mutex_);

    if (compute_call_count <= 3)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Performing GL operations");
    }

    // Make context current
    // \todo Is it OK to override the current context of OpenGL?
    glfwMakeContextCurrent(window_);

    // Update abstract scene
    mjv_updateScene(m, d, &option_, nullptr, &camera_, mjCAT_STATIC | mjCAT_DYNAMIC, &scene_);

    // Render scene in offscreen buffer
    mjr_render(viewport_, &scene_, &context_);

    // Read rgb and depth pixels
    mjr_readPixels(color_buffer_.get(), depth_buffer_.get(), viewport_, &context_);

    if (compute_call_count <= 3)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "GL operations complete");
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Exception during GL operations: %s", e.what());
    return;
  }

  // Get timestamp for all messages
  rclcpp::Time stamp_now = nh_->get_clock()->now();

  // Publish camera info always (needed by RViz2 and other tools)
  if (publish_info_)
  {
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header.stamp     = stamp_now;
    info_msg.header.frame_id  = frame_id_;
    info_msg.height           = viewport_.height;
    info_msg.width            = viewport_.width;
    info_msg.distortion_model = "plumb_bob";
    info_msg.d.resize(5, 0.0);
    info_msg.k.fill(0.0);
    info_msg.r.fill(0.0);
    info_msg.p.fill(0.0);
    double focal_scaling = (1.0 / std::tan((m_->cam_fovy[camera_id_] * M_PI / 180.0) / 2.0))
                           * viewport_.height / 2.0;
    info_msg.k[0] = info_msg.p[0] = focal_scaling;
    info_msg.k[2] = info_msg.p[2] = static_cast<double>(viewport_.width) / 2.0;
    info_msg.k[4] = info_msg.p[5] = focal_scaling;
    info_msg.k[5] = info_msg.p[6] = static_cast<double>(viewport_.height) / 2.0;
    info_msg.k[8] = info_msg.p[10] = 1.0;
    info_pub_->publish(info_msg);
  }

  // Publish color image directly from the compute thread for low latency
  if (publish_color_)
  {
    // Flip color buffer - row-wise memcpy is faster than pixel-by-pixel for large images
    const int row_bytes = viewport_.width * 3;
#pragma omp parallel for schedule(static)
    for (int h = 0; h < viewport_.height; h++)
    {
      const unsigned char* src_row = color_buffer_.get() + h * row_bytes;
      unsigned char* dst_row = color_buffer_flipped_.get() + (viewport_.height - 1 - h) * row_bytes;
      std::memcpy(dst_row, src_row, row_bytes);
    }

    // Create and publish color_msg
    sensor_msgs::msg::Image color_msg;
    color_msg.header.stamp    = stamp_now;
    color_msg.header.frame_id = frame_id_;
    color_msg.height          = viewport_.height;
    color_msg.width           = viewport_.width;
    color_msg.encoding        = "rgb8";
    color_msg.is_bigendian    = 0;
    color_msg.step = static_cast<unsigned int>(sizeof(unsigned char) * 3 * viewport_.width);
    color_msg.data.resize(sizeof(unsigned char) * 3 * viewport_.width * viewport_.height);
    std::memcpy(&color_msg.data[0], color_buffer_flipped_.get(),
                sizeof(unsigned char) * 3 * viewport_.width * viewport_.height);
    color_pub_->publish(color_msg);
  }

  // Offload depth and point cloud processing to the worker thread
  if (publish_depth_ || publish_cloud_)
  {
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      color_buffer_.swap(color_buffer_back_);
      depth_buffer_.swap(depth_buffer_back_);
      data_ready_ = true;
    }
    buffer_cv_.notify_one();
  }
}

void ImagePublisher::free()
{
  RCLCPP_DEBUG(nh_->get_logger(), "Starting cleanup");

  RCLCPP_DEBUG(nh_->get_logger(), "Stopping publish thread");
  stop_thread_ = true;
  buffer_cv_.notify_all();
  if (publish_thread_.joinable())
  {
    RCLCPP_DEBUG(nh_->get_logger(), "Waiting for thread to join");
    publish_thread_.join();
    RCLCPP_DEBUG(nh_->get_logger(), "Thread joined");
  }

  RCLCPP_DEBUG(nh_->get_logger(), "Freeing MuJoCo context and scene");
  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);

  RCLCPP_DEBUG(nh_->get_logger(), "Destroying GLFW window");
  if (window_)
  {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  RCLCPP_INFO(nh_->get_logger(), "ImagePublisher cleanup complete");
}

} // namespace MujocoRosUtils
void MujocoRosUtils::ImagePublisher::publishThread()
{
  RCLCPP_INFO(nh_->get_logger(), "publishThread started");
  print_confirm("ImagePublisher publish thread started\n");
  while (!stop_thread_)
  {
    RCLCPP_INFO(nh_->get_logger(), "publishThread is waiting for data...");
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      buffer_cv_.wait(lock, [this] {
        return stop_thread_ || data_ready_;
      });

      if (stop_thread_)
      {
        break;
      }

      data_ready_ = false;
    }

    // Null pointer safety check
    if (!m_ || !depth_buffer_back_ || !color_buffer_back_)
    {
      RCLCPP_ERROR(nh_->get_logger(), "[ImagePublisher] Null pointer detected in publish thread");
      continue;
    }

    // --- Process Depth Data ---
    float near = static_cast<float>(m_->vis.map.znear * m_->stat.extent);
    float far  = static_cast<float>(m_->vis.map.zfar * m_->stat.extent);
    
    // Precompute constant for depth conversion
    const float depth_scale = 1.0f - near / far;
    
#pragma omp parallel for schedule(static)
    for (int h = 0; h < viewport_.height; h++)
    {
      const int row_offset = h * viewport_.width;
      const int flipped_row_offset = (viewport_.height - 1 - h) * viewport_.width;
      
      for (int w = 0; w < viewport_.width; w++)
      {
        const int idx = row_offset + w;
        const int idx_flipped = flipped_row_offset + w;
        
        // Process depth with precomputed constant
        depth_buffer_back_[idx] = near / (1.0f - depth_buffer_back_[idx] * depth_scale);
        depth_buffer_flipped_[idx_flipped] = depth_buffer_back_[idx];
      }
    }

    rclcpp::Time stamp_now = nh_->get_clock()->now();

    // --- Publish Depth Image ---
    sensor_msgs::msg::Image depth_msg;
    if (publish_depth_ || publish_cloud_) // Also need depth_msg for point cloud
    {
      depth_msg.header.stamp    = stamp_now;
      depth_msg.header.frame_id = frame_id_;
      depth_msg.height          = viewport_.height;
      depth_msg.width           = viewport_.width;
      depth_msg.encoding        = "32FC1";
      depth_msg.is_bigendian    = 0;
      depth_msg.step            = static_cast<unsigned int>(sizeof(float) * viewport_.width);
      depth_msg.data.resize(sizeof(float) * viewport_.width * viewport_.height);
      std::memcpy(&depth_msg.data[0], depth_buffer_flipped_.get(),
                  sizeof(float) * viewport_.width * viewport_.height);
      if (publish_depth_)
      {
        depth_pub_->publish(depth_msg);
      }
    }

    // --- Create Camera Info for Point Cloud (if needed) ---
    sensor_msgs::msg::CameraInfo info_msg;
    if (publish_cloud_)
    {
      info_msg.header.stamp     = stamp_now;
      info_msg.header.frame_id  = frame_id_;
      info_msg.height           = viewport_.height;
      info_msg.width            = viewport_.width;
      info_msg.distortion_model = "plumb_bob";
      info_msg.d.resize(5, 0.0);
      info_msg.k.fill(0.0);
      info_msg.r.fill(0.0);
      info_msg.p.fill(0.0);
      double focal_scaling = (1.0 / std::tan((m_->cam_fovy[camera_id_] * M_PI / 180.0) / 2.0))
                             * viewport_.height / 2.0;
      info_msg.k[0] = info_msg.p[0] = focal_scaling;
      info_msg.k[2] = info_msg.p[2] = static_cast<double>(viewport_.width) / 2.0;
      info_msg.k[4] = info_msg.p[5] = focal_scaling;
      info_msg.k[5] = info_msg.p[6] = static_cast<double>(viewport_.height) / 2.0;
      info_msg.k[8] = info_msg.p[10] = 1.0;
    }

    // --- Publish Point Cloud ---
    if (publish_cloud_)
    {
      // Flip the color buffer that was passed from the main thread - row-wise memcpy
      const int row_bytes = viewport_.width * 3;
#pragma omp parallel for schedule(static)
      for (int h = 0; h < viewport_.height; h++)
      {
        const unsigned char* src_row = color_buffer_back_.get() + h * row_bytes;
        unsigned char* dst_row = color_buffer_flipped_.get() + (viewport_.height - 1 - h) * row_bytes;
        std::memcpy(dst_row, src_row, row_bytes);
      }

      // Create a temporary color message for the conversion function
      sensor_msgs::msg::Image color_msg_for_cloud;
      color_msg_for_cloud.header.stamp    = stamp_now;
      color_msg_for_cloud.header.frame_id = frame_id_;
      color_msg_for_cloud.height          = viewport_.height;
      color_msg_for_cloud.width           = viewport_.width;
      color_msg_for_cloud.encoding        = "rgb8";
      color_msg_for_cloud.is_bigendian    = 0;
      color_msg_for_cloud.step
        = static_cast<unsigned int>(sizeof(unsigned char) * 3 * viewport_.width);
      color_msg_for_cloud.data.resize(sizeof(unsigned char) * 3 * viewport_.width
                                      * viewport_.height);
      std::memcpy(&color_msg_for_cloud.data[0], color_buffer_flipped_.get(),
                  sizeof(unsigned char) * 3 * viewport_.width * viewport_.height);

      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
      sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg
        = std::make_shared<sensor_msgs::msg::PointCloud2>();
      cloud_msg->header       = depth_msg.header;
      cloud_msg->height       = depth_msg.height;
      cloud_msg->width        = depth_msg.width;
      cloud_msg->is_dense     = false;
      cloud_msg->is_bigendian = false;
      sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
      pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

      sensor_msgs::msg::Image::ConstSharedPtr color_msg_ptr(&color_msg_for_cloud,
                                                            [](const sensor_msgs::msg::Image *) {
                                                            });
      cv_bridge::CvImageConstPtr              cv_ptr
        = cv_bridge::toCvCopy(color_msg_ptr, sensor_msgs::image_encodings::RGB8);

      sensor_msgs::msg::Image::ConstSharedPtr depth_msg_ptr(&depth_msg,
                                                            [](const sensor_msgs::msg::Image *) {
                                                            });
      if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        depthimage_to_pointcloud2::convert<uint16_t>(depth_msg_ptr, cloud_msg, model, range_max_,
                                                     use_quiet_nan_, point_cloud_rotation_preset_,
                                                     cv_ptr);
      }
      else if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        depthimage_to_pointcloud2::convert<float>(depth_msg_ptr, cloud_msg, model, range_max_,
                                                  use_quiet_nan_, point_cloud_rotation_preset_,
                                                  cv_ptr);
      }
      else
      {
        RCLCPP_WARN(nh_->get_logger(), "Depth image has unsupported encoding [%s]",
                    depth_msg.encoding.c_str());
        // Can't return, so just continue to the next loop iteration
        continue;
      }

      point_cloud_pub_->publish(*cloud_msg);
    }
  }
}
