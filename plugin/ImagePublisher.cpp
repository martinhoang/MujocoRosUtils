#define GL_GLEXT_PROTOTYPES
#define GL_GLEXT_LEGACY
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>

#include "ImagePublisher.h"
#include "depth_conversions.hpp"
#include "mujoco_utils.hpp"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mujoco/mujoco.h>
#include <sensor_msgs/image_encodings.hpp>

namespace MujocoRosUtils
{

constexpr char ATTR_COLOR_FRAME_ID[]              = "color_frame_id";
constexpr char ATTR_DEPTH_FRAME_ID[]              = "depth_frame_id";
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
constexpr char ATTR_MIN_RANGE[]                   = "min_range";
constexpr char ATTR_MAX_RANGE[]                   = "max_range";
constexpr char ATTR_READBACK_MODE[]               = "readback_mode";
constexpr char ATTR_PARALLEL_PROCESSING[]         = "parallel_processing";
constexpr char ATTR_POINT_CLOUD_DOWNSAMPLE[]      = "point_cloud_downsample";

void ImagePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::ImagePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char *attributes[] = {ATTR_COLOR_FRAME_ID,
                              ATTR_DEPTH_FRAME_ID,
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
                              ATTR_MIN_RANGE,
                              ATTR_MAX_RANGE,
                              ATTR_READBACK_MODE,
                              ATTR_PARALLEL_PROCESSING,
                              ATTR_POINT_CLOUD_DOWNSAMPLE};

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
  const char *color_frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_COLOR_FRAME_ID);
  std::string color_frame_id      = "";
  if (color_frame_id_char && strlen(color_frame_id_char) > 0)
  {
    color_frame_id = std::string(color_frame_id_char);
    print_debug("[ImagePublisher::Create] color_frame_id=%s\n", color_frame_id.c_str());
  }

  const char *depth_frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_DEPTH_FRAME_ID);
  std::string depth_frame_id      = color_frame_id;
  if (depth_frame_id_char && strlen(depth_frame_id_char) > 0)
  {
    depth_frame_id = std::string(depth_frame_id_char);
    print_debug("[ImagePublisher::Create] depth_frame_id=%s\n", depth_frame_id.c_str());
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
  bool        rotate_point_cloud      = false;
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
  std::string point_cloud_rotation_preset = depthimage_to_pointcloud2::PCL_ROT_NO_PRESET;
  if (rotate_point_cloud)
  {
    point_cloud_rotation_preset = depthimage_to_pointcloud2::PCL_ROT_PRESET_RDF_TO_FLU;
    if (point_cloud_rotation_preset_char && strlen(point_cloud_rotation_preset_char) > 0)
    {
      point_cloud_rotation_preset = point_cloud_rotation_preset_char;
    }
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

  // min_range
  print_debug("[ImagePublisher::Create] Reading min_range...\n");
  const char *min_range_char = mj_getPluginConfig(m, plugin_id, ATTR_MIN_RANGE);
  double      min_range      = 0.0;
  if (min_range_char && strlen(min_range_char) > 0)
  {
    min_range = strtod(min_range_char, nullptr);
    print_debug("[ImagePublisher::Create] min_range=%f\n", min_range);
  }
  else
  {
    print_debug("[ImagePublisher::Create] min_range not specified, using default=%f\n", min_range);
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
    print_debug("[ImagePublisher::Create] max_range not specified, using default=%f\n", max_range);
  }

  ImagePublisher::ReadbackMode readback_mode = ImagePublisher::ReadbackMode::Legacy;
  const char *readback_mode_char             = mj_getPluginConfig(m, plugin_id, ATTR_READBACK_MODE);
  if (readback_mode_char && strlen(readback_mode_char) > 0)
  {
    std::string readback_mode_str(readback_mode_char);
    std::transform(readback_mode_str.begin(), readback_mode_str.end(), readback_mode_str.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });

    if (readback_mode_str == "legacy")
    {
      readback_mode = ImagePublisher::ReadbackMode::Legacy;
      print_debug("[ImagePublisher::Create] readback_mode=legacy\n");
    }
    else if (readback_mode_str == "auto")
    {
      readback_mode = ImagePublisher::ReadbackMode::Auto;
      print_debug("[ImagePublisher::Create] readback_mode=auto\n");
    }
    else if (readback_mode_str == "pbo")
    {
      readback_mode = ImagePublisher::ReadbackMode::Pbo;
      print_debug("[ImagePublisher::Create] readback_mode=pbo\n");
    }
  }
  else
  {
    print_debug("[ImagePublisher::Create] readback_mode not specified, defaulting to legacy\n");
  }

  // parallel_processing
  print_debug("[ImagePublisher::Create] Reading parallel_processing...\n");
  const char *parallel_processing_char = mj_getPluginConfig(m, plugin_id, ATTR_PARALLEL_PROCESSING);
  bool        enable_parallel          = false;
  if (parallel_processing_char && strlen(parallel_processing_char) > 0)
  {
    std::string parallel_str(parallel_processing_char);
    std::transform(parallel_str.begin(), parallel_str.end(), parallel_str.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    if (parallel_str == "true" || parallel_str == "1")
    {
      enable_parallel = true;
    }
    else if (parallel_str == "false" || parallel_str == "0")
    {
      enable_parallel = false;
    }
    else
    {
      mju_error("[ImagePublisher] `parallel_processing` must be `true (1)` or `false (0)`, not %s.",
                parallel_processing_char);
      return nullptr;
    }
    print_debug("[ImagePublisher::Create] parallel_processing=%s\n",
                enable_parallel ? "true" : "false");
  }
  else
  {
    print_debug("[ImagePublisher::Create] parallel_processing not specified, using default=%s\n",
                enable_parallel ? "true" : "false");
  }

  // point_cloud_downsample
  const char *cloud_downsample_char = mj_getPluginConfig(m, plugin_id, ATTR_POINT_CLOUD_DOWNSAMPLE);
  int         point_cloud_downsample = 1;
  if (cloud_downsample_char && strlen(cloud_downsample_char) > 0)
  {
    point_cloud_downsample = static_cast<int>(strtol(cloud_downsample_char, nullptr, 10));
    if (point_cloud_downsample < 1)
    {
      mju_error("[ImagePublisher] `point_cloud_downsample` must be >= 1.");
      return nullptr;
    }
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
  print_debug(
    "[ImagePublisher::Create] Parameters: sensor_id=%d, width=%d, height=%d, publish_rate=%f\n",
    sensor_id, width, height, publish_rate);

  ImagePublisher *instance = new ImagePublisher(
    m, d, sensor_id, color_frame_id, depth_frame_id, topic_namespace, color_topic_name,
    depth_topic_name, info_topic_name, point_cloud_topic_name, rotate_point_cloud,
    point_cloud_rotation_preset, height, width, publish_rate, min_range, max_range, readback_mode,
    enable_parallel, point_cloud_downsample);

  print_confirm("[ImagePublisher::Create] Instance created successfully at %p\n", (void *)instance);
  return instance;
}

ImagePublisher::ImagePublisher(const mjModel *m,
                               mjData *, // d
                               int sensor_id, const std::string &color_frame_id,
                               const std::string &depth_frame_id,
                               const std::string &topic_namespace, std::string color_topic_name,
                               std::string depth_topic_name, std::string info_topic_name,
                               std::string point_cloud_topic_name, bool rotate_point_cloud,
                               const std::string &point_cloud_rotation_preset, int height,
                               int width, mjtNum publish_rate, double min_range, double max_range,
                               ReadbackMode readback_mode, bool enable_parallel,
                               int point_cloud_downsample)
    : m_(m)
    , sensor_id_(sensor_id)
    , camera_id_(m->sensor_objid[sensor_id])
    , color_frame_id_(color_frame_id)
    , depth_frame_id_(depth_frame_id)
    , readback_mode_(readback_mode)
    , rotate_point_cloud_(rotate_point_cloud)
    , point_cloud_rotation_preset_(point_cloud_rotation_preset)
    , publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
    , viewport_({0, 0, width, height})
    , enable_parallel_processing_(enable_parallel)
    , point_cloud_downsample_(std::max(point_cloud_downsample, 1))
    , last_fps_log_time_(std::chrono::steady_clock::now())
    , last_frame_time_(std::chrono::steady_clock::now())
    , last_color_log_(std::chrono::steady_clock::now())
{
  // Set PBO usage based on readback mode
  if (readback_mode_ == ReadbackMode::Legacy)
  {
    use_pbo_readback_ = false;
  }
  else if (readback_mode_ == ReadbackMode::Pbo)
  {
    use_pbo_readback_ = true;
  }
  else // Auto
  {
    use_pbo_readback_ = true; // Default to PBO for Auto mode
  }

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
  // info_topic_name configures the DEPTH camera_info; color camera_info always follows color_topic
  std::string depth_info_topic_name
    = topic_namespace
      + (info_topic_name.empty() ? camera_name + "/depth/camera_info" : info_topic_name);
  std::string color_info_topic_name = color_topic_name + "/camera_info";
  point_cloud_topic_name
    = topic_namespace
      + (point_cloud_topic_name.empty() ? camera_name + "/point_cloud" : point_cloud_topic_name);

  // Derive color frame_id by replacing depth_optical_frame suffix with color_optical_frame
  color_frame_id_ = frame_id_;
  {
    const std::string from = "depth_optical_frame";
    const std::string to   = "color_optical_frame";
    auto              pos  = color_frame_id_.rfind(from);
    if (pos != std::string::npos)
      color_frame_id_.replace(pos, from.size(), to);
  }

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
  // Reset all hints to defaults first — on sim relaunch glfwInit() is a no-op
  // (GLFW stays initialized), so leftover hints from the previous instance
  // would persist without this call.
  glfwDefaultWindowHints();
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_NATIVE_CONTEXT_API);
  // Use compatibility profile (requires OpenGL 3.2+) to allow MuJoCo's display lists
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
  print_debug("[ImagePublisher] Constructor: Creating GLFW window %dx%d\n", viewport_.width,
              viewport_.height);
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

  size_t color_buffer_size = sizeof(unsigned char) * 3 * viewport_.width * viewport_.height;
  size_t depth_buffer_size = sizeof(float) * viewport_.width * viewport_.height;

  if (use_pbo_readback_)
  {
    // Create PBOs for asynchronous GPU->CPU transfer (double buffering)
    print_debug("[ImagePublisher] Constructor: Creating PBOs for async transfer\n");
    glGenBuffers(PBO_COUNT, pbo_color_);
    glGenBuffers(PBO_COUNT, pbo_depth_);

    for (int i = 0; i < PBO_COUNT; i++)
    {
      glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_color_[i]);
      glBufferData(GL_PIXEL_PACK_BUFFER, color_buffer_size, nullptr, GL_STREAM_READ);
      print_debug("[ImagePublisher] Constructor: PBO color[%d] created (size=%zu)\n", i,
                  color_buffer_size);

      glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_depth_[i]);
      glBufferData(GL_PIXEL_PACK_BUFFER, depth_buffer_size, nullptr, GL_STREAM_READ);
      print_debug("[ImagePublisher] Constructor: PBO depth[%d] created (size=%zu)\n", i,
                  depth_buffer_size);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    print_debug("[ImagePublisher] Constructor: PBOs created successfully\n");
  }
  else
  {
    print_debug("[ImagePublisher] Constructor: Skipping PBO creation (legacy readback)\n");
  }

  // Allocate buffer
  print_debug("[ImagePublisher] Constructor: Allocating image buffers (%dx%d)\n", width, height);
  print_debug(
    "[ImagePublisher] Constructor: Color buffer size=%zu bytes, Depth buffer size=%zu bytes\n",
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
    color_buffer_flipped_back_.reset(new unsigned char[3 * width * height]);
    print_debug("[ImagePublisher] Constructor: color_buffer_flipped_back_ allocated\n");
  }
  catch (const std::bad_alloc &e)
  {
    mju_error("[ImagePublisher] Failed to allocate buffers: %s", e.what());
  }

  if (!color_buffer_ || !depth_buffer_ || !color_buffer_flipped_ || !depth_buffer_flipped_
      || !color_buffer_back_ || !depth_buffer_back_ || !color_buffer_flipped_back_)
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

  // Set logger level to INFO to see diagnostic messages
  auto ret
    = rcutils_logging_set_logger_level(nh_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  if (ret != RCUTILS_RET_OK)
  {
    print_debug("[ImagePublisher] WARNING: Failed to set logger level\n");
  }

  auto qos = rclcpp::SensorDataQoS();
  RCLCPP_DEBUG(nh_->get_logger(), "Creating publishers...");
  // color_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(color_topic_name, qos);
  image_transport::ImageTransport it(nh_);
  color_pub_      = it.advertise(color_topic_name, qos.get_rmw_qos_profile());
  depth_pub_      = it.advertise(depth_topic_name, qos.get_rmw_qos_profile());
  color_info_pub_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic_name, qos);
  depth_info_pub_ = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic_name, qos);
  point_cloud_pub_
    = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_name, qos);
  RCLCPP_DEBUG(nh_->get_logger(), "Publishers created");

  // If the ros2 params are provided, they will take precedence
  range_min_     = nh_->get_parameter_or("range_min", min_range);
  range_max_     = nh_->get_parameter_or("range_max", max_range);
  use_quiet_nan_ = nh_->get_parameter_or("use_quiet_nan", true);

  RCLCPP_INFO(nh_->get_logger(), "ImagePublisher initialized (range_min=%.2f, range_max=%.2f)",
              range_min_, range_max_);
  RCLCPP_INFO(nh_->get_logger(), "  Image size: %dx%d (%zu KB uncompressed)", viewport_.width,
              viewport_.height, static_cast<size_t>(3 * viewport_.width * viewport_.height) / 1024);
  RCLCPP_WARN(nh_->get_logger(),
              "  NOTE: Large images may cause image_transport compression bottleneck!");
  RCLCPP_WARN(nh_->get_logger(),
              "  If 'ros2 topic hz' shows low rate, try: --qos-reliability best_effort");
  RCLCPP_INFO(nh_->get_logger(), "  Target publish rate: %.1f Hz (skip=%d, timestep=%.4f)",
              publish_rate, publish_skip_, m->opt.timestep);
  RCLCPP_INFO(nh_->get_logger(), "  Readback mode: %s",
              use_pbo_readback_ ? "PBO (async GPU transfer)" : "Legacy (mjr_readPixels)");
#ifdef _OPENMP
  RCLCPP_INFO(nh_->get_logger(), "  Parallel processing: %s (OpenMP available)",
              enable_parallel_processing_ ? "ENABLED" : "DISABLED");
#else
  RCLCPP_INFO(nh_->get_logger(), "  Parallel processing: DISABLED (OpenMP not compiled)");
#endif

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
    if (first_compute_warning_)
    {
      RCLCPP_WARN(nh_->get_logger(), "compute() called before initialization complete, skipping");
      first_compute_warning_ = false;
    }
    return;
  }

  sim_cnt_++;
  if (sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  compute_call_count_++;
  if (compute_call_count_ <= 5 || compute_call_count_ % 100 == 0)
  {
    RCLCPP_DEBUG(nh_->get_logger(), "compute() call #%d", compute_call_count_);
  }

  // Update subscriber counts
  publish_color_ = color_pub_.getNumSubscribers() > 0;
  publish_depth_ = depth_pub_.getNumSubscribers() > 0;
  publish_info_  = color_info_pub_->get_subscription_count() > 0
                  || depth_info_pub_->get_subscription_count() > 0;
  publish_cloud_ = point_cloud_pub_->get_subscription_count() > 0;

  // Log subscriber info periodically
  RCLCPP_DEBUG_THROTTLE(
    nh_->get_logger(), *nh_->get_clock(), 2000,
    "Active subscribers: color=%zu depth=%zu color_info=%zu depth_info=%zu cloud=%zu",
    static_cast<size_t>(color_pub_.getNumSubscribers()),
    static_cast<size_t>(depth_pub_.getNumSubscribers()),
    static_cast<size_t>(color_info_pub_->get_subscription_count()),
    static_cast<size_t>(depth_info_pub_->get_subscription_count()),
    static_cast<size_t>(point_cloud_pub_->get_subscription_count()));

  // If no one is listening, do nothing
  if (!publish_color_ && !publish_depth_ && !publish_info_ && !publish_cloud_)
  {
    return;
  }

  // FPS tracking - only count frames that are actually published
  auto now = std::chrono::steady_clock::now();
  frame_count_++;
  auto elapsed_since_log
    = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_log_time_).count();

  if (elapsed_since_log >= 2000) // Log every 2 seconds
  {
    double fps = frame_count_ * 1000.0 / elapsed_since_log;
    RCLCPP_DEBUG(nh_->get_logger(), "Published FPS: %.2f (frames=%d, elapsed=%ldms)", fps,
                 frame_count_, elapsed_since_log);
    frame_count_       = 0;
    last_fps_log_time_ = now;
  }

  if (compute_call_count_ <= 5)
  {
    RCLCPP_DEBUG(nh_->get_logger(), "Subscribers: color=%d depth=%d info=%d cloud=%d",
                 publish_color_, publish_depth_, publish_info_, publish_cloud_);
  }

  // Protect OpenGL context operations with mutex to prevent multi-threaded access
  try
  {
    std::lock_guard<std::mutex> gl_lock(gl_context_mutex_);

    auto gl_start = std::chrono::steady_clock::now();

    if (compute_call_count_ <= 3)
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

    if (use_pbo_readback_)
    {
      auto wait_for_sync = [this](int sync_index) {
        if (!pbo_sync_[sync_index])
        {
          return;
        }

        constexpr GLuint64 timeout_ns = 1000000ULL; // 1 ms per wait call
        int                attempts   = 0;
        GLenum             wait_state = GL_TIMEOUT_EXPIRED;

        while (wait_state == GL_TIMEOUT_EXPIRED && attempts < 200)
        {
          wait_state
            = glClientWaitSync(pbo_sync_[sync_index], GL_SYNC_FLUSH_COMMANDS_BIT, timeout_ns);
          attempts++;
        }

        if (wait_state == GL_WAIT_FAILED && compute_call_count_ <= 5)
        {
          RCLCPP_WARN(nh_->get_logger(),
                      "glClientWaitSync failed for PBO index %d after %d attempts", sync_index,
                      attempts);
        }

        glDeleteSync(pbo_sync_[sync_index]);
        pbo_sync_[sync_index] = nullptr;
      };

      if (context_.currentBuffer != mjFB_WINDOW)
      {
        if (context_.offSamples)
        {
          glBindFramebuffer(GL_READ_FRAMEBUFFER, context_.offFBO);
          glReadBuffer(GL_COLOR_ATTACHMENT0);
          glBindFramebuffer(GL_DRAW_FRAMEBUFFER, context_.offFBO_r);
          glDrawBuffer(GL_COLOR_ATTACHMENT0);

          glBlitFramebuffer(viewport_.left, viewport_.bottom, viewport_.left + viewport_.width,
                            viewport_.bottom + viewport_.height, viewport_.left, viewport_.bottom,
                            viewport_.left + viewport_.width, viewport_.bottom + viewport_.height,
                            GL_COLOR_BUFFER_BIT, GL_NEAREST);

          glBindFramebuffer(GL_READ_FRAMEBUFFER, context_.offFBO_r);
          glReadBuffer(GL_COLOR_ATTACHMENT0);
        }
        else
        {
          glBindFramebuffer(GL_READ_FRAMEBUFFER, context_.offFBO);
          glReadBuffer(GL_COLOR_ATTACHMENT0);
        }

        GLenum fbo_status = glCheckFramebufferStatus(GL_READ_FRAMEBUFFER);
        if (fbo_status != GL_FRAMEBUFFER_COMPLETE && compute_call_count_ <= 5)
        {
          RCLCPP_ERROR(nh_->get_logger(), "Framebuffer incomplete! Status: 0x%x", fbo_status);
        }
      }
      else
      {
        glReadBuffer(GL_BACK);
      }

      pbo_index_      = pbo_frame_count_ % PBO_COUNT;
      pbo_next_index_ = (pbo_frame_count_ + 1) % PBO_COUNT;

      if (pbo_sync_[pbo_index_])
      {
        glDeleteSync(pbo_sync_[pbo_index_]);
        pbo_sync_[pbo_index_] = nullptr;
      }

      glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_color_[pbo_index_]);
      glReadPixels(viewport_.left, viewport_.bottom, viewport_.width, viewport_.height, GL_RGB,
                   GL_UNSIGNED_BYTE, 0);

      GLenum color_error = glGetError();
      if (color_error != GL_NO_ERROR && compute_call_count_ <= 5)
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "glReadPixels color error: 0x%x (using GL_RGB=0x%x, context format was 0x%x)",
                     color_error, GL_RGB, context_.readPixelFormat);
      }

      glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_depth_[pbo_index_]);
      glReadPixels(viewport_.left, viewport_.bottom, viewport_.width, viewport_.height,
                   GL_DEPTH_COMPONENT, GL_FLOAT, 0);

      GLenum depth_error = glGetError();
      if (depth_error != GL_NO_ERROR && compute_call_count_ <= 5)
      {
        RCLCPP_ERROR(nh_->get_logger(), "glReadPixels depth error: 0x%x", depth_error);
      }

      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

      pbo_sync_[pbo_index_] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
      if (!pbo_sync_[pbo_index_] && compute_call_count_ <= 5)
      {
        RCLCPP_WARN(nh_->get_logger(), "Failed to create GL fence for PBO index %d", pbo_index_);
      }

      if (pbo_frame_count_ >= PBO_COUNT)
      {
        wait_for_sync(pbo_next_index_);

        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_color_[pbo_next_index_]);
        unsigned char *color_ptr
          = static_cast<unsigned char *>(glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY));
        if (color_ptr)
        {
          std::memcpy(color_buffer_.get(), color_ptr,
                      sizeof(unsigned char) * 3 * viewport_.width * viewport_.height);
          glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "Failed to map color PBO!");
        }

        glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_depth_[pbo_next_index_]);
        float *depth_ptr = static_cast<float *>(glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY));
        if (depth_ptr)
        {
          const size_t pixel_count
            = static_cast<size_t>(viewport_.width) * static_cast<size_t>(viewport_.height);
          std::memcpy(depth_buffer_.get(), depth_ptr, sizeof(float) * pixel_count);

          // glReadPixels returns the raw GPU depth map (reversed Z in MuJoCo's pipeline).
          // When readDepthMap requests ZERONEAR output, mirror mjr_readPixels behavior here.
          if (context_.readDepthMap == mjDEPTH_ZERONEAR)
          {
            for (size_t i = 0; i < pixel_count; i++)
            {
              depth_buffer_[i] = 1.0f - depth_buffer_[i];
            }
          }

          glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "Failed to map depth PBO!");
        }

        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
      }
      else
      {
        mjr_readPixels(color_buffer_.get(), depth_buffer_.get(), viewport_, &context_);
      }

      if (context_.currentBuffer != mjFB_WINDOW)
      {
        mjr_restoreBuffer(&context_);
      }

      pbo_frame_count_++;

      if (compute_call_count_ <= 3)
      {
        RCLCPP_DEBUG(nh_->get_logger(), "GL operations complete (PBO frame %d)", pbo_frame_count_);
      }
    }
    else
    {
      mjr_readPixels(color_buffer_.get(), depth_buffer_.get(), viewport_, &context_);
    }

    // Release the plugin's GL context so the MuJoCo viewer can reclaim its
    // own context cleanly on the next frame.  Without this, the viewer must
    // always call glfwMakeContextCurrent under contention, which can produce
    // inconsistent GL state on simulation relaunch.
    glfwMakeContextCurrent(nullptr);

    auto gl_end = std::chrono::steady_clock::now();
    auto gl_ms
      = std::chrono::duration_cast<std::chrono::microseconds>(gl_end - gl_start).count() / 1000.0;

    total_gl_time_ += gl_ms;
    gl_sample_count_++;

    if (elapsed_since_log >= 2000)
    {
      if (gl_sample_count_ > 0)
      {
        double avg_gl_ms = total_gl_time_ / gl_sample_count_;
        RCLCPP_DEBUG(nh_->get_logger(), "  GL render+readback: avg %.2fms per frame", avg_gl_ms);
      }
      total_gl_time_   = 0.0;
      gl_sample_count_ = 0;
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Exception during GL operations: %s", e.what());
    return;
  }

  // Get timestamp for all messages
  rclcpp::Time stamp_now = nh_->get_clock()->now();

  // Publish camera info for both color and depth
  if (publish_info_)
  {
    color_info_pub_->publish(buildCameraInfo(stamp_now, color_frame_id_));
    depth_info_pub_->publish(buildCameraInfo(stamp_now, frame_id_));
  }

  // Flip color buffer once if needed by any subscriber
  bool need_color_flip = publish_color_ || publish_cloud_;
  if (need_color_flip)
  {
    auto flip_start = std::chrono::steady_clock::now();

    // Flip color buffer AND convert RGB→BGR to avoid cv_bridge color conversion issues
    const int row_bytes = viewport_.width * 3;
    auto      flip_row  = [&](int h) {
      const unsigned char *src_row = color_buffer_.get() + h * row_bytes;
      unsigned char *dst_row = color_buffer_flipped_.get() + (viewport_.height - 1 - h) * row_bytes;

      // Convert RGB→BGR while copying (swap R and B channels)
      for (int w = 0; w < viewport_.width; w++)
      {
        dst_row[w * 3 + 0] = src_row[w * 3 + 2]; // B ← R
        dst_row[w * 3 + 1] = src_row[w * 3 + 1]; // G ← G
        dst_row[w * 3 + 2] = src_row[w * 3 + 0]; // R ← B
      }
    };

#ifdef _OPENMP
    if (shouldParallelizeRows(viewport_.height))
    {
#pragma omp parallel for schedule(static)
      for (int h = 0; h < viewport_.height; h++)
      {
        flip_row(h);
      }
    }
    else
#endif
    {
      for (int h = 0; h < viewport_.height; h++)
      {
        flip_row(h);
      }
    }

    auto flip_end = std::chrono::steady_clock::now();
    auto flip_ms
      = std::chrono::duration_cast<std::chrono::microseconds>(flip_end - flip_start).count()
        / 1000.0;

    total_flip_time_ += flip_ms;
    flip_sample_count_++;

    if (elapsed_since_log >= 2000)
    {
      if (flip_sample_count_ > 0)
      {
        double      avg_flip_ms = total_flip_time_ / flip_sample_count_;
        const char *mode        = shouldParallelizeRows(viewport_.height) ? "parallel" : "serial";
        RCLCPP_DEBUG(nh_->get_logger(), "  Color flip (%s): avg %.2fms per frame", mode,
                     avg_flip_ms);
      }
      total_flip_time_   = 0.0;
      flip_sample_count_ = 0;
    }
  }

  // Publish color image directly from the compute thread for low latency
  // NOTE: Do NOT use 'return' here — the depth/cloud buffer swap below must always run
  if (publish_color_)
  {
    bool color_ok = true;

    // Validate viewport dimensions before creating message
    if (viewport_.width <= 0 || viewport_.height <= 0 || viewport_.width > 10000
        || viewport_.height > 10000)
    {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                            "Invalid viewport dimensions: %dx%d", viewport_.width,
                            viewport_.height);
      color_ok = false;
    }

    // Validate buffer pointer
    if (color_ok && !color_buffer_flipped_)
    {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                            "color_buffer_flipped_ is null!");
      color_ok = false;
    }

    if (color_ok)
    {
      // Create and publish color_msg
      sensor_msgs::msg::Image color_msg;
      color_msg.header.stamp    = stamp_now;
      color_msg.header.frame_id = color_frame_id_;
      color_msg.height          = static_cast<uint32_t>(viewport_.height);
      color_msg.width           = static_cast<uint32_t>(viewport_.width);
      color_msg.encoding        = "bgr8";
      color_msg.is_bigendian    = 0;
      color_msg.step            = static_cast<uint32_t>(3) * static_cast<uint32_t>(viewport_.width);

      const size_t required_size = static_cast<size_t>(3) * static_cast<size_t>(viewport_.width)
                                   * static_cast<size_t>(viewport_.height);

      try
      {
        color_msg.data.resize(required_size);
        std::memcpy(&color_msg.data[0], color_buffer_flipped_.get(), required_size);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to prepare color message: %s", e.what());
        color_ok = false;
      }

      if (color_ok)
      {
        color_pub_.publish(color_msg);

        color_pub_count_++;

        auto now_pub = std::chrono::steady_clock::now();
        auto elapsed_pub
          = std::chrono::duration_cast<std::chrono::milliseconds>(now_pub - last_color_log_)
              .count();
        if (elapsed_pub >= 2000)
        {
          double pub_fps     = color_pub_count_ * 1000.0 / elapsed_pub;
          size_t msg_size_kb = color_msg.data.size() / 1024;
          RCLCPP_DEBUG(nh_->get_logger(), "  Color published: %.2f Hz (%d msgs, %zu KB each)",
                       pub_fps, color_pub_count_, msg_size_kb);
          color_pub_count_ = 0;
          last_color_log_  = now_pub;
        }
      }
    }
  }

  // Offload depth and point cloud processing to the worker thread
  if (publish_depth_ || publish_cloud_)
  {
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      // Swap buffers to hand off work to publish thread
      // For point cloud, we need the RGB buffer (not BGR-converted), so swap the original
      color_buffer_.swap(color_buffer_back_);
      depth_buffer_.swap(depth_buffer_back_);
      data_ready_ = true;
    }
    buffer_cv_.notify_one();
  }
}

bool ImagePublisher::shouldParallelizeRows(int rows) const
{
#ifdef _OPENMP
  return enable_parallel_processing_ && (rows >= kParallelRowThreshold);
#else
  (void)rows;
  return false;
#endif
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

  // All GL cleanup MUST happen with the plugin's own context current.
  // Without this, the calls silently corrupt the viewer's GL state or leak
  // GPU resources, causing incorrect depth rendering on the next plugin init.
  if (window_)
  {
    glfwMakeContextCurrent(window_);
  }

  RCLCPP_DEBUG(nh_->get_logger(), "Deleting PBOs");
  for (int i = 0; i < PBO_COUNT; ++i)
  {
    if (pbo_sync_[i])
    {
      glDeleteSync(pbo_sync_[i]);
      pbo_sync_[i] = nullptr;
    }
  }
  if (pbo_color_[0] != 0)
  {
    glDeleteBuffers(PBO_COUNT, pbo_color_);
    glDeleteBuffers(PBO_COUNT, pbo_depth_);
  }

  RCLCPP_DEBUG(nh_->get_logger(), "Freeing MuJoCo context and scene");
  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);

  RCLCPP_DEBUG(nh_->get_logger(), "Destroying GLFW window");
  if (window_)
  {
    // Release context before destroying window so the viewer can cleanly
    // reclaim its own context on the next frame.
    glfwMakeContextCurrent(nullptr);
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  RCLCPP_INFO(nh_->get_logger(), "ImagePublisher cleanup complete");
}

sensor_msgs::msg::CameraInfo ImagePublisher::buildCameraInfo(const rclcpp::Time &stamp,
                                                             const std::string  &frame_id) const
{
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header.stamp     = stamp;
  info_msg.header.frame_id  = frame_id;
  info_msg.height           = viewport_.height;
  info_msg.width            = viewport_.width;
  info_msg.distortion_model = "plumb_bob";
  info_msg.d.resize(5, 0.0);
  info_msg.k.fill(0.0);
  info_msg.r.fill(0.0);
  info_msg.p.fill(0.0);
  double focal_scaling
    = (1.0 / std::tan((m_->cam_fovy[camera_id_] * M_PI / 180.0) / 2.0)) * viewport_.height / 2.0;
  info_msg.k[0] = info_msg.p[0] = focal_scaling;
  info_msg.k[2] = info_msg.p[2] = static_cast<double>(viewport_.width) / 2.0;
  info_msg.k[4] = info_msg.p[5] = focal_scaling;
  info_msg.k[5] = info_msg.p[6] = static_cast<double>(viewport_.height) / 2.0;
  info_msg.k[8] = info_msg.p[10] = 1.0;
  return info_msg;
}

void ImagePublisher::publishThread()
{
  RCLCPP_DEBUG(nh_->get_logger(), "publishThread started");
  print_confirm("ImagePublisher publish thread started\n");

  auto last_publish_log    = std::chrono::steady_clock::now();
  int  publish_frame_count = 0;

  while (!stop_thread_)
  {
    RCLCPP_DEBUG(nh_->get_logger(), "publishThread is waiting for data...");

    // Snapshot buffer pointers under lock to prevent race condition with compute thread's swap
    float         *local_depth_back    = nullptr;
    float         *local_depth_flipped = nullptr;
    unsigned char *local_color_back    = nullptr;
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

      // Snapshot raw pointers while lock is held - this prevents the compute
      // thread's swap from changing what we're processing mid-iteration
      local_depth_back    = depth_buffer_back_.get();
      local_depth_flipped = depth_buffer_flipped_.get();
      local_color_back    = color_buffer_back_.get(); // RGB buffer, not BGR-converted
    }

    auto thread_start = std::chrono::steady_clock::now();
    publish_frame_count++;

    // Null pointer safety check
    if (!m_ || !local_depth_back)
    {
      RCLCPP_ERROR(nh_->get_logger(), "[ImagePublisher] Null pointer detected in publish thread");
      continue;
    }

    auto depth_start = std::chrono::steady_clock::now();

    // --- Process Depth Data ---
    float near = static_cast<float>(m_->vis.map.znear * m_->stat.extent);
    float far  = static_cast<float>(m_->vis.map.zfar * m_->stat.extent);

    // Precompute constant for depth conversion
    const float depth_scale = 1.0f - near / far;

    // Debug: Log near/far values periodically
    if (depth_log_count_ == 0)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Depth conversion: near=%.4f, far=%.4f, depth_scale=%.6f",
                   near, far, depth_scale);
    }
    depth_log_count_ = (depth_log_count_ + 1) % 100;

    auto process_depth_row = [&](int row) {
      const int row_offset         = row * viewport_.width;
      const int flipped_row_offset = (viewport_.height - 1 - row) * viewport_.width;
      for (int col = 0; col < viewport_.width; col++)
      {
        const int idx = row_offset + col;

        // Normalize depth map to ZERONEAR convention (0: near, 1: far) before linearization.
        float depth_map_value = local_depth_back[idx];
        if (context_.readDepthMap == mjDEPTH_ZEROFAR)
        {
          depth_map_value = 1.0f - depth_map_value;
        }

        const float depth_value = near / (1.0f - depth_map_value * depth_scale);
        local_depth_back[idx]   = depth_value;
        local_depth_flipped[flipped_row_offset + col] = depth_value;
      }
    };

#ifdef _OPENMP
    if (shouldParallelizeRows(viewport_.height))
    {
#pragma omp parallel for schedule(static)
      for (int h = 0; h < viewport_.height; h++)
      {
        process_depth_row(h);
      }
    }
    else
#endif
    {
      for (int h = 0; h < viewport_.height; h++)
      {
        process_depth_row(h);
      }
    }

    // Debug: Sample some depth values
    if (depth_log_count_ == 1)
    {
      int   center_idx   = (viewport_.height / 2) * viewport_.width + (viewport_.width / 2);
      float center_depth = local_depth_flipped[center_idx];
      float min_depth    = local_depth_flipped[0];
      float max_depth    = local_depth_flipped[0];
      for (int i = 0; i < viewport_.width * viewport_.height; i++)
      {
        min_depth = std::min(min_depth, local_depth_flipped[i]);
        max_depth = std::max(max_depth, local_depth_flipped[i]);
      }
      RCLCPP_DEBUG(nh_->get_logger(), "Depth values: min=%.4f, max=%.4f, center=%.4f", min_depth,
                   max_depth, center_depth);
    }

    auto depth_end = std::chrono::steady_clock::now();
    auto depth_ms
      = std::chrono::duration_cast<std::chrono::microseconds>(depth_end - depth_start).count()
        / 1000.0;

    rclcpp::Time stamp_now = nh_->get_clock()->now();

    // --- Publish Depth Image ---
    sensor_msgs::msg::Image depth_msg;
    if (publish_depth_ || publish_cloud_) // Also need depth_msg for point cloud
    {
      depth_msg.header.stamp    = stamp_now;
      depth_msg.header.frame_id = depth_frame_id_;
      depth_msg.height          = viewport_.height;
      depth_msg.width           = viewport_.width;
      depth_msg.encoding        = "32FC1";
      depth_msg.is_bigendian    = 0;
      depth_msg.step            = static_cast<unsigned int>(sizeof(float) * viewport_.width);
      depth_msg.data.resize(sizeof(float) * viewport_.width * viewport_.height);
      std::memcpy(&depth_msg.data[0], local_depth_flipped,
                  sizeof(float) * viewport_.width * viewport_.height);
      if (publish_depth_)
      {
        depth_pub_.publish(depth_msg);
      }
    }

    // --- Create Camera Info for Point Cloud (if needed) ---
    sensor_msgs::msg::CameraInfo info_msg;
    if (publish_cloud_)
    {
      info_msg = buildCameraInfo(stamp_now, frame_id_);
    }

    // --- Publish Point Cloud ---
    if (publish_cloud_)
    {
      // Flip the RGB color buffer vertically for point cloud (RGB, not BGR)
      // We need to flip because OpenGL reads bottom-to-top but ROS images are top-to-bottom
      const size_t pixel_count_rgb = static_cast<size_t>(viewport_.width) * viewport_.height * 3;
      color_flipped_rgb_.resize(pixel_count_rgb);

      const int row_bytes = viewport_.width * 3;
      for (int h = 0; h < viewport_.height; h++)
      {
        const unsigned char *src_row = local_color_back + h * row_bytes;
        unsigned char *dst_row = color_flipped_rgb_.data() + (viewport_.height - 1 - h) * row_bytes;
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
      color_msg_for_cloud.data.assign(color_flipped_rgb_.begin(), color_flipped_rgb_.end());

      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);

      // Debug: Log point cloud generation parameters
      if (pcl_log_count_ == 0)
      {
        RCLCPP_DEBUG(nh_->get_logger(),
                     "PointCloud params: range_min_=%.4f, range_max_=%.4f, use_quiet_nan_=%d",
                     range_min_, range_max_, use_quiet_nan_);
        RCLCPP_DEBUG(nh_->get_logger(), "Camera model: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                     model.fx(), model.fy(), model.cx(), model.cy());
      }
      pcl_log_count_ = (pcl_log_count_ + 1) % 100;

      sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg
        = std::make_shared<sensor_msgs::msg::PointCloud2>();
      cloud_msg->header       = depth_msg.header;
      cloud_msg->is_dense     = false;
      cloud_msg->is_bigendian = false;

      // --- Downsample depth and color for cloud if requested ---
      // This shrinks the images before cloud conversion: O(W*H / k²) instead of O(W*H).
      // The rendered image is untouched; only the cloud resolution is reduced.
      sensor_msgs::msg::Image::ConstSharedPtr depth_msg_for_cloud;
      cv_bridge::CvImageConstPtr              cv_ptr_for_cloud;
      image_geometry::PinholeCameraModel      cloud_model = model;

      if (point_cloud_downsample_ > 1)
      {
        const int small_w = std::max(1, viewport_.width / point_cloud_downsample_);
        const int small_h = std::max(1, viewport_.height / point_cloud_downsample_);

        // Resize depth image (nearest-neighbour to preserve depth values)
        cv::Mat depth_full(
          viewport_.height, viewport_.width, CV_32FC1,
          const_cast<float *>(reinterpret_cast<const float *>(depth_msg.data.data())));
        cv::Mat depth_small;
        cv::resize(depth_full, depth_small, cv::Size(small_w, small_h), 0, 0, cv::INTER_NEAREST);

        auto depth_small_msg          = std::make_shared<sensor_msgs::msg::Image>();
        depth_small_msg->header       = depth_msg.header;
        depth_small_msg->height       = small_h;
        depth_small_msg->width        = small_w;
        depth_small_msg->encoding     = depth_msg.encoding;
        depth_small_msg->is_bigendian = 0;
        depth_small_msg->step         = static_cast<unsigned int>(sizeof(float) * small_w);
        depth_small_msg->data.resize(sizeof(float) * small_w * small_h);
        std::memcpy(depth_small_msg->data.data(), depth_small.data,
                    sizeof(float) * small_w * small_h);
        depth_msg_for_cloud = depth_small_msg;

        // Resize color image
        cv::Mat rgb_full(viewport_.height, viewport_.width, CV_8UC3,
                         const_cast<unsigned char *>(color_msg_for_cloud.data.data()));
        cv::Mat rgb_small;
        cv::resize(rgb_full, rgb_small, cv::Size(small_w, small_h), 0, 0, cv::INTER_LINEAR);
        auto cv_small = std::make_shared<cv_bridge::CvImage>(
          color_msg_for_cloud.header, sensor_msgs::image_encodings::RGB8, rgb_small.clone());
        cv_ptr_for_cloud = cv_small;

        // Scale camera intrinsics
        const double                 scale       = 1.0 / point_cloud_downsample_;
        sensor_msgs::msg::CameraInfo scaled_info = buildCameraInfo(stamp_now, frame_id_);
        scaled_info.width                        = small_w;
        scaled_info.height                       = small_h;
        scaled_info.k[0] *= scale; // fx
        scaled_info.k[2] *= scale; // cx
        scaled_info.k[4] *= scale; // fy
        scaled_info.k[5] *= scale; // cy
        scaled_info.p[0] *= scale; // fx
        scaled_info.p[2] *= scale; // cx
        scaled_info.p[5] *= scale; // fy
        scaled_info.p[6] *= scale; // cy
        cloud_model.fromCameraInfo(scaled_info);

        cloud_msg->height = small_h;
        cloud_msg->width  = small_w;
      }
      else
      {
        // No downsampling
        cv::Mat rgb_mat(viewport_.height, viewport_.width, CV_8UC3,
                        const_cast<unsigned char *>(color_msg_for_cloud.data.data()));
        cv_ptr_for_cloud = std::make_shared<cv_bridge::CvImage>(
          color_msg_for_cloud.header, sensor_msgs::image_encodings::RGB8, rgb_mat);
        depth_msg_for_cloud = std::shared_ptr<sensor_msgs::msg::Image>(
          const_cast<sensor_msgs::msg::Image *>(&depth_msg), [](const sensor_msgs::msg::Image *) {
          });

        cloud_msg->height = depth_msg.height;
        cloud_msg->width  = depth_msg.width;
      }

      sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
      pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

      if (depth_msg_for_cloud->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        depthimage_to_pointcloud2::convert<uint16_t>(
          depth_msg_for_cloud, cloud_msg, cloud_model, range_max_, use_quiet_nan_,
          rotate_point_cloud_ ? point_cloud_rotation_preset_
                              : depthimage_to_pointcloud2::PCL_ROT_NO_PRESET,
          cv_ptr_for_cloud, range_min_);
      }
      else if (depth_msg_for_cloud->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        depthimage_to_pointcloud2::convert<float>(
          depth_msg_for_cloud, cloud_msg, cloud_model, range_max_, use_quiet_nan_,
          rotate_point_cloud_ ? point_cloud_rotation_preset_
                              : depthimage_to_pointcloud2::PCL_ROT_NO_PRESET,
          cv_ptr_for_cloud, range_min_);
      }
      else
      {
        RCLCPP_WARN(nh_->get_logger(), "Depth image has unsupported encoding [%s]",
                    depth_msg_for_cloud->encoding.c_str());
        continue;
      }

      point_cloud_pub_->publish(*cloud_msg);
    }

    auto thread_end = std::chrono::steady_clock::now();
    auto thread_ms
      = std::chrono::duration_cast<std::chrono::microseconds>(thread_end - thread_start).count()
        / 1000.0;

    total_depth_time_ += depth_ms;
    total_thread_time_ += thread_ms;
    thread_sample_count_++;

    auto elapsed_since_log
      = std::chrono::duration_cast<std::chrono::milliseconds>(thread_end - last_publish_log)
          .count();

    if (elapsed_since_log >= 2000)
    {
      if (thread_sample_count_ > 0)
      {
        double      avg_depth_ms  = total_depth_time_ / thread_sample_count_;
        double      avg_thread_ms = total_thread_time_ / thread_sample_count_;
        const char *mode          = shouldParallelizeRows(viewport_.height) ? "parallel" : "serial";

        RCLCPP_DEBUG(nh_->get_logger(), "  Depth processing (%s): avg %.2fms per frame", mode,
                     avg_depth_ms);
        RCLCPP_DEBUG(nh_->get_logger(),
                     "  Publish thread total: avg %.2fms per frame (depth+cloud)", avg_thread_ms);
      }

      total_depth_time_    = 0.0;
      total_thread_time_   = 0.0;
      thread_sample_count_ = 0;
      last_publish_log     = thread_end;
    }
  }
}

} // namespace MujocoRosUtils
