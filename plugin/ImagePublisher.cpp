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

constexpr char ATTR_FRAME_ID[]               = "frame_id";
constexpr char ATTR_COLOR_TOPIC_NAME[]       = "color_topic_name";
constexpr char ATTR_DEPTH_TOPIC_NAME[]       = "depth_topic_name";
constexpr char ATTR_INFO_TOPIC_NAME[]        = "info_topic_name";
constexpr char ATTR_POINT_CLOUD_TOPIC_NAME[] = "point_cloud_topic_name";
constexpr char ATTR_ROTATE_POINT_CLOUD[]     = "rotate_point_cloud";
constexpr char ATTR_HEIGHT[]                 = "height";
constexpr char ATTR_WIDTH[]                  = "width";
constexpr char ATTR_PUBLISH_RATE[]           = "publish_rate";
constexpr char ATTR_MAX_RANGE[]              = "max_range";

void ImagePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::ImagePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char *attributes[] = {ATTR_FRAME_ID,
                              ATTR_COLOR_TOPIC_NAME,
                              ATTR_DEPTH_TOPIC_NAME,
                              ATTR_INFO_TOPIC_NAME,
                              ATTR_POINT_CLOUD_TOPIC_NAME,
                              ATTR_ROTATE_POINT_CLOUD,
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
    auto *plugin_instance = ImagePublisher::Create(m, d, plugin_id);
    if (!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    auto *plugin_instance = reinterpret_cast<class ImagePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->free();
    delete reinterpret_cast<ImagePublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel *m, double *, // plugin_state
                     void *plugin_data, int plugin_id) {
    auto *plugin_instance = reinterpret_cast<class ImagePublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int // capability_bit
                    ) {
    auto *plugin_instance = reinterpret_cast<class ImagePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

ImagePublisher *ImagePublisher::Create(const mjModel *m, mjData *d, int plugin_id)
{
  // frame_id
  const char *frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_FRAME_ID);
  std::string frame_id      = "";
  if (strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
  }

  // color_topic_name
  const char *color_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_COLOR_TOPIC_NAME);
  std::string color_topic_name      = "";
  if (strlen(color_topic_name_char) > 0)
  {
    color_topic_name = std::string(color_topic_name_char);
  }

  // depth_topic_name
  const char *depth_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_DEPTH_TOPIC_NAME);
  std::string depth_topic_name      = "";
  if (strlen(depth_topic_name_char) > 0)
  {
    depth_topic_name = std::string(depth_topic_name_char);
  }

  // info_topic_name
  const char *info_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_INFO_TOPIC_NAME);
  std::string info_topic_name      = "";
  if (strlen(info_topic_name_char) > 0)
  {
    info_topic_name = std::string(info_topic_name_char);
  }

  // point_cloud_topic_name
  const char *point_cloud_topic_name_char
    = mj_getPluginConfig(m, plugin_id, ATTR_POINT_CLOUD_TOPIC_NAME);
  std::string point_cloud_topic_name = "";
  if (strlen(point_cloud_topic_name_char) > 0)
  {
    point_cloud_topic_name = std::string(point_cloud_topic_name_char);
  }

  // rotate_point_cloud
  const char *rotate_point_cloud_char = mj_getPluginConfig(m, plugin_id, ATTR_ROTATE_POINT_CLOUD);
  bool        rotate_point_cloud      = true;
  if (strlen(rotate_point_cloud_char) > 0)
  {
    if (strcmp(rotate_point_cloud_char, "false") == 0)
    {
      rotate_point_cloud = false;
    }
  }

  // height
  const char *height_char = mj_getPluginConfig(m, plugin_id, ATTR_HEIGHT);
  int         height      = 240;
  if (strlen(height_char) > 0)
  {
    height = static_cast<int>(strtol(height_char, nullptr, 10));
  }
  if (height <= 0)
  {
    mju_error("[ImagePublisher] `height` must be positive.");
    return nullptr;
  }

  // width
  const char *width_char = mj_getPluginConfig(m, plugin_id, ATTR_WIDTH);
  int         width      = 320;
  if (strlen(width_char) > 0)
  {
    width = static_cast<int>(strtol(width_char, nullptr, 10));
  }
  if (width <= 0)
  {
    mju_error("[ImagePublisher] `width` must be positive.");
    return nullptr;
  }

  // publish_rate
  const char *publish_rate_char = mj_getPluginConfig(m, plugin_id, ATTR_PUBLISH_RATE);
  mjtNum      publish_rate      = 30.0;
  if (strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if (publish_rate <= 0)
  {
    mju_error("[ImagePublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // max_range
  const char *max_range_char = mj_getPluginConfig(m, plugin_id, ATTR_MAX_RANGE);
  double      max_range      = 0.0;
  if (strlen(max_range_char) > 0)
  {
    max_range = strtod(max_range_char, nullptr);
    print_info("Max range: %s OR %f\n", max_range_char, max_range);
  }

  // Set sensor_id
  int sensor_id = 0;
  for (; sensor_id < m->nsensor; sensor_id++)
  {
    if (m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if (sensor_id == m->nsensor)
  {
    mju_error("[ImagePublisher] Plugin not found in sensors.");
    return nullptr;
  }
  if (m->sensor_objtype[sensor_id] != mjOBJ_CAMERA)
  {
    mju_error("[ImagePublisher] Plugin must be attached to a camera.");
    return nullptr;
  }

  std::cout << "[ImagePublisher] Create." << std::endl;

  return new ImagePublisher(m, d, sensor_id, frame_id, color_topic_name, depth_topic_name,
                            info_topic_name, point_cloud_topic_name, rotate_point_cloud, height,
                            width, publish_rate, max_range);
}

ImagePublisher::ImagePublisher(const mjModel *m,
                               mjData *, // d
                               int sensor_id, const std::string &frame_id,
                               std::string color_topic_name, std::string depth_topic_name,
                               std::string info_topic_name, std::string point_cloud_topic_name,
                               bool rotate_point_cloud, int height, int width, mjtNum publish_rate,
                               double max_range)
    : m_(m)
    , sensor_id_(sensor_id)
    , camera_id_(m->sensor_objid[sensor_id])
    , frame_id_(frame_id)
    , rotate_point_cloud_(rotate_point_cloud)
    , publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
    , viewport_({0, 0, width, height})
{
  std::string camera_name = std::string(mj_id2name(m, mjOBJ_CAMERA, camera_id_));
  if (frame_id_.empty())
  {
    frame_id_ = camera_name;
  }
  if (color_topic_name.empty())
  {
    color_topic_name = "mujoco/" + camera_name + "/color";
  }
  if (depth_topic_name.empty())
  {
    depth_topic_name = "mujoco/" + camera_name + "/depth";
  }
  if (info_topic_name.empty())
  {
    info_topic_name = "mujoco/" + camera_name + "/camera_info";
  }
  if (point_cloud_topic_name.empty())
  {
    point_cloud_topic_name = "mujoco/" + camera_name + "/point_cloud";
  }

  // Set GLFW error callback
  glfwSetErrorCallback([](int error, const char *description) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ImagePublisher"),
                        "GLFW Error " << error << ": " << description);
  });

  // Init OpenGL
  if (!glfwInit())
  {
    mju_error("[ImagePublisher] Could not initialize GLFW.");
  }

  // Create invisible window, single-buffered
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  window_ = glfwCreateWindow(viewport_.width, viewport_.height, "MujocoRosUtils::ImagePublisher",
                             nullptr, nullptr);
  if (!window_)
  {
    mju_error("[ImagePublisher] Could not create GLFW window.");
  }

  // Make context current
  // \todo Is it OK to override the current context of OpenGL?
  glfwMakeContextCurrent(window_);

  // Init default data for visualization structures
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);

  // Create scene and context
  mjv_makeScene(m, &scene_, 1000);
  mjr_makeContext(m, &context_, mjFONTSCALE_100);

  // Need to resize off-screen buffers
  mjr_resizeOffscreen(viewport_.width, viewport_.height, &context_);

  // Init camera
  camera_.type       = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;

  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  if (context_.currentBuffer != mjFB_OFFSCREEN)
  {
    mju_error(
      "[ImagePublisher] Offscreen rendering not supported, using default/window framebuffer.");
  }

  // Allocate buffer
  size_t color_buffer_size = sizeof(unsigned char) * 3 * viewport_.width * viewport_.height;
  size_t depth_buffer_size = sizeof(float) * viewport_.width * viewport_.height;

  color_buffer_.reset(new unsigned char[3 * width * height]);
  depth_buffer_.reset(new float[width * height]);
  color_buffer_flipped_.reset(new unsigned char[3 * width * height]);
  depth_buffer_flipped_.reset(new float[width * height]);
  color_buffer_back_.reset(new unsigned char[3 * width * height]);
  depth_buffer_back_.reset(new float[width * height]);

  if (!color_buffer_ || !depth_buffer_ || !color_buffer_flipped_ || !depth_buffer_flipped_
      || !color_buffer_back_ || !depth_buffer_back_)
  {
    mju_error("[ImagePublisher] Could not allocate image buffers.");
  }

  // Init ROS
  int    argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;

  node_options.parameter_overrides({
    {"use_sim_time", true}, // Force use simulation time
  });
  node_options.automatically_declare_parameters_from_overrides(true);

  std::string node_name = mj_id2name(m, mjOBJ_SENSOR, sensor_id);

  nh_        = rclcpp::Node::make_shared(node_name, node_options);
  auto qos   = rclcpp::SensorDataQoS();
  color_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(color_topic_name, qos);
  depth_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(depth_topic_name, qos);
  info_pub_  = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name, qos);
  point_cloud_pub_
    = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_name, qos);

  // If the ros2 params are provided, they will take precedence
  range_max_     = nh_->get_parameter_or("range_max", max_range);
  use_quiet_nan_ = nh_->get_parameter_or("use_quiet_nan", true);

  RCLCPP_INFO(nh_->get_logger(), "Setting depth range to %.4f", range_max_);

  publish_thread_ = std::thread(&ImagePublisher::publishThread, this);
}

void ImagePublisher::reset(const mjModel *, // m
                           int              // plugin_id
)
{}

void ImagePublisher::compute(const mjModel *m, mjData *d, int // plugin_id
)
{
  sim_cnt_++;
  if (sim_cnt_ % publish_skip_ != 0)
  {
    return;
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

  // Make context current
  // \todo Is it OK to override the current context of OpenGL?
  glfwMakeContextCurrent(window_);

  // Update abstract scene
  mjv_updateScene(m, d, &option_, nullptr, &camera_, mjCAT_STATIC | mjCAT_DYNAMIC, &scene_);

  // Render scene in offscreen buffer
  mjr_render(viewport_, &scene_, &context_);

  // Read rgb and depth pixels
  mjr_readPixels(color_buffer_.get(), depth_buffer_.get(), viewport_, &context_);

  // Publish color image directly from the compute thread for low latency
  if (publish_color_)
  {
    // Flip color buffer
#pragma omp parallel for
    for (int h = 0; h < viewport_.height; h++)
    {
      for (int w = 0; w < viewport_.width; w++)
      {
        int idx         = h * viewport_.width + w;
        int idx_flipped = (viewport_.height - 1 - h) * viewport_.width + w;
        for (int c = 0; c < 3; c++)
        {
          color_buffer_flipped_[3 * idx_flipped + c] = color_buffer_[3 * idx + c];
        }
      }
    }

    // Create and publish color_msg
    rclcpp::Time            stamp_now = nh_->get_clock()->now();
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
    }
    buffer_cv_.notify_one();
  }
}

void ImagePublisher::free()
{
  stop_thread_ = true;
  buffer_cv_.notify_all();
  if (publish_thread_.joinable())
  {
    publish_thread_.join();
  }

  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);

  glfwDestroyWindow(window_);
}

} // namespace MujocoRosUtils
void MujocoRosUtils::ImagePublisher::publishThread()
{
  while (!stop_thread_)
  {
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      buffer_cv_.wait(lock);
    }

    if (stop_thread_)
    {
      break;
    }

    // --- Process Depth Data ---
    float near = static_cast<float>(m_->vis.map.znear * m_->stat.extent);
    float far  = static_cast<float>(m_->vis.map.zfar * m_->stat.extent);
#pragma omp parallel for
    for (int h = 0; h < viewport_.height; h++)
    {
      for (int w = 0; w < viewport_.width; w++)
      {
        int idx         = h * viewport_.width + w;
        int idx_flipped = (viewport_.height - 1 - h) * viewport_.width + w;
        // Process depth
        depth_buffer_back_[idx] = near / (1.0f - depth_buffer_back_[idx] * (1.0f - near / far));
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

    // --- Publish Camera Info ---
    sensor_msgs::msg::CameraInfo info_msg;
    if (publish_info_ || publish_cloud_)
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
      if (publish_info_)
      {
        info_pub_->publish(info_msg);
      }
    }

    // --- Publish Point Cloud ---
    if (publish_cloud_)
    {
      // Flip the color buffer that was passed from the main thread
#pragma omp parallel for
      for (int h = 0; h < viewport_.height; h++)
      {
        for (int w = 0; w < viewport_.width; w++)
        {
          int idx         = h * viewport_.width + w;
          int idx_flipped = (viewport_.height - 1 - h) * viewport_.width + w;
          for (int c = 0; c < 3; c++)
          {
            color_buffer_flipped_[3 * idx_flipped + c] = color_buffer_back_[3 * idx + c];
          }
        }
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
                                                     use_quiet_nan_, rotate_point_cloud_, cv_ptr);
      }
      else if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        depthimage_to_pointcloud2::convert<float>(depth_msg_ptr, cloud_msg, model, range_max_,
                                                  use_quiet_nan_, rotate_point_cloud_, cv_ptr);
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
