#pragma once

#include <image_geometry/pinhole_camera_model.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <GLFW/glfw3.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace MujocoRosUtils
{

/** \brief Plugin to publish topics of color and depth images. */
class ImagePublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ImagePublisher *Create(const mjModel *m, mjData *d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ImagePublisher(ImagePublisher &&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel *m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel *m, mjData *d, int plugin_id);

  /** \brief  Convert depth image to PointCloud2
   *
   */
  static void convert(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                      sensor_msgs::msg::PointCloud2::SharedPtr      &cloud_msg,
                      const image_geometry::PinholeCameraModel      &model);

  /** \brief Free buffer. */
  void free();

protected:
  /** \brief Publishing loop for the worker thread. */
  void publishThread();
  /** \brief Constructor.
      \param m model
      \param d data
      \param sensor_id sensor ID
      \param frame_id frame ID of topics header or TF parent
      \param topic_namespace namespace prefix for all topics
      \param color_topic_name topic name of color image
      \param depth_topic_name topic name of depth image
      \param info_topic_name topic name of camera information
      \param point_cloud_topic_name topic name of point cloud
      \param rotate_point_cloud whether to rotate point cloud to align with ROS coordinate system
      \param point_cloud_rotation_preset preset for point cloud rotation
      \param height image height
      \param width image width
      \param publish_rate publish rate
  */
  ImagePublisher(const mjModel *m, mjData *d, int sensor_id, const std::string &frame_id,
                 const std::string &topic_namespace,
                 std::string color_topic_name, std::string depth_topic_name,
                 std::string info_topic_name, std::string point_cloud_topic_name,
                 bool rotate_point_cloud, const std::string &point_cloud_rotation_preset,
                 int height, int width, mjtNum publish_rate, double max_range);

protected:
  //! MuJoCo model
  const mjModel *m_;

  //! Sensor ID
  int sensor_id_ = -1;

  //! Camera ID
  int camera_id_ = -1;

  //! Frame ID
  std::string frame_id_ = "";

  //! Rotate point cloud
  bool        rotate_point_cloud_          = false;
  std::string point_cloud_rotation_preset_ = "";

  //! Number of steps to skip between publications
  int publish_skip_ = 1;

  //! Viewport
  mjrRect viewport_;

  //! Iteration count of simulation
  int sim_cnt_ = 0;

  //! Parameters for point cloud conversion
  //! @{
  double range_max_     = 0.0;
  bool   use_quiet_nan_ = true;
  //! @}

  //! Data buffer
  //! @{
  std::unique_ptr<unsigned char[]> color_buffer_;
  std::unique_ptr<float[]>         depth_buffer_;
  std::unique_ptr<unsigned char[]> color_buffer_flipped_;
  std::unique_ptr<float[]>         depth_buffer_flipped_;
  std::unique_ptr<unsigned char[]> color_buffer_back_;
  std::unique_ptr<float[]>         depth_buffer_back_;
  //! @}

  //! Variables for visualization and rendering in MuJoCo
  //! @{
  mjvScene    scene_;
  mjvCamera   camera_;
  mjvOption   option_;
  mjrContext  context_;
  GLFWwindow *window_;
  //! @}

  //! ROS variables
  //! @{
  rclcpp::Node::SharedPtr                                     nh_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  bool                                                        publish_color_ = false;
  bool                                                        publish_depth_ = false;
  bool                                                        publish_info_  = false;
  bool                                                        publish_cloud_ = false;
  //! @}

  //! Threading variables
  //! @{
  std::thread             publish_thread_;
  std::mutex              buffer_mutex_;
  std::condition_variable buffer_cv_;
  std::atomic<bool>       stop_thread_{false};
  //! @}
};

} // namespace MujocoRosUtils
