#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace MujocoRosUtils
{

/**
 * \brief Plugin to publish 2D LaserScan or 3D PointCloud2 from MuJoCo rangefinder sensors.
 *
 * Rangefinder sensors must be named "<sensor_name_prefix>-<index>" where index is a zero-padded
 * integer (e.g., "front_lidar-000", "front_lidar-001", ...).
 *
 * For 2D lidar (vertical_layers == 1), publishes sensor_msgs/LaserScan.
 * For 3D lidar (vertical_layers > 1), publishes sensor_msgs/PointCloud2.
 * In 3D mode the flat sensor index maps as: layer = idx / num_horizontal_rays,
 * horizontal = idx % num_horizontal_rays.
 *
 * Example MJCF usage:
 * \code{.xml}
 * <sensor>
 *   <plugin name="my_lidar" plugin="MujocoRosUtils::LidarPublisher" objtype="xbody" objname="lidar_body">
 *     <config key="sensor_name_prefix" value="front_lidar"/>
 *     <config key="frame_id"           value="lidar_frame"/>
 *     <config key="topic_name"         value="/scan"/>
 *     <config key="publish_rate"       value="10"/>
 *     <config key="min_angle"          value="-3.14159"/>
 *     <config key="max_angle"          value="3.14159"/>
 *     <config key="angle_increment"    value="0.0175"/>
 *     <config key="range_min"          value="0.1"/>
 *     <config key="range_max"          value="30.0"/>
 *   </plugin>
 * </sensor>
 * \endcode
 */
class LidarPublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static LidarPublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Move constructor. */
  LidarPublisher(LidarPublisher &&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

  /** \brief Visualize rays in the MuJoCo viewer.
      \param m model
      \param d data
      \param opt visualization options
      \param scn visualization scene
      \param plugin_id plugin ID
   */
  void visualize(const mjModel * m, mjData * d, const mjvOption * opt, mjvScene * scn, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param body_id ID of the body the plugin sensor is attached to
      \param frame_id frame ID of message header (used as TF child frame when output_tf=true)
      \param topic_name topic name
      \param publish_rate publish rate [Hz]
      \param min_angle horizontal start angle [rad]
      \param max_angle horizontal end angle [rad]
      \param angle_increment horizontal angle step [rad]
      \param range_min minimum valid range [m]
      \param range_max maximum valid range [m]
      \param vertical_layers number of vertical layers (1 = 2D, >1 = 3D)
      \param vertical_min_angle vertical start angle [rad] (3D only)
      \param vertical_max_angle vertical end angle [rad] (3D only)
      \param sensor_adrs sorted list of sensor_adr values for each rangefinder
      \param site_ids list of site IDs parallel to sensor_adrs (for ray visualization)
      \param output_tf whether to broadcast a TF for frame_id
      \param tf_parent_frame_id parent frame for the TF broadcast (default "world")
      \param qos ROS publisher QoS profile
      \param visualize_rays whether to draw each ray as a line in the MuJoCo viewer
      \param ray_hit_rgba RGBA color for rays that hit an object
      \param ray_miss_rgba RGBA color for rays that miss (drawn to range_max)
   */
  LidarPublisher(const mjModel * m,
                 int body_id,
                 const std::string & frame_id,
                 const std::string & topic_name,
                 mjtNum publish_rate,
                 double min_angle,
                 double max_angle,
                 double angle_increment,
                 double range_min,
                 double range_max,
                 int vertical_layers,
                 double vertical_min_angle,
                 double vertical_max_angle,
                 std::vector<int> sensor_adrs,
                 std::vector<int> site_ids,
                 bool output_tf,
                 const std::string & tf_parent_frame_id,
                 const rclcpp::QoS & qos,
                 bool visualize_rays,
                 std::array<float, 4> ray_hit_rgba,
                 std::array<float, 4> ray_miss_rgba);

protected:
  //! Body ID of the body the plugin is attached to (for TF)
  int body_id_ = -1;

  //! Whether to broadcast a TF transform for frame_id
  bool output_tf_ = false;

  //! Parent frame for the TF broadcast
  std::string tf_parent_frame_id_;

  //! TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  //! Frame ID of message header
  std::string frame_id_;

  //! Topic name
  std::string topic_name_;

  //! True when publishing PointCloud2 (vertical_layers > 1), false for LaserScan
  bool is_3d_;

  //! Horizontal scan parameters
  double min_angle_;
  double max_angle_;
  double angle_increment_;
  int num_horizontal_rays_;

  //! Vertical scan parameters (3D only)
  int vertical_layers_;
  double vertical_min_angle_;
  double vertical_max_angle_;
  double vertical_angle_increment_;

  //! Range limits
  double range_min_;
  double range_max_;

  //! Indices into d->sensordata for each rangefinder, sorted by beam index
  std::vector<int> sensor_adrs_;

  //! Site IDs parallel to sensor_adrs_ (used for ray visualization)
  std::vector<int> site_ids_;

  //! Whether to draw rays in the MuJoCo viewer
  bool visualize_rays_ = false;

  //! Flag to print one-time diagnostics on first publish
  bool debug_printed_ = false;

  //! Precomputed site position and Z-axis in body frame for each sensor (from model data at init
  //! time). Used instead of d->site_xpos/d->site_xmat to avoid any runtime init issues with
  //! fixed/kinematic bodies.
  std::vector<std::array<mjtNum, 3>> site_local_pos_;
  std::vector<std::array<mjtNum, 3>> site_local_z_;

  //! Ray colors: hit (object detected) and miss (drawn to range_max)
  std::array<float, 4> ray_hit_rgba_ = {0.0f, 1.0f, 0.0f, 0.5f};
  std::array<float, 4> ray_miss_rgba_ = {1.0f, 0.0f, 0.0f, 0.3f};

  //! ROS node handle
  rclcpp::Node::SharedPtr nh_;

  //! ROS publisher (LaserScan or PointCloud2)
  rclcpp::PublisherBase::SharedPtr pub_;

  //! Reusable 2D scan message (fields pre-filled at construction)
  sensor_msgs::msg::LaserScan scan_msg_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
