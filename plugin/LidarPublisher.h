#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
 * \brief MuJoCo sensor plugin that publishes LiDAR data via ROS 2.
 *
 * Fires all rays in parallel using mj_multiRay() and publishes either a
 * `sensor_msgs/LaserScan` (2D) or `sensor_msgs/PointCloud2` (3D).
 *
 * ## Attachment
 * Attach the plugin to a `<sensor>` element that references a `<site>`:
 * \code{.xml}
 * <sensor name="my_lidar" type="plugin" objtype="site" objname="SITE_NAME">
 *   <plugin name="my_lidar" plugin="MujocoRosUtils::LidarPublisher">
 *     <config key="site_name" value="SITE_NAME"/>
 *     ...
 *   </plugin>
 * </sensor>
 * \endcode
 *
 * ## Config keys
 *
 * ### Required
 * | Key         | Description                                                  |
 * |-------------|--------------------------------------------------------------|
 * | `site_name` | Name of the MuJoCo `<site>` used as the LiDAR origin.       |
 *
 * ### Scan pattern
 * | Key            | Default   | Description                                                    |
 * |----------------|-----------|----------------------------------------------------------------|
 * | `scan_pattern` | `custom`  | Preset: `vlp16`, `vlp32`, `hdl64`, `os128`, or `custom`.      |
 *
 * #### Preset patterns
 * | Pattern | Beams  | Hz  | Type |
 * |---------|--------|-----|------|
 * | `vlp16` | 16 × 1800 = 28800 | 10 | PointCloud2 |
 * | `vlp32` | 32 × 1800 = 57600 | 10 | PointCloud2 |
 * | `hdl64` | 64 × 1800 = 115200 | 10 | PointCloud2 |
 * | `os128` | 128 × 1024 = 131072 | 10 | PointCloud2 |
 *
 * #### Custom pattern keys (used when `scan_pattern=custom`)
 * | Key                   | Default   | Description                                           |
 * |-----------------------|-----------|-------------------------------------------------------|
 * | `min_angle`           | `-π`      | Start azimuth [rad].                                  |
 * | `max_angle`           | `π`       | End azimuth [rad].                                    |
 * | `num_horizontal_rays` | `360`     | Number of horizontal beams (overrides `angle_increment`). |
 * | `angle_increment`     | —         | Azimuth step [rad] (used if `num_horizontal_rays` is absent). |
 * | `vertical_layers`     | `1`       | Number of elevation channels. `1` → 2D LaserScan.    |
 * | `vertical_min_angle`  | —         | Lowest elevation angle [rad] (required if `vertical_layers > 1`). |
 * | `vertical_max_angle`  | —         | Highest elevation angle [rad] (required if `vertical_layers > 1`). |
 *
 * ### ROS output
 * | Key               | Default               | Description                                        |
 * |-------------------|-----------------------|----------------------------------------------------|
 * | `frame_id`        | `lidar`               | ROS frame ID in message headers.                   |
 * | `topic_name`      | `mujoco/lidar/scan`   | ROS topic to publish on.                           |
 * | `publish_rate`    | `10`                  | Publishing rate [Hz].                              |
 * | `output_tf`       | `false`               | Broadcast a TF transform for `frame_id`.           |
 * | `tf_parent_frame_id` | `world`            | Parent frame for the TF broadcast.                 |
 * | `qos`             | `best_effort;10`      | QoS string: `"N"`, `"best_effort;N"`, `"reliable;N"`. |
 *
 * ### Range
 * | Key         | Default  | Description                      |
 * |-------------|----------|----------------------------------|
 * | `range_min` | `0.0`    | Minimum valid range [m].         |
 * | `range_max` | `1000.0` | Maximum range / ray cutoff [m].  |
 *
 * ### Ray filtering
 * | Key           | Default | Description                                                                       |
 * |---------------|---------|-----------------------------------------------------------------------------------|
 * | `body_exclude` | —      | Semicolon-separated body names to exclude from ray hits, e.g. `"base;head"`. Rays that hit an excluded body are re-cast from just past the hit, up to N times (N = number of excluded bodies). |
 * | `skip_group`  | —       | Geom groups to exclude from ray casting. Syntax: `"2"` skips group 2; `">1"` skips groups 2+; `"1;3"` skips groups 1 and 3. Useful to exclude visual-only meshes (typically group 2). |
 *
 * ### Visualization
 * | Key              | Default                  | Description                                          |
 * |------------------|--------------------------|------------------------------------------------------|
 * | `visualize_rays` | `false`                  | Draw rays in the MuJoCo viewer.                      |
 * | `ray_hit_color`  | `0.0 1.0 0.0 0.5`        | RGBA for rays that hit an object (space-separated).  |
 * | `ray_miss_color` | `1.0 0.0 0.0 0.3`        | RGBA for rays that miss (drawn to `range_max`).      |
 *
 * ### Sensor noise
 * All three noise models are optional and independent; they are applied in the order listed.
 *
 * #### 1. Gaussian range noise (`noise_gaussian`)
 * Adds zero-mean Gaussian noise \f$\mathcal{N}(0,\sigma)\f$ to every valid hit distance.
 * Models range-measurement uncertainty from electronics / atmospheric turbulence.
 * \code{.xml}
 * <config key="noise_gaussian" value="0.02"/>   <!-- σ = 2 cm -->
 * \endcode
 * | Parameter | Position | Description |
 * |-----------|----------|-------------|
 * | `STDDEV`  | 0        | Standard deviation σ [m]. Typical: 0.01–0.05. |
 *
 * #### 2. Dropout noise (`noise_dropout`)
 * Randomly discards valid hits (sets them to NaN/miss) with probability PROB.
 * Models beam absorption by dark/transparent surfaces, rain, or fog.
 * \code{.xml}
 * <config key="noise_dropout" value="0.01"/>    <!-- 1 % miss rate -->
 * \endcode
 * | Parameter | Position | Description |
 * |-----------|----------|-------------|
 * | `PROB`    | 0        | Drop probability ∈ [0, 1]. Typical: 0.005–0.02. |
 *
 * #### 3. Outlier noise (`noise_outlier`)
 * Replaces valid hits with a spurious random return uniformly distributed in
 * [MIN_RANGE, MAX_RANGE] with probability PROB. Models multi-path reflections,
 * dust, or sensor cross-talk.
 * \code{.xml}
 * <config key="noise_outlier" value="0.005,0.1,30.0"/>  <!-- 0.5 %, 0.1–30 m -->
 * \endcode
 * | Parameter   | Position | Description |
 * |-------------|----------|-------------|
 * | `PROB`      | 0        | Outlier probability ∈ [0, 1]. Typical: 0.002–0.01. |
 * | `MIN_RANGE` | 1        | Minimum outlier distance [m].                       |
 * | `MAX_RANGE` | 2        | Maximum outlier distance [m].                       |
 *
 * #### 4. 3D jitter noise (`noise_jitter`)
 * Displaces each valid hit point by an **independent** Gaussian offset on each of the x, y, z
 * axes in the sensor frame. Unlike `noise_gaussian` (which shifts along the ray), jitter
 * scatters the point in all 3 directions, modelling beam divergence, surface roughness,
 * mechanical vibration, or multi-return ambiguity. Only applied to 3D (PointCloud2) output.
 * \code{.xml}
 * <config key="noise_jitter" value="0.03"/>   <!-- σ = 3 cm per axis -->
 * \endcode
 * | Parameter | Position | Description |
 * |-----------|----------|-------------|
 * | `STDDEV`  | 0        | Per-axis standard deviation σ [m]. Typical: 0.01–0.05. |
 *
 * ## Full example (VLP-16 with noise)
 * \code{.xml}
 * <sensor name="vlp16" type="plugin" objtype="site" objname="lidar_site">
 *   <plugin name="vlp16" plugin="MujocoRosUtils::LidarPublisher">
 *     <config key="site_name"       value="lidar_site"/>
 *     <config key="scan_pattern"    value="vlp16"/>
 *     <config key="frame_id"        value="lidar_frame"/>
 *     <config key="topic_name"      value="/points"/>
 *     <config key="publish_rate"    value="10"/>
 *     <config key="range_min"       value="0.9"/>
 *     <config key="range_max"       value="130.0"/>
 *     <config key="body_exclude"    value="lidar_base;lidar_head"/>
 *     <config key="skip_group"      value="2"/>
 *     <config key="visualize_rays"  value="true"/>
 *     <config key="noise_gaussian"  value="0.02"/>
 *     <config key="noise_dropout"   value="0.01"/>
 *     <config key="noise_outlier"   value="0.005,0.1,30.0"/>
 *     <config key="noise_jitter"    value="0.03"/>
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

  /** \brief Reset (no-op).
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute: cast rays via mj_multiRay and publish.
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
      \param site_id site ID used as LiDAR origin
      \param body_exclude_id body ID to exclude from ray hits (-1 = none)
      \param frame_id ROS frame ID for message headers
      \param topic_name ROS topic name
      \param publish_rate publish rate [Hz]
      \param range_min minimum valid range [m]
      \param range_max maximum valid range [m]
      \param is_3d true → PointCloud2, false → LaserScan
      \param min_angle horizontal start angle [rad] (2D only)
      \param max_angle horizontal end angle [rad] (2D only)
      \param angle_increment horizontal angle step [rad] (2D only)
      \param ray_dirs_local (nray × 3) sensor-frame unit direction vectors
      \param output_tf whether to broadcast a TF for frame_id
      \param tf_parent_frame_id parent frame for the TF broadcast
      \param qos ROS publisher QoS profile
      \param visualize_rays whether to draw rays in the MuJoCo viewer
      \param ray_hit_rgba RGBA color for rays that hit an object
      \param ray_miss_rgba RGBA color for rays that miss
   */
  LidarPublisher(const mjModel * m,
                 int site_id,
                 std::vector<int> excluded_body_ids,
                 const std::string & frame_id,
                 const std::string & topic_name,
                 mjtNum publish_rate,
                 double range_min,
                 double range_max,
                 bool is_3d,
                 double min_angle,
                 double max_angle,
                 double angle_increment,
                 std::vector<mjtNum> ray_dirs_local,
                 bool output_tf,
                 const std::string & tf_parent_frame_id,
                 const rclcpp::QoS & qos,
                 bool visualize_rays,
                 std::array<float, 4> ray_hit_rgba,
                 std::array<float, 4> ray_miss_rgba,
                 std::array<mjtByte, mjNGROUP> geomgroup_filter,
                 double noise_gaussian_stddev,
                 double noise_dropout_prob,
                 double noise_outlier_prob,
                 double noise_outlier_min,
                 double noise_outlier_max,
                 double noise_jitter_stddev);

protected:
  //! Site ID of the LiDAR origin
  int site_id_ = -1;

  //! Body IDs to exclude from ray hits (empty = none)
  std::vector<int> excluded_body_ids_;

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

  //! True when publishing PointCloud2, false for LaserScan
  bool is_3d_ = false;

  //! Horizontal scan parameters (2D / custom)
  double min_angle_ = 0.0;
  double max_angle_ = 0.0;
  double angle_increment_ = 0.0;

  //! Range limits
  double range_min_ = 0.0;
  double range_max_ = 1000.0;

  //! Number of rays
  int nray_ = 0;

  //! (nray × 3) sensor-frame unit direction vectors
  std::vector<mjtNum> ray_dirs_local_;

  //! (nray × 3) world-frame unit direction vectors (updated each compute step)
  std::vector<mjtNum> ray_dirs_world_;

  //! (nray) hit distances from mj_multiRay (-1 = miss)
  std::vector<mjtNum> ray_dist_;

  //! (nray) hit geom IDs from mj_multiRay (-1 = miss)
  std::vector<int> ray_geomid_;

  //! Per-group filter for mj_multiRay: 1 = include group, 0 = exclude (MuJoCo API semantics)
  std::array<mjtByte, mjNGROUP> geomgroup_filter_ = {};

  // ----- Sensor noise -----

  //! Gaussian noise: standard deviation [m]. 0 = disabled.
  double noise_gaussian_stddev_ = 0.0;

  //! Dropout noise: probability [0,1] that a valid hit is discarded. 0 = disabled.
  double noise_dropout_prob_ = 0.0;

  //! Outlier noise: probability [0,1] of replacing a hit with a random distance. 0 = disabled.
  double noise_outlier_prob_ = 0.0;
  double noise_outlier_min_  = 0.0; //!< Minimum outlier range [m]
  double noise_outlier_max_  = 0.0; //!< Maximum outlier range [m]

  //! 3D jitter: standard deviation [m] of per-axis Gaussian offset applied to the 3D point. 0 = disabled.
  //! Unlike range noise (shifts along the ray), jitter scatters the point in all directions.
  double noise_jitter_stddev_ = 0.0;

  //! World-frame site position cached from last compute(), used by visualize()
  mjtNum last_site_pos_[3] = {0.0, 0.0, 0.0};

  //! True after the first compute() call; guards visualize() against uninitialized data
  bool has_computed_ = false;

  //! Whether to draw rays in the MuJoCo viewer
  bool visualize_rays_ = false;

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
