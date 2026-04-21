#include "LidarPublisher.h"

#include <mujoco/mujoco.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace MujocoRosUtils
{

void LidarPublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::LidarPublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {
    "sensor_name_prefix",
    "sensor_name_prefix_list",
    "frame_id",
    "topic_name",
    "publish_rate",
    "min_angle",
    "max_angle",
    "angle_increment",
    "range_min",
    "range_max",
    "vertical_layers",
    "vertical_min_angle",
    "vertical_max_angle",
    "output_tf",
    "tf_parent_frame_id",
    "qos",
    "visualize_rays",
    "ray_hit_color",
    "ray_miss_color",
  };

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int) { return 0; };

  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };

  // Must run after forces (sensor data is available at mjSTAGE_ACC)
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * inst = LidarPublisher::Create(m, d, plugin_id);
    if(!inst) return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(inst);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<LidarPublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, void * plugin_data, int plugin_id)
  {
    reinterpret_cast<LidarPublisher *>(plugin_data)->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int)
  {
    reinterpret_cast<LidarPublisher *>(d->plugin_data[plugin_id])->compute(m, d, plugin_id);
  };

  plugin.visualize = +[](const mjModel * m, mjData * d, const mjvOption * opt, mjvScene * scn, int plugin_id)
  {
    reinterpret_cast<LidarPublisher *>(d->plugin_data[plugin_id])->visualize(m, d, opt, scn, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

// Parse QoS string from XML attribute.
// Formats:
//   "n"                      -> reliable, keep_last(n)
//   "best_effort;n"          -> best_effort, keep_last(n)
//   "best_effort;transient_local" -> best_effort, transient_local, keep_last(1)
//   "reliable;n"             -> reliable, keep_last(n)
//   "reliable;volatile"      -> reliable, volatile, keep_last(10)
// Default (empty):            best_effort, volatile, keep_last(10)
static rclcpp::QoS parse_qos_string(const std::string & s)
{
  auto trim = [](std::string t) -> std::string
  {
    t.erase(0, t.find_first_not_of(" \t"));
    if(!t.empty()) t.erase(t.find_last_not_of(" \t") + 1);
    return t;
  };
  auto is_digits = [](const std::string & t) { return !t.empty() && std::all_of(t.begin(), t.end(), ::isdigit); };

  if(s.empty())
  {
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  }

  // Plain integer → reliable, keep_last(n)
  if(is_digits(s))
  {
    return rclcpp::QoS(rclcpp::KeepLast(std::stoi(s))).reliable().durability_volatile();
  }

  // "reliability;second"
  const auto sep = s.find(';');
  const std::string rel_str = trim(s.substr(0, sep));
  const std::string second_str = (sep != std::string::npos) ? trim(s.substr(sep + 1)) : "";

  int depth = 10;
  bool transient = false;

  if(is_digits(second_str))
  {
    depth = std::stoi(second_str);
  }
  else if(second_str == "transient_local")
  {
    transient = true;
    depth = 1;
  }
  // "volatile" or empty → volatile, depth 10 (already defaults)

  rclcpp::QoS qos{rclcpp::KeepLast(depth)};

  if(rel_str == "reliable")
    qos.reliable();
  else
    qos.best_effort();

  if(transient)
    qos.transient_local();
  else
    qos.durability_volatile();

  return qos;
}

// Parse "R G B A" space-separated floats. Returns def on empty/invalid input.
static std::array<float, 4> parse_rgba_string(const std::string & s, std::array<float, 4> def)
{
  if(s.empty()) return def;
  std::istringstream ss(s);
  std::array<float, 4> out = def;
  ss >> out[0] >> out[1] >> out[2] >> out[3];
  return out;
}

LidarPublisher * LidarPublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // --- sensor_name_prefix_list (preferred for 3D multi-layer) or sensor_name_prefix ---
  const char * prefix_list_char = mj_getPluginConfig(m, plugin_id, "sensor_name_prefix_list");
  const char * prefix_char = mj_getPluginConfig(m, plugin_id, "sensor_name_prefix");

  std::vector<std::string> layer_prefixes;
  std::string primary_prefix; // used for topic default name and log messages

  if(prefix_list_char && strlen(prefix_list_char) > 0)
  {
    // Parse comma-separated list
    std::istringstream ss(prefix_list_char);
    std::string token;
    while(std::getline(ss, token, ','))
    {
      // Trim whitespace
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);
      if(!token.empty()) layer_prefixes.push_back(token);
    }
    if(layer_prefixes.empty())
    {
      mju_error("[LidarPublisher] `sensor_name_prefix_list` is empty after parsing.");
      return nullptr;
    }
    primary_prefix = layer_prefixes[0];
  }
  else if(prefix_char && strlen(prefix_char) > 0)
  {
    primary_prefix = std::string(prefix_char);
    layer_prefixes.push_back(primary_prefix);
  }
  else
  {
    mju_error("[LidarPublisher] Either `sensor_name_prefix` or `sensor_name_prefix_list` is required.");
    return nullptr;
  }

  // --- frame_id ---
  const char * frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  std::string frame_id = (frame_id_char && strlen(frame_id_char) > 0) ? std::string(frame_id_char) : "map";

  // --- topic_name ---
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = (topic_name_char && strlen(topic_name_char) > 0)
                               ? std::string(topic_name_char)
                               : ("mujoco/" + primary_prefix + "/scan");

  // --- publish_rate ---
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum publish_rate = 10.0;
  if(publish_rate_char && strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if(publish_rate <= 0)
  {
    mju_error("[LidarPublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // --- min_angle / max_angle / angle_increment (required) ---
  const char * min_angle_char = mj_getPluginConfig(m, plugin_id, "min_angle");
  const char * max_angle_char = mj_getPluginConfig(m, plugin_id, "max_angle");
  const char * angle_incr_char = mj_getPluginConfig(m, plugin_id, "angle_increment");
  if(!min_angle_char || strlen(min_angle_char) == 0 || !max_angle_char || strlen(max_angle_char) == 0
     || !angle_incr_char || strlen(angle_incr_char) == 0)
  {
    mju_error("[LidarPublisher] `min_angle`, `max_angle`, and `angle_increment` are required.");
    return nullptr;
  }
  const double min_angle = strtod(min_angle_char, nullptr);
  const double max_angle = strtod(max_angle_char, nullptr);
  const double angle_increment = strtod(angle_incr_char, nullptr);
  if(angle_increment <= 0)
  {
    mju_error("[LidarPublisher] `angle_increment` must be positive.");
    return nullptr;
  }

  // --- range_min / range_max ---
  const char * range_min_char = mj_getPluginConfig(m, plugin_id, "range_min");
  const char * range_max_char = mj_getPluginConfig(m, plugin_id, "range_max");
  const double range_min = (range_min_char && strlen(range_min_char) > 0) ? strtod(range_min_char, nullptr) : 0.0;
  const double range_max =
      (range_max_char && strlen(range_max_char) > 0) ? strtod(range_max_char, nullptr) : 1000.0;

  // --- vertical_layers: inferred from prefix list size if list was used, else explicit attribute ---
  int vertical_layers = 1;
  if(layer_prefixes.size() > 1)
  {
    // Multi-layer 3D: one prefix per layer
    vertical_layers = static_cast<int>(layer_prefixes.size());
  }
  else
  {
    const char * vlayers_char = mj_getPluginConfig(m, plugin_id, "vertical_layers");
    if(vlayers_char && strlen(vlayers_char) > 0) vertical_layers = std::stoi(vlayers_char);
  }
  if(vertical_layers < 1)
  {
    mju_error("[LidarPublisher] `vertical_layers` must be >= 1.");
    return nullptr;
  }

  // --- vertical_min_angle / vertical_max_angle (required for 3D) ---
  double vertical_min_angle = 0.0;
  double vertical_max_angle = 0.0;
  if(vertical_layers > 1)
  {
    const char * vmin_char = mj_getPluginConfig(m, plugin_id, "vertical_min_angle");
    const char * vmax_char = mj_getPluginConfig(m, plugin_id, "vertical_max_angle");
    if(!vmin_char || strlen(vmin_char) == 0 || !vmax_char || strlen(vmax_char) == 0)
    {
      mju_error("[LidarPublisher] `vertical_min_angle` and `vertical_max_angle` are required when "
                "`vertical_layers` > 1.");
      return nullptr;
    }
    vertical_min_angle = strtod(vmin_char, nullptr);
    vertical_max_angle = strtod(vmax_char, nullptr);
  }

  // --- output_tf ---
  const char * output_tf_char = mj_getPluginConfig(m, plugin_id, "output_tf");
  bool output_tf = false;
  if(output_tf_char && strlen(output_tf_char) > 0)
  {
    if(strcmp(output_tf_char, "true") != 0 && strcmp(output_tf_char, "false") != 0)
    {
      mju_error("[LidarPublisher] `output_tf` must be `true` or `false`.");
      return nullptr;
    }
    output_tf = (strcmp(output_tf_char, "true") == 0);
  }

  // --- tf_parent_frame_id ---
  const char * tf_parent_char = mj_getPluginConfig(m, plugin_id, "tf_parent_frame_id");
  std::string tf_parent_frame_id =
      (tf_parent_char && strlen(tf_parent_char) > 0) ? std::string(tf_parent_char) : "world";

  // --- Find the body that this sensor plugin is attached to (needed for TF and 3D reconstruction) ---
  int body_id = -1;
  for(int s = 0; s < m->nsensor; s++)
  {
    if(m->sensor_type[s] == mjSENS_PLUGIN && m->sensor_plugin[s] == plugin_id)
    {
      if(m->sensor_objtype[s] == mjOBJ_XBODY)
        body_id = m->sensor_objid[s];
      break;
    }
  }
  if(output_tf && body_id < 0)
  {
    mju_warning("[LidarPublisher] `output_tf` is true but could not find attached xbody. TF will not be published.");
    output_tf = false;
  }

  // --- Discover rangefinder sensors layer by layer ---
  // For each prefix in layer_prefixes, collect {index, sensor_adr, site_id} sorted by index, then append.
  // This ensures sensor_adrs is ordered: all layer0 sensors, then all layer1, etc.
  auto collect_sensors_for_prefix = [&](const std::string & pfx) -> std::vector<std::pair<int, int>>  // {adr, site_id}
  {
    const std::string search_prefix = pfx + "-";
    std::vector<std::tuple<int, int, int>> indexed;  // {index, adr, site_id}
    for(int i = 0; i < m->nsensor; i++)
    {
      if(m->sensor_type[i] != mjSENS_RANGEFINDER) continue;
      const char * name_char = mj_id2name(m, mjOBJ_SENSOR, i);
      if(!name_char) continue;
      const std::string name(name_char);
      if(name.size() <= search_prefix.size()) continue;
      if(name.substr(0, search_prefix.size()) != search_prefix) continue;
      const std::string suffix = name.substr(search_prefix.size());
      if(suffix.empty() || !std::all_of(suffix.begin(), suffix.end(), ::isdigit)) continue;
      indexed.emplace_back(std::stoi(suffix), m->sensor_adr[i], m->sensor_objid[i]);
    }
    std::sort(indexed.begin(), indexed.end());
    std::vector<std::pair<int, int>> result;
    result.reserve(indexed.size());
    for(auto & [idx, adr, sid] : indexed) result.push_back({adr, sid});
    return result;
  };

  std::vector<int> sensor_adrs;
  std::vector<int> site_ids;
  for(const auto & pfx : layer_prefixes)
  {
    auto layer_pairs = collect_sensors_for_prefix(pfx);
    if(layer_pairs.empty())
    {
      mju_error("[LidarPublisher] No rangefinder sensors found with prefix \"%s-\".", pfx.c_str());
      return nullptr;
    }
    for(auto & [adr, sid] : layer_pairs)
    {
      sensor_adrs.push_back(adr);
      site_ids.push_back(sid);
    }
  }

  const int num_sensors = static_cast<int>(sensor_adrs.size());

  if(vertical_layers > 1)
  {
    if(num_sensors % vertical_layers != 0)
    {
      mju_warning("[LidarPublisher] Found %d rangefinders which is not a multiple of %d vertical layers. "
                  "Check sensor naming.",
                  num_sensors, vertical_layers);
    }
  }
  else
  {
    const int expected_horizontal =
        static_cast<int>(std::round((max_angle - min_angle) / angle_increment)) + 1;
    if(num_sensors != expected_horizontal)
    {
      mju_warning("[LidarPublisher] Found %d rangefinders but expected %d from angle parameters. "
                  "Check sensor naming or angle parameters.",
                  num_sensors, expected_horizontal);
    }
  }

  std::cout << "[LidarPublisher] Creating " << (vertical_layers > 1 ? "3D (PointCloud2)" : "2D (LaserScan)")
            << " lidar publisher for prefix \"" << primary_prefix << "\" with " << num_sensors << " rangefinders."
            << std::endl;

  // --- qos ---
  const char * qos_char = mj_getPluginConfig(m, plugin_id, "qos");
  const rclcpp::QoS qos = parse_qos_string(qos_char ? std::string(qos_char) : "");

  // --- visualize_rays ---
  const char * vis_rays_char = mj_getPluginConfig(m, plugin_id, "visualize_rays");
  const bool visualize_rays = (vis_rays_char && strcmp(vis_rays_char, "true") == 0);

  // --- ray_hit_color / ray_miss_color ---
  const char * hit_color_char = mj_getPluginConfig(m, plugin_id, "ray_hit_color");
  const char * miss_color_char = mj_getPluginConfig(m, plugin_id, "ray_miss_color");
  const auto ray_hit_rgba =
      parse_rgba_string(hit_color_char ? std::string(hit_color_char) : "", {0.0f, 1.0f, 0.0f, 0.5f});
  const auto ray_miss_rgba =
      parse_rgba_string(miss_color_char ? std::string(miss_color_char) : "", {1.0f, 0.0f, 0.0f, 0.3f});

  return new LidarPublisher(m, body_id, frame_id, topic_name, publish_rate, min_angle, max_angle, angle_increment,
                            range_min, range_max, vertical_layers, vertical_min_angle, vertical_max_angle,
                            std::move(sensor_adrs), std::move(site_ids), output_tf, tf_parent_frame_id, qos,
                            visualize_rays, ray_hit_rgba, ray_miss_rgba);
}

LidarPublisher::LidarPublisher(const mjModel * m,
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
                               std::array<float, 4> ray_miss_rgba)
: body_id_(body_id), output_tf_(output_tf), tf_parent_frame_id_(tf_parent_frame_id),
  frame_id_(frame_id), topic_name_(topic_name), is_3d_(vertical_layers > 1), min_angle_(min_angle),
  max_angle_(max_angle), angle_increment_(angle_increment),
  num_horizontal_rays_(static_cast<int>(std::round((max_angle - min_angle) / angle_increment)) + 1),
  vertical_layers_(vertical_layers), vertical_min_angle_(vertical_min_angle),
  vertical_max_angle_(vertical_max_angle),
  vertical_angle_increment_(vertical_layers > 1 ? (vertical_max_angle - vertical_min_angle)
                                                        / std::max(vertical_layers - 1, 1)
                                                    : 0.0),
  range_min_(range_min), range_max_(range_max), sensor_adrs_(std::move(sensor_adrs)),
  site_ids_(std::move(site_ids)), visualize_rays_(visualize_rays),
  ray_hit_rgba_(ray_hit_rgba), ray_miss_rgba_(ray_miss_rgba),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
  int argc = 0;
  char ** argv = nullptr;
  if(!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({{"use_sim_time", true}});
  nh_ = rclcpp::Node::make_shared("lidar_publisher", node_options);

  // Precompute each site's body-frame position and Z-axis from model data.
  // Using m->site_pos / m->site_quat (model-time, always correct) instead of
  // d->site_xpos / d->site_xmat avoids any initialisation issue that can occur
  // with fixed/kinematic bodies where MuJoCo may not have run a forward pass yet.
  site_local_pos_.resize(site_ids_.size());
  site_local_z_.resize(site_ids_.size());
  for(std::size_t idx = 0; idx < site_ids_.size(); idx++)
  {
    const int sid = site_ids_[idx];
    // Position of site in its parent body frame
    site_local_pos_[idx] = {m->site_pos[3 * sid + 0],
                             m->site_pos[3 * sid + 1],
                             m->site_pos[3 * sid + 2]};
    // Rotation of site relative to its parent body (stored as quaternion in model)
    mjtNum local_mat[9];
    mju_quat2Mat(local_mat, m->site_quat + 4 * sid);
    // Column 2 of the 3×3 rotation matrix = Z-axis of the site in body frame
    site_local_z_[idx] = {local_mat[2], local_mat[5], local_mat[8]};
  }

  if(!is_3d_)
  {
    // Pre-fill static LaserScan fields
    scan_msg_.header.frame_id = frame_id_;
    scan_msg_.angle_min = static_cast<float>(min_angle_);
    scan_msg_.angle_max = static_cast<float>(max_angle_);
    scan_msg_.angle_increment = static_cast<float>(angle_increment_);
    scan_msg_.time_increment = 0.0f;
    scan_msg_.scan_time = static_cast<float>(1.0 / (1.0 / publish_skip_ / m->opt.timestep));
    scan_msg_.range_min = static_cast<float>(range_min_);
    scan_msg_.range_max = static_cast<float>(range_max_);
    scan_msg_.ranges.resize(sensor_adrs_.size(), 0.0f);
    scan_msg_.intensities.resize(0);

    pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, qos);
  }
  else
  {
    pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, qos);
  }

  if(output_tf_)
  {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
  }
}

void LidarPublisher::reset(const mjModel *, int) {}

void LidarPublisher::compute(const mjModel * m, mjData * d, int)
{
  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0) return;

  const rclcpp::Time now(static_cast<int32_t>(d->time),
                         static_cast<uint32_t>((d->time - static_cast<int32_t>(d->time)) * 1e9),
                         RCL_ROS_TIME);

  // --- TF broadcast ---
  if(output_tf_ && tf_br_ && body_id_ >= 0)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = tf_parent_frame_id_;
    tf_msg.child_frame_id = frame_id_;
    tf_msg.transform.translation.x = d->xpos[3 * body_id_ + 0];
    tf_msg.transform.translation.y = d->xpos[3 * body_id_ + 1];
    tf_msg.transform.translation.z = d->xpos[3 * body_id_ + 2];
    tf_msg.transform.rotation.w = d->xquat[4 * body_id_ + 0];
    tf_msg.transform.rotation.x = d->xquat[4 * body_id_ + 1];
    tf_msg.transform.rotation.y = d->xquat[4 * body_id_ + 2];
    tf_msg.transform.rotation.z = d->xquat[4 * body_id_ + 3];
    tf_br_->sendTransform(tf_msg);
  }

  if(!is_3d_)
  {
    // --- 2D: publish LaserScan ---
    scan_msg_.header.stamp = now;

    const int n = static_cast<int>(sensor_adrs_.size());
    for(int i = 0; i < n; i++)
    {
      const float r = static_cast<float>(d->sensordata[sensor_adrs_[i]]);
      scan_msg_.ranges[i] = (r < static_cast<float>(range_min_) || r > static_cast<float>(range_max_))
                                ? std::numeric_limits<float>::infinity()
                                : r;
    }

    std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pub_)->publish(scan_msg_);
  }
  else
  {
    // --- 3D: publish PointCloud2 ---

    // One-time diagnostic to verify site orientations and body frame at runtime.
    if(!debug_printed_ && !site_ids_.empty() && body_id_ >= 0)
    {
      debug_printed_ = true;

      const int sid0 = site_ids_[0];
      const mjtNum * sm = d->site_xmat + 9 * sid0;

      // Derive site Z-axis from model quat (ground truth, independent of d->site_xmat)
      mjtNum q[4] = {m->site_quat[4 * sid0 + 0], m->site_quat[4 * sid0 + 1],
                     m->site_quat[4 * sid0 + 2], m->site_quat[4 * sid0 + 3]};
      mjtNum model_mat[9];
      mju_quat2Mat(model_mat, q);

      std::cerr << "[LidarPublisher DEBUG] body_id=" << body_id_
                << " body_pos=(" << d->xpos[3 * body_id_] << ","
                << d->xpos[3 * body_id_ + 1] << "," << d->xpos[3 * body_id_ + 2] << ")"
                << " body_quat(wxyz)=(" << d->xquat[4 * body_id_] << ","
                << d->xquat[4 * body_id_ + 1] << "," << d->xquat[4 * body_id_ + 2] << ","
                << d->xquat[4 * body_id_ + 3] << ")" << std::endl;
      std::cerr << "[LidarPublisher DEBUG] site[0] id=" << sid0
                << " d->site_xmat row-major=[";
      for(int k = 0; k < 9; k++) std::cerr << sm[k] << (k < 8 ? "," : "");
      std::cerr << "]" << std::endl;
      std::cerr << "[LidarPublisher DEBUG] site[0] d->site_xmat col2(Z-axis)=("
                << sm[2] << "," << sm[5] << "," << sm[8] << ")" << std::endl;
      std::cerr << "[LidarPublisher DEBUG] site[0] model_quat mat col2(Z-axis)=("
                << model_mat[2] << "," << model_mat[5] << "," << model_mat[8] << ")" << std::endl;

      // Print first few z_local values for diagnostics
      std::cerr << "[LidarPublisher DEBUG] First 5 z_local values:" << std::endl;
      const mjtNum * bp = d->xpos + 3 * body_id_;
      const mjtNum * bm = d->xmat + 9 * body_id_;
      for(int k = 0; k < std::min(5, static_cast<int>(site_ids_.size())); k++)
      {
        const double r_k = d->sensordata[sensor_adrs_[k]];
        const int sid_k = site_ids_[k];
        const mjtNum * sp = d->site_xpos + 3 * sid_k;
        const mjtNum * sx = d->site_xmat + 9 * sid_k;
        const mjtNum pw2 = sp[2] - sx[8] * r_k;
        const mjtNum d2 = pw2 - bp[2];
        const mjtNum z_local = bm[2] * (d->site_xpos[3*sid_k+0]-bp[0])
                             + bm[5] * (d->site_xpos[3*sid_k+1]-bp[1])
                             + bm[8] * d2;
        std::cerr << "  [" << k << "] r=" << r_k << " site_xmat[8]=" << sx[8]
                  << " p_world_z=" << pw2 << " z_local=" << z_local << std::endl;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(sensor_adrs_.size());
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(sensor_adrs_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const int n = static_cast<int>(sensor_adrs_.size());
    for(int i = 0; i < n; i++, ++iter_x, ++iter_y, ++iter_z)
    {
      const double r = d->sensordata[sensor_adrs_[i]];

      if(r < range_min_ || r > range_max_)
      {
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      // Compute hit point in the lidar body frame directly from model-derived local data.
      // Rangefinder fires along -Z of the site frame; site_local_z_[i] is the site Z-axis
      // expressed in the body frame.  So the hit point in body frame is:
      //   p_body = site_local_pos - site_local_z * r
      const int site_id = site_ids_[i];
      const auto & lp = site_local_pos_[i];
      const auto & lz = site_local_z_[i];

      if(body_id_ >= 0)
      {
        *iter_x = static_cast<float>(lp[0] - lz[0] * r);
        *iter_y = static_cast<float>(lp[1] - lz[1] * r);
        *iter_z = static_cast<float>(-(lp[2] - lz[2] * r));
      }
      else
      {
        // No body attached — publish in world frame using d->site_xpos/xmat as fallback
        const mjtNum * site_pos = d->site_xpos + 3 * site_id;
        const mjtNum * site_mat = d->site_xmat + 9 * site_id;
        *iter_x = static_cast<float>(site_pos[0] - site_mat[2] * r);
        *iter_y = static_cast<float>(site_pos[1] - site_mat[5] * r);
        *iter_z = static_cast<float>(site_pos[2] - site_mat[8] * r);
      }
    }

    std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pub_)->publish(cloud);
  }
}

void LidarPublisher::visualize(const mjModel *, mjData * d, const mjvOption *, mjvScene * scn, int)
{
  if(!visualize_rays_) return;

  const int n = static_cast<int>(sensor_adrs_.size());
  for(int i = 0; i < n; i++)
  {
    if(scn->ngeom >= scn->maxgeom) break;

    const mjtNum r = d->sensordata[sensor_adrs_[i]];
    const bool hit = (r >= range_min_ && r <= range_max_);
    const mjtNum ray_range = hit ? r : static_cast<mjtNum>(range_max_);

    // Compute site world-frame position and Z-axis from body pose + precomputed local data,
    // avoiding any potential stale state in d->site_xpos / d->site_xmat.
    const auto & lp = site_local_pos_[i];
    const auto & lz = site_local_z_[i];
    mjtNum pos[3], z_world[3];
    if(body_id_ >= 0)
    {
      const mjtNum * bp = d->xpos + 3 * body_id_;
      const mjtNum * bm = d->xmat + 9 * body_id_;
      pos[0] = bp[0] + bm[0]*lp[0] + bm[1]*lp[1] + bm[2]*lp[2];
      pos[1] = bp[1] + bm[3]*lp[0] + bm[4]*lp[1] + bm[5]*lp[2];
      pos[2] = bp[2] + bm[6]*lp[0] + bm[7]*lp[1] + bm[8]*lp[2];
      z_world[0] = bm[0]*lz[0] + bm[1]*lz[1] + bm[2]*lz[2];
      z_world[1] = bm[3]*lz[0] + bm[4]*lz[1] + bm[5]*lz[2];
      z_world[2] = bm[6]*lz[0] + bm[7]*lz[1] + bm[8]*lz[2];
    }
    else
    {
      const int site_id = site_ids_[i];
      const mjtNum * sp = d->site_xpos + 3 * site_id;
      const mjtNum * sm = d->site_xmat + 9 * site_id;
      pos[0] = sp[0]; pos[1] = sp[1]; pos[2] = sp[2];
      z_world[0] = sm[2]; z_world[1] = sm[5]; z_world[2] = sm[8];
    }

    // Rangefinder fires along the -Z axis of the site frame.
    const mjtNum to[3] = {
      pos[0] - z_world[0] * ray_range,
      pos[1] - z_world[1] * ray_range,
      pos[2] - z_world[2] * ray_range,
    };

    mjvGeom * geom = scn->geoms + scn->ngeom;
    mjv_initGeom(geom, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);
    mjv_connector(geom, mjGEOM_LINE, 1.5, pos, to);
    const auto & rgba = hit ? ray_hit_rgba_ : ray_miss_rgba_;
    geom->rgba[0] = rgba[0];
    geom->rgba[1] = rgba[1];
    geom->rgba[2] = rgba[2];
    geom->rgba[3] = rgba[3];
    scn->ngeom++;
  }
}

} // namespace MujocoRosUtils
