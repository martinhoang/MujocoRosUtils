#include "LidarPublisher.h"
#include "RosContextManager.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mujoco/mujoco.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace MujocoRosUtils
{

void LidarPublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::LidarPublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char *attributes[] = {
    "site_name",           "scan_pattern",       "frame_id",
    "topic_name",          "publish_rate",       "range_min",
    "range_max",           "body_exclude",       "skip_group",
    "output_tf",           "tf_parent_frame_id", "qos",
    "visualize_rays",      "ray_hit_color",      "ray_miss_color",
    "min_angle",           "max_angle",          "angle_increment",
    "num_horizontal_rays", "vertical_layers",    "vertical_min_angle",
    "vertical_max_angle",  "noise_gaussian",     "noise_dropout",
    "noise_outlier",       "noise_jitter",
  };

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int) {
    return 0;
  };

  plugin.nsensordata = +[](const mjModel *, int, int) {
    return 0;
  };

  // Must run after forces (sensor data is available at mjSTAGE_ACC)
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    auto *inst = LidarPublisher::Create(m, d, plugin_id);
    if (!inst)
      return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(inst);
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<LidarPublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel *m, double *, void *plugin_data, int plugin_id) {
    reinterpret_cast<LidarPublisher *>(plugin_data)->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int) {
    reinterpret_cast<LidarPublisher *>(d->plugin_data[plugin_id])->compute(m, d, plugin_id);
  };

  plugin.visualize
    = +[](const mjModel *m, mjData *d, const mjvOption *opt, mjvScene *scn, int plugin_id) {
        reinterpret_cast<LidarPublisher *>(d->plugin_data[plugin_id])
          ->visualize(m, d, opt, scn, plugin_id);
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
static rclcpp::QoS parse_qos_string(const std::string &s)
{
  auto trim = [](std::string t) -> std::string {
    t.erase(0, t.find_first_not_of(" \t"));
    if (!t.empty())
      t.erase(t.find_last_not_of(" \t") + 1);
    return t;
  };
  auto is_digits = [](const std::string &t) {
    return !t.empty() && std::all_of(t.begin(), t.end(), ::isdigit);
  };

  if (s.empty())
  {
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  }

  // Plain integer → reliable, keep_last(n)
  if (is_digits(s))
  {
    return rclcpp::QoS(rclcpp::KeepLast(std::stoi(s))).reliable().durability_volatile();
  }

  // "reliability;second"
  const auto        sep        = s.find(';');
  const std::string rel_str    = trim(s.substr(0, sep));
  const std::string second_str = (sep != std::string::npos) ? trim(s.substr(sep + 1)) : "";

  int  depth     = 10;
  bool transient = false;

  if (is_digits(second_str))
  {
    depth = std::stoi(second_str);
  }
  else if (second_str == "transient_local")
  {
    transient = true;
    depth     = 1;
  }
  // "volatile" or empty → volatile, depth 10 (already defaults)

  rclcpp::QoS qos{rclcpp::KeepLast(depth)};

  if (rel_str == "reliable")
    qos.reliable();
  else
    qos.best_effort();

  if (transient)
    qos.transient_local();
  else
    qos.durability_volatile();

  return qos;
}

// Parse "R G B A" space-separated floats. Returns def on empty/invalid input.
static std::array<float, 4> parse_rgba_string(const std::string &s, std::array<float, 4> def)
{
  if (s.empty())
    return def;
  std::istringstream   ss(s);
  std::array<float, 4> out = def;
  ss >> out[0] >> out[1] >> out[2] >> out[3];
  return out;
}

// Build a geomgroup filter array for mj_multiRay/mj_ray (1=include, 0=exclude).
// skip_group config specifies which groups to EXCLUDE:
//   "N"   → exclude group N
//   ">N"  → exclude all groups > N  (i.e. N+1 … mjNGROUP-1)
//   "N;M" → exclude groups N and M
// Empty string → no exclusions (all groups included, returns nullptr-equivalent all-ones array).
static std::array<mjtByte, mjNGROUP> parse_skip_group(const std::string & s)
{
  // Start with all groups included (1 = include, per MuJoCo API)
  std::array<mjtByte, mjNGROUP> result;
  result.fill(1);
  if(s.empty())
    return result;

  std::istringstream ss(s);
  std::string        token;
  while(std::getline(ss, token, ';'))
  {
    const auto f = token.find_first_not_of(" \t");
    if(f == std::string::npos) continue;
    const auto l = token.find_last_not_of(" \t");
    token        = token.substr(f, l - f + 1);
    if(token.empty()) continue;

    try
    {
      if(token[0] == '>')
      {
        const int n = std::stoi(token.substr(1));
        for(int g = n + 1; g < mjNGROUP; ++g)
          result[static_cast<size_t>(g)] = 0; // exclude
      }
      else
      {
        const int g = std::stoi(token);
        if(g >= 0 && g < mjNGROUP)
          result[static_cast<size_t>(g)] = 0; // exclude
        else
          mju_warning("[LidarPublisher] skip_group value %d out of range [0,%d).", g, mjNGROUP);
      }
    }
    catch(const std::exception &)
    {
      mju_warning("[LidarPublisher] Could not parse skip_group token \"%s\".", token.c_str());
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// Scan pattern generators
// Each function returns a flat (nray × 3) vector of sensor-local unit direction vectors.
// Convention: X = forward, Y = left, Z = up.
//   theta = azimuth (0 = +X, increases CCW viewed from above)
//   phi   = elevation (0 = horizontal, positive = upward)
//   dir   = (cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi))
// Preset horizontal beams go from 0 to 2π exclusive (step = 2π/N_horiz).
// Outer loop = elevation channels, inner loop = azimuth beams.
// ---------------------------------------------------------------------------

static std::vector<mjtNum> gen_rays_vlp16(int n_horiz = 1800)
{
  // 16 elevation channels: -15° to +15° in 2° steps (no 0°)
  static const double elevs_deg[16] = {-15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
                                       1.0,   3.0,   5.0,   7.0,  9.0,  11.0, 13.0, 15.0};
  const int           n_vert        = 16;
  std::vector<mjtNum> dirs;
  dirs.reserve(n_horiz * n_vert * 3);
  const double h_step = 2.0 * M_PI / n_horiz;
  for (int iv = 0; iv < n_vert; iv++)
  {
    const double phi = elevs_deg[iv] * M_PI / 180.0;
    const double cp = std::cos(phi), sp = std::sin(phi);
    for (int ih = 0; ih < n_horiz; ih++)
    {
      const double theta = ih * h_step;
      dirs.push_back(cp * std::cos(theta));
      dirs.push_back(cp * std::sin(theta));
      dirs.push_back(sp);
    }
  }
  return dirs;
}

static std::vector<mjtNum> gen_rays_hdl64(int n_horiz = 1800)
{
  // 64 elevation channels: -24.9° to +2.0° uniform
  const int           n_vert = 64;
  const double        vmin   = -24.9 * M_PI / 180.0;
  const double        vmax   = 2.0 * M_PI / 180.0;
  const double        vstep  = (vmax - vmin) / (n_vert - 1);
  std::vector<mjtNum> dirs;
  dirs.reserve(n_horiz * n_vert * 3);
  const double h_step = 2.0 * M_PI / n_horiz;
  for (int iv = 0; iv < n_vert; iv++)
  {
    const double phi = vmin + iv * vstep;
    const double cp = std::cos(phi), sp = std::sin(phi);
    for (int ih = 0; ih < n_horiz; ih++)
    {
      const double theta = ih * h_step;
      dirs.push_back(cp * std::cos(theta));
      dirs.push_back(cp * std::sin(theta));
      dirs.push_back(sp);
    }
  }
  return dirs;
}

static std::vector<mjtNum> gen_rays_vlp32(int n_horiz = 1800)
{
  // 32 non-uniform elevation channels (VLP-32C spec)
  static const double elevs_deg[32]
    = {-25.0,  -22.5, -20.0, -15.0, -13.0, -10.0, -5.0,  -3.0,  -2.333, -1.0,  -0.667,
       -0.333, 0.0,   0.0,   0.333, 0.667, 1.0,   1.333, 1.667, 2.0,    2.333, 2.667,
       3.0,    3.333, 3.667, 4.0,   5.0,   7.0,   10.0,  15.0,  17.0,   20.0};
  const int           n_vert = 32;
  std::vector<mjtNum> dirs;
  dirs.reserve(n_horiz * n_vert * 3);
  const double h_step = 2.0 * M_PI / n_horiz;
  for (int iv = 0; iv < n_vert; iv++)
  {
    const double phi = elevs_deg[iv] * M_PI / 180.0;
    const double cp = std::cos(phi), sp = std::sin(phi);
    for (int ih = 0; ih < n_horiz; ih++)
    {
      const double theta = ih * h_step;
      dirs.push_back(cp * std::cos(theta));
      dirs.push_back(cp * std::sin(theta));
      dirs.push_back(sp);
    }
  }
  return dirs;
}

static std::vector<mjtNum> gen_rays_os128(int n_horiz = 1024)
{
  // 128 elevation channels: -22.5° to +22.5° uniform
  const int           n_vert = 128;
  const double        vmin   = -22.5 * M_PI / 180.0;
  const double        vmax   = 22.5 * M_PI / 180.0;
  const double        vstep  = (vmax - vmin) / (n_vert - 1);
  std::vector<mjtNum> dirs;
  dirs.reserve(n_horiz * n_vert * 3);
  const double h_step = 2.0 * M_PI / n_horiz;
  for (int iv = 0; iv < n_vert; iv++)
  {
    const double phi = vmin + iv * vstep;
    const double cp = std::cos(phi), sp = std::sin(phi);
    for (int ih = 0; ih < n_horiz; ih++)
    {
      const double theta = ih * h_step;
      dirs.push_back(cp * std::cos(theta));
      dirs.push_back(cp * std::sin(theta));
      dirs.push_back(sp);
    }
  }
  return dirs;
}

// Custom scan: inclusive horizontal endpoints, uniform vertical spacing.
// For 2D (n_vert == 1), v_min/v_max are ignored and phi = 0.
static std::vector<mjtNum> gen_rays_custom(double h_min, double h_max, int n_horiz, double v_min,
                                           double v_max, int n_vert)
{
  std::vector<mjtNum> dirs;
  dirs.reserve(n_horiz * std::max(n_vert, 1) * 3);

  const double h_step = (n_horiz > 1) ? (h_max - h_min) / (n_horiz - 1) : 0.0;

  if (n_vert <= 1)
  {
    // 2D: single horizontal ring at phi = 0
    for (int ih = 0; ih < n_horiz; ih++)
    {
      const double theta = h_min + ih * h_step;
      dirs.push_back(std::cos(theta));
      dirs.push_back(std::sin(theta));
      dirs.push_back(0.0);
    }
  }
  else
  {
    const double v_step = (n_vert > 1) ? (v_max - v_min) / (n_vert - 1) : 0.0;
    for (int iv = 0; iv < n_vert; iv++)
    {
      const double phi = v_min + iv * v_step;
      const double cp = std::cos(phi), sp = std::sin(phi);
      for (int ih = 0; ih < n_horiz; ih++)
      {
        const double theta = h_min + ih * h_step;
        dirs.push_back(cp * std::cos(theta));
        dirs.push_back(cp * std::sin(theta));
        dirs.push_back(sp);
      }
    }
  }
  return dirs;
}

// ---------------------------------------------------------------------------
// Create
// ---------------------------------------------------------------------------

LidarPublisher *LidarPublisher::Create(const mjModel *m, mjData *d, int plugin_id)
{
  (void)d;

  // --- site_name (required) ---
  const char *site_name_char = mj_getPluginConfig(m, plugin_id, "site_name");
  if (!site_name_char || strlen(site_name_char) == 0)
  {
    mju_error("[LidarPublisher] `site_name` is required.");
    return nullptr;
  }
  const int site_id = mj_name2id(m, mjOBJ_SITE, site_name_char);
  if (site_id < 0)
  {
    mju_error("[LidarPublisher] Site \"%s\" not found in model.", site_name_char);
    return nullptr;
  }

  // --- scan_pattern ---
  const char       *pattern_char = mj_getPluginConfig(m, plugin_id, "scan_pattern");
  const std::string scan_pattern
    = (pattern_char && strlen(pattern_char) > 0) ? std::string(pattern_char) : "custom";

  // --- frame_id ---
  const char       *frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  const std::string frame_id
    = (frame_id_char && strlen(frame_id_char) > 0) ? std::string(frame_id_char) : "lidar";

  // --- topic_name ---
  const char       *topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  const std::string topic_name      = (topic_name_char && strlen(topic_name_char) > 0)
                                        ? std::string(topic_name_char)
                                        : "mujoco/lidar/scan";

  // --- publish_rate ---
  const char *publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum      publish_rate      = 10.0;
  if (publish_rate_char && strlen(publish_rate_char) > 0)
    publish_rate = strtod(publish_rate_char, nullptr);
  if (publish_rate <= 0)
  {
    mju_error("[LidarPublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // --- range_min / range_max ---
  const char  *range_min_char = mj_getPluginConfig(m, plugin_id, "range_min");
  const char  *range_max_char = mj_getPluginConfig(m, plugin_id, "range_max");
  const double range_min
    = (range_min_char && strlen(range_min_char) > 0) ? strtod(range_min_char, nullptr) : 0.0;
  const double range_max
    = (range_max_char && strlen(range_max_char) > 0) ? strtod(range_max_char, nullptr) : 1000.0;

  // --- body_exclude ---
  // Semicolon-separated list of body names to exclude from ray hits.
  // e.g. "sensor_base_link;sensor_head"
  // mj_multiRay only supports one bodyexclude, so we use the last entry there
  // and re-cast any rays that still hit excluded bodies in a parallel loop.
  const char      *body_exclude_char = mj_getPluginConfig(m, plugin_id, "body_exclude");
  std::vector<int> excluded_body_ids;
  if (body_exclude_char && strlen(body_exclude_char) > 0)
  {
    std::string        body_exclude_str(body_exclude_char);
    std::istringstream ss{body_exclude_str};
    std::string        token;
    while (std::getline(ss, token, ';'))
    {
      const auto f = token.find_first_not_of(" \t");
      if (f == std::string::npos)
        continue;
      const auto l = token.find_last_not_of(" \t");
      token        = token.substr(f, l - f + 1);
      if (token.empty())
        continue;
      const int id = mj_name2id(m, mjOBJ_BODY, token.c_str());
      if (id < 0)
        mju_warning("[LidarPublisher] body_exclude body \"%s\" not found.", token.c_str());
      else
        excluded_body_ids.push_back(id);
    }
  }

  // --- output_tf ---
  const char *output_tf_char = mj_getPluginConfig(m, plugin_id, "output_tf");
  bool        output_tf      = false;
  if (output_tf_char && strlen(output_tf_char) > 0)
  {
    if (strcmp(output_tf_char, "true") != 0 && strcmp(output_tf_char, "false") != 0)
    {
      mju_error("[LidarPublisher] `output_tf` must be `true` or `false`.");
      return nullptr;
    }
    output_tf = (strcmp(output_tf_char, "true") == 0);
  }

  // --- tf_parent_frame_id ---
  const char       *tf_parent_char = mj_getPluginConfig(m, plugin_id, "tf_parent_frame_id");
  const std::string tf_parent_frame_id
    = (tf_parent_char && strlen(tf_parent_char) > 0) ? std::string(tf_parent_char) : "world";

  // --- qos ---
  const char       *qos_char = mj_getPluginConfig(m, plugin_id, "qos");
  const rclcpp::QoS qos      = parse_qos_string(qos_char ? std::string(qos_char) : "");

  // --- visualize_rays ---
  const char *vis_rays_char  = mj_getPluginConfig(m, plugin_id, "visualize_rays");
  const bool  visualize_rays = (vis_rays_char && strcmp(vis_rays_char, "true") == 0);

  // --- ray_hit_color / ray_miss_color ---
  const char *hit_color_char  = mj_getPluginConfig(m, plugin_id, "ray_hit_color");
  const char *miss_color_char = mj_getPluginConfig(m, plugin_id, "ray_miss_color");
  const auto  ray_hit_rgba    = parse_rgba_string(hit_color_char ? std::string(hit_color_char) : "",
                                                  {0.0f, 1.0f, 0.0f, 0.5f});
  const auto  ray_miss_rgba = parse_rgba_string(miss_color_char ? std::string(miss_color_char) : "",
                                                {1.0f, 0.0f, 0.0f, 0.3f});

  // --- skip_group ---
  const char *skip_group_char = mj_getPluginConfig(m, plugin_id, "skip_group");
  const auto geomgroup_filter = parse_skip_group(skip_group_char ? std::string(skip_group_char) : "");

  // --- Generate ray directions based on scan_pattern ---
  std::vector<mjtNum> ray_dirs_local;
  bool                is_3d           = true;
  double              min_angle       = -M_PI;
  double              max_angle       = M_PI;
  double              angle_increment = 0.0;

  if (scan_pattern == "vlp16")
  {
    ray_dirs_local = gen_rays_vlp16();
  }
  else if (scan_pattern == "hdl64")
  {
    ray_dirs_local = gen_rays_hdl64();
  }
  else if (scan_pattern == "vlp32")
  {
    ray_dirs_local = gen_rays_vlp32();
  }
  else if (scan_pattern == "os128")
  {
    ray_dirs_local = gen_rays_os128();
  }
  else // custom
  {
    // Horizontal parameters
    const char *min_angle_char = mj_getPluginConfig(m, plugin_id, "min_angle");
    const char *max_angle_char = mj_getPluginConfig(m, plugin_id, "max_angle");
    min_angle
      = (min_angle_char && strlen(min_angle_char) > 0) ? strtod(min_angle_char, nullptr) : -M_PI;
    max_angle
      = (max_angle_char && strlen(max_angle_char) > 0) ? strtod(max_angle_char, nullptr) : M_PI;

    // Determine num_horizontal_rays and angle_increment
    const char *nhoriz_char = mj_getPluginConfig(m, plugin_id, "num_horizontal_rays");
    const char *incr_char   = mj_getPluginConfig(m, plugin_id, "angle_increment");
    int         n_horiz     = 360;

    if (nhoriz_char && strlen(nhoriz_char) > 0)
    {
      n_horiz         = std::stoi(nhoriz_char);
      angle_increment = (max_angle - min_angle) / (n_horiz > 1 ? n_horiz - 1 : 1);
    }
    else if (incr_char && strlen(incr_char) > 0)
    {
      angle_increment = strtod(incr_char, nullptr);
      n_horiz         = static_cast<int>(std::round((max_angle - min_angle) / angle_increment)) + 1;
    }
    else
    {
      angle_increment = (max_angle - min_angle) / (n_horiz - 1);
    }

    // Vertical parameters
    const char *vlayers_char = mj_getPluginConfig(m, plugin_id, "vertical_layers");
    int         n_vert       = 1;
    if (vlayers_char && strlen(vlayers_char) > 0)
      n_vert = std::stoi(vlayers_char);

    is_3d = (n_vert > 1);

    double v_min = 0.0, v_max = 0.0;
    if (n_vert > 1)
    {
      const char *vmin_char = mj_getPluginConfig(m, plugin_id, "vertical_min_angle");
      const char *vmax_char = mj_getPluginConfig(m, plugin_id, "vertical_max_angle");
      if (!vmin_char || strlen(vmin_char) == 0 || !vmax_char || strlen(vmax_char) == 0)
      {
        mju_error("[LidarPublisher] `vertical_min_angle` and `vertical_max_angle` are required "
                  "when `vertical_layers` > 1.");
        return nullptr;
      }
      v_min = strtod(vmin_char, nullptr);
      v_max = strtod(vmax_char, nullptr);
    }

    ray_dirs_local = gen_rays_custom(min_angle, max_angle, n_horiz, v_min, v_max, n_vert);
  }

  if (ray_dirs_local.empty())
  {
    mju_error("[LidarPublisher] Failed to generate ray directions for pattern \"%s\".",
              scan_pattern.c_str());
    return nullptr;
  }

  const int nray = static_cast<int>(ray_dirs_local.size()) / 3;
  std::cout << "[LidarPublisher] Creating " << (is_3d ? "3D (PointCloud2)" : "2D (LaserScan)")
            << " lidar publisher for site \"" << site_name_char << "\" with " << nray << " rays."
            << std::endl;

  // --- noise_gaussian: "STDDEV" ---
  // Adds Gaussian N(0, stddev) noise to each valid hit distance.
  double noise_gaussian_stddev = 0.0;
  {
    const char *c = mj_getPluginConfig(m, plugin_id, "noise_gaussian");
    if (c && strlen(c) > 0)
      noise_gaussian_stddev = strtod(c, nullptr);
  }

  // --- noise_dropout: "PROB" ---
  // Each valid hit is randomly discarded (→ NaN) with probability PROB.
  double noise_dropout_prob = 0.0;
  {
    const char *c = mj_getPluginConfig(m, plugin_id, "noise_dropout");
    if (c && strlen(c) > 0)
      noise_dropout_prob = strtod(c, nullptr);
  }

  // --- noise_outlier: "PROB,MIN_RANGE,MAX_RANGE" ---
  // Each valid hit is replaced by a random distance in [MIN,MAX] with probability PROB.
  double noise_outlier_prob = 0.0, noise_outlier_min = 0.0, noise_outlier_max = 0.0;
  {
    const char *c = mj_getPluginConfig(m, plugin_id, "noise_outlier");
    if (c && strlen(c) > 0)
    {
      std::istringstream ss{std::string(c)};
      std::string        tok;
      int                idx = 0;
      while (std::getline(ss, tok, ','))
      {
        const double v = strtod(tok.c_str(), nullptr);
        if (idx == 0) noise_outlier_prob = v;
        else if (idx == 1) noise_outlier_min = v;
        else if (idx == 2) noise_outlier_max = v;
        ++idx;
      }
      if (noise_outlier_max <= noise_outlier_min)
        mju_warning("[LidarPublisher] noise_outlier: MAX_RANGE must be > MIN_RANGE; noise disabled.");
    }
  }

  // --- noise_jitter: "STDDEV" ---
  // Displaces each valid 3D point by an independent N(0, stddev) offset on x, y, z.
  // Only meaningful for 3D (PointCloud2); ignored for 2D LaserScan.
  double noise_jitter_stddev = 0.0;
  {
    const char *c = mj_getPluginConfig(m, plugin_id, "noise_jitter");
    if (c && strlen(c) > 0)
      noise_jitter_stddev = strtod(c, nullptr);
  }

  return new LidarPublisher(
    m, site_id, std::move(excluded_body_ids), frame_id, topic_name, publish_rate, range_min,
    range_max, is_3d, min_angle, max_angle, angle_increment, std::move(ray_dirs_local), output_tf,
    tf_parent_frame_id, qos, visualize_rays, ray_hit_rgba, ray_miss_rgba, geomgroup_filter,
    noise_gaussian_stddev, noise_dropout_prob, noise_outlier_prob, noise_outlier_min,
    noise_outlier_max, noise_jitter_stddev);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LidarPublisher::LidarPublisher(const mjModel *m, int site_id, std::vector<int> excluded_body_ids,
                               const std::string &frame_id, const std::string &topic_name,
                               mjtNum publish_rate, double range_min, double range_max, bool is_3d,
                               double min_angle, double max_angle, double angle_increment,
                               std::vector<mjtNum> ray_dirs_local, bool output_tf,
                               const std::string &tf_parent_frame_id, const rclcpp::QoS &qos,
                               bool visualize_rays, std::array<float, 4> ray_hit_rgba,
                               std::array<float, 4>          ray_miss_rgba,
                               std::array<mjtByte, mjNGROUP> geomgroup_filter,
                               double noise_gaussian_stddev, double noise_dropout_prob,
                               double noise_outlier_prob, double noise_outlier_min,
                               double noise_outlier_max, double noise_jitter_stddev)
    : site_id_(site_id)
    , excluded_body_ids_(std::move(excluded_body_ids))
    , output_tf_(output_tf)
    , tf_parent_frame_id_(tf_parent_frame_id)
    , frame_id_(frame_id)
    , topic_name_(topic_name)
    , is_3d_(is_3d)
    , min_angle_(min_angle)
    , max_angle_(max_angle)
    , angle_increment_(angle_increment)
    , range_min_(range_min)
    , range_max_(range_max)
    , nray_(static_cast<int>(ray_dirs_local.size()) / 3)
    , ray_dirs_local_(std::move(ray_dirs_local))
    , ray_dirs_world_(static_cast<size_t>(nray_) * 3, 0.0)
    , ray_dist_(static_cast<size_t>(nray_), -1.0)
    , ray_geomid_(static_cast<size_t>(nray_), -1)
    , geomgroup_filter_(geomgroup_filter)
    , noise_gaussian_stddev_(noise_gaussian_stddev)
    , noise_dropout_prob_(noise_dropout_prob)
    , noise_outlier_prob_(noise_outlier_prob)
    , noise_outlier_min_(noise_outlier_min)
    , noise_outlier_max_(noise_outlier_max)
    , noise_jitter_stddev_(noise_jitter_stddev)
    , visualize_rays_(visualize_rays)
    , ray_hit_rgba_(ray_hit_rgba)
    , ray_miss_rgba_(ray_miss_rgba)
    , publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
  int    argc = 0;
  char **argv = nullptr;
  ros_context_lease_.acquire(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({{"use_sim_time", true}});
  nh_ = rclcpp::Node::make_shared("lidar_publisher", node_options);

  if (!is_3d_)
  {
    scan_msg_.header.frame_id = frame_id_;
    scan_msg_.angle_min       = static_cast<float>(min_angle_);
    scan_msg_.angle_max       = static_cast<float>(max_angle_);
    scan_msg_.angle_increment = static_cast<float>(angle_increment_);
    scan_msg_.time_increment  = 0.0f;
    scan_msg_.scan_time       = static_cast<float>(publish_skip_ * m->opt.timestep);
    scan_msg_.range_min       = static_cast<float>(range_min_);
    scan_msg_.range_max       = static_cast<float>(range_max_);
    scan_msg_.ranges.resize(static_cast<size_t>(nray_), 0.0f);
    pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, qos);
  }
  else
  {
    pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, qos);
  }

  if (output_tf_)
  {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
  }
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

LidarPublisher::~LidarPublisher()
{
}

void LidarPublisher::reset(const mjModel *, int)
{}

// ---------------------------------------------------------------------------
// compute
// ---------------------------------------------------------------------------

void LidarPublisher::compute(const mjModel *m, mjData *d, int)
{
  sim_cnt_++;
  if (sim_cnt_ % publish_skip_ != 0)
    return;

  const rclcpp::Time now(static_cast<int32_t>(d->time),
                         static_cast<uint32_t>((d->time - static_cast<int32_t>(d->time)) * 1e9),
                         RCL_ROS_TIME);

  // Get site world-frame position and rotation matrix
  const mjtNum *site_pos = d->site_xpos + 3 * site_id_;
  const mjtNum *R        = d->site_xmat + 9 * site_id_;

  // Transform ray directions from sensor-local to world frame:
  //   v_w = R * v_local  (R is row-major 3×3)
  for (int i = 0; i < nray_; i++)
  {
    const mjtNum *v  = ray_dirs_local_.data() + 3 * i;
    mjtNum       *vw = ray_dirs_world_.data() + 3 * i;
    vw[0]            = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
    vw[1]            = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
    vw[2]            = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
  }

  // Cast all rays in parallel via mj_multiRay.
  // Use the last excluded body for mj_multiRay (innermost geom, minimises re-casts).
  // geomgroup_filter_: 1=include group, 0=exclude (MuJoCo API semantics).
  const int multiray_bodyexclude = excluded_body_ids_.empty() ? -1 : excluded_body_ids_.back();
#if mjVERSION_HEADER >= 3008000
  mj_multiRay(m, d, site_pos, ray_dirs_world_.data(), geomgroup_filter_.data(),
              static_cast<mjtByte>(1), // flg_static: include static geoms
              multiray_bodyexclude, ray_geomid_.data(), ray_dist_.data(),
              nullptr, // normals not needed
              nray_, static_cast<mjtNum>(range_max_));
#else
  mj_multiRay(m, d, site_pos, ray_dirs_world_.data(), geomgroup_filter_.data(),
              static_cast<mjtByte>(1), // flg_static: include static geoms
              multiray_bodyexclude, ray_geomid_.data(), ray_dist_.data(),
              nray_, static_cast<mjtNum>(range_max_));
#endif

  // Re-cast any ray that still lands on an excluded body (handles multiple excludes).
  // Each re-cast steps just past the excluded-body hit and calls mj_ray for the next.
  // Parallelised with OpenMP; mj_ray is read-only on m/d → thread-safe.
  if (!excluded_body_ids_.empty())
  {
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < nray_; i++)
    {
      const mjtNum *dir      = ray_dirs_world_.data() + 3 * i;
      const int     max_iter = static_cast<int>(excluded_body_ids_.size());

      for (int iter = 0; iter < max_iter; ++iter)
      {
        if (ray_dist_[i] < 0)
          break; // genuine miss

        // Check whether the hit geom belongs to an excluded body
        const int hit_body    = m->geom_bodyid[ray_geomid_[i]];
        bool      is_excluded = false;
        for (int eid : excluded_body_ids_)
          if (hit_body == eid)
          {
            is_excluded = true;
            break;
          }
        if (!is_excluded)
          break; // good hit, done

        // Step just past this excluded geom and find the next intersection
        const mjtNum new_start     = ray_dist_[i] + static_cast<mjtNum>(1e-6);
        const mjtNum new_origin[3] = {
          site_pos[0] + dir[0] * new_start,
          site_pos[1] + dir[1] * new_start,
          site_pos[2] + dir[2] * new_start,
        };
        int          new_geomid = -1;
#if mjVERSION_HEADER >= 3008000
        const mjtNum d_local
          = mj_ray(m, d, new_origin, dir, geomgroup_filter_.data(), 1, -1, &new_geomid, nullptr);
#else
        const mjtNum d_local
          = mj_ray(m, d, new_origin, dir, geomgroup_filter_.data(), 1, -1, &new_geomid);
#endif
        if (d_local >= 0)
        {
          ray_dist_[i]   = new_start + d_local; // distance from site_pos
          ray_geomid_[i] = new_geomid;
        }
        else
        {
          ray_dist_[i]   = -1;
          ray_geomid_[i] = -1;
        }
      }
    }
  }

  // Apply sensor noise (order: dropout → gaussian → outlier).
  // Each type is skipped when its parameter is zero/disabled.
  // Thread-local RNG: each OpenMP worker has its own engine; seeded once per thread.
  const bool do_noise = (noise_dropout_prob_ > 0.0 || noise_gaussian_stddev_ > 0.0
                         || noise_outlier_prob_ > 0.0);
  if (do_noise)
  {
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < nray_; i++)
    {
      if (ray_dist_[i] < 0)
        continue; // already a miss — nothing to corrupt

      thread_local std::mt19937 rng(std::random_device{}());

      // 1. Dropout: randomly discard valid hits (simulates absorption/rain)
      if (noise_dropout_prob_ > 0.0)
      {
        std::bernoulli_distribution drop(noise_dropout_prob_);
        if (drop(rng))
        {
          ray_dist_[i] = -1;
          continue;
        }
      }

      // 2. Gaussian range noise (simulates range measurement uncertainty)
      if (noise_gaussian_stddev_ > 0.0)
      {
        std::normal_distribution<double> nd(0.0, noise_gaussian_stddev_);
        ray_dist_[i] += nd(rng);
        if (ray_dist_[i] < 0.0)
          ray_dist_[i] = 0.0; // clamp; range check against range_min/max happens later
      }

      // 3. Outlier: replace hit with spurious random return (simulates multi-path/dust)
      if (noise_outlier_prob_ > 0.0 && noise_outlier_max_ > noise_outlier_min_)
      {
        std::bernoulli_distribution is_outlier(noise_outlier_prob_);
        if (is_outlier(rng))
        {
          std::uniform_real_distribution<double> ud(noise_outlier_min_, noise_outlier_max_);
          ray_dist_[i] = ud(rng);
        }
      }
    }
  }

  // Cache site position for visualize()
  last_site_pos_[0] = site_pos[0];
  last_site_pos_[1] = site_pos[1];
  last_site_pos_[2] = site_pos[2];
  has_computed_     = true;

  // TF broadcast (site world-frame pose)
  if (output_tf_ && tf_br_)
  {
    mjtNum quat[4];
    mju_mat2Quat(quat, R);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = now;
    tf_msg.header.frame_id         = tf_parent_frame_id_;
    tf_msg.child_frame_id          = frame_id_;
    tf_msg.transform.translation.x = site_pos[0];
    tf_msg.transform.translation.y = site_pos[1];
    tf_msg.transform.translation.z = site_pos[2];
    tf_msg.transform.rotation.w    = quat[0];
    tf_msg.transform.rotation.x    = quat[1];
    tf_msg.transform.rotation.y    = quat[2];
    tf_msg.transform.rotation.z    = quat[3];
    tf_br_->sendTransform(tf_msg);
  }

  if (!is_3d_)
  {
    // --- 2D: publish LaserScan ---
    scan_msg_.header.stamp = now;
    for (int i = 0; i < nray_; i++)
    {
      const float r = static_cast<float>(ray_dist_[i]);
      scan_msg_.ranges[i]
        = (r < 0 || r < static_cast<float>(range_min_) || r > static_cast<float>(range_max_))
            ? std::numeric_limits<float>::infinity()
            : r;
    }
    std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pub_)->publish(
      scan_msg_);
  }
  else
  {
    // --- 3D: publish PointCloud2 ---
    // Points in sensor frame: p = ray_dir_local * dist
    // (valid because rotation preserves vector direction/magnitude)
    const int            point_step = 12; // 3 × float32
    std::vector<uint8_t> data(static_cast<size_t>(nray_) * point_step);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < nray_; i++)
    {
      float       *p = reinterpret_cast<float *>(data.data() + i * point_step);
      const double r = ray_dist_[i];
      if (r < 0 || r < range_min_ || r > range_max_)
      {
        p[0] = p[1] = p[2] = std::numeric_limits<float>::quiet_NaN();
      }
      else
      {
        p[0] = static_cast<float>(ray_dirs_local_[3 * i + 0] * r);
        p[1] = static_cast<float>(ray_dirs_local_[3 * i + 1] * r);
        p[2] = static_cast<float>(ray_dirs_local_[3 * i + 2] * r);

        // 3D jitter: displace the point by an independent Gaussian offset on each axis.
        // Unlike range noise (shifts along the ray), this scatters the point in all
        // directions, modelling beam divergence, surface roughness, or vibration.
        if (noise_jitter_stddev_ > 0.0)
        {
          thread_local std::mt19937              rng(std::random_device{}());
          std::normal_distribution<float> nd(0.0f, static_cast<float>(noise_jitter_stddev_));
          p[0] += nd(rng);
          p[1] += nd(rng);
          p[2] += nd(rng);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp    = now;
    cloud.header.frame_id = frame_id_;
    cloud.height          = 1;
    cloud.width           = static_cast<uint32_t>(nray_);
    cloud.is_dense        = false;
    cloud.is_bigendian    = false;
    cloud.point_step      = point_step;
    cloud.row_step        = static_cast<uint32_t>(nray_) * point_step;
    cloud.fields.resize(3);
    for (int k = 0; k < 3; k++)
    {
      cloud.fields[k].name     = std::string(1, "xyz"[k]);
      cloud.fields[k].offset   = static_cast<uint32_t>(k * 4);
      cloud.fields[k].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[k].count    = 1;
    }
    cloud.data = std::move(data);
    std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pub_)->publish(
      cloud);
  }
}

// ---------------------------------------------------------------------------
// visualize
// ---------------------------------------------------------------------------

void LidarPublisher::visualize(const mjModel *, mjData *, const mjvOption *opt, mjvScene *scn, int)
{
  if (!visualize_rays_ || !has_computed_)
    return;

  // Skip adding debug geoms into offscreen camera scenes.
  // ImagePublisher sets option_.label = -1 (never a valid mjtLabel) as a sentinel so that plugin
  // visualize callbacks do not pollute offscreen camera renders with lidar ray visualizations.
  if (opt && opt->label < 0)
    return;

  // For large ray counts, subsample to ~1000 rays to avoid overwhelming the scene
  const int draw_step = (nray_ > 1000) ? (nray_ / 1000) : 1;

  for (int i = 0; i < nray_; i += draw_step)
  {
    if (scn->ngeom >= scn->maxgeom)
      break;

    const mjtNum r         = ray_dist_[i];
    const bool   hit       = (r >= 0 && r >= range_min_ && r <= range_max_);
    const mjtNum ray_range = hit ? r : static_cast<mjtNum>(range_max_);

    const mjtNum *vw    = ray_dirs_world_.data() + 3 * i;
    const mjtNum  to[3] = {
      last_site_pos_[0] + vw[0] * ray_range,
      last_site_pos_[1] + vw[1] * ray_range,
      last_site_pos_[2] + vw[2] * ray_range,
    };

    mjvGeom *geom = scn->geoms + scn->ngeom;
    mjv_initGeom(geom, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);
    mjv_connector(geom, mjGEOM_LINE, 1.5, last_site_pos_, to);
    const auto &rgba = hit ? ray_hit_rgba_ : ray_miss_rgba_;
    geom->rgba[0]    = rgba[0];
    geom->rgba[1]    = rgba[1];
    geom->rgba[2]    = rgba[2];
    geom->rgba[3]    = rgba[3];
    scn->ngeom++;
  }
}

} // namespace MujocoRosUtils
