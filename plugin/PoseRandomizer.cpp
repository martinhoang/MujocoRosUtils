#include "PoseRandomizer.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace MujocoRosUtils
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace
{

/** Parse a "min max" string into a 2-element array. */
std::array<double, 2> parseRange(const char * str, const char * attr_name)
{
  if(!str || strlen(str) == 0)
  {
    return {0.0, 0.0};
  }
  std::istringstream ss(str);
  double lo, hi;
  if(!(ss >> lo >> hi))
  {
    mju_error("[PoseRandomizer] Failed to parse attribute '%s': expected \"min max\"", attr_name);
    return {0.0, 0.0};
  }
  if(lo > hi)
  {
    mju_error("[PoseRandomizer] Attribute '%s': min (%f) > max (%f)", attr_name, lo, hi);
    return {0.0, 0.0};
  }
  return {lo, hi};
}

/** RPY (ZYX convention) to quaternion [w, x, y, z]. */
void rpy2quat(double roll, double pitch, double yaw, double quat[4])
{
  double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);

  quat[0] = cr * cp * cy + sr * sp * sy; // w
  quat[1] = sr * cp * cy - cr * sp * sy; // x
  quat[2] = cr * sp * cy + sr * cp * sy; // y
  quat[3] = cr * cp * sy - sr * sp * cy; // z
}

} // namespace

// ---------------------------------------------------------------------------
// RegisterPlugin
// ---------------------------------------------------------------------------
void PoseRandomizer::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::PoseRandomizer";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char * attributes[] = {"x_range", "y_range", "z_range", "roll_range", "pitch_range", "yaw_range"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int) { return 0; };

  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };

  plugin.needstage = mjSTAGE_POS;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id) -> int
  {
    auto * instance = PoseRandomizer::Create(m, d, plugin_id);
    if(!instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<PoseRandomizer *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double * /*plugin_state*/, void * plugin_data, int plugin_id)
  {
    auto * instance = reinterpret_cast<PoseRandomizer *>(plugin_data);
    instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int /*capability_bit*/)
  {
    auto * instance = reinterpret_cast<PoseRandomizer *>(d->plugin_data[plugin_id]);
    instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

// ---------------------------------------------------------------------------
// Create
// ---------------------------------------------------------------------------
PoseRandomizer * PoseRandomizer::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // -- Find the body that owns this plugin ----------------------------------
  int body_id = 0;
  for(; body_id < m->nbody; body_id++)
  {
    if(m->body_plugin[body_id] == plugin_id)
    {
      break;
    }
  }
  if(body_id == m->nbody)
  {
    mju_error("[PoseRandomizer] Plugin not found in any body. Attach as a child of <body>.");
    return nullptr;
  }

  // -- Find the freejoint of that body ---------------------------------------
  int freejoint_qposadr = -1;
  for(int jid = m->body_jntadr[body_id]; jid < m->body_jntadr[body_id] + m->body_jntnum[body_id]; jid++)
  {
    if(m->jnt_type[jid] == mjJNT_FREE)
    {
      freejoint_qposadr = m->jnt_qposadr[jid];
      break;
    }
  }
  if(freejoint_qposadr < 0)
  {
    const char * bname = mj_id2name(m, mjOBJ_BODY, body_id);
    mju_error("[PoseRandomizer] Body '%s' has no freejoint — cannot randomize pose.", bname ? bname : "(unknown)");
    return nullptr;
  }

  // -- Capture nominal pose from m->qpos0 ------------------------------------
  std::array<double, 3> nominal_pos = {m->qpos0[freejoint_qposadr + 0], m->qpos0[freejoint_qposadr + 1],
                                       m->qpos0[freejoint_qposadr + 2]};
  // MuJoCo freejoint quat layout: qpos[3]=w, qpos[4]=x, qpos[5]=y, qpos[6]=z
  std::array<double, 4> nominal_quat = {m->qpos0[freejoint_qposadr + 3], m->qpos0[freejoint_qposadr + 4],
                                        m->qpos0[freejoint_qposadr + 5], m->qpos0[freejoint_qposadr + 6]};

  // -- Parse ranges ----------------------------------------------------------
  auto x_range = parseRange(mj_getPluginConfig(m, plugin_id, "x_range"), "x_range");
  auto y_range = parseRange(mj_getPluginConfig(m, plugin_id, "y_range"), "y_range");
  auto z_range = parseRange(mj_getPluginConfig(m, plugin_id, "z_range"), "z_range");
  auto roll_range = parseRange(mj_getPluginConfig(m, plugin_id, "roll_range"), "roll_range");
  auto pitch_range = parseRange(mj_getPluginConfig(m, plugin_id, "pitch_range"), "pitch_range");
  auto yaw_range = parseRange(mj_getPluginConfig(m, plugin_id, "yaw_range"), "yaw_range");

  const char * bname = mj_id2name(m, mjOBJ_BODY, body_id);
  std::cout << "[PoseRandomizer] Create for body '" << (bname ? bname : "(unknown)") << "' "
            << "(nominal pos: [" << nominal_pos[0] << ", " << nominal_pos[1] << ", " << nominal_pos[2] << "])."
            << std::endl;

  (void)d; // unused at creation time
  return new PoseRandomizer(body_id, freejoint_qposadr, nominal_pos, nominal_quat, x_range, y_range, z_range,
                            roll_range, pitch_range, yaw_range);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
PoseRandomizer::PoseRandomizer(int body_id,
                               int freejoint_qposadr,
                               const std::array<double, 3> & nominal_pos,
                               const std::array<double, 4> & nominal_quat,
                               const std::array<double, 2> & x_range,
                               const std::array<double, 2> & y_range,
                               const std::array<double, 2> & z_range,
                               const std::array<double, 2> & roll_range,
                               const std::array<double, 2> & pitch_range,
                               const std::array<double, 2> & yaw_range)
: body_id_(body_id),
  freejoint_qposadr_(freejoint_qposadr),
  nominal_pos_(nominal_pos),
  nominal_quat_(nominal_quat),
  x_range_(x_range),
  y_range_(y_range),
  z_range_(z_range),
  roll_range_(roll_range),
  pitch_range_(pitch_range),
  yaw_range_(yaw_range)
{
  resample();
}

// ---------------------------------------------------------------------------
// resample  (draw new random offsets)
// ---------------------------------------------------------------------------
void PoseRandomizer::resample()
{
  auto uniform = [&](const std::array<double, 2> & range) -> double
  {
    if(range[0] == range[1])
    {
      return range[0];
    }
    std::uniform_real_distribution<double> dist(range[0], range[1]);
    return dist(rng_);
  };

  rand_pos_[0] = uniform(x_range_);
  rand_pos_[1] = uniform(y_range_);
  rand_pos_[2] = uniform(z_range_);
  rand_rpy_[0] = uniform(roll_range_);
  rand_rpy_[1] = uniform(pitch_range_);
  rand_rpy_[2] = uniform(yaw_range_);
}

// ---------------------------------------------------------------------------
// reset  (called by MuJoCo on mj_resetData — no mjData* available)
// ---------------------------------------------------------------------------
void PoseRandomizer::reset(const mjModel * /*m*/, int /*plugin_id*/)
{
  resample();
  pending_ = true;
}

// ---------------------------------------------------------------------------
// compute  (called every step — apply randomization on first call after reset)
// ---------------------------------------------------------------------------
void PoseRandomizer::compute(const mjModel * /*m*/, mjData * d, int /*plugin_id*/)
{
  if(!pending_)
  {
    return;
  }
  pending_ = false;

  // -- Position: nominal + offset --------------------------------------------
  d->qpos[freejoint_qposadr_ + 0] = nominal_pos_[0] + rand_pos_[0];
  d->qpos[freejoint_qposadr_ + 1] = nominal_pos_[1] + rand_pos_[1];
  d->qpos[freejoint_qposadr_ + 2] = nominal_pos_[2] + rand_pos_[2];

  // -- Orientation: nominal_quat * perturbation_quat -------------------------
  double perturb_quat[4];
  rpy2quat(rand_rpy_[0], rand_rpy_[1], rand_rpy_[2], perturb_quat);

  // mju_mulQuat: result = qa * qb  (quat layout: w x y z)
  double result_quat[4];
  mju_mulQuat(result_quat, nominal_quat_.data(), perturb_quat);

  d->qpos[freejoint_qposadr_ + 3] = result_quat[0]; // w
  d->qpos[freejoint_qposadr_ + 4] = result_quat[1]; // x
  d->qpos[freejoint_qposadr_ + 5] = result_quat[2]; // y
  d->qpos[freejoint_qposadr_ + 6] = result_quat[3]; // z
}

} // namespace MujocoRosUtils
