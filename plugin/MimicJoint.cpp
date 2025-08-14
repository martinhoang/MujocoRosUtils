#include "MimicJoint.hpp"

#include <mujoco/mujoco.h>

#include "mujoco_utils.hpp"

#include <cstring>

namespace MujocoRosUtils
{

constexpr char ATTR_MIMIC_JOINT[] = "mimic_joint";
constexpr char ATTR_GEAR[]        = "gear";

void MimicJoint::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  print_info("Registering MimicJoint plugin\n");

  plugin.name = "MujocoRosUtils::MimicJoint";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  std::vector<const char *> attributes = {ATTR_MIMIC_JOINT, ATTR_GEAR};

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  plugin.nstate = +[](const mjModel *, // m
                      int              // plugin_id
                   ) {
    return 0;
  };

  plugin.needstage = 0;

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    auto plugin_instance = MimicJoint::Create(m, d, plugin_id);
    if (!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance.release());
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<MimicJoint *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<MimicJoint *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int // capability_bit
                    ) {
    auto *plugin_instance = reinterpret_cast<class MimicJoint *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);

  print_confirm("Successfully registered 'MimicJoint' plugin\n");
};

std::unique_ptr<MimicJoint> MimicJoint::Create(const mjModel *m, mjData *d, int plugin_id)
{
  std::map<int, int> mimic_joint_ids;

  // Get master actuator id
  const char *master_joint_char = mj_getPluginConfig(m, plugin_id, ATTR_MIMIC_JOINT);

  int master_joint_id = -1;
  if (master_joint_char && strlen(master_joint_char) > 0)
  {
    master_joint_id = mj_name2id(m, mjOBJ_JOINT, master_joint_char);
  }
  else
  {
    mju_error("[MimicJoint] No master joint specified in plugin config for plugin ID %d.\n",
              plugin_id);
  }

  double      gear      = 1.0;
  const char *gear_char = mj_getPluginConfig(m, plugin_id, ATTR_GEAR);
  if (gear_char && strlen(gear_char) > 0)
  {
    gear = std::stod(gear_char);
  }
  else
  {
    print_info("[MimicJoint] No gear specified. Default to %f\n", gear);
  }

  // Since this plugin is a kind of actuator
  int this_actuator_id = -1;
  for (int i = 0; i < m->nu; ++i)
  {
    if (m->actuator_plugin[i] == plugin_id)
    {
      this_actuator_id = i;
      break;
    }
  }

  // Get the joint associated with this actuator
  int         slave_joint_id  = m->actuator_trnid[2 * this_actuator_id];
  const char *joint_name_char = mj_id2name(m, mjOBJ_JOINT, slave_joint_id);

  // Find slave actuator id
  int slave_act_id = -1;
  for (int idx = 0; idx < m->nu; ++idx)
  {
    // If the actuator is associated with this joint
    if (m->actuator_trnid[2 * idx] == slave_joint_id)
    {
      print_info("Actuator id %d is associated with joint '%s'\n", idx, joint_name_char);
      int gain_type = m->actuator_gaintype[idx];
      int bias_type = m->actuator_biastype[idx];

      if (gain_type == mjGAIN_FIXED && bias_type == mjBIAS_AFFINE)
      {
        slave_act_id = idx;
        break; // No need to check further, we found the actuator
      }
    }
  }

  if (slave_act_id < 0)
  {
    mju_error(
      "[MimicJoint] No slave actuator found for master joint '%s'. Did you forget to add "
      "a controller for the slave joint, e.g. <actuator><position></position></actuator>?\n",
      joint_name_char);
  }

  return std::unique_ptr<MimicJoint>(new MimicJoint(m, d, master_joint_id, slave_act_id, gear));
}

MimicJoint::MimicJoint(const mjModel *m, mjData *d, int master, int slave, double gear)
    : master_joint_id_(master)
    , slave_act_id_(slave)
    , gear_(gear)
{
  const char *master_joint_name = mj_id2name(m, mjOBJ_JOINT, master_joint_id_);
  slave_joint_id_               = m->actuator_trnid[2 * slave_act_id_];
  const char *slave_joint_name  = mj_id2name(m, mjOBJ_JOINT, slave_joint_id_);

  print_info("[MimicJoint] Created mimic joint '%s':\n- Slave Actuator id: %d\n- Gear %f\n- Master "
             "Joint: '%s'\n",
             slave_joint_name, slave_act_id_, gear_, master_joint_name);
}

void MimicJoint::reset(const mjModel *m, int plugin_id)
{
  // Reset the joint state
}

void MimicJoint::compute(const mjModel *m, mjData *d, int plugin_id)
{
  // Compute the joint state
  // d->ctrl[slave_act_id_] = d->qpos[master_joint_id_] * gear_;
  d->qpos[slave_joint_id_] = d->qpos[master_joint_id_] * gear_;
}

} // namespace MujocoRosUtils