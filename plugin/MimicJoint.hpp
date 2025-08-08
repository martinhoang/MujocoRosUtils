#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <limits>
#include <string> 
#include <memory>
#include <map>

namespace MujocoRosUtils
{

class MimicJoint
{
public:
    static void RegisterPlugin();

    static std::unique_ptr<MimicJoint> Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  MimicJoint(MimicJoint &&) = default;

  void reset(const mjModel * m, int plugin_id);
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
    MimicJoint(const mjModel * m, mjData * d, int master, int slave, double gear);

    int master_joint_id_;
    int slave_act_id_;
    double gear_;
};
} // namespace MujocoRosUtils