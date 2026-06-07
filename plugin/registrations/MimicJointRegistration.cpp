#include "registration_compat.hpp"
#include "../MimicJoint.hpp"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsMimicJoint)
{
  MujocoRosUtils::MimicJoint::RegisterPlugin();
}
