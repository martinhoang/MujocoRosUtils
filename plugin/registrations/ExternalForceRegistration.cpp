#include "registration_compat.hpp"
#include "../ExternalForce.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsExternalForce)
{
  MujocoRosUtils::ExternalForce::RegisterPlugin();
}
