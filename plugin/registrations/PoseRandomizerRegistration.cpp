#include "registration_compat.hpp"
#include "../PoseRandomizer.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsPoseRandomizer)
{
  MujocoRosUtils::PoseRandomizer::RegisterPlugin();
}
