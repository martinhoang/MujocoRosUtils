#include "registration_compat.hpp"
#include "../LidarPublisher.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsLidarPublisher)
{
  MujocoRosUtils::LidarPublisher::RegisterPlugin();
}
