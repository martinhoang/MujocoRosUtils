#include "registration_compat.hpp"
#include "../PosePublisher.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsPosePublisher)
{
  MujocoRosUtils::PosePublisher::RegisterPlugin();
}
