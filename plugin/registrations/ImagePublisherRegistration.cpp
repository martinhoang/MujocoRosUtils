#include "registration_compat.hpp"
#include "../ImagePublisher.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsImagePublisher)
{
  MujocoRosUtils::ImagePublisher::RegisterPlugin();
}
