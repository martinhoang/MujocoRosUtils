#include "registration_compat.hpp"
#include "../ClockPublisher.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsClockPublisher)
{
  MujocoRosUtils::ClockPublisher::RegisterPlugin();
}
