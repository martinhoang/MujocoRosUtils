#include "registration_compat.hpp"
#include "../SensorPublisher.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsSensorPublisher)
{
  MujocoRosUtils::SensorPublisher::RegisterPlugin();
}
