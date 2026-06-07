#include "registration_compat.hpp"
#include "../SimDataAggregator.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsSimDataAggregator)
{
  MujocoRosUtils::SimDataAggregator::RegisterPlugin();
}
