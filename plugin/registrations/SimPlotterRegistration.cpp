#include "registration_compat.hpp"
#include "../SimPlotter.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsSimPlotter)
{
  MujocoRosUtils::SimPlotter::RegisterPlugin();
}
