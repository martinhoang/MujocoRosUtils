#include "registration_compat.hpp"
#include "../ActuatorCommand.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsActuatorCommand)
{
  MujocoRosUtils::ActuatorCommand::RegisterPlugin();
}
