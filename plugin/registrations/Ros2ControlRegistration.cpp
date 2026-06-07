#include "registration_compat.hpp"
#include "../RosControl.hpp"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsRos2Control)
{
  MujocoRosUtils::Ros2Control::RegisterPlugin();
}
