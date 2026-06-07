#include "registration_compat.hpp"
#include "../SceneManager.h"

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsSceneManager)
{
  MujocoRosUtils::SceneManager::RegisterPlugin();
}
