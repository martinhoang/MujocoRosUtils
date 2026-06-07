#pragma once

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#if mjVERSION_HEADER >= 380
#define MUJOCO_ROS_UTILS_PLUGIN_INIT(name) mjPLUGIN_LIB_INIT(name)
#else
#define MUJOCO_ROS_UTILS_PLUGIN_INIT(name) mjPLUGIN_LIB_INIT
#endif
