#include <mujoco/mujoco.h>   // provides mjVERSION_HEADER
#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "LidarPublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"
#include "MimicJoint.hpp"
#include "PoseRandomizer.h"
#include "RosControl.hpp"
#include "SceneManager.h"
#include "SimDataAggregator.h"
#include "SimPlotter.h"

namespace MujocoRosUtils
{

// mjPLUGIN_LIB_INIT gained a required name argument in MuJoCo 3.8.0 to avoid
// initialization function name collisions between plugins.
#if mjVERSION_HEADER >= 380
mjPLUGIN_LIB_INIT(MujocoRosUtils)
#else
mjPLUGIN_LIB_INIT
#endif
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
  ImagePublisher::RegisterPlugin();
  ActuatorCommand::RegisterPlugin();
  ExternalForce::RegisterPlugin();
  SensorPublisher::RegisterPlugin();
  LidarPublisher::RegisterPlugin();
  MimicJoint::RegisterPlugin();
  PoseRandomizer::RegisterPlugin();
  Ros2Control::RegisterPlugin();
  SimDataAggregator::RegisterPlugin();
  SimPlotter::RegisterPlugin();
  SceneManager::RegisterPlugin();
}

} // namespace MujocoRosUtils
