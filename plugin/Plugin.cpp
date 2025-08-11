#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"
#include "MimicJoint.hpp"
#include "RosControl.hpp"

namespace MujocoRosUtils
{

mjPLUGIN_LIB_INIT
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
  ImagePublisher::RegisterPlugin();
  ActuatorCommand::RegisterPlugin();
  ExternalForce::RegisterPlugin();
  SensorPublisher::RegisterPlugin();
  MimicJoint::RegisterPlugin();
  Ros2Control::RegisterPlugin();
}

} // namespace MujocoRosUtils
