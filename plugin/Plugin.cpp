#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "TopicControl.h"
#include "ClockPublisher.h"
#include "ContactForcePublisher.h"
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
  TopicControl::RegisterPlugin();
  ExternalForce::RegisterPlugin();
  SensorPublisher::RegisterPlugin();
  MimicJoint::RegisterPlugin();
  Ros2Control::RegisterPlugin();
  ContactForcePublisher::RegisterPlugin();
}

} // namespace MujocoRosUtils
