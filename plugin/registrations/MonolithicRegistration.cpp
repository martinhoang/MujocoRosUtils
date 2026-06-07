#include "registration_compat.hpp"

#ifdef MUJOCO_ROS_UTILS_ENABLE_ACTUATOR_COMMAND
#include "../ActuatorCommand.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_CLOCK_PUBLISHER
#include "../ClockPublisher.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_EXTERNAL_FORCE
#include "../ExternalForce.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_IMAGE_PUBLISHER
#include "../ImagePublisher.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_LIDAR_PUBLISHER
#include "../LidarPublisher.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_MIMIC_JOINT
#include "../MimicJoint.hpp"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_POSE_PUBLISHER
#include "../PosePublisher.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_POSE_RANDOMIZER
#include "../PoseRandomizer.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_ROS2_CONTROL
#include "../RosControl.hpp"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SCENE_MANAGER
#include "../SceneManager.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SENSOR_PUBLISHER
#include "../SensorPublisher.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SIM_DATA_AGGREGATOR
#include "../SimDataAggregator.h"
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SIM_PLOTTER
#include "../SimPlotter.h"
#endif

MUJOCO_ROS_UTILS_PLUGIN_INIT(MujocoRosUtilsMonolithic)
{
#ifdef MUJOCO_ROS_UTILS_ENABLE_CLOCK_PUBLISHER
  MujocoRosUtils::ClockPublisher::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_POSE_PUBLISHER
  MujocoRosUtils::PosePublisher::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_IMAGE_PUBLISHER
  MujocoRosUtils::ImagePublisher::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_ACTUATOR_COMMAND
  MujocoRosUtils::ActuatorCommand::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_EXTERNAL_FORCE
  MujocoRosUtils::ExternalForce::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SENSOR_PUBLISHER
  MujocoRosUtils::SensorPublisher::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_LIDAR_PUBLISHER
  MujocoRosUtils::LidarPublisher::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_MIMIC_JOINT
  MujocoRosUtils::MimicJoint::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_POSE_RANDOMIZER
  MujocoRosUtils::PoseRandomizer::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_ROS2_CONTROL
  MujocoRosUtils::Ros2Control::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SIM_DATA_AGGREGATOR
  MujocoRosUtils::SimDataAggregator::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SIM_PLOTTER
  MujocoRosUtils::SimPlotter::RegisterPlugin();
#endif
#ifdef MUJOCO_ROS_UTILS_ENABLE_SCENE_MANAGER
  MujocoRosUtils::SceneManager::RegisterPlugin();
#endif
}
