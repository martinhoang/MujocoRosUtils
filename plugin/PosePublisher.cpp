#include "PosePublisher.h"

#include <mujoco/mujoco.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <iostream>

namespace MujocoRosUtils
{

constexpr char ATTR_FRAME_ID[]                = "frame_id";
constexpr char ATTR_PUBLISH_RATE[]            = "publish_rate";
constexpr char ATTR_OUTPUT_TF[]               = "output_tf";
constexpr char ATTR_TF_CHILD_FRAME_ID[]       = "tf_child_frame_id";
constexpr char ATTR_POSE_TOPIC_NAME[]         = "pose_topic_name";
constexpr char ATTR_VEL_TOPIC_NAME[]          = "vel_topic_name";

void PosePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::PosePublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {ATTR_FRAME_ID,     ATTR_POSE_TOPIC_NAME, ATTR_VEL_TOPIC_NAME,
                               ATTR_PUBLISH_RATE, ATTR_OUTPUT_TF,       ATTR_TF_CHILD_FRAME_ID};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = PosePublisher::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<PosePublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class PosePublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class PosePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

PosePublisher * PosePublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // frame_id
  const char * frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_FRAME_ID);
  std::string frame_id = "";
  if(frame_id_char && strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
  }

  // pose_topic_name
  const char * pose_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_POSE_TOPIC_NAME);
  std::string pose_topic_name = "";
  if(pose_topic_name_char && strlen(pose_topic_name_char) > 0)
  {
    pose_topic_name = std::string(pose_topic_name_char);
  }

  // vel_topic_name
  const char * vel_topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_VEL_TOPIC_NAME);
  std::string vel_topic_name = "";
  if(vel_topic_name_char && strlen(vel_topic_name_char) > 0)
  {
    vel_topic_name = std::string(vel_topic_name_char);
  }

  // publish_rate
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, ATTR_PUBLISH_RATE);
  mjtNum publish_rate = 30.0;
  if(publish_rate_char && strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if(publish_rate <= 0)
  {
    mju_error("[PosePublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // output_tf
  const char * output_tf_char = mj_getPluginConfig(m, plugin_id, ATTR_OUTPUT_TF);
  bool output_tf = false;
  if(output_tf_char && strlen(output_tf_char) > 0)
  {
    if(!(strcmp(output_tf_char, "true") == 0 || strcmp(output_tf_char, "false") == 0))
    {
      mju_error("[PosePublisher] `output_tf` must be `true` or `false`.");
      return nullptr;
    }
    output_tf = (strcmp(output_tf_char, "true") == 0);
  }

  // tf_child_frame_id
  const char * tf_child_frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_TF_CHILD_FRAME_ID);
  std::string tf_child_frame_id = "";
  if(tf_child_frame_id_char && strlen(tf_child_frame_id_char) > 0)
  {
    tf_child_frame_id = std::string(tf_child_frame_id_char);
  }

  // Set sensor_id
  int sensor_id = 0;
  for(; sensor_id < m->nsensor; sensor_id++)
  {
    if(m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if(sensor_id == m->nsensor)
  {
    mju_error("[PosePublisher] Plugin not found in sensors.");
    return nullptr;
  }
  if(m->sensor_objtype[sensor_id] != mjOBJ_XBODY)
  {
    mju_error("[PosePublisher] Plugin must be attached to a xbody.");
    return nullptr;
  }

  // Find reference body ID from frame_id
  int reference_body_id = -1;
  if(!frame_id.empty() && frame_id != "map" && frame_id != "world")
  {
    reference_body_id = mj_name2id(m, mjOBJ_XBODY, frame_id.c_str());
    if(reference_body_id == -1)
    {
      std::cout << "[PosePublisher] Warning: Reference frame '" << frame_id 
                << "' not found in model. Using world frame." << std::endl;
    }
  }

  std::cout << "[PosePublisher] Create." << std::endl;

  return new PosePublisher(m, d, sensor_id, frame_id, pose_topic_name, vel_topic_name, publish_rate, output_tf,
                           tf_child_frame_id, reference_body_id);
}

PosePublisher::PosePublisher(const mjModel * m,
                             mjData *, // d
                             int sensor_id,
                             const std::string & frame_id,
                             const std::string & pose_topic_name,
                             const std::string & vel_topic_name,
                             mjtNum publish_rate,
                             bool output_tf,
                             const std::string & tf_child_frame_id,
                             int reference_body_id)
: sensor_id_(sensor_id), body_id_(m->sensor_objid[sensor_id]), reference_body_id_(reference_body_id), frame_id_(frame_id), 
  pose_topic_name_(pose_topic_name), vel_topic_name_(vel_topic_name), 
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1)),
  output_tf_(output_tf), tf_child_frame_id_(tf_child_frame_id)
{
  std::string body_name = mj_id2name(m, mjOBJ_XBODY, body_id_);
  if(frame_id_.empty())
  {
    frame_id_ = "map";
  }
  if(pose_topic_name_.empty())
  {
    pose_topic_name_ = "mujoco/" + body_name + "/pose";
  }
  if(vel_topic_name_.empty())
  {
    vel_topic_name_ = "mujoco/" + body_name + "/vel";
  }
  if(tf_child_frame_id_.empty())
  {
    tf_child_frame_id_ = body_name;
  }

  int argc = 0;
  char ** argv = nullptr;
  if(!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;

  node_options.parameter_overrides({
    {"use_sim_time", true}, // Force use simulation time
  });
  std::string node_name = body_name + "_pose_publisher";
  nh_ = rclcpp::Node::make_shared(node_name, node_options);
  
  if(output_tf_)
  {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
  }
  else
  {
    pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 1);
    vel_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic_name, 1);
  }
}

void PosePublisher::reset(const mjModel *, // m
                          int // plugin_id
)
{
}

void PosePublisher::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  rclcpp::Time stamp_now = nh_->get_clock()->now();

  if(output_tf_)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp_now;
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = tf_child_frame_id_;

    // If we have a reference body, compute relative transform
    if(reference_body_id_ >= 0)
    {
      // Get sensor body transform in world frame
      tf2::Vector3 sensor_pos(d->xpos[3 * body_id_ + 0], 
                               d->xpos[3 * body_id_ + 1], 
                               d->xpos[3 * body_id_ + 2]);
      tf2::Quaternion sensor_quat(d->xquat[4 * body_id_ + 1], // x
                                   d->xquat[4 * body_id_ + 2], // y  
                                   d->xquat[4 * body_id_ + 3], // z
                                   d->xquat[4 * body_id_ + 0]); // w
      tf2::Transform T_world_sensor(sensor_quat, sensor_pos);

      // Get reference body transform in world frame
      tf2::Vector3 ref_pos(d->xpos[3 * reference_body_id_ + 0],
                            d->xpos[3 * reference_body_id_ + 1], 
                            d->xpos[3 * reference_body_id_ + 2]);
      tf2::Quaternion ref_quat(d->xquat[4 * reference_body_id_ + 1], // x
                                d->xquat[4 * reference_body_id_ + 2], // y
                                d->xquat[4 * reference_body_id_ + 3], // z
                                d->xquat[4 * reference_body_id_ + 0]); // w
      tf2::Transform T_world_ref(ref_quat, ref_pos);

      // Compute relative transform: T_ref_sensor = T_world_ref^-1 * T_world_sensor
      tf2::Transform T_ref_sensor = T_world_ref.inverse() * T_world_sensor;

      // Extract position and orientation
      tf2::Vector3 rel_pos = T_ref_sensor.getOrigin();
      tf2::Quaternion rel_quat = T_ref_sensor.getRotation();

      msg.transform.translation.x = rel_pos.x();
      msg.transform.translation.y = rel_pos.y();
      msg.transform.translation.z = rel_pos.z();
      msg.transform.rotation.w = rel_quat.w();
      msg.transform.rotation.x = rel_quat.x();
      msg.transform.rotation.y = rel_quat.y();
      msg.transform.rotation.z = rel_quat.z();
    }
    else
    {
      // Use world frame (original behavior)
      msg.transform.translation.x = d->xpos[3 * body_id_ + 0];
      msg.transform.translation.y = d->xpos[3 * body_id_ + 1];
      msg.transform.translation.z = d->xpos[3 * body_id_ + 2];
      msg.transform.rotation.w = d->xquat[4 * body_id_ + 0];
      msg.transform.rotation.x = d->xquat[4 * body_id_ + 1];
      msg.transform.rotation.y = d->xquat[4 * body_id_ + 2];
      msg.transform.rotation.z = d->xquat[4 * body_id_ + 3];
    }

    tf_br_->sendTransform(msg);
  }
  else
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp_now;
    pose_msg.header.frame_id = frame_id_;

    // If we have a reference body, compute relative pose
    if(reference_body_id_ >= 0)
    {
      // Get sensor body transform in world frame
      tf2::Vector3 sensor_pos(d->xpos[3 * body_id_ + 0], 
                               d->xpos[3 * body_id_ + 1], 
                               d->xpos[3 * body_id_ + 2]);
      tf2::Quaternion sensor_quat(d->xquat[4 * body_id_ + 1], // x
                                   d->xquat[4 * body_id_ + 2], // y  
                                   d->xquat[4 * body_id_ + 3], // z
                                   d->xquat[4 * body_id_ + 0]); // w
      tf2::Transform T_world_sensor(sensor_quat, sensor_pos);

      // Get reference body transform in world frame
      tf2::Vector3 ref_pos(d->xpos[3 * reference_body_id_ + 0],
                            d->xpos[3 * reference_body_id_ + 1], 
                            d->xpos[3 * reference_body_id_ + 2]);
      tf2::Quaternion ref_quat(d->xquat[4 * reference_body_id_ + 1], // x
                                d->xquat[4 * reference_body_id_ + 2], // y
                                d->xquat[4 * reference_body_id_ + 3], // z
                                d->xquat[4 * reference_body_id_ + 0]); // w
      tf2::Transform T_world_ref(ref_quat, ref_pos);

      // Compute relative transform: T_ref_sensor = T_world_ref^-1 * T_world_sensor
      tf2::Transform T_ref_sensor = T_world_ref.inverse() * T_world_sensor;

      // Extract position and orientation
      tf2::Vector3 rel_pos = T_ref_sensor.getOrigin();
      tf2::Quaternion rel_quat = T_ref_sensor.getRotation();

      pose_msg.pose.position.x = rel_pos.x();
      pose_msg.pose.position.y = rel_pos.y();
      pose_msg.pose.position.z = rel_pos.z();
      pose_msg.pose.orientation.w = rel_quat.w();
      pose_msg.pose.orientation.x = rel_quat.x();
      pose_msg.pose.orientation.y = rel_quat.y();
      pose_msg.pose.orientation.z = rel_quat.z();
    }
    else
    {
      // Use world frame (original behavior)
      pose_msg.pose.position.x = d->xpos[3 * body_id_ + 0];
      pose_msg.pose.position.y = d->xpos[3 * body_id_ + 1];
      pose_msg.pose.position.z = d->xpos[3 * body_id_ + 2];
      pose_msg.pose.orientation.w = d->xquat[4 * body_id_ + 0];
      pose_msg.pose.orientation.x = d->xquat[4 * body_id_ + 1];
      pose_msg.pose.orientation.y = d->xquat[4 * body_id_ + 2];
      pose_msg.pose.orientation.z = d->xquat[4 * body_id_ + 3];
    }

    pose_pub_->publish(pose_msg);

    // For velocity, we need to transform it to the reference frame
    geometry_msgs::msg::TwistStamped vel_msg;
    mjtNum vel[6];
    mj_objectVelocity(m, d, mjOBJ_XBODY, body_id_, vel, 0);
    vel_msg.header.stamp = stamp_now;
    vel_msg.header.frame_id = frame_id_;

    if(reference_body_id_ >= 0)
    {
      // Get reference body velocity
      mjtNum ref_vel[6];
      mj_objectVelocity(m, d, mjOBJ_XBODY, reference_body_id_, ref_vel, 0);

      // Get reference body orientation for velocity transformation
      tf2::Quaternion ref_quat(d->xquat[4 * reference_body_id_ + 1], // x
                                d->xquat[4 * reference_body_id_ + 2], // y
                                d->xquat[4 * reference_body_id_ + 3], // z
                                d->xquat[4 * reference_body_id_ + 0]); // w
      tf2::Matrix3x3 ref_rot_matrix(ref_quat);
      tf2::Matrix3x3 ref_rot_matrix_inv = ref_rot_matrix.transpose();

      // Transform linear velocity to reference frame and compute relative velocity
      tf2::Vector3 sensor_lin_vel(vel[3], vel[4], vel[5]);
      tf2::Vector3 ref_lin_vel(ref_vel[3], ref_vel[4], ref_vel[5]);
      tf2::Vector3 rel_lin_vel = ref_rot_matrix_inv * (sensor_lin_vel - ref_lin_vel);

      // Transform angular velocity to reference frame and compute relative velocity  
      tf2::Vector3 sensor_ang_vel(vel[0], vel[1], vel[2]);
      tf2::Vector3 ref_ang_vel(ref_vel[0], ref_vel[1], ref_vel[2]);
      tf2::Vector3 rel_ang_vel = ref_rot_matrix_inv * (sensor_ang_vel - ref_ang_vel);

      vel_msg.twist.linear.x = rel_lin_vel.x();
      vel_msg.twist.linear.y = rel_lin_vel.y();
      vel_msg.twist.linear.z = rel_lin_vel.z();
      vel_msg.twist.angular.x = rel_ang_vel.x();
      vel_msg.twist.angular.y = rel_ang_vel.y();
      vel_msg.twist.angular.z = rel_ang_vel.z();
    }
    else
    {
      // Use world frame (original behavior)
      vel_msg.twist.linear.x = vel[3];
      vel_msg.twist.linear.y = vel[4];
      vel_msg.twist.linear.z = vel[5];
      vel_msg.twist.angular.x = vel[0];
      vel_msg.twist.angular.y = vel[1];
      vel_msg.twist.angular.z = vel[2];
    }

    vel_pub_->publish(vel_msg);
  }
}

} // namespace MujocoRosUtils
