#include "GeomTfPublisher.h"

#include <mujoco/mujoco.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <iostream>
#include <functional>
#include <algorithm>

namespace MujocoRosUtils
{

constexpr char ATTR_PARENT_BODY[]     = "parent_body";
constexpr char ATTR_FRAME_ID[]        = "frame_id";
constexpr char ATTR_TOPIC_NAME[]      = "topic_name";
constexpr char ATTR_PUBLISH_RATE[]    = "publish_rate";
constexpr char ATTR_PUBLISH_BODIES[]  = "publish_bodies";

void GeomTfPublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::GeomTfPublisher";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char * attributes[] = {ATTR_PARENT_BODY, ATTR_FRAME_ID, ATTR_TOPIC_NAME, ATTR_PUBLISH_RATE, ATTR_PUBLISH_BODIES};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = GeomTfPublisher::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<GeomTfPublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class GeomTfPublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class GeomTfPublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

GeomTfPublisher * GeomTfPublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // parent_body
  const char * parent_body_char = mj_getPluginConfig(m, plugin_id, ATTR_PARENT_BODY);
  if(!parent_body_char || strlen(parent_body_char) == 0)
  {
    mju_error("[GeomTfPublisher] `parent_body` attribute is required.");
    return nullptr;
  }
  std::string parent_body_name = std::string(parent_body_char);

  // Find parent body ID
  int parent_body_id = mj_name2id(m, mjOBJ_BODY, parent_body_name.c_str());
  if(parent_body_id == -1)
  {
    mju_error("[GeomTfPublisher] Parent body '%s' not found in model.", parent_body_name.c_str());
    return nullptr;
  }

  // Helper function to recursively find all descendant bodies
  std::vector<int> body_ids;
  std::function<void(int)> find_descendant_bodies = [&](int body_id) {
    body_ids.push_back(body_id);
    // Find all bodies whose parent is the current body
    for(int i = 0; i < m->nbody; i++)
    {
      if(m->body_parentid[i] == body_id)
      {
        find_descendant_bodies(i);
      }
    }
  };
  
  // Find all descendant bodies starting from parent_body_id
  find_descendant_bodies(parent_body_id);

  // Find all geoms attached to the parent body and all its descendants
  std::vector<int> geom_ids;
  for(int i = 0; i < m->ngeom; i++)
  {
    int geom_body_id = m->geom_bodyid[i];
    // Check if this geom belongs to any of the descendant bodies
    if(std::find(body_ids.begin(), body_ids.end(), geom_body_id) != body_ids.end())
    {
      geom_ids.push_back(i);
    }
  }

  if(geom_ids.empty())
  {
    std::cout << "[GeomTfPublisher] Warning: No geoms found under parent body '" 
              << parent_body_name << "' or its descendants." << std::endl;
  }
  else
  {
    std::cout << "[GeomTfPublisher] Found " << geom_ids.size() << " geoms in " 
              << body_ids.size() << " bodies under '" << parent_body_name << "'." << std::endl;
  }

  // frame_id
  const char * frame_id_char = mj_getPluginConfig(m, plugin_id, ATTR_FRAME_ID);
  std::string frame_id = "map";
  if(frame_id_char && strlen(frame_id_char) > 0)
  {
    frame_id = std::string(frame_id_char);
  }

  // topic_name
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, ATTR_TOPIC_NAME);
  std::string topic_name = "/tf";
  if(topic_name_char && strlen(topic_name_char) > 0)
  {
    topic_name = std::string(topic_name_char);
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
    mju_error("[GeomTfPublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // publish_bodies
  const char * publish_bodies_char = mj_getPluginConfig(m, plugin_id, ATTR_PUBLISH_BODIES);
  bool publish_bodies = false;
  if(publish_bodies_char && strlen(publish_bodies_char) > 0)
  {
    if(!(strcmp(publish_bodies_char, "true") == 0 || strcmp(publish_bodies_char, "false") == 0))
    {
      mju_error("[GeomTfPublisher] `publish_bodies` must be `true` or `false`.");
      return nullptr;
    }
    publish_bodies = (strcmp(publish_bodies_char, "true") == 0);
  }

  std::cout << "[GeomTfPublisher] Created for parent body '" << parent_body_name 
            << "' with " << geom_ids.size() << " geoms." << std::endl;

  return new GeomTfPublisher(m, d, parent_body_id, body_ids, geom_ids, frame_id, topic_name, publish_rate, publish_bodies);
}

GeomTfPublisher::GeomTfPublisher(const mjModel * m,
                                 mjData *, // d
                                 int parent_body_id,
                                 const std::vector<int> & body_ids,
                                 const std::vector<int> & geom_ids,
                                 const std::string & frame_id,
                                 const std::string & topic_name,
                                 mjtNum publish_rate,
                                 bool publish_bodies)
: parent_body_id_(parent_body_id), body_ids_(body_ids), geom_ids_(geom_ids), frame_id_(frame_id), 
  topic_name_(topic_name), publish_bodies_(publish_bodies),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
  // Initialize ROS node
  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  nh_ = rclcpp::Node::make_shared("geom_tf_publisher_" + std::to_string(parent_body_id_));

  // Initialize TF publisher with custom topic
  tf_pub_ = nh_->create_publisher<tf2_msgs::msg::TFMessage>(topic_name_, 10);

  std::cout << "[GeomTfPublisher] Publishing TF to topic: " << topic_name_ << std::endl;
}

void GeomTfPublisher::reset(const mjModel *, // m
                            int // plugin_id
)
{
  sim_cnt_ = 0;
}

void GeomTfPublisher::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
  if(!rclcpp::ok())
  {
    return;
  }

  sim_cnt_++;

  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  // Get current timestamp
  rclcpp::Time stamp_now = nh_->get_clock()->now();

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // Publish TF for bodies if enabled
  if(publish_bodies_)
  {
    for(int body_id : body_ids_)
    {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = stamp_now;
      transform_msg.header.frame_id = frame_id_;
      
      // Get body name or use ID
      const char * body_name = mj_id2name(m, mjOBJ_BODY, body_id);
      if(body_name && strlen(body_name) > 0)
      {
        transform_msg.child_frame_id = std::string(body_name);
      }
      else
      {
        transform_msg.child_frame_id = "body_" + std::to_string(body_id);
      }

      // Get body pose in world frame
      transform_msg.transform.translation.x = d->xpos[3 * body_id + 0];
      transform_msg.transform.translation.y = d->xpos[3 * body_id + 1];
      transform_msg.transform.translation.z = d->xpos[3 * body_id + 2];
      transform_msg.transform.rotation.w = d->xquat[4 * body_id + 0];
      transform_msg.transform.rotation.x = d->xquat[4 * body_id + 1];
      transform_msg.transform.rotation.y = d->xquat[4 * body_id + 2];
      transform_msg.transform.rotation.z = d->xquat[4 * body_id + 3];
      
      transforms.push_back(transform_msg);
    }
  }

  // Publish TF for each geom
  
  for(int geom_id : geom_ids_)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp_now;
    transform_msg.header.frame_id = frame_id_;
    
    // Get geom name or use ID
    const char * geom_name = mj_id2name(m, mjOBJ_GEOM, geom_id);
    if(geom_name && strlen(geom_name) > 0)
    {
      transform_msg.child_frame_id = std::string(geom_name);
    }
    else
    {
      transform_msg.child_frame_id = "geom_" + std::to_string(geom_id);
    }

    // Get geom pose in world frame
    mjtNum geom_pos[3];
    mjtNum geom_quat[4];
    
    // For geoms, we need to get the body pose and then apply geom offset
    int body_id = m->geom_bodyid[geom_id];
    
    // Get body transformation
    mjtNum body_pos[3] = {d->xpos[3 * body_id + 0], d->xpos[3 * body_id + 1], d->xpos[3 * body_id + 2]};
    mjtNum body_quat[4] = {d->xquat[4 * body_id + 0], d->xquat[4 * body_id + 1], 
                           d->xquat[4 * body_id + 2], d->xquat[4 * body_id + 3]};
    
    // Get geom offset from body
    int geom_data_id = m->geom_dataid[geom_id];
    mjtNum * geom_pos_offset = m->geom_pos + 3 * geom_id;
    mjtNum * geom_quat_offset = m->geom_quat + 4 * geom_id;
    
    // Compute geom pose in world frame: T_world_geom = T_world_body * T_body_geom
    tf2::Vector3 body_pos_tf(body_pos[0], body_pos[1], body_pos[2]);
    tf2::Quaternion body_quat_tf(body_quat[1], body_quat[2], body_quat[3], body_quat[0]); // xyzw
    tf2::Transform T_world_body(body_quat_tf, body_pos_tf);
    
    tf2::Vector3 geom_pos_offset_tf(geom_pos_offset[0], geom_pos_offset[1], geom_pos_offset[2]);
    tf2::Quaternion geom_quat_offset_tf(geom_quat_offset[1], geom_quat_offset[2], 
                                        geom_quat_offset[3], geom_quat_offset[0]); // xyzw
    tf2::Transform T_body_geom(geom_quat_offset_tf, geom_pos_offset_tf);
    
    tf2::Transform T_world_geom = T_world_body * T_body_geom;
    
    tf2::Vector3 final_pos = T_world_geom.getOrigin();
    tf2::Quaternion final_quat = T_world_geom.getRotation();
    
    transform_msg.transform.translation.x = final_pos.x();
    transform_msg.transform.translation.y = final_pos.y();
    transform_msg.transform.translation.z = final_pos.z();
    transform_msg.transform.rotation.w = final_quat.w();
    transform_msg.transform.rotation.x = final_quat.x();
    transform_msg.transform.rotation.y = final_quat.y();
    transform_msg.transform.rotation.z = final_quat.z();
    
    transforms.push_back(transform_msg);
  }

  // Publish all transforms as a TFMessage
  if(!transforms.empty())
  {
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms = transforms;
    tf_pub_->publish(tf_msg);
  }

  // Spin ROS node
  rclcpp::spin_some(nh_);
}

} // namespace MujocoRosUtils
