#include "SensorPublisher.h"

#include <mujoco/mujoco.h>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <iostream>
#include <sstream>
#include <vector>

/*
Example usage:

Single sensor:
  <plugin plugin="MujocoRosUtils::SensorPublisher">
    <config key="sensor_name" value="gripper_upper_right_curl_1_end_site" />
    <config key="topic_name" value="/contact_force_ur_body" />
    <config key="publish_rate" value="100" />
  </plugin>

Multiple sensors (published as Float32MultiArray):
  <plugin plugin="MujocoRosUtils::SensorPublisher">
    <config key="sensor_names" value="gripper_upper_right_curl_1_end_site, gripper_upper_right_curl_2_end_site, gripper_upper_right_curl_3_end_site, gripper_upper_right_curl_4_end_site, gripper_upper_right_curl_5_end_site" />
    <config key="topic_name" value="/contact_force_array" />
    <config key="publish_rate" value="100" />
  </plugin>
*/

namespace MujocoRosUtils
{

void SensorPublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::SensorPublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {"sensor_name", "sensor_names", "frame_id", "topic_name", "publish_rate"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  // Can only run after forces have been computed
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = SensorPublisher::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<SensorPublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class SensorPublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class SensorPublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

// Helper function to parse comma-separated sensor names
static std::vector<std::string> ParseSensorNames(const std::string& input)
{
  std::vector<std::string> result;
  std::stringstream ss(input);
  std::string item;
  
  while (std::getline(ss, item, ','))
  {
    // Trim whitespace
    size_t start = item.find_first_not_of(" \t");
    size_t end = item.find_last_not_of(" \t");
    if (start != std::string::npos && end != std::string::npos)
    {
      result.push_back(item.substr(start, end - start + 1));
    }
  }
  return result;
}

SensorPublisher * SensorPublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // Check for multiple sensors first (sensor_names attribute)
  const char * sensor_names_char = mj_getPluginConfig(m, plugin_id, "sensor_names");
  std::vector<int> sensor_ids;
  MessageType msg_type = MsgScalar;
  
  if(strlen(sensor_names_char) > 0)
  {
    // Parse multiple sensor names
    std::vector<std::string> sensor_name_list = ParseSensorNames(std::string(sensor_names_char));
    
    if(sensor_name_list.empty())
    {
      mju_error("[SensorPublisher] `sensor_names` is empty after parsing.");
      return nullptr;
    }
    
    // Find all sensor IDs
    for(const auto& sensor_name_str : sensor_name_list)
    {
      int sid = mj_name2id(m, mjOBJ_SENSOR, sensor_name_str.c_str());
      if(sid < 0)
      {
        mju_error("[SensorPublisher] Sensor '%s' not found.", sensor_name_str.c_str());
        return nullptr;
      }
      sensor_ids.push_back(sid);
    }
    
    msg_type = MsgFloatArray;
  }
  else
  {
    // Fall back to single sensor (existing behavior)
    const char * sensor_name_char = mj_getPluginConfig(m, plugin_id, "sensor_name");
    if(strlen(sensor_name_char) == 0)
    {
      mju_error("[SensorPublisher] Either `sensor_name` or `sensor_names` must be provided.");
      return nullptr;
    }
    
    int sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_name_char);
    if(sensor_id < 0)
    {
      mju_error("[SensorPublisher] Sensor '%s' not found.", sensor_name_char);
      return nullptr;
    }
    
    sensor_ids.push_back(sensor_id);
    
    // Determine message type based on dimension (existing logic)
    int sensor_dim = m->sensor_dim[sensor_id];
    if(sensor_dim == 1)
    {
      msg_type = MsgScalar;
    }
    else if(sensor_dim == 3)
    {
      if(m->sensor_type[sensor_id] == mjSENS_FRAMEPOS)
      {
        msg_type = MsgPoint;
      }
      else
      {
        msg_type = MsgVector3;
      }
    }
    else if(sensor_dim == 4)
    {
      msg_type = MsgQuaternion;
    }
    else
    {
      mju_error("[SensorPublisher] Unsupported sensor dimension: %d.", sensor_dim);
      return nullptr;
    }
  }

  // frame_id
  const char * frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  std::string frame_id = strlen(frame_id_char) > 0 ? std::string(frame_id_char) : "";

  // topic_name
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = strlen(topic_name_char) > 0 ? std::string(topic_name_char) : "";

  // publish_rate
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum publish_rate = 30.0;
  if(strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if(publish_rate <= 0)
  {
    mju_error("[SensorPublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  std::cout << "[SensorPublisher] Create with " << sensor_ids.size() << " sensor(s)." << std::endl;

  return new SensorPublisher(m, d, sensor_ids, msg_type, frame_id, topic_name, publish_rate);
}

SensorPublisher::SensorPublisher(const mjModel * m,
                                 mjData *, // d
                                 const std::vector<int> & sensor_ids,
                                 MessageType msg_type,
                                 const std::string & frame_id,
                                 const std::string & topic_name,
                                 mjtNum publish_rate)
: sensor_ids_(sensor_ids), sensor_id_(sensor_ids[0]), msg_type_(msg_type), frame_id_(frame_id), 
  topic_name_(topic_name),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
  if(frame_id_.empty())
  {
    frame_id_ = "map";
  }

  std::string node_name;
  if(topic_name_.empty())
  {
    if(sensor_ids_.size() == 1)
    {
      std::string sensor_name = std::string(mj_id2name(m, mjOBJ_SENSOR, sensor_ids_[0]));
      topic_name_ = "mujoco/" + sensor_name;
      node_name = sensor_name + "_sensor_publisher";
    }
    else
    {
      topic_name_ = "mujoco/sensor_array";
      node_name = "sensor_array_publisher";
    }
  }
  else
  {
    node_name = topic_name_ + "_publisher";
    // Remove leading slash for node name
    if(node_name[0] == '/')
    {
      node_name = node_name.substr(1);
    }
    // Replace slashes with underscores
    std::replace(node_name.begin(), node_name.end(), '/', '_');
  }

  int argc = 0;
  char ** argv = nullptr;
  if(!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
      {"use_sim_time", true},
  });

  nh_ = rclcpp::Node::make_shared(node_name, node_options);
  
  if(msg_type_ == MsgFloatArray)
  {
    pub_ = nh_->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name_, 1);
  }
  else if(msg_type_ == MsgScalar)
  {
    pub_ = nh_->create_publisher<mujoco_ros_utils::msg::ScalarStamped>(topic_name_, 1);
  }
  else if(msg_type_ == MsgPoint)
  {
    pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(topic_name_, 1);
  }
  else if(msg_type_ == MsgVector3)
  {
    pub_ = nh_->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_name_, 1);
  }
  else // if(msg_type_ == MsgQuaternion)
  {
    pub_ = nh_->create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_name_, 1);
  }
}

SensorPublisher::~SensorPublisher()
{
  // Clean up publisher and node to prevent memory leaks
  try
  {
    if (pub_)
    {
      pub_.reset();
    }
    if (nh_)
    {
      nh_.reset();
    }
  }
  catch (const std::exception &e)
  {
    // Suppress exceptions during destruction
    std::cerr << "[SensorPublisher] Exception during destruction: " << e.what() << std::endl;
  }
}

void SensorPublisher::reset(const mjModel *, // m
                            int // plugin_id
)
{
}

void SensorPublisher::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  if(msg_type_ == MsgFloatArray)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "sensors";
    msg.layout.dim[0].size = sensor_ids_.size();
    msg.layout.dim[0].stride = sensor_ids_.size();
    
    for(int sid : sensor_ids_)
    {
      int sensor_adr = m->sensor_adr[sid];
      int sensor_dim = m->sensor_dim[sid];
      
      for(int i = 0; i < sensor_dim; i++)
      {
        msg.data.push_back(static_cast<float>(d->sensordata[sensor_adr + i]));
      }
    }
    
    std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub_)->publish(msg);
  }
  else
  {
    // Existing single-sensor logic
    std_msgs::msg::Header header;
    header.stamp = nh_->get_clock()->now();
    header.frame_id = frame_id_;

    int sensor_adr = m->sensor_adr[sensor_id_];
    if(msg_type_ == MsgScalar)
  {
    mujoco_ros_utils::msg::ScalarStamped msg;
    msg.header = header;
    msg.value.data = d->sensordata[sensor_adr];
    std::dynamic_pointer_cast<rclcpp::Publisher<mujoco_ros_utils::msg::ScalarStamped>>(pub_)->publish(msg);
  }
  else if(msg_type_ == MsgPoint)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header = header;
    msg.point.x = d->sensordata[sensor_adr + 0];
    msg.point.y = d->sensordata[sensor_adr + 1];
    msg.point.z = d->sensordata[sensor_adr + 2];
    std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::PointStamped>>(pub_)->publish(msg);
  }
  else if(msg_type_ == MsgVector3)
  {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header = header;
    msg.vector.x = d->sensordata[sensor_adr + 0];
    msg.vector.y = d->sensordata[sensor_adr + 1];
    msg.vector.z = d->sensordata[sensor_adr + 2];
    std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>>(pub_)->publish(msg);
  }
    else // if(msg_type_ == MsgQuaternion)
    {
      geometry_msgs::msg::QuaternionStamped msg;
      msg.header = header;
      msg.quaternion.w = d->sensordata[sensor_adr + 0];
      msg.quaternion.x = d->sensordata[sensor_adr + 1];
      msg.quaternion.y = d->sensordata[sensor_adr + 2];
      msg.quaternion.z = d->sensordata[sensor_adr + 3];
      std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>>(pub_)->publish(msg);
    }
  }
}

} // namespace MujocoRosUtils
