#include "ContactForcePublisher.h"

#include <geometry_msgs/msg/point.hpp>
#include <mujoco/mujoco.h>

#include <cmath>
#include <iostream>
#include <sstream>

/*
Example usage:
		<plugin plugin='MujocoRosUtils::ContactForcePublisher'>
			<instance name='contact'>
				<config key='geom_names' value=''/>
				<config key='frame_id' value='world'/>
				<config key='topic_name' value='/contact_forces/all'/>
				<config key='publish_rate' value='100'/>
			</instance>
		</plugin>
*/
namespace MujocoRosUtils
{

void ContactForcePublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::ContactForcePublisher";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char *attributes[] = {"geom_names", "frame_id", "topic_name", "publish_rate"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int              // plugin_id
                   ) {
    return 0;
  };

  plugin.nsensordata = +[](const mjModel *, // m
                           int,             // plugin_id
                           int              // sensor_id
                        ) {
    return 0;
  };

  // Can only run after forces have been computed
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel *m, mjData *d, int plugin_id) {
    auto *plugin_instance = ContactForcePublisher::Create(m, d, plugin_id);
    if (!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData *d, int plugin_id) {
    delete reinterpret_cast<ContactForcePublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel *m, double *, // plugin_state
                     void *plugin_data, int plugin_id) {
    auto *plugin_instance = reinterpret_cast<class ContactForcePublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel *m, mjData *d, int plugin_id, int // capability_bit
                    ) {
    auto *plugin_instance
      = reinterpret_cast<class ContactForcePublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
  std::cout << "[ContactForcePublisher] Successfully registered plugin" << std::endl;
}

// Helper function to parse comma-separated geom names
static std::vector<std::string> ParseGeomNames(const std::string &input)
{
  std::vector<std::string> result;
  std::stringstream        ss(input);
  std::string              item;

  while (std::getline(ss, item, ','))
  {
    // Trim whitespace
    size_t start = item.find_first_not_of(" \t");
    size_t end   = item.find_last_not_of(" \t");
    if (start != std::string::npos && end != std::string::npos)
    {
      result.push_back(item.substr(start, end - start + 1));
    }
  }
  return result;
}

ContactForcePublisher *ContactForcePublisher::Create(const mjModel *m, mjData *d, int plugin_id)
{
  std::cout << "[ContactForcePublisher] Creating plugin instance" << std::endl;

  // Get geom names (optional - if empty, monitor all contacts)
  const char              *geom_names_char = mj_getPluginConfig(m, plugin_id, "geom_names");
  std::vector<std::string> geom_names;

  if (geom_names_char && strlen(geom_names_char) > 0)
  {
    std::cout << "[ContactForcePublisher] Parsing geom_names: " << geom_names_char << std::endl;
    geom_names = ParseGeomNames(std::string(geom_names_char));
    // Empty result is OK - means we'll parse it but got empty string
  }

  // Get frame ID
  const char *frame_id_char = mj_getPluginConfig(m, plugin_id, "frame_id");
  std::string frame_id
    = (frame_id_char && strlen(frame_id_char) > 0) ? std::string(frame_id_char) : "world";

  // Get topic name
  const char *topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  if (!topic_name_char || strlen(topic_name_char) == 0)
  {
    std::cerr << "[ContactForcePublisher] topic_name is not specified." << std::endl;
    return nullptr;
  }
  std::string topic_name(topic_name_char);

  // Get publish rate
  const char *publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum      publish_rate      = 100.0; // Default to 100 Hz instead of 0
  if (publish_rate_char && strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }

  if (publish_rate <= 0)
  {
    std::cerr << "[ContactForcePublisher] publish_rate must be positive, using default 100 Hz."
              << std::endl;
    publish_rate = 100.0;
  }

  std::cout << "[ContactForcePublisher] Create." << std::endl;

  return new ContactForcePublisher(m, d, geom_names, frame_id, topic_name, publish_rate);
}

ContactForcePublisher::ContactForcePublisher(const mjModel *m,
                                             mjData *, // d
                                             const std::vector<std::string> &geom_names,
                                             const std::string              &frame_id,
                                             const std::string &topic_name, mjtNum publish_rate)
    : frame_id_(frame_id)
    , topic_name_(topic_name)
    , publish_skip_(1)
    , sim_cnt_(0)
{
  std::cout << "[ContactForcePublisher] Initializing plugin" << std::endl;

  // Store geom names and resolve their IDs
  for (const auto &name : geom_names)
  {
    geom_names_.insert(name);
    int geom_id = mj_name2id(m, mjOBJ_GEOM, name.c_str());
    if (geom_id >= 0)
    {
      geom_ids_.insert(geom_id);
    }
    else
    {
      std::cerr << "[ContactForcePublisher] Warning: geom '" << name << "' not found in model."
                << std::endl;
    }
  }

  // Calculate publish skip
  if (publish_rate > 0)
  {
    publish_skip_ = static_cast<int>(std::round(1.0 / (publish_rate * m->opt.timestep)));
  }
  if (publish_skip_ < 1)
  {
    publish_skip_ = 1;
  }

  // Initialize ROS node and publisher
  int    argc = 0;
  char **argv = nullptr;
  if (!rclcpp::ok())
  {
    rclcpp::init(argc, argv);
  }

  rclcpp::NodeOptions node_options;
  nh_          = rclcpp::Node::make_shared("_contact_force_publisher", node_options);
  contact_pub_ = nh_->create_publisher<mujoco_ros_utils::msg::ContactInfo>(topic_name_, 10);

  std::cout << "[ContactForcePublisher] Initialized:" << std::endl;
  std::cout << "  Topic: " << topic_name_ << std::endl;
  std::cout << "  Frame ID: " << frame_id_ << std::endl;
  std::cout << "  Publish rate: " << publish_rate << " Hz" << std::endl;
  if (geom_names_.empty())
  {
    std::cout << "  Monitoring: all geoms" << std::endl;
  }
  else
  {
    std::cout << "  Monitoring geoms: ";
    for (const auto &name : geom_names_)
    {
      std::cout << name << " ";
    }
    std::cout << std::endl;
  }
}

void ContactForcePublisher::reset(const mjModel *m, int plugin_id)
{
  sim_cnt_ = 0;
}

void ContactForcePublisher::compute(const mjModel *m, mjData *d, int plugin_id)
{
    // Check if we should publish this iteration
    if(sim_cnt_ % publish_skip_ != 0)
    {
      sim_cnt_++;
      return;
    }
    sim_cnt_++;

    // Iterate through all contacts
    for(int cnt_id = 0; cnt_id < d->ncon; cnt_id++)
    {
      mjContact & contact = d->contact[cnt_id];

      // Get geom IDs
      int geom1_id = contact.geom1;
      int geom2_id = contact.geom2;

      // Filter by geom names if specified
      if(!geom_ids_.empty())
      {
        // Skip if neither geom is in our list of interest
        if(geom_ids_.find(geom1_id) == geom_ids_.end() &&
           geom_ids_.find(geom2_id) == geom_ids_.end())
        {
          continue;
        }
      }

      // Get geom names
      const char * geom1_name = mj_id2name(m, mjOBJ_GEOM, geom1_id);
      const char * geom2_name = mj_id2name(m, mjOBJ_GEOM, geom2_id);

      // Calculate contact force
      mjtNum contact_force[6] = {0, 0, 0, 0, 0, 0};
      mj_contactForce(m, d, cnt_id, contact_force);

      // Create and populate message
      mujoco_ros_utils::msg::ContactInfo msg;
      msg.id = cnt_id;
      msg.geom1 = geom1_name ? std::string(geom1_name) : "unknown";
      msg.geom2 = geom2_name ? std::string(geom2_name) : "unknown";

      // Contact position
      msg.pos.x = contact.pos[0];
      msg.pos.y = contact.pos[1];
      msg.pos.z = contact.pos[2];

      // Contact frame (3x3 matrix stored row-wise)
      for(int i = 0; i < 9; i++)
      {
        msg.frame[i] = contact.frame[i];
      }

      // Distance
      msg.dist = contact.dist;

      // Contact force (6D)
      for(int i = 0; i < 6; i++)
      {
        msg.force[i] = contact_force[i];
      }

      // Normal force (first component)
      msg.normal_force = contact_force[0];

      // Friction force (magnitude of tangential components)
      msg.friction_force = std::sqrt(contact_force[1] * contact_force[1] +
                                     contact_force[2] * contact_force[2]);

      // Publish
      contact_pub_->publish(msg);
    }

    // Spin ROS
    rclcpp::spin_some(nh_);
}

} // namespace MujocoRosUtils
