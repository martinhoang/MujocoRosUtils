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
  contact_array_pub_ =
    nh_->create_publisher<mujoco_ros_utils::msg::ContactInfoArray>(topic_name_ + "_array", 10);

  std::cout << "[ContactForcePublisher] Initialized:" << std::endl;
  std::cout << "  Topic: " << topic_name_ << std::endl;
  std::cout << "  Array Topic: " << topic_name_ + "_array" << std::endl;
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
  if (sim_cnt_ % publish_skip_ != 0)
  {
    sim_cnt_++;
    return;
  }
  sim_cnt_++;

  mujoco_ros_utils::msg::ContactInfoArray array_msg;
  array_msg.header.frame_id = frame_id_;
  array_msg.header.stamp    = nh_->now();

  // Iterate through all contacts
  for (int cnt_id = 0; cnt_id < d->ncon; cnt_id++)
  {
    mjContact &contact = d->contact[cnt_id];

    // Get geom IDs
    int geom1_id = contact.geom[0];
    int geom2_id = contact.geom[1];

    // Determine if contacts involve flexes
    bool is_flex1 = (geom1_id < 0 && contact.flex[0] >= 0);
    bool is_flex2 = (geom2_id < 0 && contact.flex[1] >= 0);

    // Filter by geom names if specified
    if (!geom_ids_.empty())
    {
      // For flex contacts, we need to check the flex name against our filter list
      // For geom contacts, we check the geom_id
      bool match1 = false;
      bool match2 = false;

      if (is_flex1)
      {
        // Check if flex name is in our filter list
        const char *flex_name = mj_id2name(m, mjOBJ_FLEX, contact.flex[0]);
        if (flex_name && geom_names_.find(std::string(flex_name)) != geom_names_.end())
        {
          match1 = true;
        }
      }
      else if (geom1_id >= 0)
      {
        // Check if geom is in our filter list
        match1 = (geom_ids_.find(geom1_id) != geom_ids_.end());
      }

      if (is_flex2)
      {
        // Check if flex name is in our filter list
        const char *flex_name = mj_id2name(m, mjOBJ_FLEX, contact.flex[1]);
        if (flex_name && geom_names_.find(std::string(flex_name)) != geom_names_.end())
        {
          match2 = true;
        }
      }
      else if (geom2_id >= 0)
      {
        // Check if geom is in our filter list
        match2 = (geom_ids_.find(geom2_id) != geom_ids_.end());
      }

      // Skip if neither contact matches our filter
      if (!match1 && !match2)
      {
        continue;
      }
    }

    // Get names for both contacts (handle geom, body, or flex)
    std::string name1, name2;

    // For contact 1
    if (geom1_id >= 0)
    {
      // Regular geom contact
      const char *geom_name = mj_id2name(m, mjOBJ_GEOM, geom1_id);
      if (geom_name)
      {
        name1 = std::string(geom_name);
      }
      else
      {
        // Try body if geom name not found
        const char *body_name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[geom1_id]);
        name1                 = body_name ? std::string(body_name) : "unknown";
      }
    }
    else if (contact.flex[0] >= 0)
    {
      // 1. Get the general Flex Name (e.g., "FC_pelvis")
      const char *flex_name = mj_id2name(m, mjOBJ_FLEX, contact.flex[0]);
      std::string f_name    = flex_name ? std::string(flex_name) : "flex";

      int body_id = -1;
      int flex_id = contact.flex[0];

      // CASE A: The contact hit a specific vertex directly
      if (contact.vert[0] >= 0)
      {
        int global_vert_id = m->flex_vertadr[flex_id] + contact.vert[0];
        body_id            = m->flex_vertbodyid[global_vert_id];
        // Details logged only on debug request/error, skipping spam here
      }
      // CASE B: The contact hit an element (volume/face)
      // This handles the "FC_pelvis_elem..." case
      else if (contact.elem[0] >= 0)
      {
        // ---------------------------------------------------------
        // IMPORTANT: Define how many vertices make up 1 element.
        // For a 3D grid/box (composite), this is usually 8 (Hexahedron).
        // For a mesh/volume, this might be 4 (Tetrahedron).
        // ---------------------------------------------------------
        int stride = 8;

        // 1. Find the start of the element data for this flex
        int elem_start = m->flex_elemadr[flex_id];

        // 2. Look up the first vertex of this specific element
        //    We multiply elem ID by stride to find its place in the array.
        int vertex_lookup_idx = elem_start + (contact.elem[0] * stride);

        // 3. Get the Local Vertex ID
        int local_vert_id = m->flex_elem[vertex_lookup_idx];

        // 4. Convert to Global Vertex ID
        int global_vert_id = m->flex_vertadr[flex_id] + local_vert_id;

        // 5. Finally, get the Body ID attached to this vertex
        body_id = m->flex_vertbodyid[global_vert_id];
      }

      // Resolve the Body ID to a Name
      if (body_id >= 0)
      {
        const char *bname = mj_id2name(m, mjOBJ_BODY, body_id);
        name1             = bname ? std::string(bname) : f_name;
      }
      else
      {
        // Fallback if no body is found attached to the flex
        name1 = f_name + (contact.elem[0] >= 0 ? "_elem" : "_vert");
      }
    }
    else
    {
      name1 = "unknown";
    }

    // For contact 2
    if (geom2_id >= 0)
    {
      // Regular geom contact
      const char *geom_name = mj_id2name(m, mjOBJ_GEOM, geom2_id);
      if (geom_name)
      {
        name2 = std::string(geom_name);
      }
      else
      {
        // Try body if geom name not found
        const char *body_name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[geom2_id]);
        name2                 = body_name ? std::string(body_name) : "unknown";
      }
    }
    else if (contact.flex[1] >= 0)
    {
      // 1. Get the general Flex Name (e.g., "FC_pelvis")
      const char *flex_name = mj_id2name(m, mjOBJ_FLEX, contact.flex[1]);
      std::string f_name    = flex_name ? std::string(flex_name) : "flex";

      int body_id = -1;
      int flex_id = contact.flex[1];

      // CASE A: The contact hit a specific vertex directly
      if (contact.vert[1] >= 0)
      {
        int global_vert_id = m->flex_vertadr[flex_id] + contact.vert[1];
        body_id            = m->flex_vertbodyid[global_vert_id];
      }
      // CASE B: The contact hit an element (volume/face)
      // This handles the "FC_pelvis_elem..." case
      else if (contact.elem[1] >= 0)
      {
        // ---------------------------------------------------------
        // IMPORTANT: Define how many vertices make up 1 element.
        // For a 3D grid/box (composite), this is usually 8 (Hexahedron).
        // For a mesh/volume, this might be 4 (Tetrahedron).
        // ---------------------------------------------------------
        int stride = 8;

        // 1. Find the start of the element data for this flex
        int elem_start = m->flex_elemadr[flex_id];

        // 2. Look up the first vertex of this specific element
        //    We multiply elem ID by stride to find its place in the array.
        int vertex_lookup_idx = elem_start + (contact.elem[1] * stride);

        // 3. Get the Local Vertex ID
        int local_vert_id = m->flex_elem[vertex_lookup_idx];

        // 4. Convert to Global Vertex ID
        int global_vert_id = m->flex_vertadr[flex_id] + local_vert_id;

        // 5. Finally, get the Body ID attached to this vertex
        body_id = m->flex_vertbodyid[global_vert_id];
      }

      // Resolve the Body ID to a Name
      if (body_id >= 0)
      {
        const char *bname = mj_id2name(m, mjOBJ_BODY, body_id);
        name2             = bname ? std::string(bname) : f_name;
      }
      else
      {
        // Fallback if no body is found attached to the flex
        name2 = f_name + (contact.elem[1] >= 0 ? "_elem" : "_vert");
      }
    }
    else
    {
      name2 = "unknown";
    }

    // 1. Get the forces in the Local Contact Frame
    // [0]=normal, [1]=tangent1, [2]=tangent2, [3-5]=torsional/rolling
    mjtNum local_force[6] = {0, 0, 0, 0, 0, 0};
    mj_contactForce(m, d, cnt_id, local_force);

    // 2. Extract Scalar Magnitudes
    mujoco_ros_utils::msg::ContactInfo msg;
    msg.normal_force = local_force[0];
    msg.friction_force
      = std::sqrt(local_force[1] * local_force[1] + local_force[2] * local_force[2]);

    // 3. Compute World Frame Force Vector (Rotation)
    // We need to multiply the Contact Frame Matrix (3x3) by the Force Vector (3x1)
    // contact.frame is stored as: [ N_x N_y N_z | T1_x T1_y T1_z | T2_x T2_y T2_z ]

    mjtNum world_force[3] = {0, 0, 0};

    // Matrix-Vector Multiplication: World_F = Frame_Matrix * Local_F
    for (int i = 0; i < 3; i++)
    {
      // frame[i]     is Normal(i)
      // frame[i+3]   is Tangent1(i)
      // frame[i+6]   is Tangent2(i)

      world_force[i] = (local_force[0] * contact.frame[i]) +     // Normal Component
                       (local_force[1] * contact.frame[i + 3]) + // Tangent 1 Component
                       (local_force[2] * contact.frame[i + 6]);  // Tangent 2 Component
    }

    // 4. Assign to ROS message
    // Assuming msg.force is an array of size 3 (or geometry_msgs/Vector3) representing World X,Y,Z
    msg.force[0] = world_force[0];
    msg.force[1] = world_force[1];
    msg.force[2] = world_force[2];

    // Note: If msg.force is actually size 6 (wrench), indices 3-5 are torques.
    // You would perform a similar rotation for torques using local_force[3], [4], [5].

    // Fill remaining message fields
    msg.id    = cnt_id;
    msg.geom1 = name1;
    msg.geom2 = name2;
    msg.dist  = contact.dist;

    msg.pos.x = contact.pos[0];
    msg.pos.y = contact.pos[1];
    msg.pos.z = contact.pos[2];

    for (int i = 0; i < 9; i++)
    {
      msg.frame[i] = contact.frame[i];
    }

    contact_pub_->publish(msg);
    array_msg.contacts.push_back(msg);
  }

  // Publish the array message
  if (!array_msg.contacts.empty())
  {
    contact_array_pub_->publish(array_msg);
  }

  // Spin ROS
  rclcpp::spin_some(nh_);
}

} // namespace MujocoRosUtils
