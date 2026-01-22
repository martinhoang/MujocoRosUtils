#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>
#include <vector>

namespace MujocoRosUtils
{

/** \brief Plugin to publish TF frames of geoms under a specific parent body to a custom topic. */
class GeomTfPublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static GeomTfPublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  GeomTfPublisher(GeomTfPublisher &&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param parent_body_id parent body ID
      \param body_ids vector of body IDs in the hierarchy
      \param geom_ids vector of geom IDs under the parent body
      \param frame_id frame ID for TF parent
      \param topic_name custom topic name for TF messages
      \param publish_rate publish rate
      \param publish_bodies whether to publish body frames
  */
  GeomTfPublisher(const mjModel * m,
                  mjData * d,
                  int parent_body_id,
                  const std::vector<int> & body_ids,
                  const std::vector<int> & geom_ids,
                  const std::string & frame_id,
                  const std::string & topic_name,
                  mjtNum publish_rate,
                  bool publish_bodies);

protected:
  //! Parent body ID
  int parent_body_id_ = -1;

  //! Vector of body IDs in hierarchy
  std::vector<int> body_ids_;

  //! Vector of geom IDs
  std::vector<int> geom_ids_;

  //! ROS node handle
  rclcpp::Node::SharedPtr nh_;

  //! TF publisher
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  //! Frame ID for TF parent
  std::string frame_id_;

  //! Custom topic name
  std::string topic_name_;

  //! Whether to publish body frames
  bool publish_bodies_ = false;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
