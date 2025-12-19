#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mujoco_ros_utils/msg/contact_info.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>
#include <vector>
#include <set>

namespace MujocoRosUtils
{

/** \brief Plugin to publish contact force information for specific geoms. */
class ContactForcePublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ContactForcePublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ContactForcePublisher(ContactForcePublisher &&) = default;

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
      \param geom_names comma-separated list of geom names to monitor
      \param frame_id frame ID for message header
      \param topic_name topic name for publishing contact info
      \param publish_rate publish rate in Hz
  */
  ContactForcePublisher(const mjModel * m,
                        mjData * d,
                        const std::vector<std::string> & geom_names,
                        const std::string & frame_id,
                        const std::string & topic_name,
                        mjtNum publish_rate);

protected:
  //! Set of geom names to monitor (empty = monitor all)
  std::set<std::string> geom_names_;

  //! Set of geom IDs to monitor (empty = monitor all)
  std::set<int> geom_ids_;

  //! ROS node handle
  rclcpp::Node::SharedPtr nh_;

  //! ROS publisher for contact info
  rclcpp::Publisher<mujoco_ros_utils::msg::ContactInfo>::SharedPtr contact_pub_;

  //! Frame ID for message header
  std::string frame_id_;

  //! Topic name
  std::string topic_name_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
