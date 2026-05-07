#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include <array>
#include <random>

namespace MujocoRosUtils
{

/** \brief Plugin to randomize the pose of a body at every simulation reset.
 *
 *  Attach as a passive plugin on any body that owns a freejoint.
 *  Random offsets are drawn from uniform distributions over the configured
 *  per-axis ranges and applied relative to the body's nominal pose in
 *  `m->qpos0`.
 *
 *  XML usage:
 *  \code{.xml}
 *  <body name="object" pos="0 0 0.4">
 *    <freejoint/>
 *    <plugin plugin="MujocoRosUtils::PoseRandomizer">
 *      <!-- each range is "min max" in metres (xyz) or radians (rpy) -->
 *      <config key="x_range"     value="-0.05 0.05"/>
 *      <config key="y_range"     value="-0.05 0.05"/>
 *      <config key="z_range"     value="0.0   0.0"/>
 *      <config key="roll_range"  value="0.0   0.0"/>
 *      <config key="pitch_range" value="0.0   0.0"/>
 *      <config key="yaw_range"   value="-0.3  0.3"/>
 *    </plugin>
 *  </body>
 *  \endcode
 */
class PoseRandomizer
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
  */
  static PoseRandomizer * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  PoseRandomizer(PoseRandomizer &&) = default;

  /** \brief Called on simulation reset — draws new random offsets.
      \param m model
      \param plugin_id plugin ID
  */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Called every step — applies the pending randomized pose on the
      first call after each reset.
      \param m model
      \param d data
      \param plugin_id plugin ID
  */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  PoseRandomizer(int body_id,
                 int freejoint_qposadr,
                 const std::array<double, 3> & nominal_pos,
                 const std::array<double, 4> & nominal_quat,
                 const std::array<double, 2> & x_range,
                 const std::array<double, 2> & y_range,
                 const std::array<double, 2> & z_range,
                 const std::array<double, 2> & roll_range,
                 const std::array<double, 2> & pitch_range,
                 const std::array<double, 2> & yaw_range);

private:
  /** Draw new random offsets into rand_pos_ / rand_rpy_. */
  void resample();

  //! Body ID
  int body_id_ = -1;

  //! qpos address of the freejoint (xyz + wxyz), or -1 if not found
  int freejoint_qposadr_ = -1;

  //! Nominal position from m->qpos0 (xyz)
  std::array<double, 3> nominal_pos_{};

  //! Nominal quaternion from m->qpos0 (wxyz)
  std::array<double, 4> nominal_quat_{};

  //! Per-axis uniform ranges [min, max]
  std::array<double, 2> x_range_{};
  std::array<double, 2> y_range_{};
  std::array<double, 2> z_range_{};
  std::array<double, 2> roll_range_{};
  std::array<double, 2> pitch_range_{};
  std::array<double, 2> yaw_range_{};

  //! Current random position offset (xyz)
  std::array<double, 3> rand_pos_{};

  //! Current random orientation offset (roll, pitch, yaw)
  std::array<double, 3> rand_rpy_{};

  //! True when a new set of offsets has been drawn and not yet applied to d
  bool pending_ = false;

  std::mt19937 rng_{std::random_device{}()};
};

} // namespace MujocoRosUtils
