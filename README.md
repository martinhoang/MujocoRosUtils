# MujocoRosUtils

ROS 2 integration utilities for MuJoCo. The package provides loadable MuJoCo
plugins for publishing simulation data, commanding actuators, integrating
`ros2_control`, changing a running scene, recording datasets, and plotting
signals in real time.

[![License: BSD-2-Clause](https://img.shields.io/badge/license-BSD--2--Clause-blue.svg)](LICENSE)

## Highlights

- ROS 2 clock, pose, velocity, TF, image, point-cloud, lidar, and sensor output
- Topic-based actuator commands and external-force application
- `ros2_control` hardware integration with multi-interface and multi-robot
  support
- Runtime scene inspection and mutation through ROS 2 services
- In-process simulation data aggregation without a DDS round trip
- HDF5, MCAP, and LeRobot dataset recording
- Real-time ImPlot-based signal visualization and CSV export
- Pose randomization and mimic-joint support
- Modular plugin libraries by default, with a monolithic compatibility option
- Optional CUDA acceleration for image depth processing

## Compatibility

- Linux
- C++17 compiler
- ROS 2 Humble or Jazzy
- MuJoCo 2.3.5 or newer

The plugin registration layer supports both the pre-3.8 and MuJoCo 3.8+
`mjPLUGIN_LIB_INIT` APIs.

## Installation

Install MuJoCo and the ROS dependencies, then build the package in a ROS 2
workspace:

```bash
mkdir -p ~/cobot_ws/src
cd ~/cobot_ws/src
git clone https://github.com/martinhoang/MujocoRosUtils.git mujoco_ros_utils

cd ~/cobot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select mujoco_ros_utils \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DMUJOCO_ROOT_DIR=/absolute/path/to/mujoco

source install/setup.bash
```

`MUJOCO_ROOT_DIR` must contain MuJoCo's `include/` and `lib/` or `bin/`
directories. It can also be supplied through `MUJOCO_PATH`:

```bash
export MUJOCO_PATH=/absolute/path/to/mujoco
colcon build --packages-select mujoco_ros_utils
```

The package installs a ROS environment hook that adds its plugin directory to
`MUJOCO_PLUGIN_PATH`. Source the workspace before starting MuJoCo.

### Build options

| Option | Default | Description |
| --- | --- | --- |
| `MUJOCO_ROS_UTILS_PLUGIN_LAYOUT` | `MODULAR` | Build one DSO per plugin. Set to `MONOLITHIC` for a combined library. |
| `MUJOCO_ROS_UTILS_ENABLE_CUDA` | `AUTO` | Image processing acceleration: `AUTO`, `ON`, or `OFF`. |
| `MUJOCO_ROS_UTILS_BUILD_TESTING` | `ON` | Build unit and integration tests. |
| `MUJOCO_ROS_UTILS_BUILD_BENCHMARKS` | `OFF` | Build manually invoked benchmarks. |
| `INSTALL_DOCUMENTATION` | `OFF` | Generate and install Doxygen documentation. |

Each plugin also has an individual
`MUJOCO_ROS_UTILS_BUILD_<PLUGIN_NAME>` option. See
[`CMakeLists.txt`](CMakeLists.txt) for the complete list.

`SimPlotter` fetches pinned ImGui and ImPlot revisions during configuration.
Disable it with `-DMUJOCO_ROS_UTILS_BUILD_SIM_PLOTTER=OFF` when building
offline without those sources cached.

## Quick start

After building and sourcing the workspace:

```bash
"$MUJOCO_ROOT_DIR/bin/simulate" \
  "$(ros2 pkg prefix mujoco_ros_utils)/share/mujoco_ros_utils/xml/sample_mujoco_ros_utils.xml"
```

In another terminal:

```bash
source ~/cobot_ws/install/setup.bash
ros2 launch mujoco_ros_utils display.launch.py
```

To run `depth_image_proc` alongside RViz:

```bash
ros2 launch mujoco_ros_utils display.launch.py points:=true
```

The package's plugins are declared and instantiated in MJCF:

```xml
<extension>
  <plugin plugin="MujocoRosUtils::ClockPublisher"/>
</extension>

<worldbody>
  <plugin plugin="MujocoRosUtils::ClockPublisher">
    <config key="topic_name" value="/clock"/>
    <config key="publish_rate" value="100"/>
    <config key="use_sim_time" value="true"/>
  </plugin>
</worldbody>
```

Placement depends on the plugin capability. Passive plugins are normally
instantiated under `<worldbody>` or as a named `<instance>` in `<extension>`;
sensor plugins are instantiated under `<sensor>`; actuator plugins are
instantiated under `<actuator>`.

## Plugins

| Plugin | Purpose | Main configuration |
| --- | --- | --- |
| `MujocoRosUtils::ClockPublisher` | Publish `/clock` and enable simulation time | `topic_name`, `publish_rate`, `use_sim_time` |
| `MujocoRosUtils::PosePublisher` | Publish body pose/velocity and optionally TF | `frame_id`, `pose_topic_name`, `vel_topic_name`, `publish_rate`, `output_tf`, `tf_child_frame_id` |
| `MujocoRosUtils::ImagePublisher` | Publish color, depth, camera info, and point clouds | Camera names, frames, namespace, topics, image/depth dimensions, clip ranges, readback mode, parallel processing, downsampling |
| `MujocoRosUtils::LidarPublisher` | Ray-cast 2D/3D lidar with TF, QoS, visualization, and noise models | `site_name`, scan pattern/angles, range, exclusions, topics, TF, QoS, noise |
| `MujocoRosUtils::SensorPublisher` | Publish scalar, point, quaternion, and vector MuJoCo sensor data | `sensor_name`, `frame_id`, `topic_name`, `publish_rate` |
| `MujocoRosUtils::ActuatorCommand` | Command one or more actuators from ROS topics | `node_name`, `topic_name`, `publish_rate` |
| `MujocoRosUtils::ExternalForce` | Apply timed world-frame forces to a body | `topic_name`, `vis_scale` |
| `MujocoRosUtils::MimicJoint` | Drive a slave joint from a master joint | `mimic_joint`, `gear` |
| `MujocoRosUtils::PoseRandomizer` | Randomize a free body's pose on reset | `x_range`, `y_range`, `z_range`, `roll_range`, `pitch_range`, `yaw_range` |
| `MujocoRosUtils::Ros2Control` | Connect MuJoCo joints and actuators to `ros2_control` | `node_name`, `publish_rate`, `robot_param_node`, `config_file`, `parameters`, `namespace` |
| `MujocoRosUtils::SimDataAggregator` | Gather camera and joint data in process and provide recording services | `instance_name`, `camera_namespaces`, `joint_names` |
| `MujocoRosUtils::SimPlotter` | Plot MuJoCo signals in a separate SDL/OpenGL window | Window settings plus up to 8 plots with 8 lines each |
| `MujocoRosUtils::SceneManager` | Inspect and modify a running MuJoCo scene | `node_name`, `namespace` |

### Image publishing

`ImagePublisher` supports:

- Separate color and depth cameras
- Separate color/depth frame IDs and resolutions
- Camera-specific near/far depth clipping
- Direct `sensor_msgs/PointCloud2` output
- Point-cloud rotation presets and downsampling
- Parallel CPU processing and optional CUDA acceleration
- An in-process `registry_key` used by `SimDataAggregator`

For new models, attach the plugin to an `xbody` and configure
`color_camera_name` and `depth_camera_name`. Legacy camera-attached instances
remain supported.

### Lidar

`LidarPublisher` supports configurable horizontal and vertical scan patterns,
including a VLP-16 preset. It can exclude bodies or geom groups, publish TF,
visualize hit/miss rays, and apply Gaussian range noise, dropout, outliers, and
XYZ jitter.

See:

- [`xml/sample_mujoco_ros_utils_lidar.xml`](xml/sample_mujoco_ros_utils_lidar.xml)
- [`xml/lidar_vlp16_sensors.xml`](xml/lidar_vlp16_sensors.xml)
- [`xml/lidar_vlp16_sites.xml`](xml/lidar_vlp16_sites.xml)

### ROS 2 control

`Ros2Control` loads a controller-manager YAML file and connects the URDF
`ros2_control` description to the active MuJoCo model. The current
implementation includes:

- Position, velocity, and effort command interfaces
- Multiple command interfaces on a joint
- State-interface export and validation
- Joint limits and soft limits
- Multi-robot namespaces and per-instance node names
- Controller-manager update timing based on simulation time
- `/reset_simulation` support
- Retry/backoff while waiting for `robot_state_publisher`

Use `config_file` in the plugin instance to select the controller YAML:

```xml
<extension>
  <plugin plugin="MujocoRosUtils::Ros2Control">
    <instance name="robot_control">
      <config key="config_file" value="/absolute/path/to/ros2_controllers.yaml"/>
      <config key="namespace" value="/robot"/>
      <config key="publish_rate" value="1000"/>
    </instance>
  </plugin>
</extension>
```

The `mujoco_hardware_plugin` hardware interface is exported through pluginlib.
Interface examples and validation fixtures are in:

- [`xml/test_ros2_control_interfaces.xml`](xml/test_ros2_control_interfaces.xml)
- [`xml/test_ros2_control_interfaces.urdf`](xml/test_ros2_control_interfaces.urdf)

### Runtime scene management

`SceneManager` uses MuJoCo's spec API and `mj_recompile` to change a running
model while preserving simulation state. It provides services for:

- Spawning MJCF or URDF entities from a path or string
- Despawning previously managed entities
- Listing managed entities
- Getting and setting body poses
- Getting and setting geom size/color properties
- Listing model bodies, geoms, joints, and actuators
- Activating or deactivating equality constraints

Example declaration:

```xml
<extension>
  <plugin plugin="MujocoRosUtils::SceneManager">
    <instance name="scene_manager_inst">
      <config key="node_name" value="scene_manager"/>
      <config key="namespace" value=""/>
    </instance>
  </plugin>
</extension>
```

The available service types are `SpawnEntity`, `DespawnEntity`,
`ListEntities`, `SetBodyPose`, `SetGeomProperties`, `SetEqualityActive`,
`GetBodyPose`, `GetGeomProperties`, and `GetModelInfo`.

Scene recompilation must run between simulation steps. The repository includes
`libmujoco_step_hook.so` for stock MuJoCo `simulate` binaries:

```bash
PACKAGE_PREFIX="$(ros2 pkg prefix mujoco_ros_utils)"
export LD_PRELOAD="$PACKAGE_PREFIX/lib/mujoco_ros_utils/plugins/libmujoco_step_hook.so"
"$MUJOCO_ROOT_DIR/bin/simulate" \
  "$(ros2 pkg prefix mujoco_ros_utils)/share/mujoco_ros_utils/xml/test_scene_manager_world.xml"
```

A custom simulator can instead call the between-step hook directly. See
[`launch/test_scene_manager.launch.py`](launch/test_scene_manager.launch.py)
for a complete launch pattern.

### Data aggregation and recording

`SimDataAggregator` reads joint state, actuator control, and camera frames
directly inside the simulator process. This avoids serializing high-bandwidth
data through ROS before recording.

```xml
<extension>
  <plugin plugin="MujocoRosUtils::SimDataAggregator">
    <instance name="sim_data">
      <config key="instance_name" value="training"/>
      <config key="camera_namespaces" value="wrist_cam,base_cam"/>
      <config key="joint_names" value="joint1,joint2,joint3"/>
    </instance>
  </plugin>
</extension>
```

For an `instance_name` of `training`, the generated node is
`sim_data_aggregator_training` and exposes:

- `/sim_data_aggregator_training/start_recording`
- `/sim_data_aggregator_training/stop_recording`
- `/sim_data_aggregator_training/cancel_recording`

Supported formats are:

- `hdf5`
- `mcap`
- `lerobot`
- `both` (MCAP and LeRobot)
- `hdf5_and_mcap`
- `hdf5_and_lerobot`

LeRobot output includes episode videos, state/action rows, task metadata, and
episode statistics. Video encoding runs asynchronously and uses NVENC when
available, with a software encoder fallback. The JSONL-to-Parquet conversion
helper is installed from
[`scripts/jsonl_to_lerobot_parquet.py`](scripts/jsonl_to_lerobot_parquet.py).

### Real-time plotting

`SimPlotter` renders signals in a separate SDL2/OpenGL thread without routing
data through DDS. Sources include:

```text
joint.<name>.position
joint.<name>.velocity
joint.<name>.effort
actuator.<name>.ctrl
actuator.<name>.force
body.<name>.pos.x
body.<name>.quat.w
sensor.<name>
qpos.<index>
qvel.<index>
qacc.<index>
ctrl.<index>
```

Plots support line and scatter modes, dual Y axes, reset-persistent history,
pause/resume, runtime reconfiguration, and CSV export. The
`PlotCommand` service accepts `add_plot`, `del_plot`, `add_line`, `del_line`,
`mod_plot`, `mod_line`, `clear`, `pause`, `resume`, `export`, and `list`.
Its resolved name includes the generated node and plugin instance names; use
`ros2 service list | grep plot_command` to discover it.

See [`xml/sim_plotter_example.xml`](xml/sim_plotter_example.xml) for a complete
configuration.

### Pose randomization

`PoseRandomizer` samples translation and roll/pitch/yaw offsets around a
freejoint body's nominal pose whenever the simulation resets:

```xml
<body name="object">
  <freejoint/>
  <plugin plugin="MujocoRosUtils::PoseRandomizer">
    <config key="x_range" value="-0.05 0.05"/>
    <config key="y_range" value="-0.05 0.05"/>
    <config key="yaw_range" value="-0.5236 0.5236"/>
  </plugin>
</body>
```

See [`xml/sample_pose_randomizer.xml`](xml/sample_pose_randomizer.xml).

## ROS interfaces

Messages:

- `mujoco_ros_utils/msg/ExternalForce`
- `mujoco_ros_utils/msg/ScalarStamped`

Services:

- `StartRecording`, `StopRecording`
- `PlotCommand`
- `SpawnEntity`, `DespawnEntity`, `ListEntities`
- `SetBodyPose`, `GetBodyPose`
- `SetGeomProperties`, `GetGeomProperties`
- `GetModelInfo`
- `SetEqualityActive`

Inspect an interface with:

```bash
ros2 interface show mujoco_ros_utils/srv/SpawnEntity
```

## Examples

| File | Demonstrates |
| --- | --- |
| [`xml/sample_mujoco_ros_utils.xml`](xml/sample_mujoco_ros_utils.xml) | Clock, pose, TF, image, actuator command, external force, and generic sensor publishing |
| [`xml/sample_mujoco_ros_utils_lidar.xml`](xml/sample_mujoco_ros_utils_lidar.xml) | Configurable lidar and VLP-16 output |
| [`xml/sample_pose_randomizer.xml`](xml/sample_pose_randomizer.xml) | Reset-time free-body pose randomization |
| [`xml/sim_plotter_example.xml`](xml/sim_plotter_example.xml) | Multi-window line and scatter plotting |
| [`xml/test_scene_manager_world.xml`](xml/test_scene_manager_world.xml) | Runtime scene services |

## Testing

Build and run the package tests with:

```bash
colcon build \
  --packages-select mujoco_ros_utils \
  --cmake-args \
    -DMUJOCO_ROOT_DIR="$MUJOCO_ROOT_DIR" \
    -DMUJOCO_ROS_UTILS_BUILD_TESTING=ON

colcon test --packages-select mujoco_ros_utils
colcon test-result --verbose
```

The test suite covers plugin loading, shared ROS context management,
`ros2_control` interface mapping, scene-manager services, reset behavior, and
roadmap regression cases.

## Docker

Development containers for Linux and Windows hosts are provided in
[`dockerfiles/`](dockerfiles/). See
[`dockerfiles/README.md`](dockerfiles/README.md) for usage.

## License

This project is distributed under the BSD 2-Clause License. See
[`LICENSE`](LICENSE).
