# GeomTfPublisher Plugin

## Overview

The `GeomTfPublisher` plugin publishes TF (Transform) frames for all geoms (geometries) attached to a specified parent body. This is useful for:
- Visualizing sensor positions in RViz
- Tracking collision geometry transforms
- Publishing contact point locations
- Debugging geom placements in the simulation

## Features

- **Automatic Geom Discovery**: Finds all geoms attached to a specified parent body
- **Custom Topic**: Publish transforms to a custom topic (not just `/tf`)
- **Configurable Rate**: Control publishing frequency
- **Multiple Instances**: Can create multiple plugin instances for different bodies

## Configuration Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `parent_body` | string | Yes | - | Name of the parent body whose geoms will be published |
| `frame_id` | string | No | `"map"` | Parent frame ID for the TF transforms |
| `topic_name` | string | No | `"/tf"` | Custom topic name to publish transforms to |
| `publish_rate` | float | No | `30.0` | Publishing rate in Hz |
| `publish_bodies` | bool | No | `false` | Also publish TF frames for all bodies in the hierarchy |

## Usage

### Basic Example

```xml
<body name="robot_base" pos="0 0 0.5">
  <geom name="base_link" type="box" size="0.2 0.2 0.1"/>
  <geom name="sensor_1" type="sphere" size="0.05" pos="0.15 0 0.1"/>
  <geom name="sensor_2" type="sphere" size="0.05" pos="-0.15 0 0.1"/>
  
  <plugin plugin="MujocoRosUtils::GeomTfPublisher">
    <config key="parent_body" value="robot_base"/>
    <config key="frame_id" value="world"/>
    <config key="topic_name" value="/robot/geom_tf"/>
    <config key="publish_rate" value="50.0"/>
    <config key="publish_bodies" value="true"/>
  </plugin>
</body>
```

### Multiple Instances

You can create separate plugin instances for different bodies:

```xml
<body name="robot_base" pos="0 0 0.5">
  <geom name="base_geom" type="box" size="0.2 0.2 0.1"/>
  
  <plugin plugin="MujocoRosUtils::GeomTfPublisher">
    <config key="parent_body" value="robot_base"/>
    <config key="topic_name" value="/robot/base_geoms"/>
  </plugin>
</body>

<body name="manipulator" pos="1 0 0.3">
  <geom name="arm_link" type="cylinder" size="0.05 0.3"/>
  
  <plugin plugin="MujocoRosUtils::GeomTfPublisher">
    <config key="parent_body" value="manipulator"/>
    <config key="topic_name" value="/robot/arm_geoms"/>
  </plugin>
</body>
```

## Transform Computation

The plugin computes the world frame pose for each geom using:

```
T_world_geom = T_world_body * T_body_geom
```

Where:
- `T_world_body` is the body's pose in the world frame (from `d->xpos` and `d->xquat`)
- `T_body_geom` is the geom's offset from its parent body (from `m->geom_pos` and `m->geom_quat`)

## Child Frame IDs

The TF child frame ID for each geom is determined as follows:
- If the geom has a name, use that name
- Otherwise, use `geom_<id>` where `<id>` is the geom's numeric ID

## Visualization in RViz

To visualize the published transforms in RViz:

1. Add a TF display
2. Set the topic to your custom topic (e.g., `/robot/geom_tf`)
3. Configure the fixed frame to match your `frame_id` parameter

## Example Command

```bash
# Launch the simulation with the example XML
ros2 run mujoco_ros_utils mujoco_node --xml geom_tf_example.xml

# In another terminal, visualize in RViz
rviz2
```

## Notes

- The plugin uses `mjPLUGIN_PASSIVE` capability flag, so it runs at every simulation step
- Geom names should be unique to avoid TF frame conflicts
- The publish rate is automatically adjusted based on the simulation timestep
- If no geoms are found under the parent body, a warning is printed but the plugin still loads

## See Also

- [PosePublisher](./PosePublisher.md) - For publishing body poses
- [ContactForcePublisher](./ContactForcePublisher.md) - For publishing contact forces
