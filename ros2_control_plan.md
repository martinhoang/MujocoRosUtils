# MujocoRos2Control Plugin Implementation Plan

This document outlines the strategy for integrating `ros2_control` with MuJoCo through a custom plugin.

## 1. Goal

The primary goal is to create a MuJoCo plugin that acts as a `ros2_control` hardware interface. This allows `ros2_control` controllers (e.g., `joint_trajectory_controller`) to command joints in a MuJoCo simulation and read their states (position and velocity).

## 2. Plugin Architecture

The solution is based on a single class, `MujocoRosUtils::MujocoRos2Control`, which serves two main purposes:
1.  It is a **MuJoCo Plugin**, registered and managed by the MuJoCo simulation engine.
2.  It is a **ros2_control Hardware System Interface**, inheriting from `hardware_interface::SystemInterface`.

### 2.1. MuJoCo Plugin Integration

- **Registration**: The plugin is registered with MuJoCo using a static `RegisterPlugin` function. This function defines the plugin's name (`MujocoRosUtils::MujocoRos2Control`) and its attributes.
- **Instantiation**: When MuJoCo loads a model with this plugin, it calls the `init` callback, which creates an instance of the `MujocoRos2Control` class. A pointer to this instance is stored in `mjData.plugin_data`.
- **Callbacks**: The plugin uses the following MuJoCo callbacks:
    - `init`: Creates and initializes the plugin instance.
    - `destroy`: Cleans up and deletes the plugin instance.
    - `reset`: Resets the controller manager state.
    - `compute`: The main update loop, called at each simulation step.

### 2.2. ros2_control Integration

The `MujocoRos2Control` class implements the `hardware_interface::SystemInterface` by overriding its key methods:

-   **`on_init()`**: Initializes the hardware interface. It reads the joint names from the plugin configuration and prepares the internal data structures for states and commands.
-   **`export_state_interfaces()`**: Exposes the joint states (position and velocity) to `ros2_control`. It creates a `StateInterface` for each joint's position and velocity.
-   **`export_command_interfaces()`**: Exposes the command interfaces to `ros2_control`. It creates a `CommandInterface` for each joint's position command.
-   **`read()`**: Reads the current joint states (position and velocity) from the MuJoCo simulation (`mjData`) and stores them in the hardware interface's state vectors.
-   **`write()`**: Takes the commands from `ros2_control`'s command vectors and applies them to the MuJoCo simulation's actuator controls (`mjData->ctrl`).
-   **`on_activate()` / `on_deactivate()`**: Manages the lifecycle of the hardware interface.

## 3. Configuration

The plugin is configured directly within the MJCF file. This avoids the complexity of external URDF files.

-   **`node_name`**: (Optional) The name of the ROS 2 node to be created. Defaults to `mujoco_ros2_control`.
-   **`joints`**: (Required) A space-separated list of the MuJoCo joint names that should be exposed to `ros2_control`.

Example configuration in an MJCF file:
```xml
<mujoco>
  <worldbody>
    ...
  </worldbody>
  <actuator>
    ...
  </actuator>
  <plugin>
    <plugin plugin="MujocoRosUtils::MujocoRos2Control">
      <config key="node_name" value="my_robot_control_node"/>
      <config key="joints" value="joint1 joint2 joint3"/>
    </plugin>
  </plugin>
</mujoco>
```

## 4. Data Flow and Execution Model

1.  **Initialization**:
    -   MuJoCo loads the model and calls the plugin's `init` function.
    -   The `MujocoRos2Control` constructor is called.
    -   A new ROS 2 node is created.
    -   The `ControllerManager` is instantiated.
    -   A separate thread is spawned to spin the ROS executor (`MultiThreadedExecutor`), which handles ROS services, parameters, and other callbacks.

2.  **Update Loop** (within the `compute` callback):
    -   The `read()` method is called to get the latest joint states from `mjData`.
    -   The `ControllerManager::update()` method is called, which runs the active controllers. The controllers process their logic and write new commands to the hardware interface.
    -   The `write()` method is called to apply the new commands to the MuJoCo actuators (`mjData->ctrl`).

## 5. Potential Issues and Next Steps

Given that the implementation is not working, here are potential areas to investigate:

-   **Threading and ROS Spinning**: Is the ROS executor being spun correctly? Are there race conditions between the MuJoCo thread and the ROS thread?
-   **Controller Manager Lifecycle**: Is the `ControllerManager` being configured and started correctly? Are the controllers being loaded and activated?
-   **Timing and Synchronization**: Is the `update` loop being called at the correct frequency? Are the `read` and `write` operations synchronized properly with the MuJoCo simulation step?
-   **Joint/Actuator Naming**: Does the name of the joint in the `joints` config attribute correctly map to an actuator in the MuJoCo model? The current implementation assumes a one-to-one mapping.
-   **Dependencies**: Are all `ros2_control` and other dependencies correctly specified in the `CMakeLists.txt` and `package.xml`?

The next step should be a systematic debugging process based on this plan, starting with logging and introspection to verify each stage of the data flow.