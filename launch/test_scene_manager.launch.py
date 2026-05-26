#!/usr/bin/env python3
"""
Launch file for SceneManager integration tests.

Starts a dedicated Xvfb virtual display and the MuJoCo simulate binary as
separate processes (both direct children of the launch system so both are
killed cleanly on shutdown), then runs the Python test script.

Usage:
    ros2 launch mujoco_ros_utils test_scene_manager.launch.py
    ros2 launch mujoco_ros_utils test_scene_manager.launch.py headless:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_free_display(start: int = 66, end: int = 200) -> str:
    """Return the first X display number (as ':N') with no active lock file."""
    for n in range(start, end):
        if not os.path.exists(f"/tmp/.X{n}-lock"):
            return f":{n}"
    return f":{start}"


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_ros_utils")
    mujoco_dir = os.environ.get("MUJOCO_PATH", "")
    if not mujoco_dir:
        raise RuntimeError("MUJOCO_PATH environment variable is not set")

    simulate_bin  = os.path.join(mujoco_dir, "bin", "simulate")
    step_hook_lib = os.path.join(mujoco_dir, "bin", "mujoco_plugin", "libmujoco_step_hook.so")
    test_world    = os.path.join(pkg_share, "xml", "test_scene_manager_world.xml")

    # Build the environment with LD_PRELOAD to inject the between-step hook.
    # This allows SceneManager to call mj_recompile safely before each mj_step.
    sim_env = dict(os.environ)
    if os.path.exists(step_hook_lib):
        existing_preload = sim_env.get("LD_PRELOAD", "")
        sim_env["LD_PRELOAD"] = (step_hook_lib + ":" + existing_preload).rstrip(":")

    # Virtual display number used for the headless Xvfb server.
    # Pick the first free display number to avoid clashing with any existing server.
    VDISPLAY = _find_free_display()

    # Headless env: point GLFW to the virtual display and (when Nvidia prime-
    # offload is not active) force Mesa software rasterizer inside Xvfb.
    sim_env_headless = dict(sim_env)
    sim_env_headless["DISPLAY"] = VDISPLAY
    if "__NV_PRIME_RENDER_OFFLOAD" not in sim_env_headless:
        sim_env_headless["LIBGL_ALWAYS_SOFTWARE"] = "1"

    headless = LaunchConfiguration("headless", default="true")

    declared = [
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run simulate with a dedicated Xvfb virtual display",
        ),
    ]

    # ── Xvfb virtual display (headless only) ─────────────────────────────────
    # Xvfb and simulate are started as SEPARATE direct children of the launch
    # system so that both receive the SIGTERM when the launch shuts down.
    # (Using xvfb-run as a wrapper orphans simulate because xvfb-run is a shell
    # script that does not forward signals to its children.)
    xvfb = ExecuteProcess(
        cmd=["Xvfb", VDISPLAY, "-screen", "0", "1280x1024x24"],
        output="log",
        condition=IfCondition(headless),
    )

    # ── Simulate (headless) ───────────────────────────────────────────────────
    sim_headless = ExecuteProcess(
        cmd=[simulate_bin, test_world],
        output="screen",
        additional_env=sim_env_headless,
        condition=IfCondition(headless),
    )

    sim_display = ExecuteProcess(
        cmd=[simulate_bin, test_world],
        output="screen",
        additional_env=sim_env,
        condition=UnlessCondition(headless),
    )

    # ── Test node — delayed 5 s to let SceneManager services come online ──────
    test_node_action = Node(
        package="mujoco_ros_utils",
        executable="test_scene_manager_services.py",
        output="screen",
        name="scene_manager_tester",
    )

    test_with_delay = TimerAction(
        period=5.0,
        actions=[test_node_action],
    )

    # Emit a clean Shutdown when the test node exits so that simulate is
    # terminated before it crashes on DDS context teardown.
    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=test_node_action,
            on_exit=[EmitEvent(event=Shutdown(reason="test node exited"))],
        )
    )

    return LaunchDescription(declared + [xvfb, sim_headless, sim_display, test_with_delay, shutdown_when_done])
