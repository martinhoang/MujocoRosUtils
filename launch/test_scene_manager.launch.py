#!/usr/bin/env python3
"""
Launch file for SceneManager integration tests.

Starts the MuJoCo simulate binary with the test world (headless via Xvfb),
then runs the Python test script.  The launch exits when the test node exits.

Usage:
    ros2 launch mujoco_ros_utils test_scene_manager.launch.py
    ros2 launch mujoco_ros_utils test_scene_manager.launch.py headless:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_ros_utils")
    mujoco_dir = os.environ.get("MUJOCO_PATH", "")
    if not mujoco_dir:
        raise RuntimeError("MUJOCO_PATH environment variable is not set")

    simulate_bin  = os.path.join(mujoco_dir, "bin", "simulate")
    step_hook_lib = os.path.join(mujoco_dir, "bin", "mujoco_plugin", "libmujoco_step_hook.so")
    test_world    = os.path.join(pkg_share, "xml", "test_scene_manager_world.xml")
    test_script  = os.path.join(
        get_package_share_directory("mujoco_ros_utils"),
        "..", "..", "..", "..",   # ws root install/share/../..
    )

    # Build the environment with LD_PRELOAD to inject the between-step hook.
    # This allows SceneManager to call mj_recompile safely before each mj_step.
    sim_env = dict(os.environ)
    if os.path.exists(step_hook_lib):
        existing_preload = sim_env.get("LD_PRELOAD", "")
        sim_env["LD_PRELOAD"] = (step_hook_lib + ":" + existing_preload).rstrip(":")

    # Prefer the source location so edits are picked up without reinstall
    src_script = os.path.join(
        os.path.dirname(__file__), "..", "test", "test_scene_manager_services.py"
    )
    if not os.path.exists(src_script):
        src_script = os.path.join(pkg_share, "test", "test_scene_manager_services.py")

    headless = LaunchConfiguration("headless", default="true")

    declared = [
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run simulate under xvfb-run (no physical display needed)",
        ),
    ]

    # ── Simulate (headless via xvfb-run) ─────────────────────────────────────
    sim_headless = ExecuteProcess(
        cmd=["xvfb-run", "-a", simulate_bin, test_world],
        output="screen",
        additional_env=sim_env,
        condition=IfCondition(headless),
    )

    sim_display = ExecuteProcess(
        cmd=[simulate_bin, test_world],
        output="screen",
        additional_env=sim_env,
        condition=UnlessCondition(headless),
    )

    # ── Test node — delayed 5 s to let SceneManager services come online ──────
    test_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="mujoco_ros_utils",
                executable="test_scene_manager_services.py",
                output="screen",
                name="scene_manager_tester",
            )
        ],
    )

    return LaunchDescription(declared + [sim_headless, sim_display, test_node])
