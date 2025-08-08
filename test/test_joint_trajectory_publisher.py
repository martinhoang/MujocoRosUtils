#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import random
import math
import sys
import argparse

_RIGHT_ARM_JOINTS = [
    "right_arm_m1_to_shoulder_l1",
    "right_arm_m2_to_shoulder_l2",
    "right_arm_m3_to_elbow_l1",
    "right_arm_m4_to_elbow_l2",
    "right_arm_m5_to_wrist_l1",
    "right_arm_wrist_l1_to_m6",
    "right_arm_m7_to_wrist_l3",
]
_RIGHT_HAND_JOINTS = [
    "right_hand_index_1_joint",
    "right_hand_middle_1_joint",
    "right_hand_little_1_joint",
    "right_hand_ring_1_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
]
_RIGHT_WHOLE_ARM_JOINTS = _RIGHT_ARM_JOINTS + _RIGHT_HAND_JOINTS
_LEFT_ARM_JOINTS = [
    "left_arm_m1_to_shoulder_l1",
    "left_arm_m2_to_shoulder_l2",
    "left_arm_m3_to_elbow_l1",
    "left_arm_m4_to_elbow_l2",
    "left_arm_m5_to_wrist_l1",
    "left_arm_wrist_l1_to_m6",
    "left_arm_m7_to_wrist_l3",
]
_LEFT_HAND_JOINTS = [
    "left_hand_index_1_joint",
    "left_hand_middle_1_joint",
    "left_hand_little_1_joint",
    "left_hand_ring_1_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
]
_LEFT_WHOLE_ARM_JOINTS = _LEFT_ARM_JOINTS + _LEFT_HAND_JOINTS

_ALL_JOINTS = _RIGHT_WHOLE_ARM_JOINTS + _LEFT_WHOLE_ARM_JOINTS


class JointCommandPublisher(Node):
    def __init__(self, use_joint_state=False, side="both", part="whole_arm"):
        super().__init__("joint_command_publisher")
        self.use_joint_state = use_joint_state
        self.side = side
        self.part = part

        joint_map = {
            "right": {
                "arm": _RIGHT_ARM_JOINTS,
                "hand": _RIGHT_HAND_JOINTS,
                "whole_arm": _RIGHT_WHOLE_ARM_JOINTS,
            },
            "left": {
                "arm": _LEFT_ARM_JOINTS,
                "hand": _LEFT_HAND_JOINTS,
                "whole_arm": _LEFT_WHOLE_ARM_JOINTS,
            },
        }

        if self.side == "both":
            self.joints = joint_map["right"][self.part] + joint_map["left"][self.part]
        else:
            self.joints = joint_map[self.side][self.part]

        if not self.joints:
            self.get_logger().warn("No joints selected, check arguments. Exiting.")
            return

        if self.use_joint_state:
            self.publisher_ = self.create_publisher(JointState, "actuator_command_plugin/command", 10)
            self.get_logger().info(f"Publishing JointState messages for {self.side} {self.part}.")
        else:
            self.publisher_ = self.create_publisher(
                JointTrajectory, "actuator_command_plugin/command_trajectory", 10
            )
            self.get_logger().info(f"Publishing JointTrajectory messages for {self.side} {self.part}.")
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        num_joints = len(self.joints)
        positions = [random.uniform(-math.pi, math.pi) for _ in range(num_joints)]
        velocities = [0.0] * num_joints
        efforts = [0.0] * num_joints

        if self.use_joint_state:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joints
            msg.position = positions
            msg.velocity = velocities
            msg.effort = efforts
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing JointState message")
        else:
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = self.joints

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = [0.0] * num_joints
            point.effort = efforts
            point.time_from_start = Duration(sec=1, nanosec=0)

            msg.points.append(point)

            self.publisher_.publish(msg)
            self.get_logger().info("Publishing JointTrajectory message")


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Publish joint commands, either as JointTrajectory or JointState."
    )
    parser.add_argument(
        "-js",
        "--use-joint-state",
        action="store_true",
        help="Use sensor_msgs/JointState instead of trajectory_msgs/JointTrajectory.",
    )
    parser.add_argument(
        "-s",
        "--side",
        type=str,
        default="both",
        choices=["right", "left", "both"],
        help="Specify which side to command.",
    )
    parser.add_argument(
        "-p",
        "--part",
        type=str,
        default="whole_arm",
        choices=["arm", "hand", "whole_arm"],
        help="Specify which part to command.",
    )

    # rclpy.init() will remove ros-specific arguments. We need to parse our arguments
    # before rclpy.init() to avoid conflicts, but we also need to pass the original
    # arguments to rclpy.init().
    if args is None:
        args = sys.argv

    # Separate our args from ros args
    our_args, ros_args = parser.parse_known_args(args[1:])

    rclpy.init(args=ros_args)

    joint_command_publisher = JointCommandPublisher(
        use_joint_state=our_args.use_joint_state, side=our_args.side, part=our_args.part
    )
    try:
        rclpy.spin(joint_command_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            joint_command_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
