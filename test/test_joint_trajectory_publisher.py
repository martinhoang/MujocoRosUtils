#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import random
import math

_JOINTS = [
    ##### RIGHT ARM JOINTS #####
    "right_arm_m1_to_shoulder_l1",
    "right_arm_m2_to_shoulder_l2",
    "right_arm_m3_to_elbow_l1",
    "right_arm_m4_to_elbow_l2",
    "right_arm_m5_to_wrist_l1",
    "right_arm_wrist_l1_to_m6",
    "right_arm_m7_to_wrist_l3",
    ##### RIGHT HAND JOINTS #####
    "right_hand_index_1_joint",
    "right_hand_middle_1_joint",
    "right_hand_little_1_joint",
    "right_hand_ring_1_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    ##### LEFT ARM JOINTS #####
    "left_arm_m1_to_shoulder_l1",
    "left_arm_m2_to_shoulder_l2",
    "left_arm_m3_to_elbow_l1",
    "left_arm_m4_to_elbow_l2",
    "left_arm_m5_to_wrist_l1",
    "left_arm_wrist_l1_to_m6",
    "left_arm_m7_to_wrist_l3",
    ##### LEFT HAND JOINTS #####
    "left_hand_index_1_joint",
    "left_hand_middle_1_joint",
    "left_hand_little_1_joint",
    "left_hand_ring_1_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint"
]


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("joint_trajectory_publisher")
        self.publisher_ = self.create_publisher(
            JointTrajectory, "actuator_command_plugin/command_trajectory", 10
        )
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = _JOINTS

        point = JointTrajectoryPoint()
        point.positions = [random.uniform(-math.pi, math.pi) for _ in range(len(_JOINTS))]
        point.velocities = [0.0] * len(_JOINTS)
        point.accelerations = [0.0] * len(_JOINTS)
        point.effort = [0.0] * len(_JOINTS)
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Publishing JointTrajectory message")


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
