#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from mujoco_ros_utils.srv import SpawnEntity


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def parse_args(arguments=None):
    parser = argparse.ArgumentParser(
        description="Spawn an MJCF or URDF entity through SceneManager."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "-topic",
        help="std_msgs/String topic containing MJCF or URDF XML.",
    )
    source.add_argument(
        "-file",
        help="Path to an MJCF or URDF file.",
    )
    parser.add_argument("-entity", required=True, help="Unique entity name.")
    parser.add_argument(
        "-robot_namespace",
        default="",
        help="Robot namespace, accepted for Gazebo spawn_entity.py compatibility.",
    )
    parser.add_argument(
        "-service",
        default="/scene_manager/spawn_entity",
        help="SceneManager SpawnEntity service.",
    )
    parser.add_argument(
        "-attach_to",
        default="worldbody",
        help="Target MuJoCo body, or worldbody.",
    )
    parser.add_argument("-x", type=float, default=0.0)
    parser.add_argument("-y", type=float, default=0.0)
    parser.add_argument("-z", type=float, default=0.0)
    parser.add_argument("-R", type=float, default=0.0, help="Roll in radians.")
    parser.add_argument("-P", type=float, default=0.0, help="Pitch in radians.")
    parser.add_argument("-Y", type=float, default=0.0, help="Yaw in radians.")
    parser.add_argument(
        "-with_freejoint",
        action="store_true",
        help="Add a free joint to the spawned top-level body.",
    )
    parser.add_argument(
        "-timeout",
        type=float,
        default=60.0,
        help="Topic and service timeout in seconds.",
    )
    return parser.parse_args(arguments)


class EntitySpawner(Node):
    def __init__(self, args) -> None:
        super().__init__("spawn_entity")
        self.args = args
        self.xml_content = None
        self.subscription = None
        self.client = self.create_client(SpawnEntity, args.service)

    def wait_for_xml(self):
        if self.args.file:
            path = Path(self.args.file).expanduser().resolve()
            if not path.is_file():
                raise RuntimeError(f"Description file does not exist: {path}")
            return None, str(path)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            String,
            self.args.topic,
            self._description_callback,
            qos,
        )
        self.get_logger().info(
            f"Waiting for robot description on {self.args.topic}"
        )
        rclpy.spin_until_future_complete(
            self,
            _message_future(self),
            timeout_sec=self.args.timeout,
        )
        if self.xml_content is None:
            raise RuntimeError(
                f"Timed out waiting for description on {self.args.topic}"
            )
        return self.xml_content, None

    def _description_callback(self, message):
        if self.xml_content is None and message.data.strip():
            self.xml_content = message.data

    def spawn(self, xml_content, xml_path):
        if not self.client.wait_for_service(timeout_sec=self.args.timeout):
            raise RuntimeError(
                f"Service {self.args.service} was not available"
            )

        qw, qx, qy, qz = quaternion_from_rpy(
            self.args.R, self.args.P, self.args.Y
        )
        request = SpawnEntity.Request()
        request.name = self.args.entity
        request.xml_content = xml_content or ""
        request.xml_path = xml_path or ""
        request.attach_to = self.args.attach_to
        request.pos_x = self.args.x
        request.pos_y = self.args.y
        request.pos_z = self.args.z
        request.rot_qw = qw
        request.rot_qx = qx
        request.rot_qy = qy
        request.rot_qz = qz
        request.with_freejoint = self.args.with_freejoint

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.args.timeout
        )
        response = future.result()
        if response is None:
            raise RuntimeError(
                f"Timed out calling {self.args.service}"
            )
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(
            f"Spawned entity '{self.args.entity}' as "
            f"'{response.spawned_body_name}'"
        )


def _message_future(node):
    future = rclpy.task.Future()

    def check_message():
        if node.xml_content is not None and not future.done():
            future.set_result(node.xml_content)

    timer = node.create_timer(0.01, check_message)
    future.add_done_callback(lambda _: node.destroy_timer(timer))
    return future


def main():
    args = parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = EntitySpawner(args)
    exit_code = 0
    try:
        node.spawn(*node.wait_for_xml())
    except (KeyboardInterrupt, RuntimeError) as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
