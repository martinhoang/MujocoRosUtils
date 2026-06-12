#!/usr/bin/env python3
"""Call the SceneManager DeleteEntity service to delete a spawned entity."""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from mujoco_ros_utils.srv import DeleteEntity


def parse_args(arguments=None):
    parser = argparse.ArgumentParser(
        description="Delete a named entity through SceneManager."
    )
    parser.add_argument("-entity", required=True, help="Entity name to delete.")
    parser.add_argument(
        "-robot_namespace",
        default="",
        help="Accepted for spawn_entity.py compatibility; not used.",
    )
    parser.add_argument(
        "-service",
        default="/scene_manager/despawn_entity",
        help="SceneManager DeleteEntity service.",
    )
    parser.add_argument(
        "-timeout",
        type=float,
        default=30.0,
        help="Service wait timeout in seconds.",
    )
    return parser.parse_args(arguments)


class EntityDeleter(Node):
    def __init__(self, args):
        super().__init__("delete_entity")
        self.args = args
        self.client = self.create_client(DeleteEntity, args.service)

    def delete(self) -> bool:
        if not self.client.wait_for_service(timeout_sec=self.args.timeout):
            self.get_logger().error(
                f"Service {self.args.service} not available after "
                f"{self.args.timeout}s — is the simulator running?"
            )
            return False

        request = DeleteEntity.Request()
        request.name = self.args.entity
        self.get_logger().info(
            f"Calling {self.args.service} to delete '{self.args.entity}' ..."
        )
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error("Service call returned no response.")
            return False
        if not response.success:
            self.get_logger().error(
                f"Delete failed: {response.message}"
            )
            return False

        self.get_logger().info(
            f"Entity '{self.args.entity}' deleted successfully."
        )
        return True


def main():
    args = parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = EntityDeleter(args)
    try:
        ok = node.delete()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
