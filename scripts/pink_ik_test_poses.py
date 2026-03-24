#!/usr/bin/env python3
"""
Test script for Pink IK node.
Publishes a sequence of target poses to pink_ik/target_pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# Test poses relative to the arm base frame.
# The SO-100 arm reach is roughly 0.3m, so keep targets within that.
TEST_POSES = [
    {
        "name": "Forward reach",
        "position": [0.3, 0.0, 0.3],
        "orientation": [0.0, 0.0, 0.0, 1.0],  # x, y, z, w
    },
    {
        "name": "Left reach",
        "position": [0.1, 0.12, 0.3],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    },
    {
        "name": "High pose",
        "position": [0.05, 0.0, 0.28],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    },
    {
        "name": "Low forward",
        "position": [0.18, 0.0, 0.05],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    },
    {
        "name": "Right reach",
        "position": [0.1, -0.12, 0.3],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    },
]


class PinkIKTestNode(Node):
    def __init__(self):
        super().__init__("pink_ik_test_poses")
        self._pub = self.create_publisher(PoseStamped, "pink_ik/target_pose", 10)
        self._pose_idx = 0
        # Wait a bit for everything to initialize, then start sending poses
        self._timer = self.create_timer(6.0, self._send_next_pose)
        self.get_logger().info("Pink IK test node started. Sending poses every 6 seconds.")

    def _send_next_pose(self):
        if self._pose_idx >= len(TEST_POSES):
            self.get_logger().info("All test poses sent. Restarting sequence.")
            self._pose_idx = 0

        pose_data = TEST_POSES[self._pose_idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        msg.pose.position.x = pose_data["position"][0]
        msg.pose.position.y = pose_data["position"][1]
        msg.pose.position.z = pose_data["position"][2]

        msg.pose.orientation.x = pose_data["orientation"][0]
        msg.pose.orientation.y = pose_data["orientation"][1]
        msg.pose.orientation.z = pose_data["orientation"][2]
        msg.pose.orientation.w = pose_data["orientation"][3]

        self._pub.publish(msg)
        self.get_logger().info(
            f"Sent pose [{self._pose_idx + 1}/{len(TEST_POSES)}]: "
            f"{pose_data['name']} -> "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})"
        )
        self._pose_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = PinkIKTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
