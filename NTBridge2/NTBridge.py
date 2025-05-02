#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import ntcore


class LivoxOdometryBridge(Node):
    def __init__(self):
        super().__init__('livox_ntcore_bridge')

        # --- NTCore (NT4) setup ---
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("livox_ros_client")
        self.inst.setServer("10.99.99.29")  # Replace with roboRIO IP
        

        self.table = self.inst.getTable("/livoxpose")
        self.pos_pub = self.table.getDoubleArrayTopic("position").publish()

        # --- ROS 2 Subscription ---
        self.subscription = self.create_subscription(
            Odometry,
            "/lio_livox/odometry_mapped",
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Convert quaternion → yaw (Z rotation)
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y ** 2 + ori.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Publish [x, y, yaw] as float array
        self.pos_pub.set([pos.x, pos.y, yaw])
        self.get_logger().info(f"Published: x={pos.x:.2f}, y={pos.y:.2f}, yaw={yaw:.2f} rad")


def main(args=None):
    rclpy.init(args=args)
    node = LivoxOdometryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
