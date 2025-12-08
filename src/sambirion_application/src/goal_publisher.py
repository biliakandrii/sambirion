#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sin, cos

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Publisher for simple pose messages
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer to send one goal every 3 seconds
        self.timer = self.create_timer(5.0, self.publish_next_goal)

        # Predefined goals (x, y, yaw)
        self.goals = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, 0.0)
        ]

        self.index = 0
        self.get_logger().info('Simple Goal Publisher started.')

    def publish_next_goal(self):
        if self.index >= len(self.goals):
            self.get_logger().info("All goals published.")
            return

        x, y, yaw = self.goals[self.index]

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # yaw → quaternion
        msg.pose.orientation.z = sin(yaw / 2)
        msg.pose.orientation.w = cos(yaw / 2)

        self.pub.publish(msg)
        self.get_logger().info(f"Published goal {self.index+1}: x={x}, y={y}, yaw={yaw}")

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
