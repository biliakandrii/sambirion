#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import sin, cos, sqrt
import ast

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Declare parameters for goals
        # Format: "x1,y1,yaw1,x2,y2,yaw2,..."
        self.declare_parameter('goals', '2.0,0.0,0.0,2.0,2.0,1.57,0.0,2.0,3.14,0.0,0.0,0.0')
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('update_rate', 15)

        # Get parameters
        goals_str = self.get_parameter('goals').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        odom_topic = self.get_parameter('odom_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        update_rate = self.get_parameter('update_rate').value

        # Parse goals from comma-separated string
        try:
            values = [float(x.strip()) for x in goals_str.split(',')]
            if len(values) % 3 != 0:
                raise ValueError("Goals must have groups of 3 values (x, y, yaw)")
            
            self.goals = []
            for i in range(0, len(values), 3):
                self.goals.append((values[i], values[i+1], values[i+2]))
        except Exception as e:
            self.get_logger().error(f"Failed to parse goals parameter: {e}")
            self.get_logger().error("Using default goals")
            self.goals = [
                (2.0, 0.0, 0.0),
                (2.0, 2.0, 1.57),
                (0.0, 2.0, 3.14),
                (0.0, 0.0, 0.0)
            ]

        # Publisher for pose messages
        self.pub = self.create_publisher(PoseStamped, goal_topic, 10)

        # Subscriber for robot's current position
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Timer to publish goals and check if reached
        self.timer = self.create_timer(update_rate, self.update_goal)

        self.index = 0
        self.current_pose = None
        self.current_goal = None
        self.start_time = None
        self.mission_complete = False
        
        # Print configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("Goal Publisher Configuration:")
        self.get_logger().info(f"  Odometry Topic: {odom_topic}")
        self.get_logger().info(f"  Goal Topic: {goal_topic}")
        self.get_logger().info(f"  Update Rate: {update_rate} Hz")
        self.get_logger().info(f"  Goal Tolerance: {self.goal_tolerance} m")
        self.get_logger().info(f"  Number of Goals: {len(self.goals)}")
        for i, (x, y, yaw) in enumerate(self.goals):
            self.get_logger().info(f"    Goal {i+1}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        self.get_logger().info("=" * 60)
        self.get_logger().info('Waiting for odometry data...')

    def odom_callback(self, msg):
        """Update current robot position from odometry"""
        self.current_pose = msg.pose.pose

    def update_goal(self):
        """Publish current goal and check if robot reached it"""
        
        # Wait for odometry data
        if self.current_pose is None:
            return

        # Check if all goals are completed
        if self.index >= len(self.goals):
            if not self.mission_complete and self.start_time is not None:
                elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                self.get_logger().info(f"=== ALL GOALS COMPLETED ===")
                self.get_logger().info(f"Total time: {elapsed_time:.2f} seconds")
                self.mission_complete = True
            return

        # Start timer on first goal
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("=== STARTING MISSION ===")

        # Get current goal
        x, y, yaw = self.goals[self.index]

        # Publish the current goal continuously
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

        # Log only when goal changes
        if self.current_goal != self.index:
            self.get_logger().info(f"Publishing goal {self.index+1}/{len(self.goals)}: x={x}, y={y}, yaw={yaw}")
            self.current_goal = self.index

        # Check if current goal is reached
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        distance = sqrt((x - current_x)**2 + (y - current_y)**2)

        if distance < self.goal_tolerance:
            self.get_logger().info(f"Goal {self.index+1} reached! Distance: {distance:.3f}m")
            self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = GoalPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()