#!/usr/bin/env python3

"""
ROS2 Node: Trajectory Plotter
Subscribes to robot odometry and plots the trajectory on a 10x10 map
Saves plot every N seconds with unique timestamp from node start
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import os
from datetime import datetime
import time


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Parameters
        self.declare_parameter('topic_name', '/odom')
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('map_size', 10.0)  # 10x10 meters
        self.declare_parameter('output_dir', './plots/trajectory/')
        self.declare_parameter('save_interval', 3.0)  # seconds
        
        # Get parameters
        topic_name = self.get_parameter('topic_name').value
        self.max_points = self.get_parameter('max_points').value
        self.map_size = self.get_parameter('map_size').value
        self.output_dir = self.get_parameter('output_dir').value
        self.save_interval = self.get_parameter('save_interval').value
        
        # Generate filename with start time
        start_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_filename = f'trajectory_{start_time}.png'
        
        # Storage for trajectory points
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create subscriber (try Odometry first, can also handle PoseStamped)
        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odom_callback,
            10
        )
        
        # Timer for saving plots
        self.save_timer = self.create_timer(self.save_interval, self.save_plot)
        
        self.get_logger().info(f'Trajectory Plotter started, subscribing to {topic_name}')
        self.get_logger().info(f'Map size: {self.map_size}x{self.map_size} meters')
        self.get_logger().info(f'Saving to: {os.path.abspath(self.output_dir)}/{self.output_filename}')
        self.get_logger().info(f'Save interval: {self.save_interval} seconds')
        
    def odom_callback(self, msg):
        """Callback for odometry messages"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Store the position
        self.x_data.append(x)
        self.y_data.append(y)
        
    def save_plot(self):
        """Create and save the trajectory plot"""
        if len(self.x_data) == 0:
            self.get_logger().warn('No trajectory data collected, no plot saved')
            return
        
        # Create figure
        fig, ax = plt.subplots(figsize=(8, 8))
        
        # Set up the plot limits (10x10 map)
        ax.set_xlim(-self.map_size/2, self.map_size/2)
        ax.set_ylim(-self.map_size/2, self.map_size/2)
        
        # Grid and labels
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        
        # Title with timestamp and point count
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        ax.set_title(f'Robot Trajectory (10x10 Map)\n{timestamp}\nPoints: {len(self.x_data)}', 
                     fontsize=14, fontweight='bold')
        ax.set_aspect('equal')
        
        # Add origin marker
        ax.plot(0, 0, 'go', markersize=10, label='Origin', zorder=5)
        
        # Plot trajectory
        ax.plot(self.x_data, self.y_data, 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        
        # Plot current position
        ax.plot(self.x_data[-1], self.y_data[-1], 'ro', markersize=12, 
                label='Current Position', zorder=5)
        
        # Optional: Auto-adjust limits if robot goes outside map
        if len(self.x_data) > 5:
            x_min, x_max = min(self.x_data), max(self.x_data)
            y_min, y_max = min(self.y_data), max(self.y_data)
            
            # Only adjust if robot significantly exceeds boundaries
            if (x_max - x_min > self.map_size * 0.8 or 
                y_max - y_min > self.map_size * 0.8):
                margin = 1.0
                ax.set_xlim(x_min - margin, x_max + margin)
                ax.set_ylim(y_min - margin, y_max + margin)
        
        ax.legend(loc='upper right')
        
        # Use fixed filename (will overwrite previous file)
        filepath = os.path.join(self.output_dir, self.output_filename)
        
        # Save the plot
        plt.tight_layout()
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        self.get_logger().info(f'Trajectory plot saved: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()