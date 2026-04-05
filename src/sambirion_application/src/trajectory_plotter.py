#!/usr/bin/env python3

"""
ROS2 Node: Enhanced Trajectory Plotter
Subscribes to robot odometry and plots the trajectory on a 10x10 map
Tracks:
- Path length traveled
- Minimum distance to moving obstacles
Saves plot every N seconds with unique timestamp from node start
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import os
from datetime import datetime
import time
import math
import json


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Parameters
        self.declare_parameter('topic_name', '/odom')
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('map_size', 10.0)  # 10x10 meters
        self.declare_parameter('output_dir', './plots/trajectory/')
        self.declare_parameter('save_interval', 3.0)  # seconds
        self.declare_parameter('obstacle_topics', [''])  # Empty list means auto-discover
        
        # Get parameters
        topic_name = self.get_parameter('topic_name').value
        self.max_points = self.get_parameter('max_points').value
        self.map_size = self.get_parameter('map_size').value
        self.output_dir = self.get_parameter('output_dir').value
        self.save_interval = self.get_parameter('save_interval').value
        obstacle_topics_param = self.get_parameter('obstacle_topics').value
        
        # If obstacle topics is a string, convert to list
        if isinstance(obstacle_topics_param, str):
            self.obstacle_topics = [obstacle_topics_param] if obstacle_topics_param else []
        else:
            self.obstacle_topics = [t for t in obstacle_topics_param if t]  # Filter empty strings
        
        # Generate unique run ID with timestamp
        self.run_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_id = f"run_{self.run_timestamp}"
        
        # JSON file for all runs (append mode)
        self.json_filename = 'trajectory_data.json'
        self.json_filepath = os.path.join(self.output_dir, self.json_filename)
        
        # PNG file for current run visualization
        self.output_filename = f'trajectory_{self.run_timestamp}.png'
        
        # Storage for trajectory points
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        
        # Storage for path length calculation
        self.last_position = None
        self.total_path_length = 0.0
        
        # Storage for obstacle tracking
        self.obstacle_positions = {}  # Dict of {topic_name: (x, y, z)}
        self.min_obstacle_distance = float('inf')
        self.min_distance_timestamp = None
        self.distance_history = deque(maxlen=self.max_points)
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create subscriber for odometry
        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odom_callback,
            10
        )
        
        # Create subscribers for obstacle markers
        self.obstacle_subscribers = []
        
        # If no specific topics provided, we'll rely on auto-discovery
        if len(self.obstacle_topics) == 0:
            self.get_logger().info('No specific obstacle topics provided - will auto-discover')
        
        for obstacle_topic in self.obstacle_topics:
            # Ensure topic starts with /
            if not obstacle_topic.startswith('/'):
                obstacle_topic = '/' + obstacle_topic
            
            # Create a unique callback for each topic
            def create_callback(topic_name):
                def callback(msg):
                    self.obstacle_callback(msg, topic_name)
                return callback
            
            sub = self.create_subscription(
                Marker,
                obstacle_topic,
                create_callback(obstacle_topic),
                10
            )
            self.obstacle_subscribers.append(sub)
            self.get_logger().info(f'Subscribed to obstacle marker: {obstacle_topic}')
        
        # Timer to discover topics dynamically
        self.discovery_timer = self.create_timer(2.0, self.discover_obstacle_topics)
        self.discovered_topics = set()
        
        # Timer for saving plots
        self.save_timer = self.create_timer(self.save_interval, self.save_plot)
        
        self.get_logger().info(f'Trajectory Plotter started, subscribing to {topic_name}')
        self.get_logger().info(f'Map size: {self.map_size}x{self.map_size} meters')
        self.get_logger().info(f'Saving to: {os.path.abspath(self.output_dir)}/{self.output_filename}')
        self.get_logger().info(f'Save interval: {self.save_interval} seconds')
        self.get_logger().info(f'Tracking {len(self.obstacle_topics)} obstacle topics')
        self.get_logger().info(f'Run ID: {self.run_id}')
        self.get_logger().info(f'JSON data file: {self.json_filepath}')
    
    def load_json_data(self):
        """Load existing JSON data if file exists"""
        if os.path.exists(self.json_filepath):
            try:
                with open(self.json_filepath, 'r') as f:
                    data = json.load(f)
                    self.get_logger().info(f'Loaded existing data with {len(data.get("runs", []))} previous runs')
                    return data
            except Exception as e:
                self.get_logger().warn(f'Could not load existing JSON: {e}')
                return {"runs": []}
        else:
            self.get_logger().info('No existing JSON file, will create new one')
            return {"runs": []}
        
    def discover_obstacle_topics(self):
        """Discover obstacle marker topics dynamically"""
        topic_names_and_types = self.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_names_and_types:
            # Look for visualization_msgs/msg/Marker topics that contain "moving_obstacle_marker"
            if 'visualization_msgs/msg/Marker' in topic_types:
                if 'moving_obstacle_marker' in topic_name and topic_name not in self.discovered_topics:
                    self.get_logger().info(f'Discovered new obstacle topic: {topic_name}')
                    self.discovered_topics.add(topic_name)
                    
                    # Create subscriber for this topic
                    def create_callback(topic):
                        def callback(msg):
                            self.obstacle_callback(msg, topic)
                        return callback
                    
                    sub = self.create_subscription(
                        Marker,
                        topic_name,
                        create_callback(topic_name),
                        10
                    )
                    self.obstacle_subscribers.append(sub)
                    self.get_logger().info(f'Subscribed to discovered topic: {topic_name}')
    
    def obstacle_callback(self, msg, topic_name):
        """Callback for obstacle marker messages"""
        # Store the obstacle position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Log first time we see this obstacle
        if topic_name not in self.obstacle_positions:
            self.get_logger().info(f'First obstacle detection from {topic_name} at ({x:.3f}, {y:.3f}, {z:.3f})')
        
        self.obstacle_positions[topic_name] = (x, y, z)
        
        # Calculate distance to current robot position if we have one
        if len(self.x_data) > 0:
            robot_x = self.x_data[-1]
            robot_y = self.y_data[-1]
            
            distance = math.sqrt((robot_x - x)**2 + (robot_y - y)**2)
            
            # Update minimum distance
            if distance < self.min_obstacle_distance:
                self.min_obstacle_distance = distance
                self.min_distance_timestamp = self.get_clock().now()
                self.get_logger().info(
                    f'New minimum distance to obstacle: {distance:.3f} m (topic: {topic_name}, robot: ({robot_x:.3f}, {robot_y:.3f}), obstacle: ({x:.3f}, {y:.3f}))'
                )
        
    def odom_callback(self, msg):
        """Callback for odometry messages"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate path length increment
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            # Sanity check: ignore unrealistic jumps (likely from reset or teleportation)
            # If robot moves more than 1 meter in one odometry update, it's likely an error
            if distance < 1.0:
                self.total_path_length += distance
            else:
                self.get_logger().warn(f'Large position jump detected: {distance:.3f}m - not added to path length')
        else:
            # First position received
            self.get_logger().info(f'Initial robot position: X={x:.3f}, Y={y:.3f}')
        
        # Update last position
        self.last_position = (x, y)
        
        # Store the position
        self.x_data.append(x)
        self.y_data.append(y)
        
        # Calculate current minimum distance to all obstacles
        current_min_dist = float('inf')
        for obstacle_topic, (obs_x, obs_y, obs_z) in self.obstacle_positions.items():
            distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            current_min_dist = min(current_min_dist, distance)
        
        # Store distance history
        if current_min_dist != float('inf'):
            self.distance_history.append(current_min_dist)
            
            # Update global minimum
            if current_min_dist < self.min_obstacle_distance:
                self.min_obstacle_distance = current_min_dist
                self.min_distance_timestamp = self.get_clock().now()
        
    def save_plot(self):
        """Create and save the trajectory plot"""
        if len(self.x_data) == 0:
            self.get_logger().warn('No trajectory data collected, no plot saved')
            return
        
        # Create figure with single trajectory plot
        fig, ax_traj = plt.subplots(figsize=(8, 8))
        
        # Set up the plot limits (10x10 map)
        ax_traj.set_xlim(-self.map_size/2, self.map_size/2)
        ax_traj.set_ylim(-self.map_size/2, self.map_size/2)
        
        # Grid and labels
        ax_traj.grid(True, alpha=0.3)
        ax_traj.set_xlabel('X Position (m)', fontsize=12)
        ax_traj.set_ylabel('Y Position (m)', fontsize=12)
        
        # Title with timestamp and point count
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        ax_traj.set_title(f'Robot Trajectory (10x10 Map)\n{timestamp}\nPoints: {len(self.x_data)}',
                         fontsize=14, fontweight='bold')
        ax_traj.set_aspect('equal')
        
        # Add origin marker
        ax_traj.plot(0, 0, 'go', markersize=10, label='Origin', zorder=5)
        
        # Plot trajectory
        ax_traj.plot(self.x_data, self.y_data, 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        
        # Plot current position
        ax_traj.plot(self.x_data[-1], self.y_data[-1], 'ro', markersize=12,
                    label='Current Position', zorder=5)
        
        # Plot obstacle positions
        for i, (topic, (x, y, z)) in enumerate(self.obstacle_positions.items()):
            color = ['red', 'orange', 'purple', 'brown', 'pink'][i % 5]
            ax_traj.plot(x, y, 'x', color=color, markersize=12,
                        markeredgewidth=3, label=f'Obstacle {i+1}', zorder=6)
        
        # Optional: Auto-adjust limits if robot goes outside map
        if len(self.x_data) > 5:
            x_min, x_max = min(self.x_data), max(self.x_data)
            y_min, y_max = min(self.y_data), max(self.y_data)
            
            # Only adjust if robot significantly exceeds boundaries
            if (x_max - x_min > self.map_size * 0.8 or
                y_max - y_min > self.map_size * 0.8):
                margin = 1.0
                ax_traj.set_xlim(x_min - margin, x_max + margin)
                ax_traj.set_ylim(y_min - margin, y_max + margin)
        
        ax_traj.legend(loc='upper right', fontsize=9)
        
        # Use fixed filename (will overwrite previous file)
        filepath = os.path.join(self.output_dir, self.output_filename)
        
        # Save the plot
        plt.tight_layout()
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        self.get_logger().info(f'Trajectory plot saved: {filepath}')
        
        # Save statistics to JSON file
        self.save_to_json()
    
    def save_to_json(self):
        """Save trajectory statistics to JSON file (append mode)"""
        # Load existing data
        all_data = self.load_json_data()
        
        # Prepare current run data
        run_data = {
            "run_id": self.run_id,
            "metrics": {
                "total_path_length_m": round(self.total_path_length, 3),
                "min_obstacle_distance_m": round(self.min_obstacle_distance, 3) if self.min_obstacle_distance != float('inf') else None,
                "straight_line_distance_m": round(
                    math.sqrt(
                        (self.x_data[-1] - self.x_data[0])**2 + 
                        (self.y_data[-1] - self.y_data[0])**2
                    ), 3
                ) if len(self.x_data) > 1 else 0.0
            },
            "plot_filename": self.output_filename
        }
        
        # Find if this run already exists (update scenario)
        existing_run_idx = None
        for idx, run in enumerate(all_data["runs"]):
            if run["run_id"] == self.run_id:
                existing_run_idx = idx
                break
        
        if existing_run_idx is not None:
            # Update existing run
            all_data["runs"][existing_run_idx] = run_data
            self.get_logger().info(f'Updated existing run in JSON: {self.run_id}')
        else:
            # Append new run
            all_data["runs"].append(run_data)
            self.get_logger().info(f'Added new run to JSON: {self.run_id}')
        
        # Save to file
        try:
            with open(self.json_filepath, 'w') as f:
                json.dump(all_data, f, indent=2)
            self.get_logger().info(f'JSON data saved: {self.json_filepath} (total runs: {len(all_data["runs"])})')
        except Exception as e:
            self.get_logger().error(f'Failed to save JSON: {e}')


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