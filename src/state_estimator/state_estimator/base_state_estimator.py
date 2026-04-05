#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


@dataclass
class TrackedObstacle:
    """Base class for tracked obstacles"""
    uid: int
    last_update_time: float
    state: np.ndarray  # State vector (implementation dependent)
    covariance: np.ndarray  # Covariance matrix
    radius: float
    is_segment: bool = False


class BaseStateEstimator(Node, ABC):
    """
    Base class for state estimators that predict obstacle future locations
    and generate heatmaps for costmap integration.
    
    Includes rotation detection to pause predictions during radical turns.
    """
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # Declare parameters
        self._declare_parameters()
        
        # Load parameters
        self._load_parameters()
        
        # Initialize tracking dictionary
        self.tracked_obstacles: Dict[int, TrackedObstacle] = {}
        
        # Robot motion tracking
        self.robot_angular_velocity = 0.0
        self.robot_linear_velocity = 0.0
        self.is_robot_turning = False
        
        # Subscribe to obstacles
        self.obstacles_sub = self.create_subscription(
            self._get_obstacles_msg_type(),
            self.input_topic,
            self.obstacles_callback,
            10
        )
        
        # Subscribe to odometry for rotation detection
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # QoS profiles for publishers
        qos_profile_transient = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        qos_profile_volatile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Publisher for full heatmap (transient local for late joiners)
        self.heatmap_pub = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            qos_profile_transient
        )
        
        # Publisher for incremental updates (volatile, best effort for efficiency)
        self.heatmap_update_pub = self.create_publisher(
            OccupancyGridUpdate,
            self.output_topic + '_updates',
            qos_profile_volatile
        )
        
        # Timer for fixed rate updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_callback
        )
        
        # Store previous heatmap for computing updates
        self.previous_heatmap = None
        
        # Counter for periodic full updates
        self.update_counter = 0
        
        self.last_obstacles_msg = None
        self.frame_id = "map"
        
        self.get_logger().info(f'{node_name} initialized')
        self.get_logger().info(f'Grid resolution: {self.grid_resolution}m')
        self.get_logger().info(f'Prediction horizon: {self.prediction_horizon}s')
        self.get_logger().info(f'Update rate: {self.update_rate}Hz')
        self.get_logger().info(f'Rotation threshold: {self.angular_velocity_threshold} rad/s')
        self.get_logger().info(f'Publishing updates to: {self.output_topic}_updates')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        # Input/Output topics
        self.declare_parameter('input_topic', '/obstacles')
        self.declare_parameter('output_topic', '/predicted_obstacles_heatmap')
        self.declare_parameter('odom_topic', '/odom')
        
        # Update rate
        self.declare_parameter('update_rate', 10.0)
        
        # Grid parameters
        self.declare_parameter('grid_resolution', 0.1)  # meters per cell
        self.declare_parameter('grid_width', 40.0)  # meters
        self.declare_parameter('grid_height', 40.0)  # meters
        self.declare_parameter('grid_origin_x', -20.0)  # meters
        self.declare_parameter('grid_origin_y', -20.0)  # meters
        
        # Prediction parameters
        self.declare_parameter('prediction_horizon', 5.0)  # seconds
        self.declare_parameter('prediction_time_step', 0.5)  # seconds
        
        # Cost parameters
        self.declare_parameter('max_cost', 100)  # Maximum cost value (0-100 for OccupancyGrid)
        self.declare_parameter('min_cost', 10)  # Minimum cost value
        self.declare_parameter('cost_decay_rate', 0.5)  # Exponential decay rate
        
        # Tracking parameters
        self.declare_parameter('max_obstacle_age', 2.0)  # seconds
        
        # Update publishing parameters
        self.declare_parameter('full_update_interval', 10)  # Publish full grid every N updates
        
        # Rotation detection parameters
        self.declare_parameter('angular_velocity_threshold', 0.3)  # rad/s 
        self.declare_parameter('pause_predictions_on_rotation', True)  # Enable/disable feature
        self.declare_parameter('min_linear_velocity', 0.0)  # m/s - only check rotation if moving
    
    def _load_parameters(self):
        """Load parameters from ROS2 parameter server"""
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.grid_origin_x = self.get_parameter('grid_origin_x').value
        self.grid_origin_y = self.get_parameter('grid_origin_y').value
        
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.prediction_time_step = self.get_parameter('prediction_time_step').value
        
        self.max_cost = self.get_parameter('max_cost').value
        self.min_cost = self.get_parameter('min_cost').value
        self.cost_decay_rate = self.get_parameter('cost_decay_rate').value
        
        self.max_obstacle_age = self.get_parameter('max_obstacle_age').value
        self.full_update_interval = self.get_parameter('full_update_interval').value
        
        # Rotation detection parameters
        self.angular_velocity_threshold = self.get_parameter('angular_velocity_threshold').value
        self.pause_predictions_on_rotation = self.get_parameter('pause_predictions_on_rotation').value
        self.min_linear_velocity = self.get_parameter('min_linear_velocity').value
        
        # Calculate grid dimensions
        self.grid_width_cells = int(self.grid_width / self.grid_resolution)
        self.grid_height_cells = int(self.grid_height / self.grid_resolution)
        
        # Prediction time steps
        self.num_prediction_steps = int(self.prediction_horizon / self.prediction_time_step)
    
    def _get_obstacles_msg_type(self):
        """Import and return the Obstacles message type"""
        try:
            from obstacle_detector.msg import Obstacles
            return Obstacles
        except ImportError:
            self.get_logger().error("Could not import obstacle_detector messages")
            # Fallback for testing without the actual package
            return type('Obstacles', (), {})
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry messages to detect robot rotation"""
        # Extract angular velocity (rotation around z-axis)
        self.robot_angular_velocity = abs(msg.twist.twist.angular.z)
        
        # Extract linear velocity (magnitude)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_linear_velocity = math.sqrt(vx**2 + vy**2)
        
        # Determine if robot is turning radically
        # Only consider it a turn if the robot is also moving forward
        was_turning = self.is_robot_turning
        
        if self.pause_predictions_on_rotation:
            if self.robot_linear_velocity > self.min_linear_velocity:
                self.is_robot_turning = self.robot_angular_velocity > self.angular_velocity_threshold
            else:
                # If robot is stationary, don't pause predictions
                self.is_robot_turning = False
        else:
            self.is_robot_turning = False
        
        # Log state changes
        if self.is_robot_turning != was_turning:
            if self.is_robot_turning:
                self.get_logger().info(
                    f'Robot turning detected (ω={self.robot_angular_velocity:.3f} rad/s) - '
                    f'pausing predictions'
                )
            else:
                self.get_logger().info('Robot turn complete - resuming predictions')
    
    def obstacles_callback(self, msg):
        """Callback for obstacles messages"""
        self.last_obstacles_msg = msg
        self.frame_id = msg.header.frame_id
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Update tracked obstacles with new measurements
        self._update_tracked_obstacles(msg, current_time)
    
    def _update_tracked_obstacles(self, msg, current_time: float):
        """Update tracked obstacles with new measurements"""
        updated_ids = set()
        
        # Process circle obstacles
        for circle in msg.circles:
            uid = circle.uid if hasattr(circle, 'uid') else hash((circle.center.x, circle.center.y))
            
            measurement = np.array([
                circle.center.x,
                circle.center.y,
                circle.velocity.x,
                circle.velocity.y
            ])
            
            if uid in self.tracked_obstacles:
                # Update existing obstacle
                self.update_obstacle(uid, measurement, current_time)
            else:
                # Initialize new obstacle
                self.initialize_obstacle(uid, measurement, circle.radius, current_time, is_segment=False)
            
            updated_ids.add(uid)
        
        # Process segment obstacles
        for segment in msg.segments:
            uid = segment.uid
            
            # Calculate center point and average velocity
            center_x = (segment.first_point.x + segment.last_point.x) / 2.0
            center_y = (segment.first_point.y + segment.last_point.y) / 2.0
            avg_vx = (segment.first_velocity.x + segment.last_velocity.x) / 2.0
            avg_vy = (segment.first_velocity.y + segment.last_velocity.y) / 2.0
            
            # Calculate segment length as radius
            length = math.sqrt(
                (segment.last_point.x - segment.first_point.x)**2 +
                (segment.last_point.y - segment.first_point.y)**2
            )
            radius = length / 2.0
            
            measurement = np.array([center_x, center_y, avg_vx, avg_vy])
            
            if uid in self.tracked_obstacles:
                self.update_obstacle(uid, measurement, current_time)
            else:
                self.initialize_obstacle(uid, measurement, radius, current_time, is_segment=True)
            
            updated_ids.add(uid)
        
        # Remove stale obstacles
        self._remove_stale_obstacles(current_time, updated_ids)
    
    def _remove_stale_obstacles(self, current_time: float, updated_ids: set):
        """Remove obstacles that haven't been updated recently"""
        to_remove = []
        for uid, obstacle in self.tracked_obstacles.items():
            if uid not in updated_ids:
                age = current_time - obstacle.last_update_time
                if age > self.max_obstacle_age:
                    to_remove.append(uid)
        
        for uid in to_remove:
            del self.tracked_obstacles[uid]
            self.get_logger().debug(f'Removed stale obstacle {uid}')
    
    @abstractmethod
    def initialize_obstacle(self, uid: int, measurement: np.ndarray, 
                          radius: float, time: float, is_segment: bool):
        """Initialize a new tracked obstacle (implementation specific)"""
        pass
    
    @abstractmethod
    def update_obstacle(self, uid: int, measurement: np.ndarray, time: float):
        """Update an existing tracked obstacle (implementation specific)"""
        pass
    
    @abstractmethod
    def predict_obstacle(self, obstacle: TrackedObstacle, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict obstacle state after time dt
        Returns: (predicted_state, predicted_covariance)
        """
        pass
    
    def update_callback(self):
        """Fixed rate update callback"""
        # Check if robot is turning radically
        if self.is_robot_turning:
            # Publish empty heatmap during turns
            self.get_logger().debug(
                f'Skipping prediction - robot turning (ω={self.robot_angular_velocity:.3f} rad/s)'
            )
            # empty_heatmap = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.float32)
            # self._publish_heatmap(empty_heatmap)
            return
        
        if not self.tracked_obstacles:
            # Publish empty heatmap
            empty_heatmap = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.float32)
            self._publish_heatmap(empty_heatmap)
            return
        
        # Generate heatmap with predictions
        heatmap = self._generate_heatmap()
        
        # Publish heatmap (full or update)
        self._publish_heatmap(heatmap)
        
        self.update_counter += 1
    
    def _generate_heatmap(self) -> np.ndarray:
        """Generate heatmap from predicted obstacle positions"""
        heatmap = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.float32)
        
        # For each tracked obstacle
        for obstacle in self.tracked_obstacles.values():
            # Predict at multiple time steps
            for step in range(self.num_prediction_steps):
                dt = (step + 1) * self.prediction_time_step
                
                # Get prediction
                pred_state, pred_cov = self.predict_obstacle(obstacle, dt)
                
                # Extract position (assumes first 2 elements are x, y)
                pos_x = pred_state[0]
                pos_y = pred_state[1]
                
                # Extract position uncertainty (assumes 2x2 top-left of covariance)
                cov_xx = pred_cov[0, 0]
                cov_yy = pred_cov[1, 1]
                cov_xy = pred_cov[0, 1]
                
                # Calculate cost based on time delta (exponential decay)
                time_factor = np.exp(-self.cost_decay_rate * dt)
                cost = self.min_cost + (self.max_cost - self.min_cost) * time_factor
                
                # Add Gaussian distribution to heatmap
                if step > 2:
                    self._add_gaussian_to_heatmap(
                        heatmap, pos_x, pos_y, 
                        cov_xx, cov_yy, cov_xy,
                        obstacle.radius, cost
                    )
        
        # Clip values to valid range
        heatmap = np.clip(heatmap, 0, 100)
        
        return heatmap
    
    def _add_gaussian_to_heatmap(self, heatmap: np.ndarray, 
                                 pos_x: float, pos_y: float,
                                 cov_xx: float, cov_yy: float, cov_xy: float,
                                 radius: float, cost: float):
        """Add Gaussian distribution centered at position to heatmap"""
        # Convert position to grid coordinates
        grid_x = int((pos_x - self.grid_origin_x) / self.grid_resolution)
        grid_y = int((pos_y - self.grid_origin_y) / self.grid_resolution)
        
        # Use only obstacle radius, ignore uncertainty
        sigma_x = radius/12
        sigma_y = radius/12
        
        # Determine region of influence (3-sigma rule)
        extent_x = int(np.ceil(3 * sigma_x / self.grid_resolution))
        extent_y = int(np.ceil(3 * sigma_y / self.grid_resolution))
        
        # Calculate bounds
        x_min = max(0, grid_x - extent_x)
        x_max = min(self.grid_width_cells, grid_x + extent_x + 1)
        y_min = max(0, grid_y - extent_y)
        y_max = min(self.grid_height_cells, grid_y + extent_y + 1)
        
        # Create covariance matrix
        cov_matrix = np.array([[radius**2, 0],
                       [0, radius**2]])
        # Add small value to diagonal for numerical stability
        cov_matrix += np.eye(2) * 1e-6
        
        try:
            cov_inv = np.linalg.inv(cov_matrix)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular covariance matrix, using identity")
            cov_inv = np.eye(2)
        
        # Add Gaussian to each cell in region
        for i in range(y_min, y_max):
            for j in range(x_min, x_max):
                # Convert grid cell to world coordinates (cell center)
                cell_x = self.grid_origin_x + (j + 0.5) * self.grid_resolution
                cell_y = self.grid_origin_y + (i + 0.5) * self.grid_resolution
                
                # Calculate difference vector
                diff = np.array([cell_x - pos_x, cell_y - pos_y])
                
                # Calculate Mahalanobis distance
                mahal_dist = np.sqrt(diff.T @ cov_inv @ diff)
                
                # Gaussian probability (unnormalized)
                prob = np.exp(-0.5 * mahal_dist**2)
                
                # Add weighted cost to heatmap (max operation to keep highest cost)
                heatmap[i, j] = max(heatmap[i, j], cost * prob)
    
    def _compute_update_bounds(self, current_heatmap: np.ndarray, 
                              previous_heatmap: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        Compute the bounding box of changed cells
        Returns: (x_min, y_min, width, height) or None if no changes
        """
        # Find all cells that have changed
        diff = np.abs(current_heatmap - previous_heatmap) > 0.5  # Small threshold for float comparison
        
        if not np.any(diff):
            return None
        
        # Find bounding box of changes
        changed_rows = np.any(diff, axis=1)
        changed_cols = np.any(diff, axis=0)
        
        y_indices = np.where(changed_rows)[0]
        x_indices = np.where(changed_cols)[0]
        
        if len(y_indices) == 0 or len(x_indices) == 0:
            return None
        
        y_min = int(y_indices[0])
        y_max = int(y_indices[-1])
        x_min = int(x_indices[0])
        x_max = int(x_indices[-1])
        
        width = x_max - x_min + 1
        height = y_max - y_min + 1
        
        return (x_min, y_min, width, height)

    def _publish_heatmap(self, heatmap: np.ndarray):
        """Publish heatmap as OccupancyGrid every update cycle"""
        current_time = self.get_clock().now()
        
        # Always publish full OccupancyGrid at update rate
        msg = OccupancyGrid()
        
        # Header
        msg.header = Header()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = self.frame_id
        
        # Map metadata
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_width_cells
        msg.info.height = self.grid_height_cells
        
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert heatmap to OccupancyGrid format (row-major order, int8)
        data = heatmap.flatten().astype(np.int8)
        msg.data = data.tolist()
        
        self.heatmap_pub.publish(msg)
        self.get_logger().debug('Published full OccupancyGrid')
        
        # Store current heatmap for next iteration
        self.previous_heatmap = heatmap.copy()