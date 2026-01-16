#!/usr/bin/env python3

import rclpy
import numpy as np
from typing import Tuple
from state_estimator.base_state_estimator import BaseStateEstimator, TrackedObstacle

class KFStateEstimator(BaseStateEstimator):
    """
    Kalman Filter based state estimator for obstacle prediction.
    State vector: [x, y, vx, vy]
    """
    
    def __init__(self):
        super().__init__('kf_state_estimator')
        
        # Load KF-specific parameters
        self.process_noise_pos = self.get_parameter('process_noise_pos').value
        self.process_noise_vel = self.get_parameter('process_noise_vel').value
        self.measurement_noise_pos = self.get_parameter('measurement_noise_pos').value
        self.measurement_noise_vel = self.get_parameter('measurement_noise_vel').value
        
        # Process noise covariance matrix Q
        self.Q = np.diag([
            self.process_noise_pos**2,  # x
            self.process_noise_pos**2,  # y
            self.process_noise_vel**2,  # vx
            self.process_noise_vel**2   # vy
        ])
        
        # Measurement noise covariance matrix R
        self.R = np.diag([
            self.measurement_noise_pos**2,  # x
            self.measurement_noise_pos**2,  # y
            self.measurement_noise_vel**2,  # vx
            self.measurement_noise_vel**2   # vy
        ])
        
        # Measurement matrix H (we observe all states directly)
        self.H = np.eye(4)
        
        self.get_logger().info('Kalman Filter State Estimator ready')
        self.get_logger().info(f'Process noise (pos/vel): {self.process_noise_pos}/{self.process_noise_vel}')
        self.get_logger().info(f'Measurement noise (pos/vel): {self.measurement_noise_pos}/{self.measurement_noise_vel}')
    
    def initialize_obstacle(self, uid: int, measurement: np.ndarray, 
                          radius: float, time: float, is_segment: bool):
        """Initialize a new tracked obstacle with Kalman Filter"""
        # Initial state from measurement
        if radius < 1.0:
            state = measurement.copy()
            
            # Initial covariance (high uncertainty)
            covariance = np.diag([1.0, 1.0, 0.5, 0.5])
            
            # Create tracked obstacle
            self.tracked_obstacles[uid] = TrackedObstacle(
                uid=uid,
                last_update_time=time,
                state=state,
                covariance=covariance,
                radius=radius,
                is_segment=is_segment
            )
            
            self.get_logger().debug(f'Initialized obstacle {uid} (segment={is_segment})')
    
    def update_obstacle(self, uid: int, measurement: np.ndarray, time: float):
        """Update an existing tracked obstacle using Kalman Filter"""
        obstacle = self.tracked_obstacles[uid]
        
        # Calculate time delta
        dt = time - obstacle.last_update_time
        
        if dt <= 0:
            self.get_logger().warn(f'Non-positive dt={dt} for obstacle {uid}')
            return
        
        # Prediction step
        F = self._get_state_transition_matrix(dt)
        
        # Predict state
        predicted_state = F @ obstacle.state
        
        # Predict covariance
        Q_scaled = self.Q * dt  # Scale process noise by time
        predicted_covariance = F @ obstacle.covariance @ F.T + Q_scaled
        
        # Update step (Kalman gain)
        S = self.H @ predicted_covariance @ self.H.T + self.R  # Innovation covariance
        
        try:
            K = predicted_covariance @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        except np.linalg.LinAlgError:
            self.get_logger().warn(f'Singular innovation covariance for obstacle {uid}')
            K = np.zeros((4, 4))
        
        # Innovation (measurement residual)
        innovation = measurement - self.H @ predicted_state
        
        # Update state
        obstacle.state = predicted_state + K @ innovation
        
        # Update covariance
        I = np.eye(4)
        obstacle.covariance = (I - K @ self.H) @ predicted_covariance
        
        # Update timestamp
        obstacle.last_update_time = time
        
        self.get_logger().debug(f'Updated obstacle {uid}: pos=({obstacle.state[0]:.2f}, {obstacle.state[1]:.2f}), '
                               f'vel=({obstacle.state[2]:.2f}, {obstacle.state[3]:.2f})')
    
    def predict_obstacle(self, obstacle: TrackedObstacle, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict obstacle state after time dt using constant velocity model
        Returns: (predicted_state, predicted_covariance)
        """
        # State transition matrix for constant velocity model
        F = self._get_state_transition_matrix(dt)
        
        # Predict state
        predicted_state = F @ obstacle.state
        
        # Predict covariance (uncertainty grows with time)
        Q_scaled = self.Q * dt
        predicted_covariance = F @ obstacle.covariance @ F.T + Q_scaled
        
        return predicted_state, predicted_covariance
    
    def _get_state_transition_matrix(self, dt: float) -> np.ndarray:
        """
        Get state transition matrix for constant velocity model
        x(t+dt) = x(t) + vx(t) * dt
        y(t+dt) = y(t) + vy(t) * dt
        vx(t+dt) = vx(t)
        vy(t+dt) = vy(t)
        """
        F = np.array([
            [1, 0, dt, 0],   # x = x + vx*dt
            [0, 1, 0, dt],   # y = y + vy*dt
            [0, 0, 1, 0],    # vx = vx
            [0, 0, 0, 1]     # vy = vy
        ])
        return F


def main(args=None):
    rclpy.init(args=args)
    
    node = KFStateEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()