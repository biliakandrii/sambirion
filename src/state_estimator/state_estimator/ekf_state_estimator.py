#!/usr/bin/env python3

import rclpy
import numpy as np
import copy
# Use standard tuple for Python 3.9+ typing
from typing import Tuple 
from filterpy.kalman import ExtendedKalmanFilter
from state_estimator.base_state_estimator import BaseStateEstimator, TrackedObstacle

class EKFStateEstimator(BaseStateEstimator):
    """
    Extended Kalman Filter based state estimator.
    Currently implements a Linear Constant Velocity model using EKF structure
    to allow for future non-linear upgrades.
    """
    
    def __init__(self):
        super().__init__('ekf_state_estimator')
        
        # 1. Load Parameters
        self.declare_parameter('process_noise_pos', 0.1)
        self.declare_parameter('process_noise_vel', 0.5)
        self.declare_parameter('measurement_noise_pos', 0.05)
        self.declare_parameter('measurement_noise_vel', 0.1)
        
        self.p_noise_pos = self.get_parameter('process_noise_pos').value
        self.p_noise_vel = self.get_parameter('process_noise_vel').value
        self.m_noise_pos = self.get_parameter('measurement_noise_pos').value
        self.m_noise_vel = self.get_parameter('measurement_noise_vel').value
        
        self.get_logger().info('FilterPy EKF Estimator ready')

    def create_filter(self, initial_state):
        """Creates a fresh EKF instance for a new obstacle."""
        
        # Initialize EKF (4 state variables, 4 measurement variables)
        ekf = ExtendedKalmanFilter(dim_x=4, dim_z=4)
        
        # Initialize State [x, y, vx, vy]
        ekf.x = initial_state
        
        # Initialize Covariance (P)
        ekf.P = np.diag([1.0, 1.0, 0.5, 0.5])
        
        # Initialize Measurement Noise (R)
        ekf.R = np.diag([
            self.m_noise_pos**2, self.m_noise_pos**2,
            self.m_noise_vel**2, self.m_noise_vel**2
        ])
        
        # Initialize Process Noise (Q)
        # Note: In a real EKF, Q is often updated based on dt, 
        # but we set a baseline diagonal here.
        ekf.Q = np.diag([
            self.p_noise_pos**2, self.p_noise_pos**2,
            self.p_noise_vel**2, self.p_noise_vel**2
        ])
        
        return ekf

    def initialize_obstacle(self, uid: int, measurement: np.ndarray, 
                            radius: float, time: float, is_segment: bool):
        if radius < 1.0:
            ekf_filter = self.create_filter(measurement)
            
            self.tracked_obstacles[uid] = TrackedObstacle(
                uid=uid,
                last_update_time=time,
                state=ekf_filter,        # Store the Filter object
                covariance=ekf_filter.P, # explicit covariance argument (FIX)
                radius=radius,
                is_segment=is_segment
            )
            self.get_logger().info(f'EKF Initialized Obstacle {uid}')

    def update_obstacle(self, uid: int, measurement: np.ndarray, time: float):
        obstacle = self.tracked_obstacles[uid]
        dt = time - obstacle.last_update_time
        
        if dt <= 0:
            return

        # --- Define EKF Functions ---
        # 1. State Transition Function x' = f(x)
        #    For CV model: new_pos = old_pos + vel*dt
        def f_move(x, dt):
            F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            return F @ x

        # 2. Jacobian of Transition Function (J_f)
        def F_jacobian(x, dt):
            return np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

        # 3. Measurement Function z = h(x)
        def h_meas(x):
            return x # We measure [x, y, vx, vy] directly

        # 4. Jacobian of Measurement Function (J_h)
        def H_jacobian(x):
            return np.eye(4)

        # --- Execution ---
        
        # Predict
        # We pass the functions and args explicitly to predict()
        obstacle.state.predict(dt=dt, u=0, B=None, F=F_jacobian, func=f_move)
        
        # Update
        obstacle.state.update(measurement, HJacobian=H_jacobian, Hx=h_meas)
        
        # Update timestamp
        obstacle.last_update_time = time

    def predict_obstacle(self, obstacle: TrackedObstacle, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Predicts state for trajectory visualization without altering the actual filter.
        """
        # Create a deep copy to simulate future without changing current state
        temp_filter = copy.deepcopy(obstacle.state)
        
        # Define the necessary transition functions for the copy
        def f_move(x, dt):
            F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            return F @ x

        def F_jacobian(x, dt):
            return np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

        # Execute prediction on the copy
        temp_filter.predict(dt=dt, u=0, F=F_jacobian, func=f_move)
        
        return temp_filter.x, temp_filter.P

def main(args=None):
    rclpy.init(args=args)
    node = EKFStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()