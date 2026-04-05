#!/usr/bin/env python3

import rclpy
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from state_estimator.base_state_estimator import BaseStateEstimator, TrackedObstacle

class UKFStateEstimator(BaseStateEstimator):
    def __init__(self):
        super().__init__('ukf_state_estimator')
        
        # 1. Load Parameters
        self.declare_parameter('process_noise_pos', 0.1)
        self.declare_parameter('process_noise_vel', 0.5)
        self.declare_parameter('measurement_noise_pos', 0.05)
        self.declare_parameter('measurement_noise_vel', 0.1)
        self.declare_parameter('alpha', 0.001)
        self.declare_parameter('beta', 2.0)
        self.declare_parameter('kappa', 0.0)
        
        self.p_noise_pos = self.get_parameter('process_noise_pos').value
        self.p_noise_vel = self.get_parameter('process_noise_vel').value
        self.m_noise_pos = self.get_parameter('measurement_noise_pos').value
        self.m_noise_vel = self.get_parameter('measurement_noise_vel').value
        
        # UKF Sigma Point Parameters
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.kappa = self.get_parameter('kappa').value
        
        self.get_logger().info('FilterPy UKF Estimator ready')

    def create_filter(self, initial_state, dt_setting=0.1):
        """Creates a fresh UKF instance for a new obstacle."""
        
        # A. Define the State Transition Function (fx)
        # Matches your _motion_model: [x, y, vx, vy] -> CV model
        def fx(x, dt):
            # x is [x, y, vx, vy]
            F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            return F @ x

        # B. Define Measurement Function (hx)
        # Assuming you measure [x, y, vx, vy] directly
        def hx(x):
            return x

        # C. Setup Sigma Points
        points = MerweScaledSigmaPoints(n=4, alpha=self.alpha, beta=self.beta, kappa=self.kappa)

        # D. Initialize Filter
        kf = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=dt_setting, fx=fx, hx=hx, points=points)
        
        # Initialize State
        kf.x = initial_state # [x, y, vx, vy]
        kf.P *= 1.0 # Initial Covariance
        
        # Process Noise (Q)
        kf.Q = np.diag([
            self.p_noise_pos**2, self.p_noise_pos**2,
            self.p_noise_vel**2, self.p_noise_vel**2
        ])
        
        # Measurement Noise (R)
        kf.R = np.diag([
            self.m_noise_pos**2, self.m_noise_pos**2,
            self.m_noise_vel**2, self.m_noise_vel**2
        ])
        
        return kf

    def initialize_obstacle(self, uid: int, measurement: np.ndarray, 
                            radius: float, time: float, is_segment: bool):
        if radius < 1.0:
            # Create the FilterPy object and store it INSTEAD of raw matrices
            ukf_filter = self.create_filter(measurement)
            
            self.tracked_obstacles[uid] = TrackedObstacle(
                uid=uid,
                last_update_time=time,
                state=ukf_filter,  # Storing the filter object itself here
                radius=radius,
                covariance=ukf_filter.P, 
                is_segment=is_segment
            )
            self.get_logger().info(f'Initialized Obstacle {uid}')

    def update_obstacle(self, uid: int, measurement: np.ndarray, time: float):
        obstacle = self.tracked_obstacles[uid]
        dt = time - obstacle.last_update_time
        
        if dt <= 0:
            return

        # 1. Prediction Step (FilterPy handles sigma points & weights internally)
        obstacle.state.predict(dt=dt)
        
        # 2. Update Step
        obstacle.state.update(measurement)
        
        obstacle.last_update_time = time

    def predict_obstacle(self, obstacle: TrackedObstacle, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        To predict without modifying the filter's actual state (for future trajectory viz),
        we must copy the filter or manually invoke the helper functions.
        FilterPy makes this harder to do 'statelessly', so we usually copy.
        """
        # Create a lightweight copy for prediction only
        import copy
        temp_filter = copy.deepcopy(obstacle.state)
        temp_filter.predict(dt=dt)
        
        return temp_filter.x, temp_filter.P

def main(args=None):
    rclpy.init(args=args)
    node = UKFStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()