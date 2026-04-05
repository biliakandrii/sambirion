"""
Launch file for state estimators with configurable estimator type.

Usage:
  ros2 launch state_estimator state_estimator_launch.py estimator_type:=kf
  ros2 launch state_estimator state_estimator_launch.py estimator_type:=ekf
  ros2 launch state_estimator state_estimator_launch.py estimator_type:=ukf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function that gets evaluated at launch time"""
    # Get the estimator type from context
    estimator_type = LaunchConfiguration('estimator_type').perform(context)
    
    # Common parameters for all estimators
    common_params = {
        'input_topic': '/obstacles',
        'output_topic': '/predicted_obstacles_heatmap',
        'odom_topic': '/odom',
        'update_rate': 1.0,
        
        # Grid parameters
        'grid_resolution': 0.05,
        'grid_width': 10.0,
        'grid_height': 10.0,
        'grid_origin_x': -5.0,
        'grid_origin_y': -5.0,
        
        # Cost parameters
        'max_cost': 250,
        'min_cost': 100,
        'cost_decay_rate': 0.1,
        
        # Tracking parameters
        'max_obstacle_age': 2.0,
        'full_update_interval': 10,
        
        # Rotation detection
        'angular_velocity_threshold': 0.3,
        'pause_predictions_on_rotation': True,
        'min_linear_velocity': 0.0,
    }
    
    # KF-specific parameters
    kf_params = {
        **common_params,
         # Prediction parameters
        'prediction_horizon': 20.0,
        'prediction_time_step': 2.0,
        'process_noise_pos': 0.1,
        'process_noise_vel': 0.5,
        'measurement_noise_pos': 0.05,
        'measurement_noise_vel': 0.1,
    }
    
    # EKF-specific parameters (CTRV model)
    ekf_params = {
        **common_params,
         # Prediction parameters
        'prediction_horizon': 10.0,
        'prediction_time_step': 0.5,
        'process_noise_pos': 0.1,
        'process_noise_vel': 0.3,
        'process_noise_yaw': 0.1,
        'process_noise_yaw_rate': 0.5,
        'measurement_noise_pos': 0.05,
        'measurement_noise_vel': 0.1,
    }
    
    # UKF-specific parameters (CTRV model)
    ukf_params = {
        **common_params,
         # Prediction parameters
        'prediction_horizon': 10.0,
        'prediction_time_step': 0.5,
        'process_noise_pos': 0.1,
        'process_noise_vel': 0.3,
        'process_noise_yaw': 0.1,
        'process_noise_yaw_rate': 0.5,
        'measurement_noise_pos': 0.05,
        'measurement_noise_vel': 0.1,
        
        # UKF tuning parameters
        'alpha': 0.01,
        'beta': 2.0,
        'kappa': 0.0,
    }
    
    # Node configurations based on estimator type
    node_configs = {
        'kf': {
            'package': 'state_estimator',
            'executable': 'kf_state_estimator',
            'name': 'kf_state_estimator',
            'parameters': [kf_params],
            'output': 'screen',
        },
        'ekf': {
            'package': 'state_estimator',
            'executable': 'ekf_state_estimator',
            'name': 'ekf_state_estimator',
            'parameters': [ekf_params],
            'output': 'screen',
        },
        'ukf': {
            'package': 'state_estimator',
            'executable': 'ukf_state_estimator',
            'name': 'ukf_state_estimator',
            'parameters': [ukf_params],
            'output': 'screen',
        },
    }
    
    # Validate estimator type
    valid_types = ['kf', 'ekf', 'ukf']
    if estimator_type not in valid_types:
        raise ValueError(
            f"Invalid estimator_type '{estimator_type}'. "
            f"Must be one of: {', '.join(valid_types)}"
        )
    
    # Create the selected node
    selected_node = Node(**node_configs[estimator_type])
    
    return [selected_node]


def generate_launch_description():
    """Generate launch description with conditional node launching"""
    
    # Declare arguments
    estimator_type_arg = DeclareLaunchArgument(
        'estimator_type',
        default_value='ekf',
        description='Type of state estimator: kf, ekf, or ukf',
        choices=['kf', 'ekf', 'ukf']
    )
    
    return LaunchDescription([
        estimator_type_arg,
        OpaqueFunction(function=launch_setup)
    ])