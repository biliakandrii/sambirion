# State Estimator for Obstacle Prediction

A ROS2 package for predicting future obstacle locations and generating cost heatmaps using Kalman Filtering.

## Overview

This package provides a modular framework for state estimation and obstacle prediction:

- **Base State Estimator**: Abstract base class handling ROS2 interface, obstacle tracking, and heatmap generation
- **Kalman Filter Estimator**: Implementation using a standard Kalman Filter with constant velocity model
- **Extensible Design**: Easy to add other estimators (EKF, UKF, Particle Filter, etc.)

## Features

- ✅ Tracks circular and segment obstacles
- ✅ Predicts future obstacle positions with uncertainty
- ✅ Generates probabilistic heatmaps with time-decaying costs
- ✅ Gaussian uncertainty spreading in predictions
- ✅ Fixed-rate operation independent of sensor frequency
- ✅ Fully configurable via ROS2 parameters
- ✅ Publishes standard `nav_msgs/OccupancyGrid` for costmap integration

## Package Structure

```
state_estimator/
├── state_estimator/
│   ├── __init__.py
│   ├── base_state_estimator.py    # Base class with common functionality
│   └── kf_state_estimator.py      # Kalman Filter implementation
├── launch/
│   └── kf_state_estimator_launch.py
├── config/
│   └── kf_state_estimator.yaml
├── setup.py
├── package.xml
└── README.md
```

## Installation

1. Clone into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <your-repo>
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build:
```bash
colcon build --packages-select state_estimator
source install/setup.bash
```

## Usage

### Running the Node

With launch file:
```bash
ros2 launch state_estimator kf_state_estimator_launch.py
```

Directly:
```bash
ros2 run state_estimator kf_state_estimator
```

With custom config:
```bash
ros2 run state_estimator kf_state_estimator --ros-args --params-file /path/to/config.yaml
```

### Visualizing in RViz

1. Add `Map` display
2. Set topic to `/predicted_obstacles_heatmap`
3. Set color scheme (e.g., costmap)

## Configuration Parameters

### Topics
- `input_topic`: Input obstacles topic (default: `/obstacles`)
- `output_topic`: Output heatmap topic (default: `/predicted_obstacles_heatmap`)

### Update Rate
- `update_rate`: Fixed update rate in Hz (default: 10.0)

### Grid Parameters
- `grid_resolution`: Cell size in meters (default: 0.1)
- `grid_width`: Total grid width in meters (default: 40.0)
- `grid_height`: Total grid height in meters (default: 40.0)
- `grid_origin_x`: Grid origin X coordinate (default: -20.0)
- `grid_origin_y`: Grid origin Y coordinate (default: -20.0)

### Prediction Parameters
- `prediction_horizon`: How far to predict in seconds (default: 5.0)
- `prediction_time_step`: Time between predictions in seconds (default: 0.5)

### Cost Parameters
- `max_cost`: Maximum cost for earliest predictions (default: 100)
- `min_cost`: Minimum cost for furthest predictions (default: 10)
- `cost_decay_rate`: Exponential decay rate (default: 0.5)

### Tracking Parameters
- `max_obstacle_age`: Remove obstacles not updated for this long (default: 2.0s)

### Kalman Filter Parameters
- `process_noise_pos`: Position process noise (default: 0.1)
- `process_noise_vel`: Velocity process noise (default: 0.5)
- `measurement_noise_pos`: Position measurement noise (default: 0.05)
- `measurement_noise_vel`: Velocity measurement noise (default: 0.1)

## How It Works

### State Estimation

The Kalman Filter tracks each obstacle with a 4D state vector:
```
State: [x, y, vx, vy]
```

**Prediction Model** (Constant Velocity):
```
x(t+dt) = x(t) + vx(t) * dt
y(t+dt) = y(t) + vy(t) * dt
vx(t+dt) = vx(t)
vy(t+dt) = vy(t)
```

### Heatmap Generation

1. **Multiple Time Steps**: Predicts obstacle positions at regular intervals up to `prediction_horizon`
2. **Time-Decaying Costs**: Earlier predictions have higher costs using exponential decay:
   ```
   cost(t) = min_cost + (max_cost - min_cost) * exp(-cost_decay_rate * t)
   ```
3. **Uncertainty Spreading**: Each prediction is represented as a Gaussian distribution
4. **Gaussian Addition**: Adds weighted Gaussians to the grid based on prediction uncertainty

### Obstacle Handling

- **Circle Obstacles**: Tracked with center position, velocity, and radius
- **Segment Obstacles**: Treated as single obstacles at segment center with average velocity and length/2 as radius
- **Tracking**: Uses unique IDs to maintain obstacle identity across frames
- **Stale Removal**: Removes obstacles not updated within `max_obstacle_age`

## Extending with New Estimators

To create a new estimator (e.g., Extended Kalman Filter):

1. Create a new file: `ekf_state_estimator.py`
2. Inherit from `BaseStateEstimator`
3. Implement required methods:
   - `initialize_obstacle()`
   - `update_obstacle()`
   - `predict_obstacle()`

Example:
```python
from base_state_estimator import BaseStateEstimator, TrackedObstacle

class EKFStateEstimator(BaseStateEstimator):
    def __init__(self):
        super().__init__('ekf_state_estimator')
        # Add EKF-specific initialization
    
    def initialize_obstacle(self, uid, measurement, radius, time, is_segment):
        # EKF initialization logic
        pass
    
    def update_obstacle(self, uid, measurement, time):
        # EKF update logic
        pass
    
    def predict_obstacle(self, obstacle, dt):
        # EKF prediction logic
        return predicted_state, predicted_covariance
```

4. Add entry point in `setup.py`
5. Create config file and launch file

## Integration with Costmap

The output `OccupancyGrid` can be used with `nav2_costmap_2d`:

1. Create a costmap layer plugin that subscribes to `/predicted_obstacles_heatmap`
2. Merge the predicted costs into the costmap
3. Use time-decay to prioritize immediate obstacles over distant predictions

## Troubleshooting

### No output heatmap
- Check if obstacles are being received: `ros2 topic echo /obstacles`
- Verify update rate: `ros2 param get /kf_state_estimator update_rate`
- Check if obstacles are within grid bounds

### High CPU usage
- Reduce `update_rate`
- Increase `grid_resolution` (larger cells)
- Reduce `prediction_horizon`
- Increase `prediction_time_step`

### Predictions too uncertain
- Decrease process noise parameters
- Increase measurement noise parameters (trust measurements more)

## Dependencies

- ROS2 (Humble or later)
- Python 3
- NumPy
- obstacle_detector package
- nav_msgs
- geometry_msgs

## License

Apache License 2.0

## Contributing

Contributions are welcome! Ideas for future estimators:
- Extended Kalman Filter (EKF) for non-linear motion
- Unscented Kalman Filter (UKF) for better non-linearity handling
- Particle Filter for multi-modal distributions
- IMM (Interacting Multiple Models) for switching motion models
- Neural network-based prediction
