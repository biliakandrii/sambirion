ros2 launch sambirion_navigation nav2_bringup.launch.py &
ros2 launch sambirion_bringup sambirion_gazebo.launch.py &
sleep 5
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}' &
