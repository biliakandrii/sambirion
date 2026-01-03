#!/usr/bin/env bash
set -e

PIDS=()

cleanup() {
  echo
  echo "Cleaning up ROS processes..."
  
  TTY=$(tty)
  ps -t "${TTY#/dev/}" -o pid=,comm= | while read pid cmd; do
    if [[ "$cmd" != "bash" && "$pid" != "$$" ]]; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done

  echo "Cleanup complete."
}

echo "Launching Gazebo..."
ros2 launch sambirion_bringup sambirion_gazebo.launch.py &
PIDS+=($!)

echo "Launching Nav2..."
ros2 launch sambirion_navigation nav2_bringup.launch.py &
PIDS+=($!)

echo "Waiting for /initialpose topic..."

until ros2 topic list 2>/dev/null | grep -q "^/initialpose$"; do
  sleep 1
done

echo "Publishing initial pose..."
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"

echo
echo "Processes running. Type 'exit' to stop all processes and end script."

while true; do
  read -r input
  if [[ "$input" == "test" ]]; then
    ros2 run sambirion_application moving_obstacle_node.py --ros-args \
      -p model_name:=obs1 \
      -p trajectory:=linear \
      -p linear_axis:=x \
      -p start_x:=0.0 -p start_y:=-3.0 \
      -p speed:=0.05 -p radius:=3.0 &

    ros2 run sambirion_application moving_obstacle_node.py --ros-args \
      -p model_name:=obs2 \
      -p trajectory:=linear \
      -p linear_axis:=x \
      -p start_x:=0.0 -p start_y:=3.0 \
      -p speed:=0.05 -p radius:=3.0 &

    ros2 run sambirion_application moving_obstacle_node.py --ros-args \
      -p model_name:=obs5 \
      -p trajectory:=square \
      -p start_x:=-1.0 -p start_y:=-1.0 \
      -p speed:=0.05 -p radius:=4.0 &

    sleep 10

    ros2 run sambirion_application goal_publisher.py --ros-args \
      -p goals:=0.06,-4.38,0.0,-3.93,-3.01,0.0,4.06,-2.98,0.0,-3.95,2.01,0.0,0.95,4.02,0.0,4.15,4.10,0.0,-3.94,-4.03,0.0 \
      -p goal_tolerance:=0.3 &
  fi
  if [[ "$input" == "exit" ]]; then 
    cleanup
    break
  fi
done


# ros2 run sambirion_application moving_obstacle_node.py --ros-args \
#   -p model_name:=obs3 \
#   -p trajectory:=linear \
#   -p linear_axis:=y \
#   -p start_x:=3.0 -p start_y:=0.0 \
#   -p speed:=0.05 -p radius:=3.0

# ros2 run sambirion_application moving_obstacle_node.py --ros-args \
#   -p model_name:=obs4 \
#   -p trajectory:=linear \
#   -p linear_axis:=y \
#   -p start_x:=-3.0 -p start_y:=0.0 \
#   -p speed:=0.05 -p radius:=3.0

