#!/bin/bash
output_directory=$(cd "$(dirname "$0")" && pwd)/../result
mkdir -p ${output_directory}
current_time=$(date '+%Y-%m-%d-%H-%M-%S')
ros2 bag record -o ${output_directory}/turtlebot3_${current_time} \
  /cbf_debug \
  /cbf_debug_collision \
  /cmd_vel \
  /cmd_vel_ref \
  /odom \
  /collision_polygon \
  /scan \
  /tf \
  /tf_static
