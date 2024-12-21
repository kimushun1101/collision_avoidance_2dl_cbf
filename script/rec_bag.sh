#!/bin/bash
output_directory=$(cd "$(dirname "$0")" && pwd)/../result
mkdir -p ${output_directory}
current_time=$(date '+%Y-%m-%d-%H-%M-%S')
ros2 bag record -o ${output_directory}/turtlebot3_${current_time} \
  /cbf_debug \
  cbf_debug_collision \
  /cmd_vel \
  /cmd_vel_ref \
  /odom \
  /collision_polygon \
  /scan \
  /tf \
  /tf_static

  # The script above records the following topics: 
  
  # /cbf_debug 
  # /cmd_vel_ref 
  # /odom 
  # /collision_polygon 
  # /scan 
  # /cmd_vel 
  
  # The recorded bag file is saved in the same directory as the script. 
  # The following is the command to run the script: 
  # $ bash rec_bag.sh
  
  # The recorded bag file is saved in the same directory as the script.
