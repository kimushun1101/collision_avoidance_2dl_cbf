#!/bin/bash
if [[ "$1" == tb3* ]]; then
  ros2 bag record /cbf_debug /cmd_vel_ref /odom /collision_polygon /scan /cmd_vel -o $1
elif [[ "$1" == ais* ]]; then
  ros2 bag record /cbf_debug /cmd_vel_ref /odom /collision_polygon /scan /back_scan /suitcase/cmd_vel -o $1
else
  echo "Undefined arguments."
fi