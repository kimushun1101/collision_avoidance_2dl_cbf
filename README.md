# collision_avoidance_2dl_cbf

Collision Avoidance Assist Control with 2D LiDAR Control Barrier Function.

## Interfaces

### Subscribe
- cmd_vel_in
- scan

### Publish
- cmd_vel_out

### Parameters

- base_frame
- scan_frame
- gamma

### Services

- (unimplemented now) set_cbf_assist_enable
- (unimplemented now) get_cbf_assist_enable

## Install
```
export ROS_WORKSPACE=$HOME/ros2_ws
git clone https://github.com/kimushun1101/collision_avoidance_2dl_cbf.git $ROS_WORKSPACE/src/collision_avoidance_2dl_cbf
cd $ROS_WORKSPACE && rosdep install -y --from-paths src
colcon build --symlink-install
export LIBGL_ALWAYS_SOFTWARE=1
export TURTLEBOT3_MODEL=burger
```

## Sample

1. Launch Simulation and controller at the same time.
    ```
    ros2 launch collision_avoidance_2dl_cbf simulation_and_controller.launch.yaml
    ```

2. Launch controller with Params after launching simulation.
    Terminal 1
    ```
    ros2 launch collision_avoidance_2dl_cbf simulation_and_controller.launch.yaml
    ```
    Terminal 2
    ```
    ros2 run collision_avoidance_2dl_cbf collision_avoidance_2dl_cbf_node
    ```
