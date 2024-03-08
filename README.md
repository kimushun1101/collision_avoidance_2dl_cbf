# collision_avoidance_2dl_cbf

Collision Avoidance Assist Control with 2D LiDAR Control Barrier Function.

## Interfaces

### Subscribe
- cmd_vel_in

### Publish
- cmd_vel_out

### Parameters

| Parameter | Type | Default | Description |
| ---- | ---- | ---- | ---- |
| base_frame_name | String | "base_link" | TD |
| scan_topic_names | String array | ["scan"] | TD |
| gamma | Double | 1.0 |  TD |
| epsilon | Double | 0.0005 | TD |

## Install

### Requirements

- Ubuntu 22.04 (on WSL also acceptable)
- ROS 2 Humble : https://docs.ros.org/en/humble/Installation.html
- rosdep : https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html

### Install commands
```
export ROS_WORKSPACE=$HOME/ros2_ws
git clone https://github.com/kimushun1101/collision_avoidance_2dl_cbf.git $ROS_WORKSPACE/src/collision_avoidance_2dl_cbf
cd $ROS_WORKSPACE && rosdep install -y --from-paths src
colcon build --symlink-install
source $ROS_WORKSPACE/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export TURTLEBOT3_MODEL=burger
```

## Example
1. Launch Simulation and the CBF controller at the same time.
    ```
    ros2 launch collision_avoidance_2dl_cbf turtlebot3_example.launch.yaml
    ```
    Drugging on Mouse Teleop commands a reference input.

2. Command a static reference input.
    Terminal 1
    ```
    ros2 launch collision_avoidance_2dl_cbf turtlebot3_example.launch.yaml
    ```
    Terminal 2
    ```
    ros2 run collision_avoidance_2dl_cbf publish_u_ref_sample_node
    ```

3. Launch an actual robot and the CBF controller.
    Terminal 1
    ```
    ros2 launch collision_avoidance_2dl_cbf turtlebot3_example.launch.yaml use_sim_time:=false
    ```
    Terminal 2
    ```
    ros2 run collision_avoidance_2dl_cbf publish_u_ref_sample_node
    ```
