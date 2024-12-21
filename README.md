# collision_avoidance_2dl_cbf

Collision Avoidance Assist Control with 2D LiDAR Control Barrier Function.

## ROS Interfaces

### Topics

Subscribe

| Topic name | Type |
| ---- | ---- |
| cmd_vel_in | geometry_msgs/msg/Twist |
| collision_polygon | geometry_msgs/msg/PolygonStamped |
| scan | sensor_msgs/msg/LaserScan |

Publish

| Topic name | Type |
| ---- | ---- |
| cmd_vel_out | geometry_msgs/msg/Twist |

### Parameters

| Parameter | Type | Default | Description |
| ---- | ---- | ---- | ---- |
| polygon_topic_name | String | "collision_polygon" | Change the topic name of Collision boundary shape |
| scan_topic_names | String | "scan" | Change the topic name of 2D LiDAR |
| cbf_param.K | Double | 0.1 | Parameter for CBF |
| cbf_param.C | Double | 0.1 | Parameter for CBF |
| cbf_param.L | Double | 0.001 | Parameter for CBF |

## Install

### Requirements

- Ubuntu 22.04 (on WSL also acceptable)
- ROS 2 Humble : <https://docs.ros.org/en/humble/Installation.html>
- rosdep : <https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html>

### Install commands

```bash
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

    ```bash
    ros2 launch collision_avoidance_2dl_cbf turtlebot3_example_launch.yaml
    ```

    Drugging on Mouse Teleop commands a reference input.

2. Command a static reference input.
    Terminal 1

    ```bash
    ros2 launch collision_avoidance_2dl_cbf turtlebot3_example_launch.yaml
    ```

    Terminal 2

    ```bash
    ros2 run collision_avoidance_2dl_cbf publish_u_ref_sample_node
    ```

3. Launch an actual robot and the CBF controller.  
    Refer <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>  
    Terminal 1

    ```bash
    ros2 launch collision_avoidance_2dl_cbf turtlebot3_example_launch.yaml use_sim_time:=false
    ```

    Terminal 2

    ```bash
    ros2 run collision_avoidance_2dl_cbf publish_u_ref_sample_node
    ```
