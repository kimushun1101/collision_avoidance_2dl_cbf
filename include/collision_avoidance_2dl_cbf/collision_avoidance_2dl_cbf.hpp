//  Copyright 2024 Shunsuke Kimura

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <functional>
#include <memory>
#include <chrono>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include <map>

// for debug
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class CollisionAvoidance2dlCBF : public rclcpp::Node
{
public:
  CollisionAvoidance2dlCBF();
  ~CollisionAvoidance2dlCBF();

private:
  struct Point {
    double x;
    double y;
  };

  struct Scan {
    Point BtoS;
    double BtoS_yaw;
    std::vector<Point> BtoP;
  };

  void cmd_vel_inCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void collision_polygonCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg);
  void publishAssistInput();
  bool summationCBFs(const std::vector<Point> BtoP_, double& B, double& LgB1, double& LgB2);
  bool calculateLineIntersection(const Point& p1, const Point& p2, const Point& p3, const Point& p4, Point& intersection);
  bool calculatePolygonIntersection(const Point& target, Point& intersection, std::size_t& number);
  // bool getlookupTransform(const std::string& source, const std::string& target, Point& xy, double& yaw);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_values_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_collision_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr cmd_vel_in_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::ConstSharedPtr collision_poly_sub_;

  double cbf_param_K_, cbf_param_C_, cbf_param_L_;
  std::string base_frame_name_;
  std::vector<Point> collision_poly_;
  Scan detected_point_;
  double u_ref1_, u_ref2_;
  bool is_debug_;
};
