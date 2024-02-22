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
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <math.h>

class PublishCollisionPolygon : public rclcpp::Node
{
public:
  PublishCollisionPolygon();
  ~PublishCollisionPolygon();

private:
  struct Circle {
    double x;
    double y;
    double r;
  };

  void timer_callback();
  void publish_circle(const Circle& c, std::size_t plot_count, const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub);
  void publish_collision(const Circle& c1, const Circle& c2, std::size_t plot_count, const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub);

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr collision_poly_pub_, robot_poly_pub_, user_poly_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string base_frame_id_;
  int plot_count_;
  Circle robot_, user_;
};
