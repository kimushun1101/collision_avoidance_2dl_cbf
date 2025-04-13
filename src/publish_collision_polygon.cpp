//  Copyright 2023 Shunsuke Kimura

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.


#include "collision_avoidance_2dl_cbf/publish_collision_polygon.hpp"

PublishCollisionPolygon::PublishCollisionPolygon() : Node("publish_collision_polygon")
{
  this->declare_parameter("base_frame_id", "base_link");
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  this->declare_parameter("plot_count", 50);
  this->declare_parameter("circle.x", -0.032);
  circle_.x = this->get_parameter("circle.x").as_double();
  this->declare_parameter("circle.y", 0.0);
  circle_.y = this->get_parameter("circle.y").as_double();
  this->declare_parameter("circle.r", 0.35);
  circle_.r = this->get_parameter("circle.r").as_double();

  int plot_count_param = this->get_parameter("plot_count").as_int();
  if (plot_count_param < 0) {
    RCLCPP_ERROR(this->get_logger(), "plot_count must be non-negative!");
    throw std::runtime_error("Invalid plot_count parameter");
  }
  plot_count_ = static_cast<std::size_t>(plot_count_param);

  collision_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("collision_polygon", 10);
  std::chrono::milliseconds sampling_period{(int)(1000.0)};
  timer_ = this->create_wall_timer(
    sampling_period, std::bind(&PublishCollisionPolygon::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "publish_collision_polygon node has been initialised");
}

PublishCollisionPolygon::~PublishCollisionPolygon()
{
  RCLCPP_INFO(this->get_logger(), "Destroying node");
}

void PublishCollisionPolygon::timer_callback()
{
  auto poly = geometry_msgs::msg::PolygonStamped();
  poly.header.stamp = this->now();
  poly.header.frame_id = base_frame_id_;
  auto point = geometry_msgs::msg::Point32();
  double delta = 2 * M_PI / static_cast<double>(plot_count_);
  double theta = 0.0;
  for (std::size_t i = 0; i < plot_count_; i++) {
    point.x = circle_.r * cos(theta) + circle_.x;
    point.y = circle_.r * sin(theta) + circle_.y;
    poly.polygon.points.push_back(point);
    theta += delta;
  }
  collision_poly_pub_->publish(poly);
}
