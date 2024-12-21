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
  plot_count_ = this->get_parameter("plot_count").as_int();

  this->declare_parameter("robot.x", 0.06);
  this->declare_parameter("robot.y", 0.0);
  this->declare_parameter("robot.r", sqrt(0.16*0.16 + 0.085*0.085));
  robot_.x = this->get_parameter("robot.x").as_double();
  robot_.y = this->get_parameter("robot.y").as_double();
  robot_.r = this->get_parameter("robot.r").as_double();


  robot_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("robot_polygon", 10);
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
  publish_circle(robot_, 30, robot_poly_pub_);
}

void PublishCollisionPolygon::publish_circle(const Circle& c, std::size_t plot_count, const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub)
{
  auto poly = geometry_msgs::msg::PolygonStamped();
  poly.header.stamp = this->now();
  poly.header.frame_id = base_frame_id_;
  auto point = geometry_msgs::msg::Point32();
  double delta = 2*M_PI/(double)plot_count;
  double theta = 0.0;

  double r, offset_x, offset_y;
  offset_x = c.x;
  offset_y = c.y;
  r = c.r;
  for(std::size_t i = 0; i < plot_count; i++){
    point.x = r*cos(theta) + offset_x;
    point.y = r*sin(theta) + offset_y;
    poly.polygon.points.push_back(point);
    theta += delta;
  }
  pub->publish(poly);
}
