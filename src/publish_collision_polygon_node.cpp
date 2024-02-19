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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <math.h>

class PublishBodyPolygon : public rclcpp::Node
{
public:
  PublishBodyPolygon() : Node("publish_collision_polygon")
  {
    this->declare_parameter("base_frame_id", "base_link");
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    std::vector<double> values = {0.26, 0.085, -0.06, 0.085, -0.06, -0.085, 0.26, -0.085};
    this->declare_parameter("collision_polygon", values);
    poly_row_ = this->get_parameter("collision_polygon").as_double_array();

    collision_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("collision_polygon", 10);
    std::chrono::milliseconds sampling_period{(int)(1000.0)};
    timer_ = this->create_wall_timer(
      sampling_period, std::bind(&PublishBodyPolygon::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "publish_collision_polygon node has been initialised");
  }

private:
  void timer_callback()
  {
    auto poly = geometry_msgs::msg::PolygonStamped();
    poly.header.stamp = this->now();
    poly.header.frame_id = base_frame_id_;
  
    auto point = geometry_msgs::msg::Point32();
    // if (poly_row_.size() <= 6 || poly_row_.size() % 2 != 0) {
    //   RCLCPP_ERROR(this->get_logger(), "Polygon has incorrect points description");
    //   return;
    // }
    // bool first = true;
    // // navigation2/nav2_collision_monitor/src/polygon.cpp
    // for (double val : poly_row_) {
    //   if (first) {
    //     point.x = val;
    //   } else {
    //     point.y = val;
    //     poly.polygon.points.push_back(point);
    //   }
    //   first = !first;
    // }

    double r = 0.5;
    std::size_t p_num = 100;
    double offset_x = -0.3;
    double offset_y = -0.15;

    // double r = 0.3;
    // std::size_t p_num = 100;
    // double offset_x = 0.032;
    // double offset_y = 0.0;

    double delta = 2*M_PI/(double)p_num;
    double theta = 0.0;
    for(std::size_t i = 0; i < p_num; i++){
      point.x = r*cos(theta) + offset_x;
      point.y = r*sin(theta) + offset_y;
      poly.polygon.points.push_back(point);
      theta += delta;
    }
    collision_poly_pub_->publish(poly);
  }
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr collision_poly_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string base_frame_id_;
  std::vector<double> poly_row_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishBodyPolygon>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
