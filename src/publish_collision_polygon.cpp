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
  this->declare_parameter("user.x", -0.06);
  this->declare_parameter("user.y", -0.17);
  this->declare_parameter("user.r", 0.3);
  robot_.x = this->get_parameter("robot.x").as_double();
  robot_.y = this->get_parameter("robot.y").as_double();
  robot_.r = this->get_parameter("robot.r").as_double();
  user_.x = this->get_parameter("user.x").as_double();
  user_.y = this->get_parameter("user.y").as_double();
  user_.r = this->get_parameter("user.r").as_double();

  collision_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("collision_polygon", 10);
  robot_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("robot_polygon", 10);
  user_poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("user_polygon", 10);
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
  publish_circle(user_, 30, user_poly_pub_);
  publish_collision(robot_, user_, plot_count_, collision_poly_pub_);
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

void PublishCollisionPolygon::publish_collision(const Circle& c1, const Circle& c2, std::size_t plot_count, const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub)
{
  auto poly = geometry_msgs::msg::PolygonStamped();
  poly.header.stamp = this->now();
  poly.header.frame_id = base_frame_id_;

  auto point = geometry_msgs::msg::Point32();
  double delta = 2*M_PI/(double)plot_count;
  double theta = 0.0;

  // tangency
  double X = c2.x - c1.x;
  double Y = c2.y - c1.y;
  double R = c2.r - c1.r;
  double X2pY2 = X*X+Y*Y;
  double sqX2Y2mR2 = sqrt(X2pY2-R*R);

  std::vector<geometry_msgs::msg::Point32> points_of_tangency_(4);
  // Circle1 tangency -
  points_of_tangency_[0].x = (-X*R*c1.r - Y*c1.r*sqX2Y2mR2)/X2pY2 + c1.x;
  points_of_tangency_[0].y = (-Y*R*c1.r + X*c1.r*sqX2Y2mR2)/X2pY2 + c1.y;
  // Circle1 tangency +
  points_of_tangency_[1].x = (-X*R*c1.r + Y*c1.r*sqX2Y2mR2)/X2pY2 + c1.x;
  points_of_tangency_[1].y = (-Y*R*c1.r - X*c1.r*sqX2Y2mR2)/X2pY2 + c1.y;
  // Circle2 tangency -
  points_of_tangency_[2].x = (-X*R*c2.r + Y*c2.r*sqX2Y2mR2)/X2pY2 + c2.x;
  points_of_tangency_[2].y = (-Y*R*c2.r - X*c2.r*sqX2Y2mR2)/X2pY2 + c2.y;
  // Circle1 tangency +
  points_of_tangency_[3].x = (-X*R*c2.r - Y*c2.r*sqX2Y2mR2)/X2pY2 + c2.x;
  points_of_tangency_[3].y = (-Y*R*c2.r + X*c2.r*sqX2Y2mR2)/X2pY2 + c2.y;
  // Again Circle1 tangency - to make loop
  points_of_tangency_.push_back(points_of_tangency_[0]);

  theta = -M_PI;
  for(std::size_t j = 0; j < plot_count; j++){
    for (std::size_t i = 0; i < points_of_tangency_.size()-1; i++) {
      double distance, differential;
      double x1 = points_of_tangency_[i].x;
      double y1 = points_of_tangency_[i].y;
      double x2 = points_of_tangency_[i+1].x;
      double y2 = points_of_tangency_[i+1].y;
      double s = atan2(y1, x1);
      double t = atan2(y2, x2);
      if (s > t){
        if (theta > 0) t += 2*M_PI;
        else s -= 2*M_PI;
      }
      if (s < theta && theta < t) {
        if (i == 0){
          // On Circle1
          double x0 = c1.x;
          double y0 = c1.y;
          double a = c1.r;
          double r0 = sqrt(x0*x0 + y0*y0);
          double theta0 = atan2(y0,x0);
          distance = sqrt(r0*r0*cos(2*theta - 2*theta0)/2 + a*a - r0*r0/2) + r0*cos(theta-theta0);
          differential = -r0*sin(theta-theta0) - sqrt(2)*r0*r0*sin(2*theta - 2*theta0)/(2*sqrt(r0*r0*cos(2*theta - 2*theta0)+2*a-r0*r0));
        } else if (i == 2){
          // On Circle2
          double x0 = c2.x;
          double y0 = c2.y;
          double a = c2.r;
          double r0 = sqrt(x0*x0 + y0*y0);
          double theta0 = atan2(y0, x0);
          distance = sqrt(r0*r0*cos(2*theta - 2*theta0)/2 + a*a - r0*r0/2) + r0*cos(theta-theta0);
          differential = -r0*sin(theta-theta0) - sqrt(2)*r0*r0*sin(2*theta - 2*theta0)/(2*sqrt(r0*r0*cos(2*theta - 2*theta0)+2*a-r0*r0));
        } else {
          // line to another Circle
          double a  = abs(x2*y1 - x1*y2)/sqrt((y2-y1)*(y2-y1)+(x1-x2)*(x1-x2));
          double alpha = atan2(-(x2-x1),(y2-y1));
          distance = a/cos(theta - alpha);
          differential = a*tan(theta - alpha)/(cos(theta - alpha));
        }
        (void)differential;
        point.x = distance*cos(theta);
        point.y = distance*sin(theta);
        poly.polygon.points.push_back(point);
      }
    }
    theta += delta;
  }
  
  pub->publish(poly);
}