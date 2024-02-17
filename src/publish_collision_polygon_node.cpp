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


    std::size_t p_num = 100;
    double delta = 2*M_PI/(double)p_num;
    double theta = 0.0;

    {
      double x = 0.06;
      double y = 0;
      double r = sqrt(0.16*0.16 + 0.085*0.085);
      for(std::size_t i = 0; i < p_num; i++){
        point.x = r*cos(theta) + x;
        point.y = r*sin(theta) + y;
        // poly.polygon.points.push_back(point);
        theta += delta;
      }
      // collision_poly_pub_->publish(poly);
    }
    {
      double x = -0.06;
      double y = -0.17;
      double r = 0.2;
      for(std::size_t i = 0; i < p_num; i++){
        point.x = r*cos(theta) + x;
        point.y = r*sin(theta) + y;
        // poly.polygon.points.push_back(point);
        theta += delta;
      }
    }

    std::vector<geometry_msgs::msg::Point32> points_of_tangency_(4); 
    double x1 = 0.06;
    double y1 = 0;
    double r1 = sqrt(0.16*0.16 + 0.085*0.085);

    double x2 = -0.06;
    double y2 = -0.17;
    double r2 = 0.2;

    double X = x2 - x1;
    double Y = y2 - y1;
    double R = r2 - r1;
    double X2pY2 = X*X+Y*Y;
    double sqX2Y2mR2 = sqrt(X2pY2-R*R);

    points_of_tangency_[0].x = (-X*R*r1 + Y*r1*sqX2Y2mR2)/X2pY2 + x1;
    points_of_tangency_[0].y = (-Y*R*r1 - X*r1*sqX2Y2mR2)/X2pY2 + y1;

    points_of_tangency_[1].x = (-X*R*r2 + Y*r2*sqX2Y2mR2)/X2pY2 + x2;
    points_of_tangency_[1].y = (-Y*R*r2 - X*r2*sqX2Y2mR2)/X2pY2 + y2;

    points_of_tangency_[3].x = (-X*R*r1 - Y*r1*sqX2Y2mR2)/X2pY2 + x1;
    points_of_tangency_[3].y = (-Y*R*r1 + X*r1*sqX2Y2mR2)/X2pY2 + y1;

    points_of_tangency_[2].x = (-X*R*r2 - Y*r2*sqX2Y2mR2)/X2pY2 + x2;
    points_of_tangency_[2].y = (-Y*R*r2 + X*r2*sqX2Y2mR2)/X2pY2 + y2;

    points_of_tangency_.push_back(points_of_tangency_[0]);


    theta = -M_PI;
    for(std::size_t j = 0; j < p_num; j++){
      // RCLCPP_INFO_STREAM(this->get_logger(), "theta: " << theta);

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
          if (i%2 == 0){
            double a  = abs(x2*y1 - x1*y2)/sqrt((y2-y1)*(y2-y1)+(x1-x2)*(x1-x2));
            double alpha = atan2(-(x2-x1),(y2-y1));
            distance = a/cos(theta - alpha);
            differential = a*tan(theta - alpha)/(cos(theta - alpha));
          } else if (i == 1){
            double x0 = -0.06;
            double y0 = -0.17;
            double a = 0.2;
            double r0 = sqrt(x0*x0 + y0*y0);
            double theta0 = atan2(y0,x0);
            distance = sqrt(r0*r0*cos(2*theta - 2*theta0)/2 + a*a - r0*r0/2) + r0*cos(theta-theta0);
            differential = -r0*sin(theta-theta0) - sqrt(2)*r0*r0*sin(2*theta - 2*theta0)/(2*sqrt(r0*r0*cos(2*theta - 2*theta0)+2*a-r0*r0));
            RCLCPP_ERROR_STREAM(this->get_logger(), "distance: " << distance << ", theta: " << theta << ", s: " << s << ", t: " << t);
          } else if (i == 3){
            double x0 = 0.06;
            double y0 = 0;
            double a = sqrt(0.16*0.16 + 0.085*0.085);
            double r0 = sqrt(x0*x0 + y0*y0);
            double theta0 = atan2(y0, x0);
            distance = sqrt(r0*r0*cos(2*theta - 2*theta0)/2 + a*a - r0*r0/2) + r0*cos(theta-theta0);
            differential = -r0*sin(theta-theta0) - sqrt(2)*r0*r0*sin(2*theta - 2*theta0)/(2*sqrt(r0*r0*cos(2*theta - 2*theta0)+2*a-r0*r0));
          }
          (void)differential;
          point.x = distance*cos(theta);
          point.y = distance*sin(theta);
          poly.polygon.points.push_back(point);
        }
        // RCLCPP_ERROR_STREAM(this->get_logger(), "theta: " << theta << ", " << s << ", " << t);
      }
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
