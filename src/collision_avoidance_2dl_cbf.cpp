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

#include "collision_avoidance_2dl_cbf/collision_avoidance_2dl_cbf.hpp"

CollisionAvoidance2dlCBF::CollisionAvoidance2dlCBF() : Node("collision_avoidance_2dl_cbf")
{
  this->declare_parameter("base_frame_name", "base_link");
  base_frame_name_ = this->get_parameter("base_frame_name").as_string();
  this->declare_parameter("scan_frame_name", "base_scan");
  scan_frame_name_ = this->get_parameter("scan_frame_name").as_string();
  this->declare_parameter("gamma", 1.0);
  gamma_ = this->get_parameter("gamma").as_double();

  using std::placeholders::_1;
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&CollisionAvoidance2dlCBF::scanCallback, this, _1));
  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 1, std::bind(&CollisionAvoidance2dlCBF::cmd_vel_inCallback, this, _1));
  collision_poly_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "collision_polygon", 1, std::bind(&CollisionAvoidance2dlCBF::collision_polygonCallback, this, _1));

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::TransformStamped t;
  while (true) {
    try {
      t = tf_buffer_->lookupTransform(scan_frame_name_, base_frame_name_, tf2::TimePointZero);
      BxS_ = t.transform.translation.x;
      ByS_ = t.transform.translation.y;
      auto q0 = t.transform.rotation.w;
      auto q1 = t.transform.rotation.x;
      auto q2 = t.transform.rotation.y;
      auto q3 = t.transform.rotation.z;
      BthetaS_ = atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
      RCLCPP_INFO_STREAM(this->get_logger(), "[x, y, yaw] : " << BxS_ << ", " << ByS_ << ", " << BthetaS_);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", scan_frame_name_.c_str(), base_frame_name_.c_str(), ex.what());
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  double x1 = 0.06;
  double y1 = 0;
  double r1 = sqrt(0.16*0.16 + 0.085*0.085);

  double x2 = -0.06;
  double y2 = -0.17;
  double r2 = 0.35;

  double X = x2 - x1;
  double Y = y2 - y1;
  double R = r2 - r1;
  double X2pY2 = X*X+Y*Y;
  double sqX2Y2mR2 = sqrt(X2pY2-R*R);

  points_of_tangency_.resize(4);

  points_of_tangency_[0].x = (-X*R*r1 + Y*r1*sqX2Y2mR2)/X2pY2 + x1;
  points_of_tangency_[0].y = (-Y*R*r1 - X*r1*sqX2Y2mR2)/X2pY2 + y1;

  points_of_tangency_[1].x = (-X*R*r2 + Y*r2*sqX2Y2mR2)/X2pY2 + x2;
  points_of_tangency_[1].y = (-Y*R*r2 - X*r2*sqX2Y2mR2)/X2pY2 + y2;

  points_of_tangency_[3].x = (-X*R*r1 - Y*r1*sqX2Y2mR2)/X2pY2 + x1;
  points_of_tangency_[3].y = (-Y*R*r1 + X*r1*sqX2Y2mR2)/X2pY2 + y1;

  points_of_tangency_[2].x = (-X*R*r2 - Y*r2*sqX2Y2mR2)/X2pY2 + x2;
  points_of_tangency_[2].y = (-Y*R*r2 + X*r2*sqX2Y2mR2)/X2pY2 + y2;

  points_of_tangency_.push_back(points_of_tangency_[0]);

  RCLCPP_INFO(this->get_logger(), "Creating node");
}

CollisionAvoidance2dlCBF::~CollisionAvoidance2dlCBF()
{
  RCLCPP_INFO(this->get_logger(), "Destroying node");
}

void CollisionAvoidance2dlCBF::cmd_vel_inCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  u_h1_ = msg->linear.x;
  u_h2_ = msg->angular.z;
  publishAssistInput();
}

void CollisionAvoidance2dlCBF::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  r_.resize(msg->ranges.size());
  theta_.resize(r_.size());

  // step 1: calculate r_i and theta_i
  double SrP, SthetaP, BxP, ByP;
  for (std::size_t i = 0; i < r_.size(); i++) {
    SrP = msg->ranges[i];
    if(SrP > msg->range_max) SrP = msg->range_max;
    SthetaP = msg->angle_min + i * msg->angle_increment;
    BxP = SrP * cos(SthetaP + BthetaS_) + BxS_;
    ByP = SrP * sin(SthetaP + BthetaS_) + ByS_;
    r_[i] = sqrt(BxP*BxP + ByP*ByP);
    theta_[i] = atan2(ByP, BxP);
  }
  publishAssistInput();
}

void CollisionAvoidance2dlCBF::collision_polygonCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  std::size_t new_size = msg->polygon.points.size();
  collision_poly_.resize(new_size);
  for (std::size_t i = 0; i < new_size; i++) {
    collision_poly_[i].x = msg->polygon.points[i].x;
    collision_poly_[i].y = msg->polygon.points[i].y;
  }
  collision_poly_.push_back(collision_poly_[0]);
  publishAssistInput();
}

void CollisionAvoidance2dlCBF::publishAssistInput()
{
  if (collision_poly_.size() < 2 || r_.size() == 0)
    return;

  double B, LgB1, LgB2;
  B = LgB1 = LgB2 = 0;
  for (std::size_t i = 0; i < r_.size(); i++) {
    // step 2: calculate BxC and ByC
    double r_ci, drc_dtheta;
    if(!calculateCollisionDistanceAndDifferential(theta_[i], r_ci, drc_dtheta))
      return;

    // step 4: calculate B and LgB
    double L = 0.001;
    double ri_rc = r_[i] - r_ci;
    if (ri_rc < 0){
      RCLCPP_ERROR_STREAM(this->get_logger(), "r_i - r_ci < 0: " << r_[i] << ", " << r_ci);
      // continue;
    }
    double ri_rc_sq = ri_rc * ri_rc;
    B += 1.0/ri_rc + L*(r_[i]*r_[i] + r_ci*r_ci);
    LgB1 +=  (1.0/ri_rc_sq - 2.0*L*r_[i]) * cos(theta_[i]);
    LgB2 += -(1.0/ri_rc_sq + 2.0*L*r_ci) * drc_dtheta;
  }
  // step 5: calculate h, Lgh, I, J, and u
  double h = 1.0/B;
  double Lgh1 = - LgB1 / (B*B);
  double Lgh2 = - LgB2 / (B*B);
  double I = Lgh1*u_h1_ + Lgh2*u_h2_;
  double J = -gamma_*h;
  double u1, u2;
  if(I < J){
    double Lgh_sq = Lgh1*Lgh1 + Lgh2*Lgh2;
    u1 = -(I - J) * Lgh1 / Lgh_sq;
    u2 = -(I - J) * Lgh2 / Lgh_sq;
  } else {
    u1 = u2 = 0.0;
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "B, LgB1, LgB2: " << B << ", " << LgB1 << ", " << LgB2);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "u: " << u1 << ", " << u2);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "u_h: " << u_h1_ << ", " << u_h2_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "u+u_h: " << u1+u_h1_ << ", " << u2 + u_h2_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "I, J, I - J: " << I << ", " << J << ", " << (I - J));

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = u1 + u_h1_;
  msg.angular.z = u2 + u_h2_;
  cmd_vel_out_pub_->publish(msg);
}

bool CollisionAvoidance2dlCBF::calculateLineIntersection(const Point& p1, const Point& p2, const Point& p3, const Point& p4, Point& intersection)
{
  double det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  if(det == 0.0){
    RCLCPP_DEBUG_STREAM(this->get_logger(), "det = " << det);
    return false;
  }
  double t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  double s = ((p2.y - p1.y) * (p4.x - p2.x) + (p1.x - p2.x) * (p4.y - p2.y)) / det;

  if(0.0 <= t && t <= 1.0 && 0.0 <= s && s <= 1.0){
    intersection.x = t * p1.x + (1.0 - t) * p2.x;
    intersection.y = t * p1.y + (1.0 - t) * p2.y;
    return true;
  } else {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "t: " << t);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "s: " << s);
    return false;
  }
}

bool CollisionAvoidance2dlCBF::calculatePolygonIntersection(const Point& target, Point& intersection, std::size_t& number)
{
  Point origin = {0.0, 0.0};
  for (std::size_t i = 0; i < collision_poly_.size()-1; i++) {
    if (calculateLineIntersection(origin, target, collision_poly_[i], collision_poly_[i+1], intersection)){
      number = i;
      return true;
    }
  }
  return false;
}

bool CollisionAvoidance2dlCBF::calculateCollisionDistanceAndDifferential(const double& theta, double& distance, double& differential)
{
  for (std::size_t i = 0; i < points_of_tangency_.size()-1; i++) {
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
        double a = 0.35;
        double r0 = sqrt(x0*x0 + y0*y0);
        double theta0 = atan2(y0, x0);
        distance = sqrt(r0*r0*cos(2*theta - 2*theta0)/2 + a*a - r0*r0/2) + r0*cos(theta-theta0);
        differential = -r0*sin(theta-theta0) - sqrt(2)*r0*r0*sin(2*theta - 2*theta0)/(2*sqrt(r0*r0*cos(2*theta - 2*theta0)+2*a*a-r0*r0));
      } else if (i == 3){
        double x0 = 0.06;
        double y0 = 0;
        double a = sqrt(0.16*0.16 + 0.085*0.085);
        double r0 = sqrt(x0*x0 + y0*y0);
        double theta0 = atan2(y0, x0);
        distance = sqrt(r0*r0*cos(2*theta - 2*theta0)/2 + a*a - r0*r0/2) + r0*cos(theta-theta0);
        differential = -r0*sin(theta-theta0) - sqrt(2)*r0*r0*sin(2*theta - 2*theta0)/(2*sqrt(r0*r0*cos(2*theta - 2*theta0)+2*a*a-r0*r0));
      }
      return true;
    }
  }
  for (std::size_t i = 0; i < points_of_tangency_.size()-1; i++) {
    double x1 = points_of_tangency_[i].x;
    double y1 = points_of_tangency_[i].y;
    double x2 = points_of_tangency_[i+1].x;
    double y2 = points_of_tangency_[i+1].y;
    RCLCPP_ERROR_STREAM(this->get_logger(), "x1, y1, atan: (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << "), "  << atan2(y1, x1) << ", "  << atan2(y2, x2));
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "theta, r_ci, drc_dtheta: " << theta << ", " << distance << ", " << differential);
  return false;
}
