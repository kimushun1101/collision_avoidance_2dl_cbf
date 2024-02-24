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
  std::vector<std::string> v{"scan"};
  this->declare_parameter("scan_topic_names", v);
  scan_topic_names_ = this->get_parameter("scan_topic_names").as_string_array();
  scan_subs_.resize(scan_topic_names_.size());
  this->declare_parameter("gamma", 1.0);
  gamma_ = this->get_parameter("gamma").as_double();
  this->declare_parameter("epsilon", 0.001);
  epsilon_ = this->get_parameter("epsilon").as_double();
  this->declare_parameter("is_debug", false);
  is_debug_ = this->get_parameter("is_debug").as_bool();

  using std::placeholders::_1;
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  for (std::size_t i = 0; i < scan_topic_names_.size(); i++) {
    scan_subs_[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_names_[i], rclcpp::SensorDataQoS(), std::bind(&CollisionAvoidance2dlCBF::scanCallback, this, _1));
  }
  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 1, std::bind(&CollisionAvoidance2dlCBF::cmd_vel_inCallback, this, _1));
  collision_poly_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "collision_polygon", 1, std::bind(&CollisionAvoidance2dlCBF::collision_polygonCallback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if(is_debug_)
    debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("cbf_debug", 10);

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
  auto scan_frame_name = msg->header.frame_id;
  auto itr = lidar_.find(scan_frame_name);
  if (itr == lidar_.end()) {
    geometry_msgs::msg::TransformStamped t;
    while (true) {
      try {
        t = tf_buffer_->lookupTransform(base_frame_name_, scan_frame_name, tf2::TimePointZero);
        lidar_[scan_frame_name].BtoS.x = t.transform.translation.x;
        lidar_[scan_frame_name].BtoS.y = t.transform.translation.y;
        double q0, q1, q2, q3;
        q0 = t.transform.rotation.w; q1 = t.transform.rotation.x; q2 = t.transform.rotation.y; q3 = t.transform.rotation.z;
        lidar_[scan_frame_name].BthetaS = atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
        RCLCPP_INFO_STREAM(this->get_logger(), " " << scan_frame_name << " : [" << lidar_[scan_frame_name].BtoS.x << ", " << lidar_[scan_frame_name].BtoS.y << ", " << lidar_[scan_frame_name].BthetaS << "]");
        break;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", base_frame_name_.c_str(), scan_frame_name.c_str(), ex.what());
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    lidar_[scan_frame_name].BtoP.resize(msg->ranges.size());
  }
  // step 1: calculate r_i and theta_i
  double SrP, SthetaP;
  for (std::size_t i = 0; i < msg->ranges.size(); i++) {
    SrP = msg->ranges[i];
    if(SrP > msg->range_max) SrP = msg->range_max;
    SthetaP = msg->angle_min + i * msg->angle_increment;
    lidar_[scan_frame_name].BtoP[i].x = SrP * cos(SthetaP + lidar_[scan_frame_name].BthetaS) + lidar_[scan_frame_name].BtoS.x;
    lidar_[scan_frame_name].BtoP[i].y = SrP * sin(SthetaP + lidar_[scan_frame_name].BthetaS) + lidar_[scan_frame_name].BtoS.y;
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
  if (collision_poly_.empty() || lidar_.empty())
    return;

  double B, LgB1, LgB2;
  B = LgB1 = LgB2 = 0;
  bool is_collision = false;
  for(auto itr = lidar_.begin(); itr != lidar_.end(); ++itr) {
    is_collision |= summationCBFs(itr->second.BtoP, B, LgB1, LgB2);
  }
  double h = 1.0/B;
  double Lgh1 = - LgB1 / (B*B);
  double Lgh2 = - LgB2 / (B*B);
  double I = Lgh1*u_h1_ + Lgh2*u_h2_;
  double J = -gamma_*(h - epsilon_);
  double u1, u2;
  if (I < J) {
    double Lgh_sq = Lgh1*Lgh1 + Lgh2*Lgh2;
    u1 = -(I - J) * Lgh1 / Lgh_sq;
    u2 = -(I - J) * Lgh2 / Lgh_sq;
  } else {
    u1 = u2 = 0.0;
  }

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = u1 + u_h1_;
  msg.angular.z = u2 + u_h2_;
  cmd_vel_out_pub_->publish(msg);

  if (is_collision) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Set a larger epsilon. [epsilon, h]: [" << epsilon_ << ", " << h << "]");
  }

  if (is_debug_) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "B, LgB1, LgB2: " << B << ", " << LgB1 << ", " << LgB2);
    // RCLCPP_INFO_STREAM(this->get_logger(), "h, Lgh1, Lgh2: " << h << ", " << Lgh1 << ", " << Lgh2);
    // RCLCPP_INFO_STREAM(this->get_logger(), "u: " << u1 << ", " << u2);
    // RCLCPP_INFO_STREAM(this->get_logger(), "u_h: " << u_h1_ << ", " << u_h2_);
    // RCLCPP_INFO_STREAM(this->get_logger(), "u+u_h: " << u1+u_h1_ << ", " << u2 + u_h2_);
    // RCLCPP_INFO_STREAM(this->get_logger(), "I, J, I - J: " << I << ", " << J << ", " << (I - J));
    auto debug_msg = std_msgs::msg::Float64MultiArray();
    debug_msg.data.push_back(h);
    debug_msg.data.push_back(I);
    debug_msg.data.push_back(J);
    debug_msg.data.push_back(gamma_);
    debug_msg.data.push_back(epsilon_);
    debug_pub_->publish(debug_msg);
  }
}

bool CollisionAvoidance2dlCBF::summationCBFs(const std::vector<Point> BtoP, double& B, double& LgB1, double& LgB2)
{
  bool collision_check = false;
  for (std::size_t i = 0; i < BtoP.size(); i++) {
    // step 1: calculate r_i
    double r_i = sqrt(BtoP[i].x*BtoP[i].x + BtoP[i].y*BtoP[i].y);
    double theta_i = atan2(BtoP[i].y, BtoP[i].x);

    // step 2: calculate BtoC
    Point BtoC;
    std::size_t poly_num;
    if (!calculatePolygonIntersection(BtoP[i], BtoC, poly_num)) {
      // RCLCPP_ERROR(this->get_logger(), "Something is wrong with the polygon settings.");
      // RCLCPP_ERROR_STREAM(this->get_logger(), "r_i, theta_i : " << r_i << ", " << theta_i);
      continue;
    }

    // step 3: calculate r_ci and drc_dtheta
    double x1 = collision_poly_[poly_num].x;
    double y1 = collision_poly_[poly_num].y;
    double x2 = collision_poly_[poly_num+1].x;
    double y2 = collision_poly_[poly_num+1].y;
    double a  = abs(x2*y1 - x1*y2)/sqrt((y2-y1)*(y2-y1)+(x1-x2)*(x1-x2));
    double alpha = atan2(-(x2-x1),(y2-y1));
    double r_ci = a/cos(theta_i - alpha);
    double drc_dtheta = a*tan(theta_i - alpha)/(cos(theta_i - alpha));

    // step 4: calculate B and LgB
    double L = 0.001;
    double ri_rc = r_i - r_ci;
    if (ri_rc < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "r_i - r_ci < 0: " << r_i << ", " << r_ci << ", " << theta_i);
      collision_check = true;
      continue;
    }
    double ri_rc_sq = ri_rc * ri_rc;
    B += 1.0/ri_rc + L*(r_i*r_i + r_ci*r_ci);
    double dBdx1 = -1.0/ri_rc_sq + 2.0*L*r_i;
    double dBdx2 = +1.0/ri_rc_sq + 2.0*L*r_ci;
    LgB1 += dBdx1*(-cos(theta_i));
    LgB2 += dBdx2*(-drc_dtheta);
  }
  return collision_check;
}


bool CollisionAvoidance2dlCBF::calculateLineIntersection(const Point& p1, const Point& p2, const Point& p3, const Point& p4, Point& intersection)
{
  double det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  if (det == 0.0) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "det = " << det);
    return false;
  }
  double t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  double s = ((p2.y - p1.y) * (p4.x - p2.x) + (p1.x - p2.x) * (p4.y - p2.y)) / det;

  if (0.0 <= t && t <= 1.0 && 0.0 <= s && s <= 1.0) {
    intersection.x = t * p1.x + (1.0 - t) * p2.x;
    intersection.y = t * p1.y + (1.0 - t) * p2.y;
    return true;
  } else {
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "t: " << t);
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "s: " << s);
    return false;
  }
}

bool CollisionAvoidance2dlCBF::calculatePolygonIntersection(const Point& target, Point& intersection, std::size_t& number)
{
  Point origin = {0.0, 0.0};
  for (std::size_t i = 0; i < collision_poly_.size()-1; i++) {
    if (calculateLineIntersection(origin, target, collision_poly_[i], collision_poly_[i+1], intersection)) {
      number = i;
      return true;
    }
  }
  return false;
}
