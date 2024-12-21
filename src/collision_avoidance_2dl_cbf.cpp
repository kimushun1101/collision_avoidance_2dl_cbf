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
  this->declare_parameter("polygon_topic_name", "collision_polygon");
  auto polygon_topic_name = this->get_parameter("polygon_topic_name").as_string();
  std::vector<std::string> v{"scan"};
  this->declare_parameter("scan_topic_names", v);
  auto scan_topic_names = this->get_parameter("scan_topic_names").as_string_array();
  scan_subs_.resize(scan_topic_names.size());
  this->declare_parameter("cbf_param.K", 0.1);
  cbf_param_K_ = this->get_parameter("cbf_param.K").as_double();
  this->declare_parameter("cbf_param.C", 0.1);
  cbf_param_C_ = this->get_parameter("cbf_param.C").as_double();
  this->declare_parameter("cbf_param.L", 0.001);
  cbf_param_L_ = this->get_parameter("cbf_param.L").as_double();
  this->declare_parameter("is_debug", false);
  is_debug_ = this->get_parameter("is_debug").as_bool();

  using std::placeholders::_1;
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  for (std::size_t i = 0; i < scan_topic_names.size(); i++) {
    scan_subs_[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_names[i], rclcpp::SensorDataQoS(), std::bind(&CollisionAvoidance2dlCBF::scanCallback, this, _1));
  }
  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 1, std::bind(&CollisionAvoidance2dlCBF::cmd_vel_inCallback, this, _1));
  collision_poly_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    polygon_topic_name, 1, std::bind(&CollisionAvoidance2dlCBF::collision_polygonCallback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if(is_debug_){
    debug_values_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("cbf_debug", 10);
    debug_collision_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cbf_debug_collision", 10);
  }

  RCLCPP_INFO(this->get_logger(), "Creating node");
}

CollisionAvoidance2dlCBF::~CollisionAvoidance2dlCBF()
{
  RCLCPP_INFO(this->get_logger(), "Destroying node");
}

void CollisionAvoidance2dlCBF::cmd_vel_inCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  u_ref1_ = msg->linear.x;
  u_ref2_ = msg->angular.z;
  publishAssistInput();
}

void CollisionAvoidance2dlCBF::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  auto scan_frame_name = msg->header.frame_id;
  auto itr = detected_point_.find(scan_frame_name);
  // scan_frame_name is not found
  if (itr == detected_point_.end()) {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(base_frame_name_, scan_frame_name, tf2::TimePointZero);
      detected_point_[scan_frame_name].BtoS.x = t.transform.translation.x;
      detected_point_[scan_frame_name].BtoS.y = t.transform.translation.y;
      double q0, q1, q2, q3;
      q0 = t.transform.rotation.w; q1 = t.transform.rotation.x; q2 = t.transform.rotation.y; q3 = t.transform.rotation.z;
      detected_point_[scan_frame_name].BtoS_yaw = atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
      RCLCPP_INFO_STREAM(this->get_logger(), "scan frame name is " << scan_frame_name << " : [" << detected_point_[scan_frame_name].BtoS.x << ", " << detected_point_[scan_frame_name].BtoS.y << ", " << detected_point_[scan_frame_name].BtoS_yaw << "]");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", base_frame_name_.c_str(), scan_frame_name.c_str(), ex.what());
      return;
    }
    detected_point_[scan_frame_name].BtoP.resize(msg->ranges.size());
  }
  double SrP, StoP_yaw;
  for (std::size_t i = 0; i < msg->ranges.size(); i++) {
    SrP = msg->ranges[i];
    if(SrP > msg->range_max) SrP = msg->range_max;
    StoP_yaw = msg->angle_min + i * msg->angle_increment;
    detected_point_[scan_frame_name].BtoP[i].x = SrP * cos(StoP_yaw + detected_point_[scan_frame_name].BtoS_yaw) + detected_point_[scan_frame_name].BtoS.x;
    detected_point_[scan_frame_name].BtoP[i].y = SrP * sin(StoP_yaw + detected_point_[scan_frame_name].BtoS_yaw) + detected_point_[scan_frame_name].BtoS.y;
  }
  // publishAssistInput();
}

void CollisionAvoidance2dlCBF::collision_polygonCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  base_frame_name_ = msg->header.frame_id;
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Polygon base frame name : " << base_frame_name_);
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
  if (collision_poly_.empty() || detected_point_.empty())
    return;

  double B, LgB1, LgB2;
  B = LgB1 = LgB2 = 0;
  bool is_collision = false;
  for(auto itr = detected_point_.begin(); itr != detected_point_.end(); ++itr) {
    is_collision |= summationCBFs(itr->second.BtoP, B, LgB1, LgB2);
  }

  double I = LgB1*u_ref1_ + LgB2*u_ref2_;
  double J = cbf_param_K_ * B + cbf_param_C_;
  double u1, u2;
  if (I > J) {
    double LgB_sq = LgB1*LgB1 + LgB2*LgB2;
    u1 = -(I - J) * LgB1 / LgB_sq;
    u2 = -(I - J) * LgB2 / LgB_sq;
  } else {
    u1 = u2 = 0.0;
  }

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = u1 + u_ref1_;
  msg.angular.z = u2 + u_ref2_;
  cmd_vel_out_pub_->publish(msg);

  if (is_collision) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Set a smaller K: " << cbf_param_K_);
  }

  if (is_debug_) {
    auto debug_values_msg = std_msgs::msg::Float64MultiArray();
    debug_values_msg.data.push_back(B);
    debug_values_msg.data.push_back(I);
    debug_values_msg.data.push_back(J);
    debug_values_msg.data.push_back(cbf_param_K_);
    debug_values_msg.data.push_back(cbf_param_C_);
    debug_values_msg.data.push_back(cbf_param_L_);
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      double q0, q1, q2, q3;
      q0 = t.transform.rotation.w; q1 = t.transform.rotation.x; q2 = t.transform.rotation.y; q3 = t.transform.rotation.z;
      debug_values_msg.data.push_back(t.transform.translation.x);
      debug_values_msg.data.push_back(t.transform.translation.y);
      debug_values_msg.data.push_back(atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3));
    } catch (const tf2::TransformException & ex) {
    }
    debug_values_pub_->publish(debug_values_msg);
  }
}

bool CollisionAvoidance2dlCBF::summationCBFs(const std::vector<Point> BtoP, double& B, double& LgB1, double& LgB2)
{
  bool collision_check = false;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  std::size_t N = BtoP.size();
  for (std::size_t i = 0; i < N; i++) {
    // step 1: calculate r_i
    double r_i = sqrt(BtoP[i].x*BtoP[i].x + BtoP[i].y*BtoP[i].y);
    double theta_i = atan2(BtoP[i].y, BtoP[i].x);

    // step 2: calculate BtoC
    Point BtoC;
    std::size_t poly_num;
    if (!calculatePolygonIntersection(BtoP[i], BtoC, poly_num)) {
      RCLCPP_ERROR(this->get_logger(), "Something is wrong with the polygon settings.");
      RCLCPP_ERROR_STREAM(this->get_logger(), "r_i, theta_i : " << r_i << ", " << theta_i);
      continue;
    }
    if(is_debug_){
      pcl::PointXYZI point;
      point.x = BtoC.x;
      point.y = BtoC.y;
      point.z = 0.01;
      point.intensity = 1.0;
      cloud->points.push_back(point);
    }

    // step 3: calculate r_c(theta_i) and drc_dtheta(theta_i)
    double p1x = collision_poly_[poly_num].x;
    double p1y = collision_poly_[poly_num].y;
    double p2x = collision_poly_[poly_num+1].x;
    double p2y = collision_poly_[poly_num+1].y;
    double r_perp  = abs(p2x*p1y - p1x*p2y)/sqrt((p2y-p1y)*(p2y-p1y)+(p1x-p2x)*(p1x-p2x));
    double theta_perp = atan2(-(p2x-p1x),(p2y-p1y));
    double r_ci = r_perp/cos(theta_i - theta_perp);
    double drc_dtheta = r_perp*tan(theta_i - theta_perp)/(cos(theta_i - theta_perp));

    // step 4: calculate B and LgB
    double ri_rc = r_i - r_ci;
    if (ri_rc < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "r_i - r_ci < 0: " << r_i << ", " << r_ci << ", " << theta_i);
      collision_check = true;
      continue;
    }
    double ri_rc_sq = ri_rc * ri_rc;
    double L = cbf_param_L_;
    B += 1.0/ri_rc + L*(r_i*r_i + 1.0/r_ci);
    double dBdx1 = -1.0/ri_rc_sq + 2.0*L*r_i;
    double dBdx2 = +1.0/ri_rc_sq + L/(r_ci*r_ci);
    LgB1 += dBdx1*(-cos(theta_i));
    LgB2 += dBdx1*sin(theta_i)/r_i*drc_dtheta + dBdx2*(-drc_dtheta);
  }
  if(is_debug_){
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = base_frame_name_;
    debug_collision_pub_->publish(cloud_msg);
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
  double s1 = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  double s2 = ((p2.y - p1.y) * (p4.x - p2.x) + (p1.x - p2.x) * (p4.y - p2.y)) / det;

  if (0.0 <= s1 && s1 <= 1.0 && 0.0 <= s2 && s2 <= 1.0) {
    intersection.x = s1 * p1.x + (1.0 - s1) * p2.x;
    intersection.y = s1 * p1.y + (1.0 - s1) * p2.y;
    return true;
  } else {
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "s1: " << s1);
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "s2: " << s2);
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
