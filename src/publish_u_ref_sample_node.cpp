#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class PublishUrefSample : public rclcpp::Node
{
  public:
    PublishUrefSample() : Node("publish_u_ref_sample")
    {
      cmd_vel_ref_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_ref", 10);
      timer_ = this->create_wall_timer(20ms, std::bind(&PublishUrefSample::timer_callback, this));
      t0_ = this->now().seconds();
    }

  private:
    void timer_callback()
    {
        int t = this->now().seconds() - t0_;
        double u_ref_1, u_ref_2;
        if (t < 5) {
          u_ref_1 = 0.0;
          u_ref_2 = 0.0;
        } else if (t < 15) {
          u_ref_1 = 0.5;
          u_ref_2 = 0.0;
        } else if (t < 30) {
          u_ref_1 = -0.5;
          u_ref_2 = 0.0;
        } else if (t < 50) {
          u_ref_1 = 0.5;
          u_ref_2 = 0.2;
        } else {
          u_ref_1 = 0.0;
          u_ref_2 = 0.0;
          RCLCPP_WARN(this->get_logger(), "End of sample u_ref");
        }
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = u_ref_1;
        msg.angular.z = u_ref_2;
        cmd_vel_ref_pub_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_ref_pub_;
    int t0_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishUrefSample>());
  rclcpp::shutdown();
  return 0;
}