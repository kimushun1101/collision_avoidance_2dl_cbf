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

int main(int argc, char * argv[])
{
  // Ctrl + C を押下したとき速度を0 とする処理
  signal(
    SIGINT, [](int) {
      auto node = rclcpp::Node::make_shared("stop");
      auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      pub->publish(geometry_msgs::msg::Twist());
      rclcpp::shutdown();
    }
  );

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionAvoidance2dlCBF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
