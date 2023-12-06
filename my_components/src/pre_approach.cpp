#include "my_components/pre_approach.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach_node", options) {

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_callback(msg);
      });

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diffbot_base_controller/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });

  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
}

void PreApproach::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

if(operation){
  double obs = 0.4;
  int deg = -95;

  RCLCPP_INFO(get_logger(), "Target: %f; Curr: %f", obs, msg->ranges[540]);

  geometry_msgs::msg::Twist cmd_vel_msg;

  if (!is_rotating_) {
    if (std::abs(msg->ranges[540] - obs) < 0.05) {
      cmd_vel_msg.linear.x = 0.0;
      is_rotating_ = true;
    } else {
      cmd_vel_msg.linear.x = 0.5;
      cmd_vel_msg.angular.z = 0.0;
    }
  }

  target_yaw_ = deg * M_PI / 180.0;

  if (is_rotating_) {
    RCLCPP_INFO(get_logger(), "Curr: %f; Targ: %f; Err: %f", current_yaw_,
                target_yaw_, std::abs(current_yaw_ - target_yaw_));

    initial_yaw_ = current_yaw_;

    if (std::abs(current_yaw_ - target_yaw_) > 0.05) {
      cmd_vel_msg.angular.z = -0.2;
      cmd_vel_msg.linear.x = 0.0;
      is_rotating_ = true;
    } else {
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Pre-Operation Successful!");
      operation = false;
    }
  }

  cmd_vel_publisher_->publish(cmd_vel_msg);
}
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
