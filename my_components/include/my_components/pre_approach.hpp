#ifndef COMPOSITION__PRE_APPROACH_HPP_
#define COMPOSITION__PRE_APPROACH_HPP_

#include "my_components/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>
#include <math.h>

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  int obs;
  int deg;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  bool is_rotating_ = false;
  double initial_yaw_;
  double target_yaw_;
  double current_yaw_;
  bool operation = true;

protected:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace composition

#endif  // COMPOSITION__PRE_APPROACH_HPP_
