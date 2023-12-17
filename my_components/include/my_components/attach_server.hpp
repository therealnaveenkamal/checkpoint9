#ifndef COMPOSITION__ATTACH_SERVER_HPP_
#define COMPOSITION__ATTACH_SERVER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>
#include <math.h>
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/msg/string.hpp"

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  std::vector<double> msgvalue;
  static const size_t NUM_LASER_READINGS = 1080;
  std::vector<int> globalLegPos;
  double angleLegs = 0.0;
  double xDistance = 0.0;
  double yDistance = 0.0;
  double xDistance_once = 0.0;
  double yDistance_once = 0.0;
  bool ready = false;
  bool move_extra = false;
  bool elevated = false;
  int extratime = 12;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  tf2::Quaternion cart_quat_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_stamped;

  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher_down;

protected:
  void handle_approach_request(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);
  void timer_callback();
  void publish_cart_frame_transform();
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

} // namespace my_components

#endif // COMPOSITION__ATTACH_SERVER_HPP_
