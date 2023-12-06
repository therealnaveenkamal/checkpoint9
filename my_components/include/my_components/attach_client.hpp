#ifndef COMPOSITION__ATTACH_CLIENT_HPP_
#define COMPOSITION__ATTACH_CLIENT_HPP_

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>
#include <math.h>
#include <std_msgs/msg/string.hpp>

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;

protected:
  void timer_callback();
  void response_callback(
      rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future);
};

} // namespace my_components

#endif // COMPOSITION__ATTACH_CLIENT_HPP_
