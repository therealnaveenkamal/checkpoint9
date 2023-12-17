#include "my_components/attach_server.hpp"
#include "rclcpp/utilities.hpp"

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("attach_server", options) {
  auto clock = this->get_clock();
  tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  approach_service_ = this->create_service<attach_shelf::srv::GoToLoading>(
      "/approach_shelf",
      std::bind(&AttachServer::handle_approach_request, this,
                std::placeholders::_1, std::placeholders::_2));

  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&AttachServer::scan_callback, this, std::placeholders::_1));

  msgvalue.resize(NUM_LASER_READINGS, 0.0);
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000),
                              std::bind(&AttachServer::timer_callback, this));

  elevator_publisher =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
      elevator_publisher_down =
      this->create_publisher<std_msgs::msg::String>("/elevator_down", 10);
}

void AttachServer::handle_approach_request(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(), "------%d", globalLegPos.size());
  if (globalLegPos.size() != 2) {
    response->complete = false;
  }

  if (request->attach_to_shelf) {
    RCLCPP_INFO(this->get_logger(), "------%d", msgvalue.size());

    RCLCPP_INFO(get_logger(), "Printing leg positions:");
    for (const auto &value : globalLegPos) {
      RCLCPP_INFO(get_logger(), "Value: %d", value);
    }

    RCLCPP_INFO(this->get_logger(), "------%d", msgvalue.size());
    ready = true;
    response->complete = true;

  } else {
    response->complete = false;
  }
}

void AttachServer::timer_callback() {
  if (move_extra && !elevated) {
    if (extratime >= 0) {
      //RCLCPP_INFO(this->get_logger(), "Move 30 cm INIT: %d", extratime);
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_msg.linear.x = 0.18;
      cmd_vel_publisher_->publish(cmd_vel_msg);
      extratime--;
    } else {
      RCLCPP_INFO(this->get_logger(), "Elevation Start");
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_msg);

      std_msgs::msg::String msg;
      elevator_publisher->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Elevation Successfull");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std_msgs::msg::String msg1;
      elevator_publisher_down->publish(msg1);

      elevated = true;
      RCLCPP_INFO(this->get_logger(),
                  "Attach Operation Successful, Shutting Down!");
      rclcpp::shutdown();
    }
  }
}

void AttachServer::publish_cart_frame_transform() {
  geometry_msgs::msg::Twist cmd_vel_msg;

  if (ready) {

    try {
      auto odom_to_base_transform = tf_buffer->lookupTransform(
          "odom", "robot_base_link", tf2::TimePointZero);
      auto base_to_laser_transform = tf_buffer->lookupTransform(
          "robot_base_link", "robot_front_laser_base_link", tf2::TimePointZero);
      tf2::Vector3 base_coords(xDistance, yDistance, 0.0);
      tf2::Quaternion laser_quat;
      tf2::fromMsg(base_to_laser_transform.transform.rotation, laser_quat);

      tf2::Vector3 laser_coords = tf2::quatRotate(laser_quat, base_coords);

      tf2::Quaternion odom_quat;
      tf2::fromMsg(odom_to_base_transform.transform.rotation, odom_quat);

      tf2::Vector3 odom_coords = tf2::quatRotate(odom_quat, laser_coords);
      odom_coords +=
          tf2::Vector3(odom_to_base_transform.transform.translation.x,
                       odom_to_base_transform.transform.translation.y,
                       odom_to_base_transform.transform.translation.z);

      if (ready) {

        if (xDistance_once == 0.0) {
          xDistance_once = odom_coords.x();
        }

        if (yDistance_once == 0.0) {
          yDistance_once = odom_coords.y();
        }

        transform_stamped.header.stamp = odom_to_base_transform.header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "cart_frame";
        transform_stamped.transform.translation.x = xDistance_once;
        transform_stamped.transform.translation.y = yDistance_once;
        transform_stamped.transform.translation.z = odom_coords.z();
        transform_stamped.transform.rotation.x = cart_quat_.x();
        transform_stamped.transform.rotation.y = cart_quat_.y();
        transform_stamped.transform.rotation.z = cart_quat_.z();
        transform_stamped.transform.rotation.w = cart_quat_.w();
        broadcaster->sendTransform(transform_stamped);

        double tfDistance = sqrt(pow(xDistance, 2) + pow(yDistance, 2));
        RCLCPP_INFO(this->get_logger(), "Dist: %f", tfDistance);

        if (tfDistance >= 0.4) {
          cmd_vel_msg.linear.x = 0.1;
          cmd_vel_msg.angular.z = 0.0;
          cmd_vel_publisher_->publish(cmd_vel_msg);
        } else {
          xDistance_once = odom_coords.x();
          yDistance_once = odom_coords.y();
          cmd_vel_msg.linear.x = 0.0;
          cmd_vel_msg.linear.y = 0.0;
          cmd_vel_msg.angular.z = 0.0;
          cmd_vel_publisher_->publish(cmd_vel_msg);

          ready = false;
          move_extra = true;
        }
      } else if (ready == false && move_extra == true) {
      }

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
      return;
    }
  }
}

void AttachServer::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  if (ready) {
    for (size_t i = 0; i < msg->intensities.size(); ++i) {
      msgvalue[i] = msg->ranges[i];
    }

    double threshold = 7000;
    int j = 0;
    std::vector<int> legPositions;

    float tempIntensity = 0.0;

    for (int i = 1; i < msg->intensities.size() - 1; ++i) {
      if (msg->intensities[i] > threshold &&
          msg->intensities[i + 1] > threshold) {
        tempIntensity = 0.0;
        if (i + 10 < msg->intensities.size()) {
          for (j = i; j <= i + 10; ++j) {
            tempIntensity += msg->intensities[j];
          }
          if (tempIntensity >= 60000) {
            legPositions.push_back(i + 5);
            i = j;
          }
        }
      }
    }

    globalLegPos = legPositions;

    if (legPositions.size() == 2) {
      double distance =
          (msg->ranges[legPositions[0]] + msg->ranges[legPositions[1]]) / 2.0;
      angleLegs =
          (msg->angle_min +
           ((legPositions[0] + legPositions[1]) / 2.0) * msg->angle_increment);

      xDistance = distance * cos(angleLegs);
      yDistance = distance * sin(angleLegs);

      cart_quat_.setRPY(0.0, 0.0, angleLegs);

      RCLCPP_INFO(this->get_logger(), "%d, %d, (%f, %f)", legPositions[0],
                  legPositions[1], xDistance, yDistance);

      publish_cart_frame_transform();
    } else {
      // RCLCPP_INFO(this->get_logger(), "TWO LEGS NOT FOUND");
      publish_cart_frame_transform();
    }
  }
}
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)