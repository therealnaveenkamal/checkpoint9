#include "my_components/attach_client.hpp"

namespace my_components {
AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {
  client =
      this->create_client<attach_shelf::srv::GoToLoading>("approach_shelf");

  timer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&AttachClient::timer_callback, this));

        
}

void AttachClient::timer_callback() {
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Service not available");
    return;
  }

  auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
  request->attach_to_shelf = true;

  auto result = client->async_send_request(
      request,
      std::bind(&AttachClient::response_callback, this, std::placeholders::_1));
}

void AttachClient::response_callback(
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
  RCLCPP_INFO(this->get_logger(), "Service response: %d",
              future.get()->complete);
  timer->cancel();
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
