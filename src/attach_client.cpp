#include "components_attach_shelf/attach_client.hpp"

#include <chrono>
#include <functional>

#include "components_attach_shelf/srv/go_to_loading.hpp"
#include "components_attach_shelf/visibility_control.h"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

using GoToLoading = components_attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options), client_{this->create_client<GoToLoading>(
                                          "approach_shelf")} {

  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = true; // final approach will be performed

  auto result_future = client_->async_send_request(
      request, std::bind(&AttachClient::service_response_callback, this,
                         std::placeholders::_1));
}

void AttachClient::service_response_callback(
    rclcpp::Client<GoToLoading>::SharedFuture future) {
  auto status = future.wait_for(1s);
  bool final_approach_complete = false;
  if (status != std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Service '/approach_shelf' in progress...");
  } else {
    RCLCPP_DEBUG(this->get_logger(),
                 "Service '/approach_shelf' response received");
    auto result = future.get();
    final_approach_complete = result->complete;
    RCLCPP_INFO(this->get_logger(), "Final approach complete: '%s'",
                final_approach_complete ? "true" : "false");
  }
}

} // namespace my_components

// namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)