#ifndef ATTACH_CLIENT_HPP_
#define ATTACH_CLIENT_HPP_

#include "components_attach_shelf/srv/go_to_loading.hpp"
#include "components_attach_shelf/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

using GoToLoading = components_attach_shelf::srv::GoToLoading;

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);
  ~AttachClient() = default;

private:
  rclcpp::Client<GoToLoading>::SharedPtr client_;
  void
  service_response_callback(rclcpp::Client<GoToLoading>::SharedFuture future);
};

} // namespace my_components

#endif // ATTACH_CLIENT_HPP_