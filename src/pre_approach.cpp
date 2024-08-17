#include "components_attach_shelf/pre_approach.hpp"

#include <chrono>
#include <functional>
#include <ios>
#include <sstream>
#include <string>

#include "components_attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals;
using std::placeholders::_1;
using GoToLoading = components_attach_shelf::srv::GoToLoading;

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach", options), timer_{this->create_wall_timer(
                                         100ms,
                                         std::bind(&PreApproach::on_timer,
                                                   this))},
      vel_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 1)},
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10, std::bind(&PreApproach::laser_scan_callback, this, _1))},
      odom_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&PreApproach::odometry_callback, this, _1))},
      client_{this->create_client<GoToLoading>("approach_shelf")},
      motion_{Motion::FORWARD}, moving_forward_{true}, turning_{false},
      have_odom_{false}, have_scan_{false}, laser_scanner_parametrized_{false} {
  obstacle_ = OBSTACLE;
  degrees_ = DEGREES;
  auto logger = this->get_logger();

  // Set the log level
  std::map<int, std::string> levels = {
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG, "DEBUG"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO, "INFO"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN, "WARN"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR, "ERROR"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL, "FATAL"}};

  // Set the log severity level here
  int level = RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO;

  if (rcutils_logging_set_logger_level(logger.get_name(), level) !=
      RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to set logger level '%s' for %s.",
                 (levels[level]).c_str(), this->get_name());
  } else {
    RCLCPP_INFO(logger, "Successfully set logger level '%s' for %s.",
                (levels[level]).c_str(), this->get_name());
  }

  RCLCPP_DEBUG(this->get_logger(), "obstacle_ = %f, degrees+ = %f", obstacle_,
               degrees_);

  wait_for_laser_scan_publisher();
  wait_for_odometery_publisher();
}

inline void PreApproach::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  last_laser_ = *msg;
  have_scan_ = true;
}

inline void
PreApproach::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //   RCLCPP_DEBUG(this->get_logger(), "Odometry callback");
  odom_data_ = *msg;
  yaw_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //   RCLCPP_DEBUG(this->get_logger(), "Current orientation is %f", yaw_);
  have_odom_ = true;
}

inline double PreApproach::yaw_from_quaternion(double x, double y, double z,
                                               double w) {
  return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

inline double PreApproach::normalize_angle(double angle) {
  double res = angle;
  while (res > PI_)
    res -= 2.0 * PI_;
  while (res < -PI_)
    res += 2.0 * PI_;
  return res;
}

void PreApproach::on_timer() {
  // Motion: go to wall (obstacle), then turn (degrees)
  // This is a fall-through loop, which sets the Twist
  // and publishes before exiting
  if (!have_odom_ || !have_scan_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for data");
    return;
  }

  if (!laser_scanner_parametrized_) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for laser scanner to be parametrized");
    parametrize_laser_scanner(last_laser_);
    return;
  }

  geometry_msgs::msg::Twist twist;

  switch (motion_) {
  case Motion::FORWARD:
    if (moving_forward_) {
      if (front_obstacle_dist() >= obstacle_ + LINEAR_TOLERANCE) {
        RCLCPP_DEBUG(this->get_logger(), "Approaching wall, distance = %f m",
                     front_obstacle_dist());
        twist.linear.x = LINEAR_BASE;
        twist.angular.z = 0.0;
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Stopping at wall, distance = %f m",
                     front_obstacle_dist());
        moving_forward_ = false;
        motion_ = Motion::TURN;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
      }
    }
    break;
  case Motion::TURN:
    static double last_angle;
    static double turn_angle;
    static double goal_angle;

    // if not turning, initialize to start
    if (!turning_) {
      last_angle = yaw_;
      turn_angle = 0;
      goal_angle = degrees_ * DEG2RAD;
    }

    if ((goal_angle > 0 &&
         (abs(turn_angle + ANGULAR_TOLERANCE) < abs(goal_angle))) ||
        (goal_angle < 0 && (abs(turn_angle - ANGULAR_TOLERANCE) <
                            abs(goal_angle)))) { // need to turn (more)
      RCLCPP_DEBUG(this->get_logger(), "Starting rotation, angle = %f deg",
                   turn_angle * RAD2DEG);
      twist.linear.x = 0.0;
      twist.angular.z = (goal_angle > 0) ? ANGULAR_BASE : -ANGULAR_BASE;

      double temp_yaw = yaw_;
      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;

      turning_ = true;
    } else {
      // reached goal angle within tolerance, stop turning
      //   RCLCPP_DEBUG(this->get_logger(), "Resulting yaw %f", yaw_);
      RCLCPP_DEBUG(this->get_logger(), "Stopping rotation, angle = %f deg",
                   turn_angle * RAD2DEG);
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;

      turning_ = false;
      motion_ = Motion::STOP;
    }
    break;
  case Motion::STOP:
    RCLCPP_INFO(this->get_logger(), "Pre-approach completed");
    // If robot not stopped, stop it, else call service
    if (abs(twist.linear.x) > 0.0 || abs(twist.angular.z) > 0.0) {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Robot stopped");
      timer_->cancel(); // timer not needed
    }
    break;
  default:
    RCLCPP_WARN(this->get_logger(), "Unrecognized motion. Stopping robot");
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
  }

  vel_pub_->publish(twist);
}

void PreApproach::parametrize_laser_scanner(
    sensor_msgs::msg::LaserScan &scan_data) {
  int size = static_cast<int>(scan_data.ranges.size());
  front = static_cast<int>(round(size / 2.0));
  RCLCPP_DEBUG(this->get_logger(), "front index = %d", front);
  RCLCPP_DEBUG(this->get_logger(), "front range = %f",
               last_laser_.ranges[front]);

  laser_scanner_parametrized_ = true;

  RCLCPP_INFO(this->get_logger(), "Laser scanner parametrized");
}

double PreApproach::front_obstacle_dist() {
  double front_dist = 0.0;
  for (int i = front - FRONT_FANOUT; i <= front + FRONT_FANOUT; ++i) {
    if (!std::isinf(last_laser_.ranges[i]) &&
        last_laser_.ranges[i] > front_dist)
      front_dist = last_laser_.ranges[i];
  }
  return front_dist;
}

void PreApproach::wait_for_laser_scan_publisher() {
  // ROS 2 does't have an equivalent to wait_for_publisher
  // this is one way to solve the problem
  while (this->count_publishers("scan") == 0) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for 'scan' topic publisher. Exiting.");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "'scan' topic publisher not available, waiting...");
  }
  RCLCPP_INFO(this->get_logger(), "'scan' topic publisher acquired");
}

void PreApproach::wait_for_odometery_publisher() {
  // ROS 2 does't have an equivalent to wait_for_publisher
  // this is one way to solve the problem
  while (this->count_publishers("odom") == 0) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for 'odom' topic publisher. Exiting.");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "'odom' topic publisher not available, waiting...");
  }
  RCLCPP_INFO(this->get_logger(), "'odom' topic publisher acquired");
}

void PreApproach::service_response_callback(
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

// Register as component (shared library)

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)