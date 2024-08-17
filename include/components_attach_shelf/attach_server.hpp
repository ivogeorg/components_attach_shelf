#ifndef ATTACH_SERVER_HPP_
#define ATTACH_SERVER_HPP_

/**
 * @file attach_server.hpp
 * @brief Implements a final approach server component for the RB1 robot.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include <tuple>

#include "components_attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace my_components {

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals;
using GoToLoading = components_attach_shelf::srv::GoToLoading;
using LaserScan = sensor_msgs::msg::LaserScan;

/**
 * @class CustomSubscriptionOptions
 * @brief Used to set callback groups for topic subsctiptions.
 * A very simple derived class to allow init-list initialization
 * of callback groups for topic subscriptions due to the signature
 * of rclcpp::Node::create_subscription.
 */
// Necessary for topic subscriptions
class CustomSubscriptionOptions : public rclcpp::SubscriptionOptions {
public:
  CustomSubscriptionOptions(rclcpp::CallbackGroup::SharedPtr cb_group)
      : rclcpp::SubscriptionOptions() {
    this->callback_group = cb_group;
  }
  ~CustomSubscriptionOptions() = default;
};

/**
 * @class AttachServer
 * @brief A multi-callback node implementing the final approach service.
 */
class AttachServer : public rclcpp::Node {
private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  CustomSubscriptionOptions topic_pub_sub_options_;

  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_up_pub_;

  bool have_scan_;
  bool have_odom_;
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry last_odom_;
  double last_yaw_;

  const double REFLECTIVE_INTENSITY_VALUE = 8000; // for reflective points
  const int POINT_DIST_THRESHOLD = 10;            // for point set segmentation

  std::string odom_frame_;
  std::string laser_frame_;
  std::string cart_frame_;

  bool broadcast_odom_cart_;  // connecting cart_frame to TF tree
  bool listen_to_odom_laser_; // odom_laser_t_ serves as basis for odom_cart_t_
  bool listen_to_robot_base_cart_; // laser_cart_t_ serves in the final approach

  geometry_msgs::msg::TransformStamped odom_cart_t_;
  geometry_msgs::msg::TransformStamped odom_laser_t_;
  geometry_msgs::msg::TransformStamped laser_cart_t_;
  geometry_msgs::msg::TransformStamped base_cart_t_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; // TODO: Consider StaticTL
  rclcpp::TimerBase::SharedPtr listener_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

  const double LINEAR_TOLERANCE = 0.08;  // m
  const double ANGULAR_TOLERANCE = 0.01; // rad
  const double LINEAR_BASE = 0.5;        // m/s
  const double ANGULAR_BASE = 0.1;       // rad/s

  /**
   * @class MotionDirection
   * @brief An enum type for the directions of motion.
   * @see method move() uses as parameter
   */
  enum class MotionDirection { FORWARD, BACKWARD };

  /**
   * @brief Linear distance the robot should correct for.
   * Before the robot rotates to orient itself toward the
   * origin of `cart_frame`, it should move forward the
   * distance between `robot_base_frame` and
   * `robot_front_laser_base_link`.
   */
  const double BASE_LINK_TO_LASER_LINK = 0.21; // m

public:
  AttachServer(const rclcpp::NodeOptions &options);
  ~AttachServer() = default;

private:
  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  inline void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  inline double normalize_angle(double angle);
  inline double yaw_from_quaternion(double x, double y, double z, double w);
  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response);
  std::vector<std::vector<int>> segment(std::vector<int> &v,
                                        const int threshold);
  void listener_cb();
  void broadcaster_cb();
  std::tuple<double, double, double>
  solve_sas_triangle(double left_side, double right_side, double sas_angle);
  void move(double dist_m, MotionDirection dir, double speed);
  void turn(double angle_rad, double speed);
};

} // namespace my_components

#endif // ATTACH_SERVER_HPP_