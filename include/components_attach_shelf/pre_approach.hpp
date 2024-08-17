#ifndef PRE_APPROACH_HPP_
#define PRE_APPROACH_HPP_

#include "components_attach_shelf/visibility_control.h"
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

// using namespace std::chrono_literals;
// using std::placeholders::_1;
using GoToLoading = components_attach_shelf::srv::GoToLoading;

class PreApproach : public rclcpp::Node {
private:
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry odom_data_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<GoToLoading>::SharedPtr client_;

  enum class Motion { FORWARD, TURN, STOP };
  Motion motion_;

  const double LINEAR_BASE = 0.5;
  const double ANGULAR_BASE = 0.45;
  const double LINEAR_TOLERANCE = 0.005;  // meters
  const double ANGULAR_TOLERANCE = 0.012; // about 2/3 of a deg
  const double ANGULAR_TOLERANCE_DEG = ANGULAR_TOLERANCE * RAD2DEG;
  bool moving_forward_;
  bool turning_;
  bool have_odom_;
  bool have_scan_;
  bool laser_scanner_parametrized_;
  double yaw_;
  const int FRONT_FANOUT = 4;
  double angle_increment, range_min;
  int front;

  double obstacle_;
  double degrees_;

  // Constants
  const double OBSTACLE = 0.45;
  const double DEGREES = -90.0;

public:
  PreApproach(const rclcpp::NodeOptions & options);
  ~PreApproach() = default;

private:
  void on_timer();
  void parametrize_laser_scanner(sensor_msgs::msg::LaserScan &scan_data);
  double front_obstacle_dist();
  void wait_for_laser_scan_publisher();
  void wait_for_odometery_publisher();
  void service_response_callback(rclcpp::Client<GoToLoading>::SharedFuture future);

  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  inline void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  inline double yaw_from_quaternion(double x, double y, double z, double w);
  inline double normalize_angle(double angle);
};

} // namespace my_components

#endif // PRE_APPROACH_HPP_

