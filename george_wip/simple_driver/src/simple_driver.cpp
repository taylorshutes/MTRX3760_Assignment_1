#include "simple_driver/simple_driver.hpp"
#include <limits>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono;

SimpleDriver::SimpleDriver() : rclcpp::Node("simple_driver")
{
  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS().best_effort(),
    std::bind(&SimpleDriver::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&SimpleDriver::odom_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    100ms,
    std::bind(&SimpleDriver::on_timer, this));

  front_distance_ = left_distance_ = right_distance_ = std::numeric_limits<float>::infinity();

  state_ = DriveState::FORWARD;
  current_yaw_ = 0.0;
  target_yaw_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Right-first wall-follow driver with ODOM started");
}

SimpleDriver::~SimpleDriver()
{
  stop_robot();
}

void SimpleDriver::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;  // radians, between -pi and pi
}

void SimpleDriver::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int n = msg->ranges.size();
  if (n == 0) return;

  float angle_min = msg->angle_min;
  float angle_inc = msg->angle_increment;

  auto get_range_at_angle = [&](float angle) {
    int idx = (int)((angle - angle_min) / angle_inc);
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;
    float r = msg->ranges[idx];
    if (std::isnan(r) || std::isinf(r)) return msg->range_max;
    return r;
  };

  // Angles: front = 0 rad, right = -90° = -1.57 rad, left = +90° = 1.57 rad
  front_distance_ = get_range_at_angle(0.0);
  right_distance_ = get_range_at_angle(-1.57);
  left_distance_  = get_range_at_angle(1.57);
}

void SimpleDriver::on_timer()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";

  double stop_dist = 0.5;
  double angular_speed = 0.6;   // rad/s
  double tol = 0.1;             // rad tolerance (~6°)

  switch (state_) {
    case DriveState::FORWARD:
      if (front_distance_ < stop_dist) {
        state_ = DriveState::TURN_RIGHT_CHECK;
        target_yaw_ = normalize_angle(current_yaw_ - M_PI/2);
        RCLCPP_INFO(this->get_logger(), "Blocked ahead, turning right to check");
      } else {
        cmd.twist.linear.x = 0.2;
      }
      break;

    case DriveState::TURN_RIGHT_CHECK:
      if (!reached_target_yaw(tol)) {
        cmd.twist.angular.z = -angular_speed;
      } else {
        if (right_distance_ > stop_dist) {
          state_ = DriveState::FORWARD;
          RCLCPP_INFO(this->get_logger(), "Right side clear → go forward");
        } else {
          state_ = DriveState::TURN_LEFT_CHECK;
          target_yaw_ = normalize_angle(current_yaw_ + M_PI); // 180°
          RCLCPP_INFO(this->get_logger(), "Right blocked, turning left 180°");
        }
      }
      break;

    case DriveState::TURN_LEFT_CHECK:
      if (!reached_target_yaw(tol)) {
        cmd.twist.angular.z = angular_speed;
      } else {
        if (left_distance_ > stop_dist) {
          state_ = DriveState::FORWARD;
          RCLCPP_INFO(this->get_logger(), "Left side clear → go forward");
        } else {
          state_ = DriveState::TURN_BACK;
          target_yaw_ = normalize_angle(current_yaw_ + M_PI/2); // +90°
          RCLCPP_INFO(this->get_logger(), "Left also blocked → turning back 90°");
        }
      }
      break;

    case DriveState::TURN_BACK:
      if (!reached_target_yaw(tol)) {
        cmd.twist.angular.z = angular_speed;
      } else {
        state_ = DriveState::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Heading back the way we came");
      }
      break;
  }

  pub_->publish(cmd);
}

void SimpleDriver::stop_robot()
{
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header.frame_id = "base_link";
  msg.header.stamp = this->get_clock()->now();
  msg.twist.linear.x = 0.0;
  msg.twist.angular.z = 0.0;
  pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Stop command published");
}

// === Helper functions ===
double SimpleDriver::normalize_angle(double a) {
  while (a > M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

bool SimpleDriver::reached_target_yaw(double tol) {
  double diff = normalize_angle(target_yaw_ - current_yaw_);
  return fabs(diff) < tol;
}
