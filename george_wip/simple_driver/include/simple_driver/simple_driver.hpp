#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

enum class DriveState { FORWARD, TURN_RIGHT_CHECK, TURN_LEFT_CHECK, TURN_BACK };

class SimpleDriver : public rclcpp::Node {
public:
  SimpleDriver();
  ~SimpleDriver();

  void stop_robot();

private:
  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_timer();

  // Helpers
  double normalize_angle(double a);
  bool reached_target_yaw(double tol);

  // State machine
  DriveState state_;
  double current_yaw_;
  double target_yaw_;

  // Sensor distances
  float front_distance_;
  float left_distance_;
  float right_distance_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

