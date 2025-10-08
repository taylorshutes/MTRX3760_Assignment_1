#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

enum class TaylorState { FORWARD, TURN, RIGHT_WALL_FIND };

class TaylorDriver : public rclcpp::Node {
public:
  TaylorDriver();
  ~TaylorDriver();
  void stop_robot();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_timer();

  double normalize_angle(double a);
  bool reached_target_yaw(double tol);

  // --- State
  TaylorState state_;
  double current_yaw_;
  double target_yaw_;

  // --- Distances
  float front_distance_;
  float right_distance_;   // ✅ added
  float left_distance_;
  float front_right_distance_;

  // --- ROS
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

