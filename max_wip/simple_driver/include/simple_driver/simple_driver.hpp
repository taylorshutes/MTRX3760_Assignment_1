#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

enum class DriveState { FORWARD, TURN };

class SimpleDriver : public rclcpp::Node {
public:
  SimpleDriver();
  ~SimpleDriver();

  void stop_robot();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_timer();

  DriveState state_;
  rclcpp::Time state_start_time_;  

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
