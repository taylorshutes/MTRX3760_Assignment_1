#include "simple_driver/simple_driver.hpp"
#include <chrono>

using std::chrono::milliseconds;

SimpleDriver::SimpleDriver() : rclcpp::Node("simple_driver")
{
  // Publisher: TwistStamped messages to /cmd_vel
  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

  // Subscriber: listen to /scan topic
  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS().best_effort(),
    std::bind(&SimpleDriver::scan_callback, this, std::placeholders::_1));

  // Timer: publish every 100ms (10 Hz)
  timer_ = this->create_wall_timer(
    milliseconds(100),
    std::bind(&SimpleDriver::on_timer, this));

  state_start_time_ = this->now();
}


SimpleDriver::~SimpleDriver()
{
  stop_robot();  // ensure robot stops on destruction
}



void SimpleDriver::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "scan angle_min = %.3f rad", msg->angle_min);
}

void SimpleDriver::on_timer()
{
  // calculate how long since entered state
  auto elapsed = (this->now() - state_start_time_).seconds();

  // fill in header of msg
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";   // adjust if needed

  switch (state_) {
    case DriveState::FORWARD:
      if (elapsed < 2.0) { // drive forward
        cmd.twist.linear.x = 0.2;
        cmd.twist.angular.z = 0.0;
      } 

      else {  // start turning
        state_ = DriveState::TURN;
        state_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Switching to TURN state");
      }
      break;

    case DriveState::TURN:
      if (elapsed < 2.0) {  // turn
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 1.57 / 2.0;   // pi/2 over 2 secs
      } 

      else {  // start going forward
        state_ = DriveState::FORWARD;
        state_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Switching to FORWARD state");
      }
      break;


      default:
        RCLCPP_WARN(this->get_logger(), "Unknown state!");
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
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;

  pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Stop command published");

}
