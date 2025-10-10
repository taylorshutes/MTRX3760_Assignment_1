#ifndef BASE_DRIVER_HPP
#define BASE_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

/**
 The BaseDriver class is a platform upon which different drivers can be built. 
 It periodically calls for a new command for the robot to be computed, sets up subs/pubs
 all driver will need, and stores data that would be needed by all drivers about the current state of the turtlebot
 */
class BaseDriver : public rclcpp::Node
{
public:
  /**
   * Construct a BaseDriver node with a given name, initialising all ROS interfaces.
   */
  BaseDriver(const std::string &node_name);

  /**
   * Destructor – cleanly stop the robot and log shutdown.
   */
  virtual ~BaseDriver();

protected:
  /**
   * Called periodically by timer to request motion command from derived class.
   */
  void on_timer();

  /**
   * Must be implemented by subclasses to compute a TwistStamped command.
   */
  virtual geometry_msgs::msg::TwistStamped compute_command() = 0;

  /**
   * Publishes a zero TwistStamped message to immediately stop the robot.
   */
  void stop_robot();

  // --- Common data for all drivers ---
  double current_yaw_{0.0};
  double front_distance_{0.0};
  double right_distance_{0.0};
  double left_distance_{0.0};
  double front_right_distance_{0.0};
  std::string last_colour_{"none"};

  // --- ROS Interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wall_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Shared Callbacks ---
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void wall_callback(const std_msgs::msg::String::SharedPtr msg);
};

#endif // BASE_DRIVER_HPP