#include "maze_driver/base_driver.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

// --- Constructor ---
BaseDriver::BaseDriver(const std::string &node_name)
: rclcpp::Node(node_name)
{
  // create necessary pub/subs
  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS().best_effort(),
    std::bind(&BaseDriver::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&BaseDriver::odom_callback, this, std::placeholders::_1));

  wall_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/wall_colour", 10,
    std::bind(&BaseDriver::wall_callback, this, std::placeholders::_1));
  
  // start timer
  timer_ = this->create_wall_timer(100ms, std::bind(&BaseDriver::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "BaseDriver initialised for node: %s", node_name.c_str());
}

// --- Destructor ---
BaseDriver::~BaseDriver()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down BaseDriver node: %s", this->get_name());
  stop_robot();
}

// --- Periodic timer handler ---
void BaseDriver::on_timer()
{
  auto cmd = compute_command();  // derived class override
  pub_->publish(cmd);
}

// --- Stop the robot immediately ---
void BaseDriver::stop_robot()
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.twist.linear.x = 0.0;
  msg.twist.angular.z = 0.0;
  pub_->publish(msg);
}

// --- Common sensor callbacks ---
void BaseDriver::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (msg->ranges.empty()) return;


  auto get_min_range_at_angle = [&](float center_angle, float window) {
    float min_dist = msg->range_max;
    int n = msg->ranges.size();
    float angle_min = msg->angle_min;
    float angle_inc = msg->angle_increment;
    int center_idx = static_cast<int>((center_angle - angle_min) / angle_inc);
    int half_window = static_cast<int>(window / angle_inc);

    for (int i = center_idx - half_window; i <= center_idx + half_window; ++i) {
      if (i >= 0 && i < n) {
        float r = msg->ranges[i];
        if (!std::isnan(r) && !std::isinf(r))
          min_dist = std::min(min_dist, r);
      }
    }
    return min_dist;
  };

  // compute distances of obstacles around the turtlebot
  front_distance_       = get_min_range_at_angle(0.0, 0.17);
  right_distance_       = get_min_range_at_angle(3 * M_PI / 2 + 0.3, 0.26);
  left_distance_        = get_min_range_at_angle(M_PI / 2, 0.26);
  front_right_distance_ = get_min_range_at_angle(7 * M_PI / 4, 0.26);
}

void BaseDriver::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // retrieve values from odometer
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

void BaseDriver::wall_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_colour_ = msg->data;
}
