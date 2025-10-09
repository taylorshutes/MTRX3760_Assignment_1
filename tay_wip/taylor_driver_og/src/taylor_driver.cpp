#include "taylor_driver/taylor_driver.hpp"
#include <limits>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

TaylorDriver::TaylorDriver() : rclcpp::Node("taylor_driver")
{
  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS().best_effort(),
    std::bind(&TaylorDriver::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&TaylorDriver::odom_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(100ms, std::bind(&TaylorDriver::on_timer, this));

  front_distance_ = std::numeric_limits<float>::infinity();
  current_yaw_ = 0.0;
  target_yaw_ = 0.0;
  state_ = TaylorState::FORWARD;

  RCLCPP_INFO(this->get_logger(), "Taylor driver started (forward + 90° turns)");
}

TaylorDriver::~TaylorDriver()
{
  stop_robot();
}

void TaylorDriver::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int n = msg->ranges.size();
  if (n == 0) return;

  float angle_min = msg->angle_min;
  float angle_inc = msg->angle_increment;

  auto get_min_range_at_angle = [&](float center_angle, float window) {
    float min_dist = msg->range_max;
    int center_idx = static_cast<int>((center_angle - angle_min) / angle_inc);
    int half_window = static_cast<int>(window / angle_inc);

    for (int i = center_idx - half_window; i <= center_idx + half_window; i++) {
      if (i >= 0 && i < n) {
        float r = msg->ranges[i];
        if (!std::isnan(r) && !std::isinf(r)) {
          min_dist = std::min(min_dist, r);
        }
      }
    }
    return min_dist;
  };

  // ✅ This is the orientation that behaved correctly in your maze
  // 0 rad = forward, −90° = right, +90° = left
  front_distance_ = get_min_range_at_angle(0.0, 0.17);        // 0° ±10°

  right_distance_ = get_min_range_at_angle(3 * M_PI / 2 + 0.3, 0.26);   // 270° + ~17° (tilt back a little)

  left_distance_  = get_min_range_at_angle(M_PI / 2, 0.26);   // +90° ±15°

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Front=%.2f, Right=%.2f, Left=%.2f, Yaw=%.2f",
                       front_distance_, right_distance_, left_distance_, current_yaw_);
}

void TaylorDriver::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
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

void TaylorDriver::on_timer()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";

  static int pause_ticks = 0;  // counter for short pauses
  double angular_speed = 0.6;
  double tol = 0.05;
  double stop_dist = 0.5;
  double free_limit = 0.6;

  switch (state_)
  {
    case TaylorState::FORWARD:
    {
      if (front_distance_ < stop_dist)
      {
        // stop and print once
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;

        if (pause_ticks == 0)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Wall ahead! F=%.2f  R=%.2f  L=%.2f  Yaw=%.2f",
                      front_distance_, right_distance_, left_distance_, current_yaw_);
        }

        pause_ticks++;

        // stay stopped for ~1.5 seconds (15 * 100ms = 1.5s)
        if (pause_ticks > 15)
        {
          pause_ticks = 0;

        // --- decide which turn to make ---
        if (right_distance_ > free_limit && left_distance_ > free_limit)
        {
          // both sides open → choose the wider one
          if (right_distance_ > left_distance_)
          {
            state_ = TaylorState::TURN;
            target_yaw_ = normalize_angle(current_yaw_ - M_PI / 2);
            RCLCPP_INFO(this->get_logger(),
                        "Both open → right slightly wider → turning right");
          }
          else
          {
            state_ = TaylorState::TURN;
            target_yaw_ = normalize_angle(current_yaw_ + M_PI / 2);
            RCLCPP_INFO(this->get_logger(),
                        "Both open → left wider → turning left");
          }
        }
        else if (right_distance_ > free_limit)
        {
          state_ = TaylorState::TURN;
          target_yaw_ = normalize_angle(current_yaw_ - M_PI / 2);
          RCLCPP_INFO(this->get_logger(),
                      "Right free → turning right 90°");
        }
        else if (left_distance_ > free_limit)
        {
          state_ = TaylorState::TURN;
          target_yaw_ = normalize_angle(current_yaw_ + M_PI / 2);
          RCLCPP_INFO(this->get_logger(),
                      "Right blocked, left free → turning left 90°");
        }
        else
        {
          state_ = TaylorState::TURN;
          target_yaw_ = normalize_angle(current_yaw_ + M_PI);
          RCLCPP_INFO(this->get_logger(),
                      "Dead end → turning 180°");
        }

        }
      }
      else
      {
        cmd.twist.linear.x = 0.2;
        pause_ticks = 0;  // reset pause timer if no wall
      }
      break;
    }

    case TaylorState::TURN:
      if (!reached_target_yaw(tol))
      {
        cmd.twist.angular.z =
            (normalize_angle(target_yaw_ - current_yaw_) > 0)
                ? angular_speed
                : -angular_speed;
      }
      else
      {
        state_ = TaylorState::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Turn complete → forward again");
      }
      break;
  }

  pub_->publish(cmd);
}

void TaylorDriver::stop_robot()
{
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_link";
  msg.twist.linear.x = 0.0;
  msg.twist.angular.z = 0.0;
  pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Stop command published");
}

double TaylorDriver::normalize_angle(double a)
{
  while (a > M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

bool TaylorDriver::reached_target_yaw(double tol)
{
  double diff = normalize_angle(target_yaw_ - current_yaw_);
  return fabs(diff) < tol;
}
