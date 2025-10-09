#include "taylor_driver/taylor_driver.hpp"
#include "taylor_driver/taylor_driver.hpp"
#include <limits>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/string.hpp>

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

  wall_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/wall_colour", 10, 
    std::bind(&TaylorDriver::wall_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(100ms, std::bind(&TaylorDriver::on_timer, this));

  front_distance_ = std::numeric_limits<float>::infinity();
  current_yaw_ = 0.0;
  target_yaw_ = 0.0;
  state_ = TaylorState::FORWARD;
  right_wall_seen_ = 0;
  green_wall_seen_ = 0;
  skip_right_ = 0;
  counter = 0;
  target_counter = 1;

  RCLCPP_INFO(this->get_logger(), "Taylor driver started (forward + 90° turns)");
}

TaylorDriver::~TaylorDriver()
{
  stop_robot();
}

void TaylorDriver::wall_callback(const std_msgs::msg::String::SharedPtr msg)
{
last_colour_ = msg->data;
if (last_colour_ == "red") {
  right_wall_seen_ = 1; // skip the very next right opportunity
//RCLCPP_INFO(this->get_logger(), "ColourDetector: RED → set skip_right_=1 (one-shot)");
} else if (last_colour_ == "green") {
  green_wall_seen_ = 1;
}
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

  front_right_distance_ = get_min_range_at_angle(7 * M_PI / 4, 0.26);  // ~315° ±7.5°

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
  double forward_speed = 0.2;
  double tol = 0.05;
  double stop_dist = 0.4;
  double open_dist = 0.7;
  double free_limit = 0.6;


  switch (state_)
  {
    case TaylorState::FORWARD:
    {

      const bool right_open = ((right_distance_ > open_dist) && (right_distance_ < 3.5));
      const bool front_open = (front_distance_ > stop_dist);
      const bool left_open = (left_distance_ > open_dist);
      
      if (right_open)
      {
        if (skip_right_)
        {    
          cmd.twist.linear.x = forward_speed;
          cmd.twist.angular.z = 0.0;

          target_counter++;

          if (pause_ticks == 0)
          {
            RCLCPP_INFO(get_logger(), "Skipping this right (one-shot)");
          }

          pause_ticks++;

          // stay stopped for ~1 second (10 * 100ms = 1s)
          if (pause_ticks > 30)
          {
            pause_ticks = 0;
            skip_right_ = 0;
            state_ = TaylorState::RIGHT_WALL_FIND;
          }
        }

        else
        {
          // stop and print once
          cmd.twist.linear.x = forward_speed;
          cmd.twist.angular.z = 0.0;

          if (pause_ticks == 0)
          {
            RCLCPP_INFO(this->get_logger(),
                        "Right open → turning RIGHT 90°  F=%.2f  R=%.2f  L=%.2f  Yaw=%.2f",
                        front_distance_, right_distance_, left_distance_, current_yaw_);
          }

          pause_ticks++;

          // stay stopped for ~1 second (10 * 100ms = 1s)
          if (pause_ticks > 15)
          {
            pause_ticks = 0;
            state_ = TaylorState::TURN;
            target_yaw_ = normalize_angle(current_yaw_ - 4 * M_PI / 9); // turn ~80°
          }
        }
      }

      else if (front_open) 
      {
        //tunables
        double desired_right_distance_ = 0.35;
        double dist_gain = 0.4;
        double angle_gain = 0.3;
        double max_angle = 1.0;
        double error_dist = 0.02;

        double error_distance = desired_right_distance_ - right_distance_;
        if (std::fabs(error_distance) < error_dist) error_distance = 0.0;

        //double desired_diagonal = desired_right_distance_ * std::sqrt(2.0);
        double error_diagonal =  right_distance_ * std::sqrt(2.0) - front_right_distance_;
        if (front_right_distance_ >= 1.8 * right_distance_) {
          error_diagonal = 0;
        }
        else if (std::fabs(error_diagonal) < error_dist) error_diagonal = 0.0;

        double omega = dist_gain * error_distance + angle_gain * error_diagonal;
        if (omega > max_angle) omega = max_angle;
        if (omega < -max_angle) omega = -max_angle;


        cmd.twist.linear.x = forward_speed;
        cmd.twist.angular.z = omega;
        pause_ticks = 0;
      }

      else if (left_open)
      {

        if (green_wall_seen_) {
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "MAZE COMPLETED!");
            pub_->publish(cmd);
            timer_->cancel();   // stop periodic callbacks
            return;
        }


        else if (!green_wall_seen_)
        {
          cmd.twist.linear.x = 0.0;
          cmd.twist.angular.z = 0.0;

          if (pause_ticks == 0)
          {
            RCLCPP_INFO(this->get_logger(),
                        "Front blocked, Left open → turning LEFT 90°  F=%.2f  R=%.2f  L=%.2f  Yaw=%.2f",
                        front_distance_, right_distance_, left_distance_, current_yaw_);
          }

          pause_ticks++;

          // stay stopped for ~1.5 seconds (15 * 100ms = 1.5s)
          if (pause_ticks > 15)
          {
            pause_ticks = 0;
            state_ = TaylorState::TURN;
            target_yaw_ = normalize_angle(current_yaw_ +  M_PI / 2);
          }
          if (right_wall_seen_ && !skip_right_)  {
            counter++;
            if (counter == target_counter)  {
              skip_right_ = 1;
              right_wall_seen_ = 0; 
              counter = 0;
              RCLCPP_INFO(this->get_logger(), "Will skip right turn");
            }
          }
        }

      }
          
      else
      {
        // stop and print once
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;

        if (pause_ticks == 0)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Dead end → turning 180°  F=%.2f  R=%.2f  L=%.2f  Yaw=%.2f",
                      front_distance_, right_distance_, left_distance_, current_yaw_);
        }

        pause_ticks++;

        // stay stopped for ~1.5 seconds (15 * 100ms = 1.5s)
        if (pause_ticks > 15)
        {
          pause_ticks = 0;
          state_ = TaylorState::TURN;
          target_yaw_ = normalize_angle(current_yaw_ + M_PI / 2);
        }    
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
        state_ = TaylorState::RIGHT_WALL_FIND; 
        RCLCPP_INFO(this->get_logger(), "Turn complete → seek right wall");
      }
      break;

    case TaylorState::RIGHT_WALL_FIND:
    {
      double desired_right_distance_ = 0.4;
      const bool right_open = (right_distance_ > open_dist);
      const bool front_open = (front_distance_ > stop_dist);
      const bool left_open = (left_distance_ > open_dist);

      if (right_distance_ <= desired_right_distance_)
      {
        state_ = TaylorState::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Right wall found → forward");
      }
      if (front_open)
      {
        cmd.twist.linear.x = forward_speed;
        cmd.twist.angular.z = 0.0;

      }
      else if (!front_open)
      {

        if (pause_ticks == 0)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Front blocked, Left open → turning LEFT 90°  F=%.2f  R=%.2f  L=%.2f  Yaw=%.2f",
                      front_distance_, right_distance_, left_distance_, current_yaw_);
        }

        pause_ticks++;

                // stay stopped for ~1.5 seconds (15 * 100ms = 1.5s)
        if (pause_ticks > 15)
        {
          pause_ticks = 0;
          state_ = TaylorState::TURN;
          target_yaw_ = normalize_angle(current_yaw_ + M_PI / 2);
        } 
      }
      break;
    }
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