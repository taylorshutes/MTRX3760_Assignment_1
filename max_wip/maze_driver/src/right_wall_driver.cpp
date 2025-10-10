#include "maze_driver/right_wall_driver.hpp"
#include <cmath>
using namespace std::chrono_literals;

// --- Constructor ---
RightWallDriver::RightWallDriver() : BaseDriver("right_wall_driver"), state_(State::FORWARD)
{
  RCLCPP_INFO(this->get_logger(), "Right-wall-following driver started.");
}

// --- Destructor ---
RightWallDriver::~RightWallDriver()
{
  RCLCPP_INFO(this->get_logger(), "Closing right wall driver node.");
  stop_robot();
}

// --- Helper functions ---
double RightWallDriver::normalize_angle(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

bool RightWallDriver::reached_target_yaw(double tol)
{
  double diff = normalize_angle(target_yaw_ - current_yaw_);
  return fabs(diff) < tol;
}

// --- Main behaviour loop ---
geometry_msgs::msg::TwistStamped RightWallDriver::compute_command()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";

  // tweakable parameters depending on maze size
  double forward_speed = 0.2;
  double angular_speed = 0.6;
  double tol = 0.05;
  double stop_dist = 0.4;
  double open_dist = 0.7;

  // check if at end
  if (last_colour_ == "green") {
    green_wall_seen_ = true;
    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Maze complete — green wall detected!");
  }

  if (green_wall_seen_) return cmd; // remain stopped

  // decide what to do based off of state
  switch (state_)
  {
    case State::FORWARD:
    {
      bool right_open = right_distance_ > open_dist && right_distance_ < 3.5;
      bool front_open = front_distance_ > stop_dist;
      bool left_open  = left_distance_ > open_dist;

      if (right_open && !skip_right_) {
        state_ = State::TURN;
        target_yaw_ = normalize_angle(current_yaw_ - M_PI / 2);
        RCLCPP_INFO(this->get_logger(), "Turning RIGHT 90°");
      }
      else if (front_open) {
        double error_distance = 0.30 - right_distance_;
        double error_diagonal = right_distance_ * std::sqrt(2.0) - front_right_distance_;
        double omega = 0.7 * error_distance + 0.5 * error_diagonal;
        omega = std::clamp(omega, -1.5, 1.5);
        cmd.twist.linear.x  = forward_speed;
        cmd.twist.angular.z = omega;
      }
      else if (left_open) {
        state_ = State::TURN;
        target_yaw_ = normalize_angle(current_yaw_ + M_PI / 2);
        RCLCPP_INFO(this->get_logger(), "Left open — turning LEFT 90°");
      }
      else {
        state_ = State::TURN;
        target_yaw_ = normalize_angle(current_yaw_ + M_PI);
        RCLCPP_INFO(this->get_logger(), "Dead end — turning 180°");
      }
      break;
    }

    case State::TURN:
      if (!reached_target_yaw(tol)) {
        cmd.twist.angular.z =
          (normalize_angle(target_yaw_ - current_yaw_) > 0)
            ? angular_speed
            : -angular_speed;
      } else {
        state_ = State::FIND_RIGHT;
        RCLCPP_INFO(this->get_logger(), "Turn complete — seeking right wall");
      }
      break;

    case State::FIND_RIGHT:
      if (right_distance_ <= 0.35) {
        state_ = State::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Right wall found — moving forward");
      } else {
        cmd.twist.linear.x = forward_speed;
      }
      break;
  }

  return cmd;
}
