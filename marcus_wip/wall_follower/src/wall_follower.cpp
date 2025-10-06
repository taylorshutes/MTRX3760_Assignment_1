#include "wall_follower/wall_follower.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

WallFollower::WallFollower() 
    : rclcpp::Node("wall_follower"),
      state_(NavigationState::FORWARD),
      current_yaw_(0.0),
      target_yaw_(0.0),
      pause_counter_(0)
{
    // Create publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10);

    // Subscribe to laser scan data
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS().best_effort(),
        std::bind(&WallFollower::scan_callback, this, std::placeholders::_1));

    // Subscribe to odometry for orientation tracking
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

    // Control loop timer - 10Hz
    control_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&WallFollower::on_timer, this));

    RCLCPP_INFO(this->get_logger(), 
                "Wall Follower started - using LidarProcessor for sensor abstraction");
}

WallFollower::~WallFollower()
{
    stop_robot();
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // ✅ Clean sensor abstraction - all LiDAR processing handled by LidarProcessor
    lidar_processor_.process_scan(msg);
    
    // Log sensor data periodically
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), 
        *this->get_clock(), 
        1000,
        "Sensors: Front=%.2fm, Right=%.2fm, Left=%.2fm | Yaw=%.2f rad",
        lidar_processor_.get_front_distance(),
        lidar_processor_.get_right_distance(),
        lidar_processor_.get_left_distance(),
        current_yaw_);
}

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract yaw from quaternion orientation
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    current_yaw_ = yaw;  // Range: [-π, π]
}

void WallFollower::on_timer()
{
    // Prepare velocity command message
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    switch (state_) {
        case NavigationState::FORWARD:
        {
            // Check if path ahead is blocked
            if (lidar_processor_.has_obstacle_ahead(STOP_DISTANCE)) {
                // Stop and transition to pause state
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = 0.0;
                
                state_ = NavigationState::PAUSED;
                pause_counter_ = 0;
                
                RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected! Front=%.2fm, Right=%.2fm, Left=%.2fm",
                    lidar_processor_.get_front_distance(),
                    lidar_processor_.get_right_distance(),
                    lidar_processor_.get_left_distance());
            } else {
                // Path clear - drive forward
                cmd.twist.linear.x = FORWARD_SPEED;
                cmd.twist.angular.z = 0.0;
            }
            break;
        }

        case NavigationState::PAUSED:
        {
            // Stay stopped while sensors stabilize
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            
            pause_counter_++;
            
            // After pause period, decide which way to turn
            if (pause_counter_ >= PAUSE_TICKS) {
                target_yaw_ = choose_best_turn_direction();
                state_ = NavigationState::TURNING;
                pause_counter_ = 0;
                
                RCLCPP_INFO(this->get_logger(),
                    "Pause complete - turning to yaw=%.2f rad", target_yaw_);
            }
            break;
        }

        case NavigationState::TURNING:
        {
            if (!reached_target_yaw(YAW_TOLERANCE)) {
                // Still turning - determine direction
                double angle_diff = normalize_angle(target_yaw_ - current_yaw_);
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = (angle_diff > 0) ? ANGULAR_SPEED : -ANGULAR_SPEED;
            } else {
                // Turn complete - resume forward motion
                state_ = NavigationState::FORWARD;
                RCLCPP_INFO(this->get_logger(), "Turn complete - resuming forward");
            }
            break;
        }
    }

    cmd_vel_pub_->publish(cmd);
}

double WallFollower::choose_best_turn_direction()
{
    // ✅ Clean decision logic using LidarProcessor queries
    bool right_clear = lidar_processor_.get_right_distance() > FREE_THRESHOLD;
    bool left_clear = lidar_processor_.get_left_distance() > FREE_THRESHOLD;

    if (right_clear && left_clear) {
        // Both sides open - choose the wider one
        if (lidar_processor_.get_right_distance() > lidar_processor_.get_left_distance()) {
            RCLCPP_INFO(this->get_logger(), 
                "Both sides clear - right wider (%.2fm vs %.2fm) - turning right 90°",
                lidar_processor_.get_right_distance(),
                lidar_processor_.get_left_distance());
            return normalize_angle(current_yaw_ - M_PI / 2);  // Right 90°
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "Both sides clear - left wider (%.2fm vs %.2fm) - turning left 90°",
                lidar_processor_.get_left_distance(),
                lidar_processor_.get_right_distance());
            return normalize_angle(current_yaw_ + M_PI / 2);  // Left 90°
        }
    } else if (right_clear) {
        RCLCPP_INFO(this->get_logger(), 
            "Right clear (%.2fm), left blocked - turning right 90°",
            lidar_processor_.get_right_distance());
        return normalize_angle(current_yaw_ - M_PI / 2);  // Right 90°
    } else if (left_clear) {
        RCLCPP_INFO(this->get_logger(), 
            "Left clear (%.2fm), right blocked - turning left 90°",
            lidar_processor_.get_left_distance());
        return normalize_angle(current_yaw_ + M_PI / 2);  // Left 90°
    } else {
        RCLCPP_INFO(this->get_logger(), "Dead end - turning 180°");
        return normalize_angle(current_yaw_ + M_PI);  // 180° turn
    }
}

void WallFollower::stop_robot()
{
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = 0.0;
    msg.twist.angular.z = 0.0;
    
    cmd_vel_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Robot stopped");
}

double WallFollower::normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool WallFollower::reached_target_yaw(double tolerance)
{
    double diff = normalize_angle(target_yaw_ - current_yaw_);
    return std::abs(diff) < tolerance;
}
