#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "lidar_processor/lidar_processor.hpp"

/**
 * @brief Wall-following robot controller with intelligent path selection
 * 
 * This node implements a wall-following algorithm that:
 * - Drives forward until blocked
 * - Intelligently chooses best turning direction
 * - Uses odometry for precise angle-based turns
 * - Leverages LidarProcessor for clean sensor abstraction
 */
class WallFollower : public rclcpp::Node {
public:
    /**
     * @brief Constructor - initializes publishers, subscribers, and state
     */
    WallFollower();
    
    /**
     * @brief Destructor - ensures robot stops cleanly
     */
    ~WallFollower();
    
    /**
     * @brief Emergency stop - publishes zero velocity
     */
    void stop_robot();

private:
    // ==================== Callback Methods ====================
    
    /**
     * @brief Process incoming laser scan data
     * @param msg LaserScan message from /scan topic
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Process odometry data to track robot orientation
     * @param msg Odometry message from /odom topic
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Main control loop - runs at 10Hz
     */
    void on_timer();

    // ==================== Navigation States ====================
    
    enum class NavigationState {
        FORWARD,        // Driving forward
        PAUSED,         // Stopped, deciding which way to turn
        TURNING         // Executing a turn to target orientation
    };

    // ==================== Helper Methods ====================
    
    /**
     * @brief Normalize angle to [-π, π] range
     * @param angle Input angle in radians
     * @return Normalized angle
     */
    double normalize_angle(double angle);
    
    /**
     * @brief Check if robot has reached target orientation
     * @param tolerance Acceptable error in radians
     * @return true if within tolerance of target yaw
     */
    bool reached_target_yaw(double tolerance);
    
    /**
     * @brief Decide best turning direction based on LiDAR data
     * @return Target yaw angle in radians
     */
    double choose_best_turn_direction();

    // ==================== State Variables ====================
    
    NavigationState state_;           // Current navigation state
    double current_yaw_;              // Current robot orientation (radians)
    double target_yaw_;               // Target orientation for turns (radians)
    int pause_counter_;               // Tick counter for pause state

    // ==================== Sensor Processing ====================
    
    LidarProcessor lidar_processor_;  // Processes LiDAR data

    // ==================== ROS 2 Interfaces ====================
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ==================== Control Parameters ====================
    
    static constexpr double FORWARD_SPEED = 0.2;      // m/s
    static constexpr double ANGULAR_SPEED = 0.6;      // rad/s
    static constexpr double STOP_DISTANCE = 0.5;      // metres
    static constexpr double FREE_THRESHOLD = 0.6;     // metres
    static constexpr double YAW_TOLERANCE = 0.05;     // radians (~3°)
    static constexpr int PAUSE_TICKS = 15;            // 1.5 seconds at 10Hz
};
