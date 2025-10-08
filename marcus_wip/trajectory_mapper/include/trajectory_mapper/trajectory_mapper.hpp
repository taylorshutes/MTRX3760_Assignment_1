#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <fstream>
#include <string>

/**
 * @brief Records and visualises robot trajectory for maze solving
 * 
 * This class subscribes to odometry data and:
 * - Records the robot's position over time
 * - Publishes a Path message for RViz visualisation
 * - Publishes markers to show start/end points
 * - Can save trajectory data to file for reports
 */
class TrajectoryMapper : public rclcpp::Node {
public:
    TrajectoryMapper();
    ~TrajectoryMapper();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_timer_callback();
    
    double calculate_distance(double x1, double y1, double x2, double y2) const;
    void save_trajectory_to_file(const std::string& filename);
    
    visualization_msgs::msg::Marker create_start_marker();
    visualization_msgs::msg::Marker create_current_marker();

    nav_msgs::msg::Path trajectory_path_;
    
    double last_x_;
    double last_y_;
    double start_x_;
    double start_y_;
    bool first_pose_received_;
    
    double min_distance_threshold_;
    double total_distance_travelled_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    static constexpr double MIN_DISTANCE = 0.05;
    static constexpr int PUBLISH_RATE_MS = 500;
};
