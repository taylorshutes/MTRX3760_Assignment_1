#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <fstream>
#include <string>

/**
 * @brief Records and visualises robot trajectory using SLAM-corrected poses
 * 
 * This class uses TF2 to get SLAM-corrected positions in the map frame:
 * - Listens to transforms from /map to /base_footprint
 * - Records drift-corrected robot positions over time
 * - Publishes Path message for RViz visualisation
 * - Publishes markers to show start/end points
 * - Saves trajectory data to file for reports
 */
class TrajectoryMapper : public rclcpp::Node {
public:
    TrajectoryMapper();
    ~TrajectoryMapper();

private:
    void timer_callback();
    void publish_visualization();
    
    double calculate_distance(double x1, double y1, double x2, double y2) const;
    void save_trajectory_to_file(const std::string& filename);
    
    visualization_msgs::msg::Marker create_start_marker();
    visualization_msgs::msg::Marker create_current_marker();

    nav_msgs::msg::Path trajectory_path_;
    
    // TF2 for SLAM-corrected poses
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    double last_x_;
    double last_y_;
    double start_x_;
    double start_y_;
    bool first_pose_received_;
    
    double min_distance_threshold_;
    double total_distance_travelled_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    static constexpr double MIN_DISTANCE = 0.10;  // Increased from 0.05
    static constexpr int UPDATE_RATE_MS = 100;     // 10 Hz update rate
    static constexpr int PUBLISH_RATE_MS = 500;    // 2 Hz visualization rate
    
    int update_counter_;
};