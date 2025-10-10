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
#include <optional>

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
    // ==================== Main Control Loop ====================
    
    /**
     * @brief Main timer callback - checks for new position and records if moved
     */
    void timer_callback();
    
    // ==================== Transform & Recording ====================
    
    /**
     * @brief Attempt to get current robot transform from SLAM
     * @return Transform if available, std::nullopt if not ready
     */
    std::optional<geometry_msgs::msg::TransformStamped> get_robot_transform();
    
    /**
     * @brief Process and record a new robot position
     * @param transform Current robot transform from SLAM
     */
    void record_position(const geometry_msgs::msg::TransformStamped& transform);
    
    // ==================== Visualization ====================
    
    /**
     * @brief Publish trajectory path and position markers to RViz
     */
    void publish_visualization();
    
    /**
     * @brief Create marker for start position (green sphere)
     * @return Marker message for start position
     */
    visualization_msgs::msg::Marker create_start_marker();
    
    /**
     * @brief Create marker for current position (red sphere)
     * @return Marker message for current position
     */
    visualization_msgs::msg::Marker create_current_marker();
    
    // ==================== Utilities ====================
    
    /**
     * @brief Calculate Euclidean distance between two points
     * @param x1 X coordinate of first point
     * @param y1 Y coordinate of first point
     * @param x2 X coordinate of second point
     * @param y2 Y coordinate of second point
     * @return Distance in metres
     */
    double calculate_distance(double x1, double y1, double x2, double y2) const;
    
    /**
     * @brief Save recorded trajectory to CSV file
     * @param filename Name of output file
     */
    void save_trajectory_to_file(const std::string& filename);

    // ==================== Data Members ====================
    
    nav_msgs::msg::Path trajectory_path_;
    
    // TF2 for SLAM-corrected poses
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Position tracking
    double last_x_;
    double last_y_;
    double start_x_;
    double start_y_;
    bool first_pose_received_;
    
    // Trajectory metrics
    double min_distance_threshold_;
    double total_distance_travelled_;
    
    // ROS 2 interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Control parameters
    static constexpr double MIN_DISTANCE = 0.10;     // Minimum movement to record (metres)
    static constexpr int UPDATE_RATE_MS = 100;       // Position check rate (10 Hz)
    static constexpr int PUBLISH_RATE_MS = 500;      // Visualization rate (2 Hz)
    
    int update_counter_;
};