#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

/**
 * @brief Processes LiDAR scan data to extract useful navigation information
 * 
 * This class takes raw LaserScan messages and provides:
 * - Distance measurements in key directions
 * - Wall detection and tracking
 * - Gap/opening detection
 * - Clearance analysis
 */
class LidarProcessor {
public:
    /**
     * @brief Constructor
     */
    LidarProcessor();

    /**
     * @brief Process a new laser scan message
     * @param msg Shared pointer to LaserScan message
     */
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // ==================== Basic Distance Queries ====================
    
    /**
     * @brief Get distance directly in front of robot
     * @return Distance in metres (or inf if no obstacle)
     */
    float get_front_distance() const;

    /**
     * @brief Get distance to the right of robot (90° right)
     * @return Distance in metres
     */
    float get_right_distance() const;

    /**
     * @brief Get distance to the left of robot (90° left)
     * @return Distance in metres
     */
    float get_left_distance() const;

    /**
     * @brief Get distance at right-front diagonal (45° right)
     * @return Distance in metres
     */
    float get_right_diagonal() const;

    /**
     * @brief Get distance at left-front diagonal (45° left)
     * @return Distance in metres
     */
    float get_left_diagonal() const;

    // ==================== Wall Detection ====================
    
    /**
     * @brief Check if wall exists on right side
     * @param threshold Maximum distance to consider as "wall present" (metres)
     * @return true if wall detected within threshold
     */
    bool has_wall_on_right(float threshold = 0.5f) const;

    /**
     * @brief Check if wall exists on left side
     * @param threshold Maximum distance to consider as "wall present" (metres)
     * @return true if wall detected within threshold
     */
    bool has_wall_on_left(float threshold = 0.5f) const;

    /**
     * @brief Check if obstacle/wall exists ahead
     * @param threshold Maximum distance to consider as "obstacle ahead" (metres)
     * @return true if obstacle detected within threshold
     */
    bool has_obstacle_ahead(float threshold = 0.4f) const;

    // ==================== Gap/Opening Detection ====================
    
    /**
     * @brief Detect if right wall has disappeared (gap detection)
     * Useful for detecting when wall-following should pause
     * @return true if right wall was present but now absent
     */
    bool right_wall_gap_detected() const;

    /**
     * @brief Detect if there's a wide opening ahead
     * @param min_width Minimum width to consider as "opening" (metres)
     * @return true if opening detected
     */
    bool opening_ahead(float min_width = 0.8f) const;

    // ==================== Clearance Analysis ====================
    
    /**
     * @brief Get angle of direction with maximum clearance
     * Useful for open navigation when no walls to follow
     * @return Angle in radians (0 = front, positive = left, negative = right)
     */
    float get_clearest_direction() const;

    /**
     * @brief Check if area around robot is clear
     * @param radius Distance to check in all directions (metres)
     * @return true if no obstacles within radius
     */
    bool is_area_clear(float radius = 1.0f) const;

    /**
     * @brief Get minimum distance in any direction
     * @return Closest obstacle distance in metres
     */
    float get_min_distance() const;

    // ==================== Advanced Queries ====================
    
    /**
     * @brief Get distance at a specific angle
     * @param angle Angle in radians (0 = front, positive = left)
     * @return Distance in metres
     */
    float get_distance_at_angle(float angle) const;

    /**
     * @brief Get minimum distance within an angular range
     * @param start_angle Start angle in radians
     * @param end_angle End angle in radians
     * @return Minimum distance found in range (metres)
     */
    float get_min_in_range(float start_angle, float end_angle) const;

    /**
     * @brief Check if LiDAR data has been received
     * @return true if scan data is available
     */
    bool is_data_valid() const;

private:
    // ==================== Internal State ====================
    
    // Cached distance values (updated by process_scan)
    float front_distance_;
    float right_distance_;
    float left_distance_;
    float right_diagonal_;
    float left_diagonal_;
    float min_distance_;

    // Wall tracking (for gap detection)
    bool previous_right_wall_present_;
    bool current_right_wall_present_;

    // Data validity
    bool data_valid_;

    // Store last scan message for advanced queries
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    // ==================== Helper Methods ====================
    
    /**
     * @brief Convert angle to array index in ranges vector
     * @param angle Angle in radians (0 = front, positive = left)
     * @param msg LaserScan message
     * @return Index in ranges array
     */
    int angle_to_index(float angle, const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    /**
     * @brief Normalize angle to [-π, π]
     * @param angle Input angle in radians
     * @return Normalized angle
     */
    float normalize_angle(float angle) const;

    /**
     * @brief Check if a range value is valid
     * @param range Distance value
     * @param msg LaserScan message (for min/max bounds)
     * @return true if valid reading
     */
    bool is_valid_range(float range, const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    /**
     * @brief Extract distance at specific angle from scan
     * @param angle Target angle in radians
     * @param msg LaserScan message
     * @return Distance in metres (inf if invalid)
     */
    float extract_distance_at_angle(float angle, const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    /**
     * @brief Get average distance over multiple adjacent readings
     * Reduces noise by averaging nearby points
     * @param center_index Central index in ranges array
     * @param num_points Number of points to average (must be odd)
     * @param msg LaserScan message
     * @return Average distance
     */
    float get_averaged_distance(int center_index, int num_points, 
                               const sensor_msgs::msg::LaserScan::SharedPtr msg) const;
};
