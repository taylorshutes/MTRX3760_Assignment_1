#include "lidar_processor/lidar_processor.hpp"
#include <algorithm>
#include <limits>

LidarProcessor::LidarProcessor()
    : front_distance_(std::numeric_limits<float>::infinity()),
      right_distance_(std::numeric_limits<float>::infinity()),
      left_distance_(std::numeric_limits<float>::infinity()),
      right_diagonal_(std::numeric_limits<float>::infinity()),
      left_diagonal_(std::numeric_limits<float>::infinity()),
      min_distance_(std::numeric_limits<float>::infinity()),
      previous_right_wall_present_(false),
      current_right_wall_present_(false),
      data_valid_(false),
      last_scan_(nullptr)
{
}

void LidarProcessor::process_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!msg || msg->ranges.empty()) {
        data_valid_ = false;
        return;
    }

    last_scan_ = msg;
    data_valid_ = true;

    // Extract key directions
    front_distance_ = extract_distance_at_angle(0.0f, msg);
    right_distance_ = extract_distance_at_angle(-M_PI / 2.0f, msg);  // -90°
    left_distance_ = extract_distance_at_angle(M_PI / 2.0f, msg);    // +90°
    right_diagonal_ = extract_distance_at_angle(-M_PI / 4.0f, msg);  // -45°
    left_diagonal_ = extract_distance_at_angle(M_PI / 4.0f, msg);    // +45°

    // Find minimum distance (closest obstacle)
    min_distance_ = std::numeric_limits<float>::infinity();
    for (const auto& range : msg->ranges) {
        if (is_valid_range(range, msg)) {
            min_distance_ = std::min(min_distance_, range);
        }
    }

    // Update wall tracking for gap detection
    previous_right_wall_present_ = current_right_wall_present_;
    current_right_wall_present_ = has_wall_on_right();
}

// ==================== Basic Distance Queries ====================

float LidarProcessor::get_front_distance() const
{
    return front_distance_;
}

float LidarProcessor::get_right_distance() const
{
    return right_distance_;
}

float LidarProcessor::get_left_distance() const
{
    return left_distance_;
}

float LidarProcessor::get_right_diagonal() const
{
    return right_diagonal_;
}

float LidarProcessor::get_left_diagonal() const
{
    return left_diagonal_;
}

// ==================== Wall Detection ====================

bool LidarProcessor::has_wall_on_right(float threshold) const
{
    return data_valid_ && (right_distance_ < threshold);
}

bool LidarProcessor::has_wall_on_left(float threshold) const
{
    return data_valid_ && (left_distance_ < threshold);
}

bool LidarProcessor::has_obstacle_ahead(float threshold) const
{
    return data_valid_ && (front_distance_ < threshold);
}

// ==================== Gap/Opening Detection ====================

bool LidarProcessor::right_wall_gap_detected() const
{
    // Wall was present, but now it's gone
    return data_valid_ && previous_right_wall_present_ && !current_right_wall_present_;
}

bool LidarProcessor::opening_ahead(float min_width) const
{
    if (!data_valid_ || !last_scan_) {
        return false;
    }

    // Check a cone ahead (e.g., -30° to +30°)
    float left_cone = get_min_in_range(0.0f, M_PI / 6.0f);   // 0° to 30° left
    float right_cone = get_min_in_range(-M_PI / 6.0f, 0.0f); // 30° right to 0°

    // If both sides are far enough, there's an opening
    return (left_cone > min_width) && (right_cone > min_width);
}

// ==================== Clearance Analysis ====================

float LidarProcessor::get_clearest_direction() const
{
    if (!data_valid_ || !last_scan_) {
        return 0.0f;  // Default to straight ahead
    }

    float max_distance = 0.0f;
    float best_angle = 0.0f;

    // Sample directions every 15 degrees
    const int num_samples = 24;  // 360° / 15° = 24
    for (int i = 0; i < num_samples; ++i) {
        float angle = -M_PI + (2.0f * M_PI * i / num_samples);
        
        // Get average distance in a small cone around this angle
        float dist = get_min_in_range(angle - 0.1f, angle + 0.1f);
        
        if (dist > max_distance) {
            max_distance = dist;
            best_angle = angle;
        }
    }

    return best_angle;
}

bool LidarProcessor::is_area_clear(float radius) const
{
    return data_valid_ && (min_distance_ > radius);
}

float LidarProcessor::get_min_distance() const
{
    return min_distance_;
}

// ==================== Advanced Queries ====================

float LidarProcessor::get_distance_at_angle(float angle) const
{
    if (!data_valid_ || !last_scan_) {
        return std::numeric_limits<float>::infinity();
    }
    
    return extract_distance_at_angle(angle, last_scan_);
}

float LidarProcessor::get_min_in_range(float start_angle, float end_angle) const
{
    if (!data_valid_ || !last_scan_) {
        return std::numeric_limits<float>::infinity();
    }

    // Normalize angles
    start_angle = normalize_angle(start_angle);
    end_angle = normalize_angle(end_angle);

    // Convert to indices
    int start_idx = angle_to_index(start_angle, last_scan_);
    int end_idx = angle_to_index(end_angle, last_scan_);

    // Ensure indices are valid
    int num_points = static_cast<int>(last_scan_->ranges.size());
    start_idx = std::max(0, std::min(start_idx, num_points - 1));
    end_idx = std::max(0, std::min(end_idx, num_points - 1));

    // Handle wrap-around if end < start
    float min_dist = std::numeric_limits<float>::infinity();
    
    if (start_idx <= end_idx) {
        // Normal case
        for (int i = start_idx; i <= end_idx; ++i) {
            if (is_valid_range(last_scan_->ranges[i], last_scan_)) {
                min_dist = std::min(min_dist, last_scan_->ranges[i]);
            }
        }
    } else {
        // Wrap around case
        for (int i = start_idx; i < num_points; ++i) {
            if (is_valid_range(last_scan_->ranges[i], last_scan_)) {
                min_dist = std::min(min_dist, last_scan_->ranges[i]);
            }
        }
        for (int i = 0; i <= end_idx; ++i) {
            if (is_valid_range(last_scan_->ranges[i], last_scan_)) {
                min_dist = std::min(min_dist, last_scan_->ranges[i]);
            }
        }
    }

    return min_dist;
}

bool LidarProcessor::is_data_valid() const
{
    return data_valid_;
}

// ==================== Helper Methods ====================

int LidarProcessor::angle_to_index(float angle, const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    if (!msg || msg->ranges.empty()) {
        return 0;
    }

    // Normalize angle to match scan range
    angle = normalize_angle(angle);

    // Calculate index
    // index = (angle - angle_min) / angle_increment
    float angle_diff = angle - msg->angle_min;
    int index = static_cast<int>(std::round(angle_diff / msg->angle_increment));

    // Clamp to valid range
    int num_points = static_cast<int>(msg->ranges.size());
    index = std::max(0, std::min(index, num_points - 1));

    return index;
}

float LidarProcessor::normalize_angle(float angle) const
{
    // Normalize to [-π, π]
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

bool LidarProcessor::is_valid_range(float range, const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    if (!msg) {
        return false;
    }

    // Check if range is within valid bounds
    return std::isfinite(range) && 
           (range >= msg->range_min) && 
           (range <= msg->range_max);
}

float LidarProcessor::extract_distance_at_angle(float angle, 
                                                const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    if (!msg || msg->ranges.empty()) {
        return std::numeric_limits<float>::infinity();
    }

    int index = angle_to_index(angle, msg);
    
    // Use averaged distance for noise reduction (average 5 adjacent points)
    return get_averaged_distance(index, 5, msg);
}

float LidarProcessor::get_averaged_distance(int center_index, int num_points,
                                           const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    if (!msg || msg->ranges.empty()) {
        return std::numeric_limits<float>::infinity();
    }

    int num_ranges = static_cast<int>(msg->ranges.size());
    int half_window = num_points / 2;
    
    float sum = 0.0f;
    int count = 0;

    for (int offset = -half_window; offset <= half_window; ++offset) {
        int idx = center_index + offset;
        
        // Handle wrap-around
        if (idx < 0) {
            idx += num_ranges;
        } else if (idx >= num_ranges) {
            idx -= num_ranges;
        }

        if (is_valid_range(msg->ranges[idx], msg)) {
            sum += msg->ranges[idx];
            count++;
        }
    }

    if (count > 0) {
        return sum / count;
    } else {
        return std::numeric_limits<float>::infinity();
    }
}
