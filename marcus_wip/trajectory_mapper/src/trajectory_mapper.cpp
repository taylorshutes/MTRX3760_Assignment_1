#include "trajectory_mapper/trajectory_mapper.hpp"
#include <cmath>
#include <chrono>
#include <iomanip>

using namespace std::chrono_literals;

TrajectoryMapper::TrajectoryMapper() 
    : rclcpp::Node("trajectory_mapper"),
      last_x_(0.0),
      last_y_(0.0),
      start_x_(0.0),
      start_y_(0.0),
      first_pose_received_(false),
      min_distance_threshold_(MIN_DISTANCE),
      total_distance_travelled_(0.0)
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&TrajectoryMapper::odom_callback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/robot_trajectory", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/trajectory_markers", 10);

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(PUBLISH_RATE_MS),
        std::bind(&TrajectoryMapper::publish_timer_callback, this));

    trajectory_path_.header.frame_id = "odom";

    RCLCPP_INFO(this->get_logger(), 
                "Trajectory Mapper started - recording robot path");
    RCLCPP_INFO(this->get_logger(), 
                "Minimum recording distance: %.2fm", min_distance_threshold_);
}

TrajectoryMapper::~TrajectoryMapper()
{
    save_trajectory_to_file("trajectory_data.csv");
    
    RCLCPP_INFO(this->get_logger(), 
                "Trajectory Mapper shutting down");
    RCLCPP_INFO(this->get_logger(), 
                "Total distance travelled: %.2fm", total_distance_travelled_);
    RCLCPP_INFO(this->get_logger(), 
                "Total waypoints recorded: %zu", trajectory_path_.poses.size());
}

void TrajectoryMapper::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    if (!first_pose_received_) {
        start_x_ = current_x;
        start_y_ = current_y;
        last_x_ = current_x;
        last_y_ = current_y;
        first_pose_received_ = true;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose = msg->pose.pose;
        trajectory_path_.poses.push_back(pose_stamped);

        RCLCPP_INFO(this->get_logger(), 
                    "Start position recorded: (%.2f, %.2f)", start_x_, start_y_);
        return;
    }

    double distance = calculate_distance(current_x, current_y, last_x_, last_y_);

    if (distance >= min_distance_threshold_) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose = msg->pose.pose;

        trajectory_path_.poses.push_back(pose_stamped);

        total_distance_travelled_ += distance;
        last_x_ = current_x;
        last_y_ = current_y;

        if (trajectory_path_.poses.size() % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                        "Waypoints: %zu | Distance: %.2fm | Current: (%.2f, %.2f)",
                        trajectory_path_.poses.size(),
                        total_distance_travelled_,
                        current_x, current_y);
        }
    }
}

void TrajectoryMapper::publish_timer_callback()
{
    if (trajectory_path_.poses.empty()) {
        return;
    }

    trajectory_path_.header.stamp = this->now();
    path_pub_->publish(trajectory_path_);

    auto start_marker = create_start_marker();
    marker_pub_->publish(start_marker);

    auto current_marker = create_current_marker();
    marker_pub_->publish(current_marker);
}

double TrajectoryMapper::calculate_distance(double x1, double y1, 
                                            double x2, double y2) const
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

void TrajectoryMapper::save_trajectory_to_file(const std::string& filename)
{
    if (trajectory_path_.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No trajectory data to save");
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Failed to open file: %s", filename.c_str());
        return;
    }

    file << "timestamp,x,y,z,qx,qy,qz,qw\n";
    file << std::fixed << std::setprecision(6);

    for (const auto& pose_stamped : trajectory_path_.poses) {
        double timestamp = pose_stamped.header.stamp.sec + 
                          pose_stamped.header.stamp.nanosec * 1e-9;
        
        const auto& pos = pose_stamped.pose.position;
        const auto& orient = pose_stamped.pose.orientation;

        file << timestamp << ","
             << pos.x << "," << pos.y << "," << pos.z << ","
             << orient.x << "," << orient.y << "," 
             << orient.z << "," << orient.w << "\n";
    }

    file.close();

    RCLCPP_INFO(this->get_logger(), 
                "Trajectory saved to: %s", filename.c_str());
    RCLCPP_INFO(this->get_logger(), 
                "Saved %zu waypoints", trajectory_path_.poses.size());
}

visualization_msgs::msg::Marker TrajectoryMapper::create_start_marker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = start_x_;
    marker.pose.position.y = start_y_;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration(0, 0);

    return marker;
}

visualization_msgs::msg::Marker TrajectoryMapper::create_current_marker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    if (!trajectory_path_.poses.empty()) {
        const auto& last_pose = trajectory_path_.poses.back();
        marker.pose.position = last_pose.pose.position;
        marker.pose.position.z = 0.1;
    }
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration(0, 0);

    return marker;
}
