#include "trajectory_mapper/trajectory_mapper.hpp"
#include <memory>

/**
 * @brief Main entry point for trajectory mapping node
 * 
 * Records robot trajectory from odometry data and visualises in RViz
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryMapper>();

    RCLCPP_INFO(node->get_logger(), 
                "Trajectory Mapper running - use Ctrl+C to stop and save");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
