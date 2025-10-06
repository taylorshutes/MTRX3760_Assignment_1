#include "wall_follower/wall_follower.hpp"
#include <memory>

/**
 * @brief Main entry point for wall-following robot controller
 * 
 * This demonstrates proper integration of:
 * - Marcus's LidarProcessor (clean sensor abstraction)
 * - Taylor's intelligent path selection logic
 * - Object-oriented design principles
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WallFollower>();

    // Register shutdown callback to ensure clean stop
    rclcpp::on_shutdown([node]() {
        node->stop_robot();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
