#include "simple_driver/simple_driver.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimpleDriver>();

    // When ROS is shutting down (Ctrl-C), call stopRobot()
    rclcpp::on_shutdown([node]() {
        node->stop_robot();
    });

  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
