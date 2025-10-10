#include "maze_driver/right_wall_driver.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RightWallDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
