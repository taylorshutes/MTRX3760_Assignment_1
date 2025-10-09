#include "taylor_driver/taylor_driver.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaylorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

