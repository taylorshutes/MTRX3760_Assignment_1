#include "colour_detector/colour_detector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColourDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
