#ifndef COLOUR_DETECTOR_HPP
#define COLOUR_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

/**
 * The Colour Detector node subscribes to the raw camera data, decides what colour wall
 * the robot is facing (red, green, or none), and publishes this to the /wall_colour topic
 * to be used by the driver.
 */
class ColourDetector : public rclcpp::Node
{
public:
  /**
   * Instantiate the Node, subscribe to camera data, and create publisher to /wall_colour topic.
   */
  ColourDetector();

  /**
   * Destructor — shuts down the node and logs a closing message.
   */
  ~ColourDetector();

private:
  /**
   * Image callback function to perform an operation when a new image frame message is received from camera.
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

  /**
   * Function to determine the dominant colour of a wall in a Mat image.
   * Returns a string "red", "green", or "none".
   */
  std::string detect_colour(const cv::Mat &image);

  // --- ROS interfaces ---
  image_transport::Subscriber sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // --- Threshold constants ---
  static constexpr double RED_ACTIVE = 80.0;        // level at which R/G is considered dominant
  static constexpr double GREEN_ACTIVE = 80.0;
  static constexpr double RED_INACTIVE = 50.0;      // R/G is only dominant if others are below this
  static constexpr double GREEN_INACTIVE = 50.0;
  static constexpr double BLUE_INACTIVE = 50.0;
};

#endif // COLOUR_DETECTOR_HPP
