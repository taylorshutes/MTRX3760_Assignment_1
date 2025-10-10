#include "colour_detector/colour_detector.hpp"
#include <cv_bridge/cv_bridge.hpp>

// --- Constructor ---
ColourDetector::ColourDetector() : Node("colour_detector")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("/wall_colour", 10);

  sub_ = image_transport::create_subscription(
    this,
    "/camera/image_raw",
    std::bind(&ColourDetector::image_callback, this, std::placeholders::_1),
    "raw"
  );

  RCLCPP_INFO(this->get_logger(), "Colour detector node started. Listening to /camera/image_raw...");
}

// --- Destructor ---
ColourDetector::~ColourDetector()
{
  RCLCPP_INFO(this->get_logger(), "Closing colour detector node.");
}

// --- Image callback ---
void ColourDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty image received!");
    return;
  }

  // Detect colour
  std::string colour = detect_colour(image);

  // Publish result
  std_msgs::msg::String msg_out;
  msg_out.data = colour;
  publisher_->publish(msg_out);
}

// --- Colour detection logic ---
std::string ColourDetector::detect_colour(const cv::Mat &image)
{
  // Define a region of interest (center third of the image)
  cv::Rect center(image.cols / 3, image.rows / 3, image.cols / 3, image.rows / 3);
  cv::Mat roi = image(center);

  // Compute mean BGR
  cv::Scalar avg = cv::mean(roi);
  double blue = avg[0];
  double green = avg[1];
  double red = avg[2];

  std::string colour = "none";

  if (red > RED_ACTIVE && green < GREEN_INACTIVE && blue < BLUE_INACTIVE)
    colour = "red";
  else if (green > GREEN_ACTIVE && red < RED_INACTIVE && blue < BLUE_INACTIVE)
    colour = "green";

  RCLCPP_INFO(
    this->get_logger(),
    "Avg BGR = (%.0f, %.0f, %.0f) -> %s wall",
    blue, green, red, colour.c_str()
  );

  return colour;
}
