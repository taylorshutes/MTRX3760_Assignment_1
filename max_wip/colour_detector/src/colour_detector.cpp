#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class ColourDetector : public rclcpp::Node
{
public:
  ColourDetector() : Node("colour_detector")
  {
    // --- Publisher ---
    // Sends out the detected wall colour as a string ("red", "green", "none")
    publisher_ = this->create_publisher<std_msgs::msg::String>("/wall_colour", 10);

    // --- Subscriber ---
    // Subscribes to the TurtleBot3 camera feed
    sub_ = image_transport::create_subscription(
      this,
      "/camera/image_raw",  // check with `ros2 topic list` if different
      std::bind(&ColourDetector::image_callback, this, std::placeholders::_1),
      "raw"
    );

    RCLCPP_INFO(this->get_logger(), "Colour detector node started. Listening to /camera/image_raw...");
  }

private:
  // --- Callback triggered every time a new camera frame arrives ---
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv::Mat image;
    try {
      // Convert ROS image message to an OpenCV BGR image
      image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty image received!");
      return;
    }

    // Define a region of interest (center third of the image)
    cv::Rect center(image.cols / 3, image.rows / 3, image.cols / 3, image.rows / 3);
    cv::Mat roi = image(center);

    // Compute the average B, G, R values in that region
    cv::Scalar avg = cv::mean(roi);
    double blue = avg[0];
    double green = avg[1];
    double red = avg[2];

    // --- Threshold logic ---
    const double RED_ACTIVE = 80.0;
    const double GREEN_ACTIVE = 80.0;

    const double RED_INACTIVE = 50.0;
    const double GREEN_INACTIVE = 50.0;
    const double BLUE_INACTIVE = 50.0;

    std::string colour = "none";
    if (red > RED_ACTIVE && green < GREEN_INACTIVE && blue < BLUE_INACTIVE)
      colour = "red";
    else if (green > GREEN_INACTIVE && red < RED_INACTIVE && blue < BLUE_INACTIVE)
      colour = "green";

    // --- Publish the detected colour ---
    auto msg_out = std_msgs::msg::String();
    msg_out.data = colour;
    publisher_->publish(msg_out);

    // --- Log the result ---
    RCLCPP_INFO(this->get_logger(),
                "Avg BGR = (%.0f, %.0f, %.0f)  ->  %s wall",
                blue, green, red, colour.c_str());
  }

  // --- Member variables ---
  image_transport::Subscriber sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

// --- Main function ---
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColourDetector>());
  rclcpp::shutdown();
  return 0;
}
