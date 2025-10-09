cmake_minimum_required(VERSION 3.8)
project(colour_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# --- Build the executable ---
add_executable(colour_detector_node src/colour_detector.cpp)

ament_target_dependencies(colour_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Explicitly link OpenCV (this sometimes helps resolve cv_bridge include issues)
target_link_libraries(colour_detector_node
  ${OpenCV_LIBRARIES}
)

# --- Install target ---
install(TARGETS
  colour_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
