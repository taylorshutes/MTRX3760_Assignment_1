cmake_minimum_required(VERSION 3.8)
project(colour_detector)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# --- Include directories ---
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# --- Build executable ---
add_executable(colour_detector_node
  src/colour_detector.cpp
  src/main.cpp
)

ament_target_dependencies(colour_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(colour_detector_node
  ${OpenCV_LIBRARIES}
)

# --- Installation ---
install(TARGETS
  colour_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
