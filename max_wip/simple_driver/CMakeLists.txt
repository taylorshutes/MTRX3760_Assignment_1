cmake_minimum_required(VERSION 3.8)
project(simple_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(simple_driver
  src/main.cpp
  src/simple_driver.cpp)
ament_target_dependencies(simple_driver rclcpp geometry_msgs sensor_msgs)

install(TARGETS simple_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
