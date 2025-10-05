cmake_minimum_required(VERSION 3.8)
project(simple_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)   # ✅ added

include_directories(include)

add_executable(simple_driver_node
  src/main.cpp
  src/simple_driver.cpp
)

ament_target_dependencies(simple_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs   # ✅ added
)

install(TARGETS
  simple_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

