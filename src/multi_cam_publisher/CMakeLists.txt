cmake_minimum_required(VERSION 3.8)
project(multi_cam_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(
  multi_cam_publisher_node
  src/camera_v4l2.cpp
  src/multi_cam_node.cpp
)

target_include_directories(
  multi_cam_publisher_node
  PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(
  multi_cam_publisher_node
  rclcpp
  sensor_msgs
)

target_link_libraries(
  multi_cam_publisher_node
  ${OpenCV_LIBS}
  Threads::Threads
)

install(
  TARGETS multi_cam_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
