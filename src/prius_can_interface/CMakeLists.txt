cmake_minimum_required(VERSION 3.8)
project(prius_can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PriusCanStatus.msg"
  DEPENDENCIES builtin_interfaces
)

add_executable(prius_can_status_node src/prius_can_status_node.cpp)
target_compile_features(prius_can_status_node PUBLIC cxx_std_17)
ament_target_dependencies(prius_can_status_node rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(prius_can_status_node "${cpp_typesupport_target}")

install(TARGETS
  prius_can_status_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
