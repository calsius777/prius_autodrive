cmake_minimum_required(VERSION 3.8)
project(prius_vehicle_status_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(prius_can_interface REQUIRED)

# Optional: Autoware Universe vehicle status messages.
# If ros-humble-autoware-vehicle-msgs is not installed, the basic adapter still builds.
find_package(autoware_vehicle_msgs QUIET)

add_executable(prius_basic_vehicle_status_adapter_node
  src/prius_basic_vehicle_status_adapter_node.cpp
)
target_compile_features(prius_basic_vehicle_status_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(prius_basic_vehicle_status_adapter_node
  rclcpp
  std_msgs
  geometry_msgs
  prius_can_interface
)

install(TARGETS
  prius_basic_vehicle_status_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(autoware_vehicle_msgs_FOUND)
  message(STATUS "Found autoware_vehicle_msgs: building Autoware status adapter")
  add_executable(prius_autoware_vehicle_status_adapter_node
    src/prius_autoware_vehicle_status_adapter_node.cpp
  )
  target_compile_features(prius_autoware_vehicle_status_adapter_node PUBLIC cxx_std_17)
  ament_target_dependencies(prius_autoware_vehicle_status_adapter_node
    rclcpp
    prius_can_interface
    autoware_vehicle_msgs
  )
  install(TARGETS
    prius_autoware_vehicle_status_adapter_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "autoware_vehicle_msgs not found. Only the basic Prius vehicle status adapter will be built.")
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
