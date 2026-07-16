cmake_minimum_required(VERSION 3.10)

project(v4l2_camera_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(
    v4l2_camera_test
    v4l2_camera_test.cpp
)

target_include_directories(
    v4l2_camera_test
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    v4l2_camera_test
    PRIVATE
    ${OpenCV_LIBS}
)
