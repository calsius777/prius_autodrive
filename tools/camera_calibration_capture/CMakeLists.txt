cmake_minimum_required(VERSION 3.10)

project(camera_calibration_capture)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(
    camera_calibration_capture
    multi_camera_calibration_capture.cpp
)

target_compile_options(
    camera_calibration_capture
    PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_include_directories(
    camera_calibration_capture
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    camera_calibration_capture
    PRIVATE
    ${OpenCV_LIBS}
    Threads::Threads
)
