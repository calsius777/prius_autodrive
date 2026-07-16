cmake_minimum_required(VERSION 3.10)

project(multi_camera_stress_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_executable(
    multi_camera_stress_test
    multi_camera_stress_test.cpp
)

target_compile_options(
    multi_camera_stress_test
    PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_link_libraries(
    multi_camera_stress_test
    PRIVATE
    Threads::Threads
)
