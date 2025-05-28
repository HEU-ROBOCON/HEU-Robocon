cmake_minimum_required(VERSION 3.10)
project(Circle_Position VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(Circle_Position Circle_Position.cpp)

target_link_libraries(Circle_Position PRIVATE realsense2 ${OpenCV_LIBS})