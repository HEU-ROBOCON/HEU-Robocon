cmake_minimum_required(VERSION 3.10)
project(RealSense_Color_Circle_Distance VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(RealSense_Color_Circle_Distance RealSense_D435i_Color_Circle_Depth.cpp)

target_link_libraries(RealSense_Color_Circle_Distance PRIVATE realsense2 ${OpenCV_LIBS})