cmake_minimum_required(VERSION 3.10.0)
project(tor_r2_vision VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version      : ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs : ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries    : ${OpenCV_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS})

# ------------------------------------------------------------
# 通用函数：添加只依赖 OpenCV 的目标
# ------------------------------------------------------------
function(add_opencv_executable target_name source_file)
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}")
        add_executable(${target_name} ${source_file})
        target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBRARIES})

        if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${target_name} PRIVATE -O3 -march=native)
        endif()

        message(STATUS "Enabled target ${target_name} from ${source_file}")
    else()
        message(WARNING "Skip target ${target_name}: ${source_file} not found")
    endif()
endfunction()

# ------------------------------------------------------------
# D455 / OpenCV V4L2 版本
# blue_d455.py 应该编译并运行这个目标：
#   build/getcubeinfo_d455
# ------------------------------------------------------------
add_opencv_executable(getcubeinfo_d455 getcubeinfo_d455.cpp)

# D455 HSV 调试程序，可选
add_opencv_executable(d455_hsv_debug d455_hsv_debug.cpp)

# ------------------------------------------------------------
# Azure Kinect / k4a 版本
# blue_k4a.py 应该编译并运行这个目标：
#   build/getcubeinfo_k4a
# ------------------------------------------------------------
find_package(k4a QUIET)

if (k4a_FOUND)
    message(STATUS "k4a found")

    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/getcubeinfo_k4a.cpp")
        add_executable(getcubeinfo_k4a getcubeinfo_k4a.cpp)
        target_link_libraries(getcubeinfo_k4a PRIVATE
            ${OpenCV_LIBRARIES}
            k4a::k4a
        )

        if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(getcubeinfo_k4a PRIVATE -O3 -march=native)
        endif()

        message(STATUS "Enabled target getcubeinfo_k4a from getcubeinfo_k4a.cpp")
    else()
        message(WARNING "Skip target getcubeinfo_k4a: getcubeinfo_k4a.cpp not found")
    endif()

    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/k4a_hsv_debug.cpp")
        add_executable(k4a_hsv_debug k4a_hsv_debug.cpp)
        target_link_libraries(k4a_hsv_debug PRIVATE
            ${OpenCV_LIBRARIES}
            k4a::k4a
        )

        if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(k4a_hsv_debug PRIVATE -O3 -march=native)
        endif()

        message(STATUS "Enabled target k4a_hsv_debug from k4a_hsv_debug.cpp")
    else()
        message(WARNING "Skip target k4a_hsv_debug: k4a_hsv_debug.cpp not found")
    endif()
else()
    message(WARNING "k4a not found. Azure Kinect targets getcubeinfo_k4a / k4a_hsv_debug will not be built.")
    message(WARNING "If k4a is installed, run cmake with: -Dk4a_DIR=/usr/lib/x86_64-linux-gnu/cmake/k4a")
endif()
