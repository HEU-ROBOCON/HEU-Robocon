cmake_minimum_required(VERSION 3.9.1)
project(RC2023)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(Cudart_LIBS "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so")
enable_language(CUDA)
find_package(CUDA REQUIRED)
find_package(k4a REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(
		Yolo
        ${OpenCV_INCLUDE_DIRS}
		/usr/local/cuda/include
		${CMAKE_CURRENT_LIST_DIR}
)
add_library(
		Yolo
		Yolo/yolov5.cpp
)

cuda_add_library(myplugins SHARED ./Yolo/yololayer.cu)


add_executable(${PROJECT_NAME}
		       main.cpp
		       Yolo/yolov5.cpp
			   src/Tower_Detection.cpp
			   src/serial_port.cpp
			   Yolo/calibrator.cpp
			   src/Control_method.cpp
        )
target_link_libraries(
	${PROJECT_NAME}
	k4a::k4a
		nvinfer
		cudart
		myplugins
		/usr/local/cuda/lib64
		${Cudart_LIBS}
		${OpenCV_LIBS}
        )
