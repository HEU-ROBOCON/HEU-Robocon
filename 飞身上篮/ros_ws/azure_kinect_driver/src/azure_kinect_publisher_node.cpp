#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <opencv2/opencv.hpp>

#include <k4a/k4a.hpp> // Azure Kinect SDK C++ API
#include <k4a/k4a.h>
#include <iostream>
#include <vector>

// 将 k4a 图像数据转换为 cv::Mat
// 注意：此函数假定 k4a_image_t 是 BGRA8 格式的彩色图或 16 位深度图
cv::Mat k4a_image_to_cv_mat(const k4a::image& k4a_image) {
    k4a_image_format_t format = k4a_image.get_format();
    int width = k4a_image.get_width_pixels();
    int height = k4a_image.get_height_pixels();
    const uint8_t* buffer = k4a_image.get_buffer();

    if (format == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
        // 直接从缓冲区创建 Mat (BGRA)
        cv::Mat bgra_image(height, width, CV_8UC4, (void*)buffer);
        cv::Mat bgr_image;
        // 转换为 BGR 用于 OpenCV 和 ROS
        cv::cvtColor(bgra_image, bgr_image, cv::COLOR_BGRA2BGR);
        return bgr_image;
    } else if (format == K4A_IMAGE_FORMAT_DEPTH16) {
        // 直接从缓冲区创建 Mat (16位单通道)
        return cv::Mat(height, width, CV_16UC1, (void*)buffer);
    } else if (format == K4A_IMAGE_FORMAT_IR16) {
        // IR 图像，如果需要可以添加处理
        return cv::Mat(height, width, CV_16UC1, (void*)buffer);
    }
    ROS_WARN("Azure Kinect image formats that aren't supported!");
    return cv::Mat();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "azure_kinect_publisher"); // 初始化 ROS 节点
    ros::NodeHandle nh("~"); // 创建私有节点句柄，用于获取参数

    // --- ROS 发布器 ---
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_color = it.advertise("/k4a/color/image_raw", 1);
    image_transport::Publisher pub_depth = it.advertise("/k4a/aligned_depth_to_color/image_raw", 1); // 发布对齐后的深度图
    ros::Publisher pub_color_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/k4a/color/camera_info", 1);
    // ros::Publisher pub_depth_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/k4a/aligned_depth_to_color/camera_info", 1); // 通常与彩色相机信息相同

    // --- Azure Kinect 设备配置参数 ---
    int device_id_param;
    std::string color_resolution_param;
    std::string depth_mode_param;
    int camera_fps_param;

    nh.param<int>("device_id", device_id_param, K4A_DEVICE_DEFAULT); // 设备 ID，默认为第一个找到的设备
    nh.param<std::string>("color_resolution", color_resolution_param, "720P"); // 彩色分辨率
    nh.param<std::string>("depth_mode", depth_mode_param, "NFOV_UNBINNED");   // 深度模式
    nh.param<int>("camera_fps", camera_fps_param, 30);                    // 帧率

    k4a_color_resolution_t color_resolution = K4A_COLOR_RESOLUTION_720P;
    if (color_resolution_param == "OFF") color_resolution = K4A_COLOR_RESOLUTION_OFF;
    else if (color_resolution_param == "720P") color_resolution = K4A_COLOR_RESOLUTION_720P;
    else if (color_resolution_param == "1080P") color_resolution = K4A_COLOR_RESOLUTION_1080P;
    else if (color_resolution_param == "1440P") color_resolution = K4A_COLOR_RESOLUTION_1440P;
    else if (color_resolution_param == "1536P") color_resolution = K4A_COLOR_RESOLUTION_1536P;
    else if (color_resolution_param == "2160P") color_resolution = K4A_COLOR_RESOLUTION_2160P;
    else if (color_resolution_param == "3072P") color_resolution = K4A_COLOR_RESOLUTION_3072P;
    else ROS_WARN("do not support: %s. use default 720P.", color_resolution_param.c_str());

    k4a_depth_mode_t depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    if (depth_mode_param == "OFF") depth_mode = K4A_DEPTH_MODE_OFF;
    else if (depth_mode_param == "NFOV_2X2BINNED") depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    else if (depth_mode_param == "NFOV_UNBINNED") depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    else if (depth_mode_param == "WFOV_2X2BINNED") depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    else if (depth_mode_param == "WFOV_UNBINNED") depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    else if (depth_mode_param == "PASSIVE_IR") depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
    else ROS_WARN("do not support: %s. use default NFOV_UNBINNED.", depth_mode_param.c_str());

    k4a_fps_t camera_fps = K4A_FRAMES_PER_SECOND_30;
    if (camera_fps_param == 5) camera_fps = K4A_FRAMES_PER_SECOND_5;
    else if (camera_fps_param == 15) camera_fps = K4A_FRAMES_PER_SECOND_15;
    else if (camera_fps_param == 30) camera_fps = K4A_FRAMES_PER_SECOND_30;
    else ROS_WARN("do not support: %d. use default 30 FPS.", camera_fps_param);


    ROS_INFO("try to open Azure Kinect ID: %d", device_id_param);
    k4a::device device = nullptr;
    try {
        device = k4a::device::open(device_id_param);
    } catch (const k4a::error& e) {
        ROS_ERROR("open Azure Kinect failed: %s", e.what());
        return 1;
    }
    ROS_INFO("open Azure Kinect successfully. serialnum: %s", device.get_serialnum().c_str());

    // --- 配置相机 ---
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // SDK 输出 BGRA, 我们在回调中转为 BGR
    config.color_resolution = color_resolution;
    config.depth_mode = depth_mode;
    config.camera_fps = camera_fps;
    config.synchronized_images_only = true; // 确保彩色和深度图像同步

    ROS_INFO("Configure camera parameters: Color Resolution=%s, Depth Mode=%s, FPS=%d",
             color_resolution_param.c_str(), depth_mode_param.c_str(), camera_fps_param);

    // if (K4A_RESULT_SUCCEEDED != device.start_cameras(&config)) {
    //     ROS_ERROR("启动 Azure Kinect 相机失败。");
    //     device.close();
    //     return 1;
    // }
    try {
        device.start_cameras(&config); // 无返回值，通过异常反馈错误
        ROS_INFO("The Azure Kinect camera started successfully.");
    } catch (const k4a::error& e) {
        ROS_ERROR("Starting the Azure Kinect camera failed: %s", e.what());
        device.close();
        return 1;
    }

    // --- 获取标定数据和转换句柄 ---
    k4a::calibration calibration;
    try {
        calibration = device.get_calibration(config.depth_mode, config.color_resolution);
    } catch (const k4a::error& e) {
        ROS_ERROR("Failed to obtain camera calibration data: %s", e.what());
        device.close();
        return 1;
    }
    k4a::transformation transformation(calibration); // 用于将深度图转换到彩色相机坐标系

    // --- 填充相机内参消息 (CameraInfo) ---
    // 我们主要关心彩色相机的内参，因为深度图会对齐到它
    sensor_msgs::CameraInfo color_cam_info_msg;
    auto& color_intrinsics = calibration.color_camera_calibration.intrinsics.parameters.param;
    color_cam_info_msg.width = calibration.color_camera_calibration.resolution_width;
    color_cam_info_msg.height = calibration.color_camera_calibration.resolution_height;
    color_cam_info_msg.K[0] = color_intrinsics.fx; // fx
    color_cam_info_msg.K[2] = color_intrinsics.cx; // cx
    color_cam_info_msg.K[4] = color_intrinsics.fy; // fy
    color_cam_info_msg.K[5] = color_intrinsics.cy; // cy
    color_cam_info_msg.K[8] = 1.0;

    // 畸变参数 k1, k2, p1, p2, k3, k4, k5, k6
    // Azure Kinect 使用 Brown Conrady (rational 6) 模型
    color_cam_info_msg.distortion_model = "rational_polynomial"; // 或 "plumb_bob" 如果只用前5个
    color_cam_info_msg.D.resize(8); // K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6 使用8个参数, k4,k5,k6可能为0
    color_cam_info_msg.D[0] = color_intrinsics.k1;
    color_cam_info_msg.D[1] = color_intrinsics.k2;
    color_cam_info_msg.D[2] = color_intrinsics.p1;
    color_cam_info_msg.D[3] = color_intrinsics.p2;
    color_cam_info_msg.D[4] = color_intrinsics.k3;
    color_cam_info_msg.D[5] = color_intrinsics.k4;
    color_cam_info_msg.D[6] = color_intrinsics.k5;
    color_cam_info_msg.D[7] = color_intrinsics.k6;


    // 旋转矩阵 R (单位阵，因为 CameraInfo 的 R 是相对于立体相机基线的，这里是单目信息)
    color_cam_info_msg.R[0] = 1.0; color_cam_info_msg.R[4] = 1.0; color_cam_info_msg.R[8] = 1.0;

    // 投影矩阵 P
    color_cam_info_msg.P[0] = color_intrinsics.fx; // fx
    color_cam_info_msg.P[2] = color_intrinsics.cx; // cx
    color_cam_info_msg.P[5] = color_intrinsics.fy; // fy
    color_cam_info_msg.P[6] = color_intrinsics.cy; // cy
    color_cam_info_msg.P[10] = 1.0;
    // P[3] 和 P[7] (Tx, Ty) 对于单目相机为0

    std::string color_frame_id = "k4a_color_optical_frame";
    std::string depth_frame_id = "k4a_color_optical_frame"; // 因为深度图已对齐到彩色相机坐标系

    // --- 主循环：获取图像并发布 ---
    k4a::capture capture;
    ros::Rate rate(camera_fps_param); // 控制发布频率，尽量与相机帧率匹配

    ROS_INFO("Start capturing and publishing Azure Kinect data...");
    while (ros::ok() && nh.ok()) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (device.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE))) {
            header.frame_id = color_frame_id;
            color_cam_info_msg.header = header;
            pub_color_cam_info.publish(color_cam_info_msg);

            // --- 处理彩色图像 ---
            k4a::image k4a_color_image = capture.get_color_image();
            if (k4a_color_image) {
                cv::Mat cv_color_image = k4a_image_to_cv_mat(k4a_color_image);
                if (!cv_color_image.empty()) {
                    sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_color_image).toImageMsg();
                    pub_color.publish(color_msg);
                }
                // k4a_color_image 会在 capture 析构时自动释放，或者手动 capture.reset_color_image();
            } else {
                ROS_WARN_THROTTLE(1, "Failed to acquire color images.");
            }

            // --- 处理深度图像 (并对齐到彩色相机) ---
            k4a::image k4a_depth_image = capture.get_depth_image();
            if (k4a_depth_image) {
                k4a::image transformed_depth_image = nullptr;
                try {
                    // 创建一个用于存放转换后深度图的图像对象
                    // 尺寸与彩色相机相同，格式为 DEPTH16
                    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                                                 calibration.color_camera_calibration.resolution_width,
                                                                 calibration.color_camera_calibration.resolution_height,
                                                                 calibration.color_camera_calibration.resolution_width * (int)sizeof(uint16_t));
                    // 执行转换
                    transformation.depth_image_to_color_camera(k4a_depth_image, &transformed_depth_image);

                    cv::Mat cv_transformed_depth_image = k4a_image_to_cv_mat(transformed_depth_image);
                    if (!cv_transformed_depth_image.empty()) {
                        std_msgs::Header depth_header = header;
                        depth_header.frame_id = depth_frame_id; // 确保 frame_id 正确
                        sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(depth_header, sensor_msgs::image_encodings::TYPE_16UC1, cv_transformed_depth_image).toImageMsg();
                        pub_depth.publish(depth_msg);
                    }
                } catch (const k4a::error& e) {
                    ROS_ERROR("Conversion of depth image failed: %s", e.what());
                }
                // transformed_depth_image 和 k4a_depth_image 会自动管理内存
            } else {
                ROS_WARN_THROTTLE(1, "Failed to acquire depth image.");
            }
            capture.reset(); // 释放捕获中的图像内存
        } else {
            ROS_WARN_THROTTLE(1, "Getting a capture from Azure Kinect failed.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Stop the camera and turn off the device...");
    device.stop_cameras();
    device.close();
    ROS_INFO("The Azure Kinect device is turned off.");

    return 0;
}