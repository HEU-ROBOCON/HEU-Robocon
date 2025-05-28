/*
文件名：Circle_Position.cpp
功能：通过颜色和深度图像检测圆形并计算其深度
*/
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

int main() try {
    // 创建 RealSense 管道
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);

    // 创建 OpenCV 窗口
    cv::namedWindow("Color Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) < 0) {
        // 等待下一帧
        rs2::frameset frames = pipe.wait_for_frames();

        // 获取彩色和深度帧
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame) continue;

        // 将帧转换为 OpenCV Mat
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // 将彩色图像从 BGR 转换到 HSV
        cv::Mat hsv_image;
        cv::cvtColor(color_image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义绿色和红色的 HSV 范围
        cv::Scalar green_lower(35, 100, 100);
        cv::Scalar green_upper(85, 255, 255);
        cv::Scalar red_lower1(0, 100, 100);
        cv::Scalar red_upper1(10, 255, 255);
        cv::Scalar red_lower2(160, 100, 100);
        cv::Scalar red_upper2(180, 255, 255);

        // 提取绿色区域
        cv::Mat green_mask;
        cv::inRange(hsv_image, green_lower, green_upper, green_mask);

        // 提取红色区域
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv_image, red_lower1, red_upper1, red_mask1);
        cv::inRange(hsv_image, red_lower2, red_upper2, red_mask2);
        red_mask = red_mask1 | red_mask2;

        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);

        // 检测绿色圆形
        std::vector<std::vector<cv::Point>> green_contours;
        cv::findContours(green_mask, green_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto& contour : green_contours) {
            if (contour.size() > 5) {
                cv::RotatedRect ellipse = cv::fitEllipse(contour);
                float aspect_ratio = std::min(ellipse.size.width, ellipse.size.height) / 
                                   std::max(ellipse.size.width, ellipse.size.height);
                if (aspect_ratio > 0.8) {
                    cv::Point2f center = ellipse.center;
                    // 坐标范围检查
                    if (center.x < 0 || center.x >= color_image.cols || 
                        center.y < 0 || center.y >= color_image.rows) {
                        continue; // 跳过不在图像范围内的点
                    }
                    float depth = depth_frame.as<rs2::depth_frame>().get_distance(center.x, center.y);
                    if (depth > 0) {
                        cv::ellipse(color_image, ellipse, cv::Scalar(0, 255, 0), 2);
                        cv::putText(color_image, "Green: " + std::to_string(depth * 1000) + " mm", 
                                  center, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                        std::cout << "Green circle depth: " << depth * 1000 << " mm" << std::endl;
                    }
                }
            }
        }

        // 检测红色圆形
        std::vector<std::vector<cv::Point>> red_contours;
        cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto& contour : red_contours) {
            if (contour.size() > 5) {
                cv::RotatedRect ellipse = cv::fitEllipse(contour);
                float aspect_ratio = std::min(ellipse.size.width, ellipse.size.height) / 
                                   std::max(ellipse.size.width, ellipse.size.height);
                if (aspect_ratio > 0.8) {
                    cv::Point2f center = ellipse.center;
                    // 坐标范围检查
                    if (center.x < 0 || center.x >= color_image.cols || 
                        center.y < 0 || center.y >= color_image.rows) {
                        continue; // 跳过不在图像范围内的点
                    }
                    float depth = depth_frame.as<rs2::depth_frame>().get_distance(center.x, center.y);
                    if (depth > 0) {
                        cv::ellipse(color_image, ellipse, cv::Scalar(0, 0, 255), 2);
                        cv::putText(color_image, "Red: " + std::to_string(depth * 1000) + " mm", 
                                  center, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                        std::cout << "Red circle depth: " << depth * 1000 << " mm" << std::endl;
                    }
                }
            }
        }

        // 显示彩色图像
        cv::imshow("Color Image", color_image);

        // 显示深度图像
        cv::Mat depth_colormap;
        // 先创建中间 Mat 存储转换后的深度数据
        cv::Mat depth_8bit;
        cv::convertScaleAbs(depth_image, depth_8bit, 0.03); // 调整 alpha 参数以正确缩放深度值
        cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
        cv::imshow("Depth Image", depth_colormap);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}