//
// Created by zgh on 2024/4/14.
//
/*
#include <opencv2/opencv.hpp>

int main() {
    // 打开摄像头
    cv::VideoCapture cap(0); // 参数 0 表示第一个摄像头，如果有多个摄像头，可以尝试不同的参数值

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera." << std::endl;
        return -1;
    }

    // 设置摄像头的分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 创建窗口用于显示图像
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    // 循环读取并显示摄像头图像
    while (true) {
        cv::Mat frame;
        // 从摄像头读取图像帧
        cap.read(frame);

        // 检查是否成功读取到图像帧
        if (frame.empty()) {
            std::cerr << "Error: Unable to read frame from camera." << std::endl;
            break;
        }

        // 显示图像帧
        cv::imshow("Camera", frame);

        // 检测按键，如果按下 ESC 键则退出循环
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    // 释放摄像头资源
    cap.release();

    // 关闭窗口
    cv::destroyAllWindows();

    return 0;
}
*/

#include <opencv2/opencv.hpp>

// 定义颜色区域检测函数
int detectColorRegion(const cv::Mat& frame) {
    // 定义颜色阈值
    cv::Scalar lower_red = cv::Scalar(160, 70, 50);
    cv::Scalar upper_red = cv::Scalar(180, 255, 255);
    cv::Scalar lower_blue = cv::Scalar(100, 150, 0);
    cv::Scalar upper_blue = cv::Scalar(160, 255, 255);
    cv::Scalar lower_purple = cv::Scalar(120, 50, 50);
    cv::Scalar upper_purple = cv::Scalar(160, 255, 255);

    // 转换图像到HSV色彩空间
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 创建掩码并执行颜色检测
    cv::Mat mask_red, mask_blue, mask_purple;
    cv::inRange(hsv, lower_red, upper_red, mask_red);
    cv::inRange(hsv, lower_blue, upper_blue, mask_blue);
    cv::inRange(hsv, lower_purple, upper_purple, mask_purple);

    // 计算颜色区域的占比
    double red_area = cv::countNonZero(mask_red);
    double blue_area = cv::countNonZero(mask_blue);
    double purple_area = cv::countNonZero(mask_purple);
    double total_area = frame.rows * frame.cols;

    // 判断红色、蓝色和紫色区域的占比
    if (red_area / total_area > 0.5) {
        return 1; // 红色区域占比超过50%
    } else if (blue_area / total_area > 0.5) {
        return 2; // 蓝色区域占比超过50%
    } else if (purple_area / total_area > 0.5) {
        return 3; // 紫色区域占比超过50%
    }

    return 0; // 没有检测到任何符合条件的颜色区域
}

int main() {
    // 打开摄像头
    cv::VideoCapture cap(0); // 参数 0 表示第一个摄像头，如果有多个摄像头，可以尝试不同的参数值

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera." << std::endl;
        return -1;
    }

    // 设置摄像头的分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 创建窗口用于显示图像
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    // 循环读取并显示摄像头图像
    while (true) {
        cv::Mat frame;
        // 从摄像头读取图像帧
        cap.read(frame);

        // 检查是否成功读取到图像帧
        if (frame.empty()) {
            std::cerr << "Error: Unable to read frame from camera." << std::endl;
            break;
        }

        // 检测颜色区域并获取标志位
        int color_flag = detectColorRegion(frame);

        // 根据标志位进行处理
        if (color_flag == 1) {
            // 红色区域占比超过50%
            std::cout << "Detected red color region!" << std::endl;
            // 给标志位readone置1
            // 这里假设你已经有一个名为readone的变量，并且在这里将其置为1
        } else if (color_flag == 2) {
            // 蓝色区域占比超过50%
            std::cout << "Detected blue color region!" << std::endl;
            // 给标志位readone置2
            // 这里假设你已经有一个名为readone的变量，并且在这里将其置为2
        } else if (color_flag == 3) {
            // 紫色区域占比超过50%
            std::cout << "Detected purple color region!" << std::endl;
            // 给标志位readone置3
            // 这里假设你已经有一个名为readone的变量，并且在这里将其置为3
        }

        // 显示图像帧
        cv::imshow("Camera", frame);

        // 检测按键，如果按下 ESC 键则退出循环
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    // 释放摄像头资源
    cap.release();

    // 关闭窗口
    cv::destroyAllWindows();

    return 0;
}
