#include <opencv2/opencv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <thread>

using namespace cv;
using namespace std;
class CubeProcessor {
private:
    int min_color_area;
    int min_white_area;
    
    // 颜色阈值 (保持不变)
    Scalar lower_blue = Scalar(100, 110, 46);
    Scalar upper_blue = Scalar(124, 255, 255);
    Scalar lower_red1 = Scalar(0, 110, 66);
    Scalar upper_red1 = Scalar(10, 255, 255);
    Scalar lower_red2 = Scalar(156, 110, 66);
    Scalar upper_red2 = Scalar(180, 255, 255);

public:
    CubeProcessor() {}

    // --- Masked Otsu SIMD 工具函数 (保持不变) ---
    static int maskedOtsu(const cv::Mat& src, const cv::Mat& mask) {
        CV_Assert(src.type() == CV_8UC1);
        CV_Assert(mask.empty() || (mask.type() == CV_8UC1 && mask.size() == src.size()));
        const int rows = src.rows;
        const int cols = src.cols;

        std::vector<std::array<int, 256>> localHists(rows);
        for (auto& h : localHists) h.fill(0);

        cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
            for (int y = range.start; y < range.end; ++y) {
                const uchar* s = src.ptr<uchar>(y);
                const uchar* m = mask.empty() ? nullptr : mask.ptr<uchar>(y);
                auto& hist = localHists[y];

                for (int x = 0; x < cols; ++x) {
                    if (m && m[x] == 0) continue;
                    uchar val = s[x];
                    if (val > 0) hist[val]++;
                }
            }
        });

        int hist[256] = {0};
        long long total = 0;
        for (size_t t = 0; t < localHists.size(); ++t) {
            for (int i = 1; i < 256; ++i) hist[i] += localHists[t][i];
        }
        for (int i = 1; i < 256; ++i) total += hist[i];
            
        if (total == 0) return 0;

        long long sum = 0;
        for (int i = 1; i < 256; ++i) sum += (long long)i * hist[i];
        long long sumB = 0;
        int wB = 0;
        double maxVar = -1.0;
        int threshold = 0;

        for (int t = 1; t < 256; ++t) {
            wB += hist[t];
            if (wB == 0) continue;
            int wF = (int)(total - wB);
            if (wF == 0) break;
            sumB += (long long)t * hist[t];
            double mB = (double)sumB / wB;
            double mF = (double)(sum - sumB) / wF;
            double diff = mB - mF;
            double betweenVar = (double)wB * (double)wF * diff * diff;
            if (betweenVar > maxVar) {
                maxVar = betweenVar;
                threshold = t;
            }
        }
        return threshold;
    }

    static void applyMaskedOtsu(const cv::Mat& src, cv::Mat& dst, int thresh, const cv::Mat& mask) {
        CV_Assert(src.type() == CV_8UC1);
        CV_Assert(mask.empty() || (mask.type() == CV_8UC1 && mask.size() == src.size()));
        dst.create(src.size(), CV_8UC1);
        dst.setTo(0);
        const int rows = src.rows;
        const int cols = src.cols;

        cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
            for (int y = range.start; y < range.end; ++y) {
                const uchar* s = src.ptr<uchar>(y);
                const uchar* m = mask.empty() ? nullptr : mask.ptr<uchar>(y);
                uchar* d = dst.ptr<uchar>(y);
                for (int x = 0; x < cols; ++x) {
                    if (m && m[x] == 0) { d[x] = 0; continue; }
                    uchar val = s[x];
                    d[x] = (val > 0 && val > thresh) ? 255 : 0;
                }
            }
        });
    }

    // --- 修改后的核心处理函数 ---
    // 逻辑：找到白色轮廓 -> 计算凸包 -> 找最大凸包 -> 计算凸包质心 -> 返回偏移量
    Point2f process(const Mat& input, Mat& debug_img) {
        if (input.empty()) return Point2f(0, 0);
        
        Size isz = input.size();
        Point2f img_center(isz.width / 2.0f, isz.height / 2.0f);
        
        // 动态计算最小面积阈值
        int iarea = isz.area();
        min_color_area = iarea / 400; 
        min_white_area = iarea / 1600;

        Mat hsv;
        cvtColor(input, hsv, COLOR_BGR2HSV);

        // 1. 准备全局掩膜
        Mat mask_blue_hull = Mat::zeros(isz, CV_8UC1);
        Mat mask_red_hull = Mat::zeros(isz, CV_8UC1);

        // 2. 准备绿色通道 (Green Channel)
        Mat green_channel;
        vector<Mat> channels;
        split(input, channels);
        green_channel = channels[1]; 
        GaussianBlur(green_channel, green_channel, Size(5, 5), 0);

        // --- A. 提取蓝色区域 ---
        {
            Mat mask_blue_raw;
            inRange(hsv, lower_blue, upper_blue, mask_blue_raw);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(mask_blue_raw, mask_blue_raw, MORPH_OPEN, kernel);
            morphologyEx(mask_blue_raw, mask_blue_raw, MORPH_CLOSE, kernel);
            
            vector<vector<Point>> contours;
            findContours(mask_blue_raw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
            vector<vector<Point>> blue_hulls;
            for(const auto& cnt : contours) {
                if(contourArea(cnt) > min_color_area) {
                    vector<Point> hull;
                    convexHull(cnt, hull);
                    blue_hulls.push_back(hull);
                }
            }
            drawContours(mask_blue_hull, blue_hulls, -1, Scalar(255), -1);
        }

        // --- B. 提取红色区域 ---
        {
            Mat mask_red1, mask_red2, mask_red_raw;
            inRange(hsv, lower_red1, upper_red1, mask_red1);
            inRange(hsv, lower_red2, upper_red2, mask_red2);
            bitwise_or(mask_red1, mask_red2, mask_red_raw);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(mask_red_raw, mask_red_raw, MORPH_OPEN, kernel);
            morphologyEx(mask_red_raw, mask_red_raw, MORPH_CLOSE, kernel);

            vector<vector<Point>> contours;
            findContours(mask_red_raw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            vector<vector<Point>> red_hulls;
            for(const auto& cnt : contours) {
                if(contourArea(cnt) > min_color_area) {
                    vector<Point> hull;
                    convexHull(cnt, hull);
                    red_hulls.push_back(hull);
                }
            }
            drawContours(mask_red_hull, red_hulls, -1, Scalar(255), -1);
        }

        // --- C. Masked Otsu 并在所有颜色区域中寻找最大凸包 ---

        double max_hull_area_found = 0; // 记录当前发现的最大凸包面积
        Point2f best_centroid(0, 0);    // 最佳凸包的质心
        bool found_target = false;
        vector<Point> best_hull_visual; // 用于画图显示的凸包

        struct Task { Mat mask; string color; };
        vector<Task> tasks = { {mask_blue_hull, "Blue"}, {mask_red_hull, "Red"} };

        for(const auto& task : tasks) {
            // 如果Mask是空的，跳过
            if (countNonZero(task.mask) == 0) continue;

            // 1. Masked Otsu 计算阈值并二值化
            int otsu_thresh = maskedOtsu(green_channel, task.mask);
            if (otsu_thresh < 80) otsu_thresh = 80;

            Mat binary_white;
            applyMaskedOtsu(green_channel, binary_white, otsu_thresh, task.mask);

            // 2. 找白色轮廓
            vector<vector<Point>> white_contours;
            findContours(binary_white, white_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            // 3. 遍历轮廓，找凸包，比较凸包面积
            for (const auto& w_cnt : white_contours) {
                // 先算凸包
                vector<Point> hull;
                convexHull(w_cnt, hull);
                
                // 计算凸包面积
                double hull_area = contourArea(hull);

                if (hull_area < min_white_area) continue;

                // 如果这个凸包比之前找到的都要大
                if (hull_area > max_hull_area_found) {
                    // 对【凸包】计算矩 (Moments)
                    Moments mu = moments(hull);
                    
                    if (mu.m00 > 0) { // 防止除以0
                        max_hull_area_found = hull_area;
                        
                        // 计算质心
                        best_centroid = Point2f(static_cast<float>(mu.m10 / mu.m00), 
                                                static_cast<float>(mu.m01 / mu.m00));
                        
                        // 保存下来用于画图
                        best_hull_visual = hull;
                        found_target = true;
                    }
                }
            }
        }

        // --- D. 结果绘制与返回 ---
        if (found_target) {
            // 画出最佳凸包 (黄色)
            vector<vector<Point>> draw_cnt = {best_hull_visual};
            drawContours(debug_img, draw_cnt, 0, Scalar(0, 255, 255), 2); 
            
            // 画出质心 (红色实心点)
            circle(debug_img, best_centroid, 5, Scalar(0, 0, 255), -1); 
            
            // 画出图像中心 (蓝色空心圆)
            circle(debug_img, img_center, 5, Scalar(255, 0, 0), 2);
            
            // 连线
            line(debug_img, img_center, best_centroid, Scalar(255, 255, 0), 1);

            // 返回偏移量 (目标 - 中心)
            return (best_centroid - img_center);
        }

        return Point2f(0, 0);
    }
};
int main(int argc, char **argv) {
    string pth;
    bool isv = false;
    
    // 1. 参数解析 (去掉了 -m 模型参数)
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "-p" || arg == "--pic") {
            if (i + 1 < argc) pth = argv[++i];
        }
        if (arg == "-v" || arg == "--video") {
            if (i + 1 < argc) { pth = argv[++i]; isv = true; }
        }
    }

    if (pth.empty()) {
        cout << "Usage: ./program -p image.jpg OR ./program -v video.mp4" << endl;
        return 0;
    }

    // 2. 初始化资源
    VideoCapture cap;
    Mat frame;
    
    if (isv) {
        cap.open(pth);
        if (!cap.isOpened()) {
            cerr << "错误: 无法打开视频 -> " << pth << endl;
            return -1;
        }
    } else {
        frame = imread(pth);
        if (frame.empty()) {
            cerr << "错误: 无法打开图片 -> " << pth << endl;
            return -1;
        }
    }

    // 实例化处理器
    CubeProcessor processor;
    
    cout << "Starting tracking... Press 'ESC' or 'q' to quit." << endl;

    // --- 循环处理 ---
    while (true) {
        // 视频模式下读取新的一帧
        if (isv) {
            cap >> frame;
            if (frame.empty()) {
                cout << "视频结束." << endl;
                break; 
            }
        }
        
        // 复制一份图像用于画图 (debug_img)
        Mat debug_img = frame.clone();

        // --- 核心处理开始 ---
        int64 start_time = cv::getTickCount();

        // 调用处理函数，直接获得偏移量 (dx, dy)
        // 注意：process 函数内部已经画了 轮廓、质心 和 连线
        Point2f offset = processor.process(frame, debug_img);

        int64 end_time = cv::getTickCount();
        // --- 核心处理结束 ---

        // 计算 FPS
        double frame_time = (end_time - start_time) / cv::getTickFrequency();
        double fps = 1.0 / frame_time;

        // --- 可视化信息绘制 ---
        
        // 1. 绘制 FPS
        string fps_text = "FPS: " + to_string((int)fps);
        putText(debug_img, fps_text, Point(20, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2);

        // 2. 绘制偏移量数值
        // 如果 offset 是 (0,0) 说明没找到目标
        if (offset.x == 0 && offset.y == 0) {
            string status = "Status: LOST";
            putText(debug_img, status, Point(20, 80), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
        } else {
            string offset_text = "Offset: X=" + to_string((int)offset.x) + " Y=" + to_string((int)offset.y);
            putText(debug_img, offset_text, Point(20, 80), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 2);
        }

        // 显示结果
        imshow("Tracking Result", debug_img);

        // 按键处理
        // 视频模式：延时 1ms，实现连续播放
        // 图片模式：延时 0ms，无限等待直到按键
        int wait_time = isv ? 1 : 0;
        char key = (char)waitKey(wait_time);
        
        if (key == 27 || key == 'q') break;

        // 如果是单张图片，处理完一次并按键后就退出
        if (!isv) break;
    }

    if (isv) cap.release();
    destroyAllWindows();
    return 0;
}