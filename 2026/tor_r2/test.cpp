#include <opencv2/opencv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <thread>
#include "CubeClassifier.hpp" // 引入刚才写的头文件

using namespace cv;
using namespace std;

// 1. 定义结构体：包含图片、原始位置、颜色标签
struct DetectedPattern {
    Mat pattern_img;       
    Rect original_rect;    
    string color_type;     
    int class_id;  // <--- 新增：神经网络识别出的编号 (0-31)
};

class CubeProcessor {
private:
    int target_size = 100; // 输出图片大小 64x64 或 100x100 适合神经网络
    int min_color_area;
    int min_white_area;
    
    Scalar lower_blue = Scalar(100, 100, 46);
    Scalar upper_blue = Scalar(124, 255, 255);
    Scalar lower_red1 = Scalar(0, 53, 66);
    Scalar upper_red1 = Scalar(10, 255, 255);
    Scalar lower_red2 = Scalar(156, 53, 66);
    Scalar upper_red2 = Scalar(180, 255, 255);

public:
    CubeProcessor() {}

    // --- 新增：Masked Otsu SIMD 工具函数 ---
    static int maskedOtsu(const cv::Mat& src, const cv::Mat& mask) {
    CV_Assert(src.type() == CV_8UC1);
    CV_Assert(mask.empty() || (mask.type() == CV_8UC1 && mask.size() == src.size()));
    const int rows = src.rows;
    const int cols = src.cols;
    int nThreads = cv::getNumThreads();

    std::vector<std::array<int, 256>> localHists(rows);
for (auto& h : localHists) h.fill(0);

cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
    for (int y = range.start; y < range.end; ++y) {
        const uchar* s = src.ptr<uchar>(y);
        const uchar* m = mask.empty() ? nullptr : mask.ptr<uchar>(y);
        auto& hist = localHists[y];  // 每行一个 hist，天然线程安全

        for (int x = 0; x < cols; ++x) {
            if (m && m[x] == 0) continue;
            uchar val = s[x];
            if (val > 0) hist[val]++;
        }
    }
});


    // 归约局部直方图到全局直方图
    int hist[256] = {0};
    long long total = 0;
    
    // 遍历所有线程的直方图
    for (size_t t = 0; t < localHists.size(); ++t) {
        for (int i = 1; i < 256; ++i) {
            hist[i] += localHists[t][i];
        }
    }

    for (int i = 1; i < 256; ++i){total += hist[i]; 
        // cout<<i<<":"<<hist[i]<<endl;
    }
        
    if (total == 0) return 0;

    long long sum = 0;
    for (int i = 1; i < 256; ++i)
        sum += (long long)i * hist[i];
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
        // cout<<t<<":"<<(double)wB * (double)wF<<endl;
        if (betweenVar > maxVar) {
            maxVar = betweenVar;
            threshold = t;
        }
    }
    // cout<<threshold<<endl;
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
                if (m && m[x] == 0) {
                    d[x] = 0;
                    continue;
                }
                uchar val = s[x];
                d[x] = (val > 0 && val > thresh) ? 255 : 0;
            }
        }
    });
    }


    void sortCorners(std::vector<Point2f>& corners, Point2f sorted_pts[4]) {
        std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b) { return a.x < b.x; });
        std::vector<Point2f> lefts = {corners[0], corners[1]};
        std::vector<Point2f> rights = {corners[2], corners[3]};
        if (lefts[0].y < lefts[1].y) { sorted_pts[0] = lefts[0]; sorted_pts[3] = lefts[1]; } 
        else { sorted_pts[0] = lefts[1]; sorted_pts[3] = lefts[0]; }
        if (rights[0].y < rights[1].y) { sorted_pts[1] = rights[0]; sorted_pts[2] = rights[1]; } 
        else { sorted_pts[1] = rights[1]; sorted_pts[2] = rights[0]; }
    }

    // --- 修改后的核心处理函数 ---
    vector<DetectedPattern> process(const Mat& input, Mat& debug_img) {
        if (input.empty()) return {};
        Size isz = input.size();
        int iarea = isz.area();
        min_color_area = iarea/400;
        min_white_area = iarea/1600;
        Mat hsv;
        cvtColor(input, hsv, COLOR_BGR2HSV);

        // 1. 准备全局掩膜 (Masks)
        Mat mask_blue_hull = Mat::zeros(input.size(), CV_8UC1);
        Mat mask_red_hull = Mat::zeros(input.size(), CV_8UC1);

        // 2. 准备绿色通道 (Green Channel)
        Mat green_channel;
        vector<Mat> channels;
        split(input, channels);
        green_channel = channels[1]; // BGR 中的 G
        // 稍微做一点模糊，去噪，利于 Otsu
        GaussianBlur(green_channel, green_channel, Size(5, 5), 0);

        // --- 步骤 A: 提取蓝色凸包掩膜 ---
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
            // 一次性画出所有蓝色凸包
            drawContours(mask_blue_hull, blue_hulls, -1, Scalar(255), -1);
        }

        // --- 步骤 B: 提取红色凸包掩膜 ---
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

        vector<DetectedPattern> results;

        // -----------------------------------------------------------
        // 步骤 C: 全局 Masked Otsu 处理 (核心修改点)
        // -----------------------------------------------------------
        // 我们需要分别处理 Blue 和 Red，因为它们可能在同一张图里
        // 为了代码复用，做一个 Lambda 或者循环处理
        
        struct Task {
            Mat mask;
            string color;
        };
        vector<Task> tasks = { {mask_blue_hull, "Blue"}, {mask_red_hull, "Red"} };

        for(const auto& task : tasks) {
            // 如果这个颜色完全没有出现，mask是全黑的，跳过
            if (countNonZero(task.mask) == 0) continue;

            // 1. 计算掩膜 Otsu 阈值 (SIMD 加速)
            int otsu_thresh = maskedOtsu(green_channel, task.mask);
            
            // 保底阈值: 防止环境太暗导致阈值过低
            if (otsu_thresh < 80) otsu_thresh = 80;

            // 2. 应用阈值得到二值图 (仅在 mask 范围内有效)
            Mat binary_white;
            applyMaskedOtsu(green_channel, binary_white, otsu_thresh, task.mask);

            // Debug: 可以显示一下这张神奇的全局二值图
            // imshow("Binary " + task.color, binary_white);

            // 3. 在二值图里找白色图案
            vector<vector<Point>> white_contours;
            findContours(binary_white, white_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            for (const auto& w_cnt : white_contours) {
                if (contourArea(w_cnt) < min_white_area) continue;

                // --- 接下来的逻辑和之前一模一样 (凸包 -> 拟合 -> 膨胀 -> 透视) ---
                vector<Point> hull;
                convexHull(w_cnt, hull);
                
                // 计算紧致 Bounding Box
                Rect tight_rect = boundingRect(hull);
                
                // Debug 画凸包 (黄色)
                vector<vector<Point>> hull_wrapper = {hull};
                drawContours(debug_img, hull_wrapper, 0, Scalar(0, 255, 255), 2);

                vector<Point> approx;
                double peri = arcLength(hull, true);
                approxPolyDP(hull, approx, 0.04 * peri, true);

                vector<Point2f> src_pts_vec;
                bool is4 = false;

                if (approx.size() == 4) {
                    is4 = true;
                    for(Point p : approx) src_pts_vec.push_back(Point2f((float)p.x, (float)p.y));
                    for(int j=0; j<4; j++) line(debug_img, approx[j], approx[(j+1)%4], Scalar(255, 0, 0), 2);
                } else {
                    RotatedRect r_rect = minAreaRect(w_cnt);
                    Point2f rect_pts[4]; r_rect.points(rect_pts);
                    for(int j=0; j<4; j++) src_pts_vec.push_back(rect_pts[j]);
                    for(int j=0; j<4; j++) line(debug_img, src_pts_vec[j], src_pts_vec[(j+1)%4], Scalar(0, 255, 0), 2);
                }

                if (is4) {
                    Point2f center(0, 0);
                    for(const auto& p : src_pts_vec) center += p;
                    center /= 4.0;
                    float scale_factor = 1.15;
                    for(auto& p : src_pts_vec) p = center + (p - center) * scale_factor;
                }

                Point2f src_pts[4];
                sortCorners(src_pts_vec, src_pts);
                Point2f dst_pts[4] = {
                    Point2f(0, 0), Point2f((float)target_size-1, 0),
                    Point2f((float)target_size-1, (float)target_size-1), Point2f(0, (float)target_size-1)
                };

                Mat M = getPerspectiveTransform(src_pts, dst_pts);
                Mat warped;
                // 注意：这里透视变换的源图，用全局的彩色图

                warpPerspective(input, warped, M, Size(target_size, target_size));

                DetectedPattern pattern;
                pattern.pattern_img = warped;
                pattern.original_rect = tight_rect;
                pattern.color_type = task.color; 
                results.push_back(pattern);
            }
        }
        return results;
    }

    /**
     * @brief 拼图 (适配 DetectedPattern 结构体)
     */
    Mat stitchImages(const vector<DetectedPattern>& patterns) {
        if (patterns.empty()) return Mat();

        int n = patterns.size();
        int cols = ceil(sqrt(n));
        int rows = ceil((double)n / cols);
        
        int w = 100; // 固定尺寸
        int h = 100;

        Mat canvas = Mat::zeros(rows * h, cols * w, CV_8UC3);

        for (int i = 0; i < n; i++) {
            int r = i / cols;
            int c = i % cols;
            Rect roi(c * w, r * h, w, h);
            
            // 拷贝小图
            patterns[i].pattern_img.copyTo(canvas(roi));

            // 在小图上画个标记，指示它是红是蓝
            Scalar txt_color = (patterns[i].color_type == "Blue") ? Scalar(255, 0, 0) : Scalar(0, 0, 255);
            // putText(canvas, patterns[i].color_type.substr(0, 1), Point(c*w + 5, r*h + 20), 
            //         FONT_HERSHEY_SIMPLEX, 0.7, txt_color, 2);
        }
        return canvas;
    }
};

int main(int argc, char **argv) {
    string pth,mpth;
    mpth = "cube_model_final.onnx";
    bool isv = false;
    
    // 参数解析
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "-p" || arg == "--pic") {
            if (i + 1 < argc) pth = argv[++i];
        }
        if (arg == "-v" || arg == "--video") {
            if (i + 1 < argc) { pth = argv[++i]; isv = true; }
        }
        if (arg == "-m" || arg == "--model") {
            if (i + 1 < argc) { mpth = argv[++i]; }
        }
    }

    if (pth.empty()||mpth.empty()) {
        cout << "Usage: ./program -p image.jpg -m model_path OR ./program -v video.mp4 -m model_path" << endl;
        // 默认 fallback (方便调试)
        // pth = "test.jpg";
        return 0;
    }
    // 1. 初始化模型
    // 检查模型文件是否存在
    ifstream f(mpth.c_str());
    if (!f.good()) {
        cerr << "错误: 找不到 ONNX 模型文件 -> " << mpth << endl;
        return -1;
    }
    
    // 实例化推理器
    CubeClassifier classifier(mpth);
    VideoCapture cap;
    if (isv) {
        cap.open(pth);
        if (!cap.isOpened()) {
            cerr << "Error opening video: " << pth << endl;
            return -1;
        }
    }

    CubeProcessor processor;
    Mat frame;

    cout << "Starting... Press 'ESC' or 'q' to quit." << endl;
    
    // --- FPS 变量初始化 ---
    double fps = 0.0;
    int frame_counter = 0;
    int64 start_time = cv::getTickCount();
    while (true) {
        int64 frame_start = cv::getTickCount();
        if (isv) {
            cap >> frame;
            if (frame.empty()) break;
        } else {
            frame = imread(pth);
            if (frame.empty()) { cerr << "Error opening image: " << pth << endl; return -1; }
        }
        Mat debug_img = frame.clone();
        // int64 kaishi = cv::getTickCount();
        // 1. 获取检测结果 (包含颜色)
        vector<DetectedPattern> results = processor.process(frame, debug_img);
        // --- 批量推理准备 ---
        vector<Mat> batch_inputs;
        for (const auto& pattern : results) {
            Mat input_gray;
            cvtColor(pattern.pattern_img, input_gray, COLOR_BGR2GRAY);
            batch_inputs.push_back(input_gray);
        }

        if (!batch_inputs.empty()) {
        // 调用我们刚写的 predict_batch
        
        vector<int> ids = classifier.predict_batch(batch_inputs);
        
        // 将结果填回 results
        for (size_t i = 0; i < results.size(); i++) {
            results[i].class_id = ids[i];

            // 画图逻辑放回这里
            string text = to_string(results[i].class_id);
            Point center = results[i].original_rect.tl();
            if(ids[i]!=0)
            {
                putText(debug_img, text, center, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 3);
                putText(debug_img, text, center, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 1);
            }
            putText(results[i].pattern_img, text, Point(10 ,60), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 3);
            putText(results[i].pattern_img, text, Point(10 ,60), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 1);
        }
    }
        // int64 jieshu = cv::getTickCount();

        // 2. 拼图显示
        Mat result_grid = processor.stitchImages(results);

        // 3. 在这里，你可以访问 results[i].color_type 来获取颜色了！
        // 示例：打印检测到的数量
        // cout << "Detected " << results.size() << " patterns." << endl;
        
        // 2. 计算 FPS 逻辑
        // 每一帧结束时计算耗时
        int64 frame_end = cv::getTickCount();
        double frame_time = (frame_end - frame_start) / cv::getTickFrequency();
        double current_fps = 1.0 / frame_time;
        // cout<<(jieshu -kaishi)<<"abab"<<(frame_end-frame_start)<<endl;
        fps = current_fps;

        // 3. 将 FPS 画在左上角
        string fps_text = "FPS: " + to_string((int)fps);
        
        // 画个黑色背景框，保证字能看清
        rectangle(debug_img, Point(0, 0), Point(150, 40), Scalar(0, 0, 0), -1);
        putText(debug_img, fps_text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2);

        // 调整 debug 窗口大小防止太大
        Mat disp_debug;
        float scale = 1; // 如果屏幕够大可以设为 1.0
        resize(debug_img, disp_debug, Size(), scale, scale);
        
        imshow("Main Debug View", disp_debug);
        if (!result_grid.empty()) {
            imshow("Stitched Patterns (R/B)", result_grid);
            if(!isv) cv::imwrite("111.jpg", result_grid);
        }

        // 图片模式只显示一次等待按键，视频模式持续刷新
        int wait_time = isv ? 1 : 0;
        char key = (char)waitKey(wait_time);
        if (key == 27 || key == 'q') break;
        
        // 如果是单张图片，处理完一次就退出循环，或者留着窗口
        if (!isv) break; 
    }

    if (isv) cap.release();
    destroyAllWindows();
    return 0;
}