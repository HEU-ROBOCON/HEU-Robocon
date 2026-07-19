#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <filesystem> // C++17 标准库，用于文件操作
#include <string>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

// 结构体：仅用于内部传递
struct ProcessedSample {
    Mat image;          // 灰度图
    string color_type;  // "Red" or "Blue"
};

class DatasetProcessor {
private:
    int target_size = 100; // 输出图片大小 64x64 或 100x100 适合神经网络
    int min_color_area = 10000;
    int min_white_area = 8000;
    
    // 颜色阈值 (请确保这些阈值在你现在的环境下是准的)
    Scalar lower_blue = Scalar(100, 80, 46);
    Scalar upper_blue = Scalar(124, 255, 255);
    
    Scalar lower_red1 = Scalar(0, 53, 66);
    Scalar upper_red1 = Scalar(10, 255, 255);
    Scalar lower_red2 = Scalar(156, 53, 66);
    Scalar upper_red2 = Scalar(180, 255, 255);

public:
    void sortCorners(std::vector<Point2f>& corners, Point2f sorted_pts[4]) {
        std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b) { return a.x < b.x; });
        std::vector<Point2f> lefts = {corners[0], corners[1]};
        std::vector<Point2f> rights = {corners[2], corners[3]};
        if (lefts[0].y < lefts[1].y) { sorted_pts[0] = lefts[0]; sorted_pts[3] = lefts[1]; } 
        else { sorted_pts[0] = lefts[1]; sorted_pts[3] = lefts[0]; }
        if (rights[0].y < rights[1].y) { sorted_pts[1] = rights[0]; sorted_pts[2] = rights[1]; } 
        else { sorted_pts[1] = rights[1]; sorted_pts[2] = rights[0]; }
    }

    vector<ProcessedSample> process_single_image(const Mat& input) {
        vector<ProcessedSample> samples;
        if (input.empty()) return samples;

        Mat hsv, mask_blue, mask_red, mask_red1, mask_red2;
        cvtColor(input, hsv, COLOR_BGR2HSV);
        
        vector<pair<vector<Point>, string>> candidates;

        // 1. 蓝色处理
        inRange(hsv, lower_blue, upper_blue, mask_blue);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(mask_blue, mask_blue, MORPH_OPEN, kernel);
        morphologyEx(mask_blue, mask_blue, MORPH_CLOSE, kernel);
        
        vector<vector<Point>> contours_blue;
        findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (const auto& cnt : contours_blue) {
            if (contourArea(cnt) > min_color_area) {
                vector<Point> hull;
                convexHull(cnt, hull);
                candidates.push_back({hull, "Blue"});
            }
        }

        // 2. 红色处理
        inRange(hsv, lower_red1, upper_red1, mask_red1);
        inRange(hsv, lower_red2, upper_red2, mask_red2);
        bitwise_or(mask_red1, mask_red2, mask_red);
        morphologyEx(mask_red, mask_red, MORPH_OPEN, kernel);
        morphologyEx(mask_red, mask_red, MORPH_CLOSE, kernel);

        vector<vector<Point>> contours_red;
        findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (const auto& cnt : contours_red) {
            if (contourArea(cnt) > min_color_area) {
                vector<Point> hull;
                convexHull(cnt, hull);
                candidates.push_back({hull, "Red"});
            }
        }

        // 3. 提取ROI并处理
        for (const auto& item : candidates) {
            const auto& cnt = item.first;
            string color_type = item.second;

            Rect roi_rect = boundingRect(cnt);
            roi_rect &= Rect(0, 0, input.cols, input.rows);
            if (roi_rect.area() == 0) continue;

            // --- 核心修改：生成灰度图 ---
            Mat raw_roi = input(roi_rect);
            
            // 制作掩膜去除背景
            Mat roi_mask = Mat::zeros(raw_roi.size(), CV_8UC1);
            vector<Point> shifted_cnt;
            Point offset = roi_rect.tl();
            for (const auto& p : cnt) shifted_cnt.push_back(p - offset);
            vector<vector<Point>> shifted_wrapper = {shifted_cnt};
            drawContours(roi_mask, shifted_wrapper, 0, Scalar(255), -1);

            Mat masked_roi = Mat::zeros(raw_roi.size(), raw_roi.type());
            raw_roi.copyTo(masked_roi, roi_mask);

            // **关键：提取绿色通道作为最终的灰度数据**
            // 相比普通转灰度，绿色通道对比度最高，最利于神经网络学习形状
            vector<Mat> channels;
            split(masked_roi, channels);
            Mat gray_roi = channels[1]; // Green Channel

            // 预处理
            GaussianBlur(gray_roi, gray_roi, Size(5, 5), 0);
            
            // 二值化找轮廓
            Mat binary_white;
            double otsu = threshold(gray_roi, binary_white, 0, 255, THRESH_BINARY | THRESH_OTSU);
            if (otsu < 100) threshold(gray_roi, binary_white, 80, 255, THRESH_BINARY);

            vector<vector<Point>> white_contours;
            findContours(binary_white, white_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            for (const auto& w_cnt : white_contours) {
                if (contourArea(w_cnt) < min_white_area) continue;

                // 几何处理 (凸包 -> 拟合 -> 膨胀)
                vector<Point> hull;
                convexHull(w_cnt, hull);
                
                vector<Point> approx;
                double peri = arcLength(hull, true);
                approxPolyDP(hull, approx, 0.04 * peri, true);

                vector<Point2f> src_pts_vec;
                bool is4 = false;

                if (approx.size() == 4) {
                    is4 = true;
                    for(auto p : approx) src_pts_vec.push_back(p);
                } else {
                    RotatedRect rr = minAreaRect(w_cnt);
                    Point2f pts[4]; rr.points(pts);
                    for(int j=0; j<4; j++) src_pts_vec.push_back(pts[j]);
                }

                if (is4) {
                    Point2f center(0,0);
                    for(auto p : src_pts_vec) center += p;
                    center /= 4.0;
                    float scale = 1.15;
                    for(auto& p : src_pts_vec) p = center + (p - center) * scale;
                }

                // 透视变换
                Point2f src_pts[4];
                sortCorners(src_pts_vec, src_pts);
                Point2f dst_pts[4] = {
                    Point2f(0, 0), Point2f((float)target_size-1, 0),
                    Point2f((float)target_size-1, (float)target_size-1), Point2f(0, (float)target_size-1)
                };

                Mat M = getPerspectiveTransform(src_pts, dst_pts);
                Mat warped;
                
                // **核心修改：直接变换 gray_roi (绿色通道灰度图)**
                // 这样输出的就是单通道灰度图
                warpPerspective(gray_roi, warped, M, Size(target_size, target_size));

                samples.push_back({warped, color_type});
            }
        }
        return samples;
    }
};

// ... 前面的 includes 和 DatasetProcessor 类代码保持不变 ...

int main(int argc, char** argv) {
    // 1. 配置路径
    string input_folder = "../tor_r2/pic/xinzeng";      
    string output_base = "dataset_output"; 
    
    if (argc >= 2) input_folder = argv[1];
    if (argc >= 3) output_base = argv[2];

    // 2. 检查输入目录
    if (!fs::exists(input_folder)) {
        cerr << "错误: 输入文件夹不存在 -> " << input_folder << endl;
        return -1;
    }

    // 3. 创建目录
    string out_red = output_base + "/Red";
    string out_blue = output_base + "/Blue";
    
    try {
        if (!fs::exists(output_base)) fs::create_directory(output_base);
        if (!fs::exists(out_red)) fs::create_directory(out_red);
        if (!fs::exists(out_blue)) fs::create_directory(out_blue);
    } catch (fs::filesystem_error& e) {
        cerr << "创建目录失败: " << e.what() << endl;
        return -1;
    }

    DatasetProcessor processor;
    
    // 4. 获取图片列表
    vector<String> filenames;
    glob(input_folder + "/*.jpg", filenames, false);
    vector<String> png_files;
    glob(input_folder + "/*.png", png_files, false);
    filenames.insert(filenames.end(), png_files.begin(), png_files.end());

    if (filenames.empty()) {
        cerr << "警告: 文件夹里没有图片" << endl;
        return -1;
    }

    cout << "找到 " << filenames.size() << " 张图片，开始处理..." << endl;

    int total_samples = 0;
    int processed_files_count = 0;

    // 5. 批量处理循环
    for (const auto& file_path : filenames) {
        Mat frame = imread(file_path);
        if (frame.empty()) continue;

        // --- [修改点 1]：提取原始文件名 (不含扩展名) ---
        // 假设 file_path 是 "/home/user/data/frame_001.jpg"
        fs::path p(file_path);
        string original_name = p.stem().string(); // 得到 "frame_001"

        // 调用处理
        vector<ProcessedSample> results = processor.process_single_image(frame);

        // 如果一张图里有多个方块，用 index 区分
        int index_in_this_image = 0;

        for (const auto& sample : results) {
            string save_dir = (sample.color_type == "Blue") ? out_blue : out_red;
            
            // --- [修改点 2]：构建新的文件名 ---
            // 格式: 目录/原文件名_序号.jpg
            // 示例: dataset_output/Red/frame_001_0.jpg
            stringstream ss;
            ss << save_dir << "/" << original_name << "_" << index_in_this_image << ".jpg";
            string filename = ss.str();

            if (imwrite(filename, sample.image)) {
                total_samples++;
                index_in_this_image++;
            }
        }

        processed_files_count++;
        // 进度显示
        if (processed_files_count % 10 == 0) {
            cout << "处理进度: " << processed_files_count << "/" << filenames.size() 
                 << " (已生成样本: " << total_samples << ")" << "\r" << flush;
        }
    }

    cout << endl << "处理完成！" << endl;
    cout << "总共生成样本数: " << total_samples << endl;
    cout << "保存在: " << output_base << endl;

    return 0;
}