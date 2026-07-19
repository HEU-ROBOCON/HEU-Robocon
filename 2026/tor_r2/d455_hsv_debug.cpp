#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

enum class BlockColor {
    EMPTY,
    RED,
    BLUE,
    UNKNOWN
};

string colorToString(BlockColor color)
{
    switch (color) {
        case BlockColor::RED:
            return "RED";
        case BlockColor::BLUE:
            return "BLUE";
        case BlockColor::UNKNOWN:
            return "UNKNOWN";
        default:
            return "EMPTY";
    }
}

// ============================================================
// D455 / V4L2 摄像头读取类
// ============================================================

class D455Camera {
private:
    cv::VideoCapture cap_;
    string camera_arg_;
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;

    static bool isNumber(const string& s)
    {
        if (s.empty()) return false;

        for (char ch : s) {
            if (!isdigit(static_cast<unsigned char>(ch))) {
                return false;
            }
        }

        return true;
    }

    bool tryOpenWithFourcc(const string& fourcc)
    {
        if (cap_.isOpened()) {
            cap_.release();
        }

        bool opened = false;

        if (isNumber(camera_arg_)) {
            opened = cap_.open(atoi(camera_arg_.c_str()), cv::CAP_V4L2);
        } else {
            opened = cap_.open(camera_arg_, cv::CAP_V4L2);
        }

        if (!opened || !cap_.isOpened()) {
            return false;
        }

        // D455 常见彩色格式：YUYV / MJPG
        if (fourcc.size() == 4) {
            cap_.set(
                cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3])
            );
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_CONVERT_RGB, 1);

        cv::Mat testFrame;

        for (int i = 0; i < 30; ++i) {
            if (cap_.read(testFrame) && !testFrame.empty()) {
                return true;
            }

            cv::waitKey(30);
        }

        cap_.release();
        return false;
    }

public:
    ~D455Camera()
    {
        closeCamera();
    }

    bool openCamera(const string& cameraArg, int width, int height, int fps)
    {
        camera_arg_ = cameraArg;
        width_ = width;
        height_ = height;
        fps_ = fps;

        cout << "Camera    : D455 / OpenCV V4L2" << endl;
        cout << "Camera arg: " << camera_arg_ << endl;
        cout << "Request   : " << width_ << "x" << height_ << " @" << fps_ << "fps" << endl;

        if (tryOpenWithFourcc("YUYV")) {
            cout << "D455 opened with YUYV." << endl;
        } else if (tryOpenWithFourcc("MJPG")) {
            cout << "D455 opened with MJPG." << endl;
        } else if (tryOpenWithFourcc("")) {
            cout << "D455 opened without fixed FOURCC." << endl;
        } else {
            cerr << "Error: 无法打开 D455 彩色相机: " << camera_arg_ << endl;
            cerr << "请用下面命令确认 D455 彩色通道:" << endl;
            cerr << "  v4l2-ctl --list-devices" << endl;
            cerr << "  v4l2-ctl -d " << camera_arg_ << " --list-formats-ext" << endl;
            return false;
        }

        double actualW = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actualH = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double actualFps = cap_.get(cv::CAP_PROP_FPS);

        int fourccInt = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
        char fourccChars[] = {
            static_cast<char>(fourccInt & 0xFF),
            static_cast<char>((fourccInt >> 8) & 0xFF),
            static_cast<char>((fourccInt >> 16) & 0xFF),
            static_cast<char>((fourccInt >> 24) & 0xFF),
            '\0'
        };

        cout << "Actual    : " << actualW << "x" << actualH << " @" << actualFps << "fps" << endl;
        cout << "FOURCC    : " << fourccChars << endl;

        return true;
    }

    bool getFrame(cv::Mat& frame_bgr)
    {
        frame_bgr.release();

        if (!cap_.isOpened()) {
            return false;
        }

        cv::Mat frame;

        if (!cap_.read(frame) || frame.empty()) {
            return false;
        }

        if (frame.channels() == 3) {
            frame_bgr = frame;
        } else if (frame.channels() == 4) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_BGRA2BGR);
        } else if (frame.channels() == 1) {
            cv::cvtColor(frame, frame_bgr, cv::COLOR_GRAY2BGR);
        } else {
            return false;
        }

        return !frame_bgr.empty();
    }

    void closeCamera()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }
};

// ============================================================
// HSV 九宫格调试逻辑
// ============================================================

cv::Rect shrinkRect(const cv::Rect& rect, double scale)
{
    int newW = static_cast<int>(rect.width * scale);
    int newH = static_cast<int>(rect.height * scale);

    int newX = rect.x + (rect.width - newW) / 2;
    int newY = rect.y + (rect.height - newH) / 2;

    return cv::Rect(newX, newY, newW, newH);
}

int largestComponentArea(const cv::Mat& binaryMask)
{
    if (binaryMask.empty()) {
        return 0;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;

    int numLabels = cv::connectedComponentsWithStats(
        binaryMask,
        labels,
        stats,
        centroids,
        8,
        CV_32S
    );

    int maxArea = 0;

    for (int i = 1; i < numLabels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        if (area > maxArea) {
            maxArea = area;
        }
    }

    return maxArea;
}

vector<cv::Rect> buildGrid(
    int frameWidth,
    int frameHeight,
    int gridSidePercent,
    int offsetX,
    int offsetY
) {
    vector<cv::Rect> grids;

    gridSidePercent = max(10, min(100, gridSidePercent));

    int gridSide = static_cast<int>(min(frameWidth, frameHeight) * (gridSidePercent / 100.0));
    int cellSize = gridSide / 3;

    int startX = (frameWidth - cellSize * 3) / 2 + offsetX;
    int startY = (frameHeight - cellSize * 3) / 2 + offsetY;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            grids.emplace_back(
                startX + c * cellSize,
                startY + r * cellSize,
                cellSize,
                cellSize
            );
        }
    }

    return grids;
}

BlockColor judgeOneCell(
    const cv::Mat& blueMask,
    const cv::Mat& redMask,
    const cv::Rect& cellRect,
    const cv::Size& frameSize,
    double coreScale,
    double thresholdRatio,
    int& bluePixels,
    int& redPixels,
    int& blueLargest,
    int& redLargest,
    int& threshold
) {
    cv::Rect validRect = cellRect & cv::Rect(0, 0, frameSize.width, frameSize.height);

    if (validRect.area() <= 0) {
        return BlockColor::UNKNOWN;
    }

    cv::Rect coreRect = shrinkRect(validRect, coreScale);
    coreRect = coreRect & cv::Rect(0, 0, frameSize.width, frameSize.height);

    if (coreRect.area() <= 0) {
        return BlockColor::UNKNOWN;
    }

    cv::Mat blueROI = blueMask(coreRect);
    cv::Mat redROI = redMask(coreRect);

    bluePixels = cv::countNonZero(blueROI);
    redPixels = cv::countNonZero(redROI);

    blueLargest = largestComponentArea(blueROI);
    redLargest = largestComponentArea(redROI);

    threshold = static_cast<int>(coreRect.area() * thresholdRatio);

    bool blueValid = blueLargest > threshold;
    bool redValid = redLargest > threshold;

    if (blueValid && redValid) {
        int maxArea = max(blueLargest, redLargest);
        int minArea = min(blueLargest, redLargest);

        if (minArea > 0 && maxArea < minArea * 1.5) {
            return BlockColor::UNKNOWN;
        }

        return blueLargest > redLargest ? BlockColor::BLUE : BlockColor::RED;
    }

    if (blueValid) {
        return BlockColor::BLUE;
    }

    if (redValid) {
        return BlockColor::RED;
    }

    return BlockColor::EMPTY;
}

void printCurrentParams(
    int blue_h_min,
    int blue_h_max,
    int blue_s_min,
    int blue_v_min,
    int red1_h_min,
    int red1_h_max,
    int red2_h_min,
    int red2_h_max,
    int red_s_min,
    int red_v_min,
    int threshold_percent,
    int grid_percent,
    int core_percent,
    int offset_x_track,
    int offset_y_track
) {
    int offsetX = offset_x_track - 400;
    int offsetY = offset_y_track - 400;

    cout << "\n\n========== 当前调试参数 ==========" << endl;

    cout << "blue: "
         << "H=" << blue_h_min << "~" << blue_h_max
         << " S>=" << blue_s_min
         << " V>=" << blue_v_min << endl;

    cout << "red1: "
         << "H=" << red1_h_min << "~" << red1_h_max
         << " S>=" << red_s_min
         << " V>=" << red_v_min << endl;

    cout << "red2: "
         << "H=" << red2_h_min << "~" << red2_h_max
         << " S>=" << red_s_min
         << " V>=" << red_v_min << endl;

    cout << "thresholdRatio = " << threshold_percent / 100.0 << endl;
    cout << "gridSidePercent = " << grid_percent << endl;
    cout << "corePercent = " << core_percent << endl;
    cout << "offsetX = " << offsetX << endl;
    cout << "offsetY = " << offsetY << endl;

    cout << "\n可复制回 getcubeinfo_d455.cpp 的 HSV 参数：" << endl;

    cout << "cv::Scalar(" << blue_h_min << ", " << blue_s_min << ", " << blue_v_min << "), "
         << "cv::Scalar(" << blue_h_max << ", 255, 255)" << endl;

    cout << "cv::Scalar(" << red1_h_min << ", " << red_s_min << ", " << red_v_min << "), "
         << "cv::Scalar(" << red1_h_max << ", 255, 255)" << endl;

    cout << "cv::Scalar(" << red2_h_min << ", " << red_s_min << ", " << red_v_min << "), "
         << "cv::Scalar(" << red2_h_max << ", 255, 255)" << endl;

    cout << "float thresholdRatio = " << threshold_percent / 100.0 << "f;" << endl;

    cout << "grid_percent = " << grid_percent << endl;
    cout << "core_scale = " << core_percent / 100.0 << endl;
    cout << "offset_x = " << offsetX << endl;
    cout << "offset_y = " << offsetY << endl;

    cout << "==================================\n" << endl;
}

int main(int argc, char** argv)
{
    // 用法：
    // ./d455_hsv_debug /dev/video6
    // ./d455_hsv_debug /dev/video6 640 480 30
    // ./d455_hsv_debug 6 640 480 30

    string cameraArg = "/dev/video6";
    int cameraWidth = 640;
    int cameraHeight = 480;
    int cameraFps = 30;

    if (argc >= 2) {
        cameraArg = argv[1];
    }

    if (argc >= 3) {
        cameraWidth = atoi(argv[2]);
    }

    if (argc >= 4) {
        cameraHeight = atoi(argv[3]);
    }

    if (argc >= 5) {
        cameraFps = atoi(argv[4]);
    }

    D455Camera camera;

    if (!camera.openCamera(cameraArg, cameraWidth, cameraHeight, cameraFps)) {
        return -1;
    }

    int blue_h_min = 100;
    int blue_h_max = 130;
    int blue_s_min = 120;
    int blue_v_min = 70;

    int red1_h_min = 0;
    int red1_h_max = 10;
    int red2_h_min = 170;
    int red2_h_max = 180;
    int red_s_min = 120;
    int red_v_min = 70;

    int threshold_percent = 18;
    int grid_percent = 75;
    int core_percent = 65;

    // 0~800，对应 -400~400
    int offset_x_track = 400;
    int offset_y_track = 400;

    cv::namedWindow("HSV Trackbars", cv::WINDOW_NORMAL);

    cv::createTrackbar("blue_h_min", "HSV Trackbars", &blue_h_min, 180);
    cv::createTrackbar("blue_h_max", "HSV Trackbars", &blue_h_max, 180);
    cv::createTrackbar("blue_s_min", "HSV Trackbars", &blue_s_min, 255);
    cv::createTrackbar("blue_v_min", "HSV Trackbars", &blue_v_min, 255);

    cv::createTrackbar("red1_h_min", "HSV Trackbars", &red1_h_min, 180);
    cv::createTrackbar("red1_h_max", "HSV Trackbars", &red1_h_max, 180);
    cv::createTrackbar("red2_h_min", "HSV Trackbars", &red2_h_min, 180);
    cv::createTrackbar("red2_h_max", "HSV Trackbars", &red2_h_max, 180);
    cv::createTrackbar("red_s_min", "HSV Trackbars", &red_s_min, 255);
    cv::createTrackbar("red_v_min", "HSV Trackbars", &red_v_min, 255);

    cv::createTrackbar("threshold_%", "HSV Trackbars", &threshold_percent, 50);
    cv::createTrackbar("grid_%", "HSV Trackbars", &grid_percent, 100);
    cv::createTrackbar("core_%", "HSV Trackbars", &core_percent, 100);
    cv::createTrackbar("offset_x", "HSV Trackbars", &offset_x_track, 800);
    cv::createTrackbar("offset_y", "HSV Trackbars", &offset_y_track, 800);

    cout << "D455 HSV 调试程序启动" << endl;
    cout << "按 q 或 ESC 退出" << endl;
    cout << "按 s 打印当前参数" << endl;
    cout << "Blue Mask / Red Mask 中白色区域就是被识别成对应颜色的区域" << endl;

    cv::Mat frame;

    while (true) {
        if (!camera.getFrame(frame) || frame.empty()) {
            cerr << "Warning: empty D455 frame" << endl;
            cv::waitKey(10);
            continue;
        }

        blue_h_min = min(blue_h_min, blue_h_max);
        red1_h_min = min(red1_h_min, red1_h_max);
        red2_h_min = min(red2_h_min, red2_h_max);

        int offsetX = offset_x_track - 400;
        int offsetY = offset_y_track - 400;

        double thresholdRatio = max(1, threshold_percent) / 100.0;
        double coreScale = max(10, core_percent) / 100.0;

        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);

        cv::Mat hsv;
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        cv::Mat blueMask;
        cv::Mat redMask1;
        cv::Mat redMask2;
        cv::Mat redMask;

        cv::inRange(
            hsv,
            cv::Scalar(blue_h_min, blue_s_min, blue_v_min),
            cv::Scalar(blue_h_max, 255, 255),
            blueMask
        );

        cv::inRange(
            hsv,
            cv::Scalar(red1_h_min, red_s_min, red_v_min),
            cv::Scalar(red1_h_max, 255, 255),
            redMask1
        );

        cv::inRange(
            hsv,
            cv::Scalar(red2_h_min, red_s_min, red_v_min),
            cv::Scalar(red2_h_max, 255, 255),
            redMask2
        );

        cv::bitwise_or(redMask1, redMask2, redMask);

        cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

        cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, kernelOpen);
        cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, kernelClose);

        cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernelOpen);
        cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernelClose);

        vector<cv::Rect> grids = buildGrid(
            frame.cols,
            frame.rows,
            grid_percent,
            offsetX,
            offsetY
        );

        cv::Mat display = frame.clone();

        for (int i = 0; i < 9; ++i) {
            int bluePixels = 0;
            int redPixels = 0;
            int blueLargest = 0;
            int redLargest = 0;
            int threshold = 0;

            BlockColor color = judgeOneCell(
                blueMask,
                redMask,
                grids[i],
                frame.size(),
                coreScale,
                thresholdRatio,
                bluePixels,
                redPixels,
                blueLargest,
                redLargest,
                threshold
            );

            cv::Scalar drawColor(255, 255, 255);

            if (color == BlockColor::RED) {
                drawColor = cv::Scalar(0, 0, 255);
            } else if (color == BlockColor::BLUE) {
                drawColor = cv::Scalar(255, 0, 0);
            } else if (color == BlockColor::UNKNOWN) {
                drawColor = cv::Scalar(0, 255, 255);
            }

            cv::Rect validRect = grids[i] & cv::Rect(0, 0, frame.cols, frame.rows);
            cv::Rect coreRect = shrinkRect(validRect, coreScale);
            coreRect = coreRect & cv::Rect(0, 0, frame.cols, frame.rows);

            cv::rectangle(display, validRect, drawColor, 2);
            cv::rectangle(display, coreRect, cv::Scalar(0, 255, 255), 1);

            string label = to_string(i) + ":" + colorToString(color);
            cv::putText(
                display,
                label,
                cv::Point(validRect.x + 5, validRect.y + 28),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                drawColor,
                2
            );

            string info =
                "B" + to_string(blueLargest) +
                " R" + to_string(redLargest) +
                " T" + to_string(threshold);

            cv::putText(
                display,
                info,
                cv::Point(validRect.x + 5, validRect.y + 55),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                cv::Scalar(0, 255, 255),
                1
            );
        }

        string topText =
            "s: print params | q/ESC: quit | threshold=" +
            to_string(threshold_percent) +
            "% grid=" + to_string(grid_percent) +
            "% core=" + to_string(core_percent) +
            "% cam=" + cameraArg;

        cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols, 42), cv::Scalar(0, 0, 0), -1);
        cv::putText(
            display,
            topText,
            cv::Point(10, 28),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(0, 255, 0),
            2
        );

        cv::imshow("D455 HSV Grid Debug", display);
        cv::imshow("Blue Mask", blueMask);
        cv::imshow("Red Mask", redMask);

        char key = static_cast<char>(cv::waitKey(1));

        if (key == 'q' || key == 27) {
            break;
        }

        if (key == 's') {
            printCurrentParams(
                blue_h_min,
                blue_h_max,
                blue_s_min,
                blue_v_min,
                red1_h_min,
                red1_h_max,
                red2_h_min,
                red2_h_max,
                red_s_min,
                red_v_min,
                threshold_percent,
                grid_percent,
                core_percent,
                offset_x_track,
                offset_y_track
            );
        }
    }

    camera.closeCamera();
    cv::destroyAllWindows();

    return 0;
}
