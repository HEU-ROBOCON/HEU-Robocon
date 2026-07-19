#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

using namespace std;

// ============================================================
// 1. 基础状态定义
// ============================================================

enum class BlockColor {
    EMPTY,
    RED,
    BLUE,
    UNKNOWN
};

constexpr int EMPTY = 0;
constexpr int OWN = 1;
constexpr int OPP = 2;
constexpr int UNKNOWN = 3;

constexpr int BOTTOM = 0;
constexpr int MIDDLE = 1;
constexpr int TOP = 2;

struct Cell {
    int r;
    int c;
};

bool operator==(const Cell& a, const Cell& b) {
    return a.r == b.r && a.c == b.c;
}

using Board = array<array<int, 3>, 3>;

// ============================================================
// 2. 参数配置
// ============================================================

bool g_own_is_blue = true;
constexpr bool IMAGE_TOP_ROW_IS_TOP_LAYER = true;
constexpr int STABLE_FRAMES_REQUIRED = 3;
constexpr int SERIAL_SEND_INTERVAL_MS = 200;
constexpr int SERIAL_RESEND_INTERVAL_MS = 1000;

struct VisionParams {
    int blue_h_min = 100;
    int blue_h_max = 130;
    int blue_s_min = 110;
    int blue_v_min = 70;
    int blue_v_max = 255;

    int red1_h_min = 0;
    int red1_h_max = 10;
    int red2_h_min = 170;
    int red2_h_max = 180;
    int red_s_min = 120;
    int red_v_min = 70;
    int red_v_max = 255;

    int grid_percent = 75;
    double core_scale = 0.65;
    double threshold_ratio = 0.18;

    int offset_x = 0;
    int offset_y = 0;
};

struct CellDebugInfo {
    cv::Rect valid_rect;
    cv::Rect core_rect;
    int blue_pixels = 0;
    int red_pixels = 0;
    int blue_largest = 0;
    int red_largest = 0;
    int threshold = 0;
};

// ============================================================
// 3. 胜利线定义
// ============================================================

const vector<array<Cell, 3>> LINES = {
    array<Cell, 3>{Cell{BOTTOM, 0}, Cell{MIDDLE, 0}, Cell{TOP, 0}},
    array<Cell, 3>{Cell{BOTTOM, 1}, Cell{MIDDLE, 1}, Cell{TOP, 1}},
    array<Cell, 3>{Cell{BOTTOM, 2}, Cell{MIDDLE, 2}, Cell{TOP, 2}},
    array<Cell, 3>{Cell{BOTTOM, 0}, Cell{MIDDLE, 1}, Cell{TOP, 2}},
    array<Cell, 3>{Cell{BOTTOM, 2}, Cell{MIDDLE, 1}, Cell{TOP, 0}},
};

// ============================================================
// 4. 字符串工具
// ============================================================

string colorToString(BlockColor color) {
    switch (color) {
        case BlockColor::RED: return "RED";
        case BlockColor::BLUE: return "BLUE";
        case BlockColor::UNKNOWN: return "UNKNOWN";
        default: return "EMPTY";
    }
}

string stateToString(int state) {
    switch (state) {
        case OWN: return "OWN";
        case OPP: return "OPP";
        case UNKNOWN: return "UNKNOWN";
        default: return "EMPTY";
    }
}

string cellName(Cell cell) {
    if (cell.r == BOTTOM) return "B" + to_string(cell.c);
    if (cell.r == MIDDLE) return "M" + to_string(cell.c);
    if (cell.r == TOP) return "T" + to_string(cell.c);
    return "NONE";
}

// ============================================================
// 5. 串口类
// ============================================================

class SerialPort {
private:
    int fd_ = -1;

    speed_t baudToSpeed(int baud) {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default:
                cerr << "Unsupported baud " << baud << ", use 115200 instead." << endl;
                return B115200;
        }
    }

public:
    ~SerialPort() { closePort(); }

    bool openPort(const string& device, int baud) {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            cerr << "Serial open failed: " << device << " : " << strerror(errno) << endl;
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            cerr << "tcgetattr failed: " << strerror(errno) << endl;
            close(fd_);
            fd_ = -1;
            return false;
        }

        cfmakeraw(&tty);
        speed_t speed = baudToSpeed(baud);
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            cerr << "tcsetattr failed: " << strerror(errno) << endl;
            close(fd_);
            fd_ = -1;
            return false;
        }

        cout << "Serial opened: " << device << " @ " << baud << endl;
        return true;
    }

    bool writeLine(const string& line) {
        if (fd_ < 0) return false;
        ssize_t n = write(fd_, line.c_str(), line.size());
        tcdrain(fd_);
        return n == static_cast<ssize_t>(line.size());
    }

    void closePort() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }
};

// ============================================================
// 6. Azure Kinect 摄像头类
// ============================================================

class AzureKinectCamera {
private:
    k4a_device_t device_ = nullptr;
    bool started_ = false;

public:
    ~AzureKinectCamera() { closeCamera(); }

    bool openCamera() {
        uint32_t deviceCount = k4a_device_get_installed_count();
        if (deviceCount == 0) {
            cerr << "Error: 没有检测到 Azure Kinect." << endl;
            return false;
        }

        cout << "Azure Kinect device count: " << deviceCount << endl;

        if (k4a_device_open(0, &device_) != K4A_RESULT_SUCCEEDED) {
            cerr << "Error: 无法打开 Azure Kinect 0." << endl;
            device_ = nullptr;
            return false;
        }

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        config.depth_mode = K4A_DEPTH_MODE_OFF;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.synchronized_images_only = false;
        config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        config.subordinate_delay_off_master_usec = 0;
        config.depth_delay_off_color_usec = 0;

        if (k4a_device_start_cameras(device_, &config) != K4A_RESULT_SUCCEEDED) {
            cerr << "Error: Azure Kinect 启动相机失败." << endl;
            k4a_device_close(device_);
            device_ = nullptr;
            return false;
        }

        started_ = true;
        cout << "Azure Kinect opened: color BGRA32 720P 15fps" << endl;

        cv::Mat testFrame;
        for (int i = 0; i < 30; ++i) {
            if (getFrame(testFrame) && !testFrame.empty()) {
                cout << "First frame received: " << testFrame.cols << "x" << testFrame.rows << endl;
                return true;
            }
            cv::waitKey(30);
        }

        cerr << "Error: Azure Kinect 已打开，但没有收到彩色帧." << endl;
        return false;
    }

    bool getFrame(cv::Mat& frame_bgr) {
        frame_bgr.release();
        if (device_ == nullptr || !started_) return false;

        k4a_capture_t capture = nullptr;
        k4a_wait_result_t result = k4a_device_get_capture(device_, &capture, 1000);

        if (result == K4A_WAIT_RESULT_TIMEOUT) {
            cerr << "Warning: Azure Kinect 获取图像超时." << endl;
            return false;
        }

        if (result == K4A_WAIT_RESULT_FAILED) {
            cerr << "Warning: Azure Kinect 获取图像失败." << endl;
            return false;
        }

        k4a_image_t colorImage = k4a_capture_get_color_image(capture);
        if (colorImage == nullptr) {
            cerr << "Warning: 没有获取到彩色图像." << endl;
            k4a_capture_release(capture);
            return false;
        }

        int width = k4a_image_get_width_pixels(colorImage);
        int height = k4a_image_get_height_pixels(colorImage);
        uint8_t* buffer = k4a_image_get_buffer(colorImage);

        cv::Mat bgra(height, width, CV_8UC4, buffer);
        cv::cvtColor(bgra, frame_bgr, cv::COLOR_BGRA2BGR);

        k4a_image_release(colorImage);
        k4a_capture_release(capture);
        return !frame_bgr.empty();
    }

    void closeCamera() {
        if (device_ != nullptr) {
            if (started_) {
                k4a_device_stop_cameras(device_);
                started_ = false;
            }
            k4a_device_close(device_);
            device_ = nullptr;
        }
    }
};

// ============================================================
// 7. Debug 版视觉识别核心
// ============================================================

cv::Rect shrinkRect(const cv::Rect& rect, double scale) {
    int newW = static_cast<int>(rect.width * scale);
    int newH = static_cast<int>(rect.height * scale);
    int newX = rect.x + (rect.width - newW) / 2;
    int newY = rect.y + (rect.height - newH) / 2;
    return cv::Rect(newX, newY, newW, newH);
}

int largestComponentArea(const cv::Mat& binaryMask) {
    if (binaryMask.empty()) return 0;

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;

    int numLabels = cv::connectedComponentsWithStats(
        binaryMask, labels, stats, centroids, 8, CV_32S
    );

    int maxArea = 0;
    for (int i = 1; i < numLabels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > maxArea) maxArea = area;
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

    int gridSide = static_cast<int>(
        min(frameWidth, frameHeight) * (gridSidePercent / 100.0)
    );
    int cellSize = gridSide / 3;

    int startX = (frameWidth - cellSize * 3) / 2 + offsetX;
    int startY = (frameHeight - cellSize * 3) / 2 + offsetY;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            grids.emplace_back(startX + c * cellSize, startY + r * cellSize, cellSize, cellSize);
        }
    }

    return grids;
}

void buildMasks(
    const cv::Mat& frame,
    const VisionParams& params,
    cv::Mat& blueMask,
    cv::Mat& redMask
) {
    cv::Mat blurred;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);

    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::Mat redMask1;
    cv::Mat redMask2;

    cv::inRange(
        hsv,
        cv::Scalar(params.blue_h_min, params.blue_s_min, params.blue_v_min),
        cv::Scalar(params.blue_h_max, 255, params.blue_v_max),
        blueMask
    );

    cv::inRange(
        hsv,
        cv::Scalar(params.red1_h_min, params.red_s_min, params.red_v_min),
        cv::Scalar(params.red1_h_max, 255, params.red_v_max),
        redMask1
    );

    cv::inRange(
        hsv,
        cv::Scalar(params.red2_h_min, params.red_s_min, params.red_v_min),
        cv::Scalar(params.red2_h_max, 255, params.red_v_max),
        redMask2
    );

    cv::bitwise_or(redMask1, redMask2, redMask);

    cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

    cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, kernelOpen);
    cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, kernelClose);

    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernelOpen);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernelClose);
}

BlockColor judgeOneCell(
    const cv::Mat& blueMask,
    const cv::Mat& redMask,
    const cv::Rect& cellRect,
    const cv::Size& frameSize,
    double coreScale,
    double thresholdRatio,
    CellDebugInfo& info
) {
    cv::Rect validRect = cellRect & cv::Rect(0, 0, frameSize.width, frameSize.height);
    info.valid_rect = validRect;

    if (validRect.area() <= 0) return BlockColor::UNKNOWN;

    cv::Rect coreRect = shrinkRect(validRect, coreScale);
    coreRect = coreRect & cv::Rect(0, 0, frameSize.width, frameSize.height);
    info.core_rect = coreRect;

    if (coreRect.area() <= 0) return BlockColor::UNKNOWN;

    cv::Mat blueROI = blueMask(coreRect);
    cv::Mat redROI = redMask(coreRect);

    info.blue_pixels = cv::countNonZero(blueROI);
    info.red_pixels = cv::countNonZero(redROI);
    info.blue_largest = largestComponentArea(blueROI);
    info.red_largest = largestComponentArea(redROI);
    info.threshold = static_cast<int>(coreRect.area() * thresholdRatio);

    bool blueValid = info.blue_largest > info.threshold;
    bool redValid = info.red_largest > info.threshold;

    if (blueValid && redValid) {
        int maxArea = max(info.blue_largest, info.red_largest);
        int minArea = min(info.blue_largest, info.red_largest);

        if (minArea > 0 && maxArea < minArea * 1.5) {
            return BlockColor::UNKNOWN;
        }

        return info.blue_largest > info.red_largest ? BlockColor::BLUE : BlockColor::RED;
    }

    if (blueValid) return BlockColor::BLUE;
    if (redValid) return BlockColor::RED;
    return BlockColor::EMPTY;
}

vector<BlockColor> getGridColorsDebugBased(
    const cv::Mat& frame,
    const VisionParams& params,
    vector<cv::Rect>& grids,
    vector<CellDebugInfo>& debugInfos,
    cv::Mat& blueMask,
    cv::Mat& redMask
) {
    vector<BlockColor> colors(9, BlockColor::EMPTY);
    debugInfos.assign(9, CellDebugInfo{});

    if (frame.empty()) return colors;

    buildMasks(frame, params, blueMask, redMask);

    grids = buildGrid(frame.cols, frame.rows, params.grid_percent, params.offset_x, params.offset_y);

    for (int i = 0; i < 9; ++i) {
        colors[i] = judgeOneCell(
            blueMask,
            redMask,
            grids[i],
            frame.size(),
            params.core_scale,
            params.threshold_ratio,
            debugInfos[i]
        );
    }

    return colors;
}

// ============================================================
// 8. 视觉格子 -> 棋盘状态
// ============================================================

Cell visualIndexToBoardCell(int index) {
    int visualRow = index / 3;
    int col = index % 3;

    int layer;
    if constexpr (IMAGE_TOP_ROW_IS_TOP_LAYER) {
        layer = TOP - visualRow;
    } else {
        layer = visualRow;
    }

    return Cell{layer, col};
}

int blockColorToBoardState(BlockColor color) {
    if (color == BlockColor::EMPTY) return EMPTY;
    if (color == BlockColor::UNKNOWN) return UNKNOWN;

    if (g_own_is_blue) {
        if (color == BlockColor::BLUE) return OWN;
        if (color == BlockColor::RED) return OPP;
    } else {
        if (color == BlockColor::RED) return OWN;
        if (color == BlockColor::BLUE) return OPP;
    }

    return UNKNOWN;
}

Board colorsToBoard(const vector<BlockColor>& colors) {
    Board board{};

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            board[r][c] = EMPTY;
        }
    }

    for (int i = 0; i < 9 && i < static_cast<int>(colors.size()); ++i) {
        Cell cell = visualIndexToBoardCell(i);
        board[cell.r][cell.c] = blockColorToBoardState(colors[i]);
    }

    return board;
}

// ============================================================
// 9. 决策结构与决策函数
// ============================================================

struct Decision {
    string robot = "R2";
    string action = "WAIT";
    string target = "NONE";
    Cell target_rc{-1, -1};
    string reason = "";
    int score = 0;
};

bool boardValid(const Board& board) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            int v = board[r][c];
            if (v != EMPTY && v != OWN && v != OPP && v != UNKNOWN) return false;
        }
    }
    return true;
}

bool hasUnknown(const Board& board) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (board[r][c] == UNKNOWN) return true;
        }
    }
    return false;
}

bool isEmpty(const Board& board, Cell cell) {
    return board[cell.r][cell.c] == EMPTY;
}

bool checkWin(const Board& board, int side) {
    for (const auto& line : LINES) {
        bool ok = true;
        for (const auto& cell : line) {
            if (board[cell.r][cell.c] != side) {
                ok = false;
                break;
            }
        }
        if (ok) return true;
    }
    return false;
}

vector<Cell> findImmediateWinCells(const Board& board, int side) {
    vector<Cell> result;

    for (const auto& line : LINES) {
        int sideCount = 0;
        vector<Cell> emptyCells;
        bool blocked = false;

        for (const auto& cell : line) {
            int value = board[cell.r][cell.c];
            if (value == side) sideCount++;
            else if (value == EMPTY) emptyCells.push_back(cell);
            else blocked = true;
        }

        if (sideCount == 2 && emptyCells.size() == 1 && !blocked) {
            Cell target = emptyCells[0];
            bool exists = false;
            for (const auto& old : result) {
                if (old == target) {
                    exists = true;
                    break;
                }
            }
            if (!exists) result.push_back(target);
        }
    }

    return result;
}

vector<Cell> getThreatEmptyCells(const Board& board, int side) {
    vector<Cell> threats;

    for (const auto& line : LINES) {
        int sideCount = 0;
        vector<Cell> emptyCells;
        bool blocked = false;

        for (const auto& cell : line) {
            int value = board[cell.r][cell.c];
            if (value == side) sideCount++;
            else if (value == EMPTY) emptyCells.push_back(cell);
            else blocked = true;
        }

        if (sideCount == 2 && emptyCells.size() == 1 && !blocked) {
            threats.push_back(emptyCells[0]);
        }
    }

    return threats;
}

int countFutureThreatsAfterPlace(Board& board, int side, Cell cell) {
    if (board[cell.r][cell.c] != EMPTY) return -999;
    board[cell.r][cell.c] = side;
    int count = static_cast<int>(getThreatEmptyCells(board, side).size());
    board[cell.r][cell.c] = EMPTY;
    return count;
}

int countLinesIncludingCell(Cell cell) {
    int count = 0;
    for (const auto& line : LINES) {
        for (const auto& x : line) {
            if (x == cell) {
                count++;
                break;
            }
        }
    }
    return count;
}

bool r2CanPlace(Cell cell) {
    return cell.r == MIDDLE;
}

vector<Cell> r2LegalMiddleCells(const Board& board) {
    vector<Cell> cells;
    for (int c = 0; c < 3; ++c) {
        Cell cell{MIDDLE, c};
        if (board[MIDDLE][c] == EMPTY) cells.push_back(cell);
    }
    return cells;
}

int middleCellBaseScore(Cell cell) {
    if (cell.r != MIDDLE) return -999;
    if (cell.c == 1) return 100;
    return 60;
}

int scoreMiddleCell(Board& board, Cell cell) {
    if (!r2CanPlace(cell)) return -999999;
    if (!isEmpty(board, cell)) return -999999;

    int score = 0;
    int ownFutureThreats = countFutureThreatsAfterPlace(board, OWN, cell);
    score += 800 * ownFutureThreats;

    int oppFutureThreats = countFutureThreatsAfterPlace(board, OPP, cell);
    score += 700 * oppFutureThreats;

    score += middleCellBaseScore(cell);
    score += 100 * countLinesIncludingCell(cell);

    for (const auto& line : LINES) {
        bool includes = false;
        for (const auto& x : line) {
            if (x == cell) {
                includes = true;
                break;
            }
        }
        if (!includes) continue;

        int ownCount = 0;
        int oppCount = 0;
        int emptyCount = 0;

        for (const auto& x : line) {
            int value = board[x.r][x.c];
            if (value == OWN) ownCount++;
            else if (value == OPP) oppCount++;
            else if (value == EMPTY) emptyCount++;
        }

        if (oppCount == 0) {
            score += 120 * ownCount;
            score += 20 * emptyCount;
        }

        if (ownCount == 0) {
            score += 100 * oppCount;
        }
    }

    return score;
}

Cell chooseBestMiddleCell(Board& board, bool& ok) {
    vector<Cell> candidates = r2LegalMiddleCells(board);
    if (candidates.empty()) {
        ok = false;
        return Cell{-1, -1};
    }

    Cell bestCell = candidates[0];
    int bestScore = -999999;

    for (const auto& cell : candidates) {
        int score = scoreMiddleCell(board, cell);
        if (score > bestScore) {
            bestScore = score;
            bestCell = cell;
        }
    }

    ok = true;
    return bestCell;
}

Decision decideR2MiddleOnly(Board board, int r2KfsCount) {
    if (!boardValid(board)) {
        return Decision{"R2", "RESCAN", "NONE", Cell{-1, -1}, "INVALID_BOARD", 0};
    }

    if (hasUnknown(board)) {
        return Decision{"R2", "RESCAN", "NONE", Cell{-1, -1}, "BOARD_HAS_UNKNOWN", 0};
    }

    if (r2KfsCount <= 0) {
        return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "R2_NO_KFS", 0};
    }

    if (checkWin(board, OWN)) {
        return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "OWN_ALREADY_WIN", 0};
    }

    if (checkWin(board, OPP)) {
        return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "OPP_ALREADY_WIN", 0};
    }

    vector<Cell> ownWinCells = findImmediateWinCells(board, OWN);
    for (const auto& cell : ownWinCells) {
        if (r2CanPlace(cell) && isEmpty(board, cell)) {
            return Decision{"R2", "PLACE", cellName(cell), cell, "R2_MIDDLE_PLACE_TO_WIN", 100000};
        }
    }

    vector<Cell> oppWinCells = findImmediateWinCells(board, OPP);
    vector<Cell> middleBlockCells;

    for (const auto& cell : oppWinCells) {
        if (r2CanPlace(cell) && isEmpty(board, cell)) middleBlockCells.push_back(cell);
    }

    if (!middleBlockCells.empty()) {
        Cell bestBlock = middleBlockCells[0];
        int bestScore = -999999;
        for (const auto& cell : middleBlockCells) {
            int score = scoreMiddleCell(board, cell);
            if (score > bestScore) {
                bestScore = score;
                bestBlock = cell;
            }
        }
        return Decision{"R2", "PLACE", cellName(bestBlock), bestBlock, "R2_MIDDLE_BLOCK_OPP_WIN", 95000};
    }

    bool ok = false;
    Cell bestCell = chooseBestMiddleCell(board, ok);
    if (ok) {
        int score = scoreMiddleCell(board, bestCell);
        return Decision{"R2", "PLACE", cellName(bestCell), bestCell, "R2_BEST_MIDDLE_PLACE", score};
    }

    return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "NO_EMPTY_MIDDLE_CELL", 0};
}

// ============================================================
// 10. 串口协议与显示
// ============================================================

string makeSerialPacket(const Decision& decision) {
    int actionCode = 1;   // 默认 WAIT
    int targetId = 9;     // 默认无目标

    if (decision.action == "PLACE") {
        // 0 表示 PLACE
        actionCode = 0;

        if (decision.target_rc.r == MIDDLE &&
            decision.target_rc.c >= 0 &&
            decision.target_rc.c <= 2) {
            targetId = decision.target_rc.c;
        }
    } else if (decision.action == "RESCAN") {
        // 2 表示 RESCAN
        actionCode = 2;
        targetId = 9;
    } else {
        // 1 表示 WAIT
        actionCode = 1;
        targetId = 9;
    }

    // 新协议：
    // R2,0,0\n  -> PLACE M0
    // R2,0,1\n  -> PLACE M1
    // R2,0,2\n  -> PLACE M2
    // R2,1,9\n  -> WAIT
    // R2,2,9\n  -> RESCAN
    stringstream ss;
    ss << "R2," << actionCode << "," << targetId << "\n";

    return ss.str();
}

string makeDecisionKey(const Decision& decision) {
    return decision.action + ":" + decision.target + ":" + decision.reason;
}

void printBoard(const Board& board) {
    cout << " B:[" << stateToString(board[BOTTOM][0]) << ","
         << stateToString(board[BOTTOM][1]) << ","
         << stateToString(board[BOTTOM][2]) << "]";

    cout << " M:[" << stateToString(board[MIDDLE][0]) << ","
         << stateToString(board[MIDDLE][1]) << ","
         << stateToString(board[MIDDLE][2]) << "]";

    cout << " T:[" << stateToString(board[TOP][0]) << ","
         << stateToString(board[TOP][1]) << ","
         << stateToString(board[TOP][2]) << "]";
}

void drawDebugView(
    cv::Mat& display,
    const vector<cv::Rect>& grids,
    const vector<BlockColor>& colors,
    const vector<CellDebugInfo>& infos,
    const Decision& decision,
    const string& packet
) {
    for (int i = 0; i < 9; ++i) {
        Cell boardCell = visualIndexToBoardCell(i);
        cv::Scalar drawColor(255, 255, 255);

        if (colors[i] == BlockColor::RED) drawColor = cv::Scalar(0, 0, 255);
        else if (colors[i] == BlockColor::BLUE) drawColor = cv::Scalar(255, 0, 0);
        else if (colors[i] == BlockColor::UNKNOWN) drawColor = cv::Scalar(0, 255, 255);

        if (infos[i].valid_rect.area() > 0) cv::rectangle(display, infos[i].valid_rect, drawColor, 2);
        if (infos[i].core_rect.area() > 0) cv::rectangle(display, infos[i].core_rect, cv::Scalar(0, 255, 255), 1);

        string label = cellName(boardCell) + ":" + colorToString(colors[i]);
        cv::putText(display, label, cv::Point(grids[i].x + 5, grids[i].y + 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, drawColor, 2);

        string statText = "B" + to_string(infos[i].blue_largest) +
                          " R" + to_string(infos[i].red_largest) +
                          " T" + to_string(infos[i].threshold);
        cv::putText(display, statText, cv::Point(grids[i].x + 5, grids[i].y + 55),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
    }

    string decisionText = "DECISION: " + decision.action + " " + decision.target +
                          "  TX: " + packet.substr(0, packet.size() - 1) +
                          "  reason=" + decision.reason;

    cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols, 42), cv::Scalar(0, 0, 0), -1);
    cv::putText(display, decisionText, cv::Point(10, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
}

// ============================================================
// 11. 主函数
// ============================================================

int main(int argc, char** argv) {
    string cameraArg = "k4a";
    string serialDevice = "none";
    int baud = 115200;
    string ownColor = "blue";
    int r2KfsCount = 3;

    if (argc >= 2) cameraArg = argv[1];
    if (argc >= 3) serialDevice = argv[2];
    if (argc >= 4) baud = atoi(argv[3]);
    if (argc >= 5) {
        ownColor = argv[4];
        transform(ownColor.begin(), ownColor.end(), ownColor.begin(),
                  [](unsigned char ch) { return static_cast<char>(tolower(ch)); });
    }
    if (argc >= 6) r2KfsCount = atoi(argv[5]);

    g_own_is_blue = (ownColor != "red");

    VisionParams params;

    cout << "Camera    : Azure Kinect SDK" << endl;
    cout << "Camera arg: " << cameraArg << " (ignored, use k4a SDK)" << endl;
    cout << "Own color : " << (g_own_is_blue ? "BLUE" : "RED") << endl;
    cout << "R2 KFS    : " << r2KfsCount << endl;
    cout << "HSV blue  : H=" << params.blue_h_min << "~" << params.blue_h_max
         << " S>=" << params.blue_s_min
         << " V=" << params.blue_v_min << "~" << params.blue_v_max << endl;
    cout << "HSV red1  : H=" << params.red1_h_min << "~" << params.red1_h_max
         << " S>=" << params.red_s_min
         << " V=" << params.red_v_min << "~" << params.red_v_max << endl;
    cout << "HSV red2  : H=" << params.red2_h_min << "~" << params.red2_h_max
         << " S>=" << params.red_s_min
         << " V=" << params.red_v_min << "~" << params.red_v_max << endl;
    cout << "Grid      : " << params.grid_percent << "%, core="
         << params.core_scale << ", threshold=" << params.threshold_ratio << endl;

    AzureKinectCamera camera;
    if (!camera.openCamera()) return -1;

    SerialPort serial;
    bool serialEnabled = false;

    if (!serialDevice.empty() && serialDevice != "none") {
        serialEnabled = serial.openPort(serialDevice, baud);
        if (!serialEnabled) cerr << "Serial disabled, only print decision." << endl;
    } else {
        cout << "Serial disabled. Use: ./getcubeinfo k4a /dev/ttyUSB1 115200 blue" << endl;
    }

    string lastCandidateKey = "";
    int stableCount = 0;
    string lastSentPacket = "";
    auto lastSendTime = chrono::steady_clock::now() - chrono::milliseconds(5000);

    cv::Mat frame;
    cv::Mat blueMask;
    cv::Mat redMask;

    cout << "Started. Press q or ESC to exit." << endl;

    while (true) {
        if (!camera.getFrame(frame) || frame.empty()) {
            cerr << "Warning: empty Azure Kinect frame." << endl;
            cv::waitKey(10);
            continue;
        }

        vector<cv::Rect> grids;
        vector<CellDebugInfo> infos;

        vector<BlockColor> colors = getGridColorsDebugBased(frame, params, grids, infos, blueMask, redMask);
        Board board = colorsToBoard(colors);
        Decision decision = decideR2MiddleOnly(board, r2KfsCount);

        string decisionKey = makeDecisionKey(decision);
        string packet = makeSerialPacket(decision);

        if (decisionKey == lastCandidateKey) {
            stableCount++;
        } else {
            lastCandidateKey = decisionKey;
            stableCount = 1;
        }

        auto now = chrono::steady_clock::now();
        auto sinceLastSend = chrono::duration_cast<chrono::milliseconds>(now - lastSendTime).count();
        bool shouldSend = false;

        if (stableCount >= STABLE_FRAMES_REQUIRED) {
            if (packet != lastSentPacket && sinceLastSend >= SERIAL_SEND_INTERVAL_MS) shouldSend = true;
            if (packet == lastSentPacket && sinceLastSend >= SERIAL_RESEND_INTERVAL_MS) shouldSend = true;
        }

        if (shouldSend) {
            if (serialEnabled) {
                bool ok = serial.writeLine(packet);
                if (!ok) cerr << "\nSerial write failed." << endl;
            }
            lastSentPacket = packet;
            lastSendTime = now;
        }

        cout << "\r";
        printBoard(board);
        cout << " | DECISION: " << decision.action
             << " " << decision.target
             << " | reason=" << decision.reason
             << " | score=" << decision.score
             << " | tx=" << packet.substr(0, packet.size() - 1)
             << " | stable=" << stableCount
             << "      " << flush;

        cv::Mat display = frame.clone();
        drawDebugView(display, grids, colors, infos, decision, packet);

        cv::imshow("R2 Debug-Based Decision - Azure Kinect", display);
        cv::imshow("Blue Mask", blueMask);
        cv::imshow("Red Mask", redMask);

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 27) break;
    }

    cout << endl;
    camera.closeCamera();
    serial.closePort();
    cv::destroyAllWindows();
    return 0;
}
