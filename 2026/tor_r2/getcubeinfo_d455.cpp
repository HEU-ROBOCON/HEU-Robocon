#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
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

enum class BlockColor { EMPTY, RED, BLUE, UNKNOWN };

constexpr int EMPTY = 0;
constexpr int OWN = 1;
constexpr int OPP = 2;
constexpr int UNKNOWN = 3;
constexpr int BOTTOM = 0;
constexpr int MIDDLE = 1;
constexpr int TOP = 2;

struct Cell { int r; int c; };
bool operator==(const Cell& a, const Cell& b) { return a.r == b.r && a.c == b.c; }
using Board = array<array<int, 3>, 3>;

bool g_own_is_blue = true;
constexpr bool IMAGE_TOP_ROW_IS_TOP_LAYER = true;
constexpr int STABLE_FRAMES_REQUIRED = 3;
constexpr int SERIAL_SEND_INTERVAL_MS = 200;
constexpr int SERIAL_RESEND_INTERVAL_MS = 1000;

constexpr int R2_TOTAL_BLOCKS = 2;
constexpr int R2_TOP_RESERVED_BLOCKS = 1;
constexpr int R2_MIDDLE_TARGET_BLOCKS = R2_TOTAL_BLOCKS - R2_TOP_RESERVED_BLOCKS;
static_assert(R2_MIDDLE_TARGET_BLOCKS == 1,
              "R2 must reserve one of its two blocks for the top layer");

constexpr int DEFAULT_CAMERA_WIDTH = 640;
constexpr int DEFAULT_CAMERA_HEIGHT = 480;
constexpr int DEFAULT_CAMERA_FPS = 30;

struct VisionParams {
    int blue_h_min = 100;
    int blue_h_max = 130;
    int blue_s_min = 90;
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

const vector<array<Cell, 3>> LINES = {
    array<Cell, 3>{Cell{BOTTOM, 0}, Cell{MIDDLE, 0}, Cell{TOP, 0}},
    array<Cell, 3>{Cell{BOTTOM, 1}, Cell{MIDDLE, 1}, Cell{TOP, 1}},
    array<Cell, 3>{Cell{BOTTOM, 2}, Cell{MIDDLE, 2}, Cell{TOP, 2}},
    array<Cell, 3>{Cell{BOTTOM, 0}, Cell{MIDDLE, 1}, Cell{TOP, 2}},
    array<Cell, 3>{Cell{BOTTOM, 2}, Cell{MIDDLE, 1}, Cell{TOP, 0}},
};

struct Decision {
    string robot = "R2";
    string action = "WAIT";
    string target = "NONE";
    Cell target_rc{-1, -1};
    string reason = "";
    int score = 0;
};

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

class D455Camera {
private:
    cv::VideoCapture cap_;
    string camera_arg_;
    int width_ = DEFAULT_CAMERA_WIDTH;
    int height_ = DEFAULT_CAMERA_HEIGHT;
    int fps_ = DEFAULT_CAMERA_FPS;

    bool isNumber(const string& s) {
        if (s.empty()) return false;
        for (char ch : s) {
            if (!isdigit(static_cast<unsigned char>(ch))) return false;
        }
        return true;
    }

    bool tryOpenWithFourcc(const string& fourcc) {
        if (cap_.isOpened()) cap_.release();

        bool opened = false;
        if (isNumber(camera_arg_)) {
            opened = cap_.open(atoi(camera_arg_.c_str()), cv::CAP_V4L2);
        } else {
            opened = cap_.open(camera_arg_, cv::CAP_V4L2);
        }

        if (!opened || !cap_.isOpened()) return false;

        if (fourcc.size() == 4) {
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_CONVERT_RGB, 1);

        cv::Mat test;
        for (int i = 0; i < 20; ++i) {
            if (cap_.read(test) && !test.empty()) return true;
            cv::waitKey(30);
        }

        cap_.release();
        return false;
    }

public:
    bool openCamera(const string& cameraArg, int width, int height, int fps) {
        camera_arg_ = cameraArg;
        width_ = width;
        height_ = height;
        fps_ = fps;

        cout << "D455 camera arg: " << camera_arg_ << endl;
        cout << "Requested size : " << width_ << "x" << height_ << " @" << fps_ << "fps" << endl;

        if (tryOpenWithFourcc("YUYV")) {
            cout << "D455 opened with YUYV." << endl;
        } else if (tryOpenWithFourcc("MJPG")) {
            cout << "D455 opened with MJPG." << endl;
        } else if (tryOpenWithFourcc("")) {
            cout << "D455 opened without fixed FOURCC." << endl;
        } else {
            cerr << "Error: 无法打开 D455 彩色相机: " << camera_arg_ << endl;
            cerr << "请用 v4l2-ctl --list-devices 确认彩色通道，比如 /dev/video6。" << endl;
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

        cout << "Actual size    : " << actualW << "x" << actualH << " @" << actualFps << "fps" << endl;
        cout << "Actual FOURCC  : " << fourccChars << endl;
        return true;
    }

    bool getFrame(cv::Mat& frame_bgr) {
        frame_bgr.release();
        if (!cap_.isOpened()) return false;

        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) return false;

        if (frame.channels() == 3) frame_bgr = frame;
        else if (frame.channels() == 4) cv::cvtColor(frame, frame_bgr, cv::COLOR_BGRA2BGR);
        else if (frame.channels() == 1) cv::cvtColor(frame, frame_bgr, cv::COLOR_GRAY2BGR);
        else return false;

        return !frame_bgr.empty();
    }

    void closeCamera() {
        if (cap_.isOpened()) cap_.release();
    }
};

cv::Rect shrinkRect(const cv::Rect& rect, double scale) {
    int newW = static_cast<int>(rect.width * scale);
    int newH = static_cast<int>(rect.height * scale);
    int newX = rect.x + (rect.width - newW) / 2;
    int newY = rect.y + (rect.height - newH) / 2;
    return cv::Rect(newX, newY, newW, newH);
}

int largestComponentArea(const cv::Mat& binaryMask) {
    if (binaryMask.empty()) return 0;
    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(binaryMask, labels, stats, centroids, 8, CV_32S);
    int maxArea = 0;
    for (int i = 1; i < numLabels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > maxArea) maxArea = area;
    }
    return maxArea;
}

vector<cv::Rect> buildGrid(int frameWidth, int frameHeight, int gridSidePercent, int offsetX, int offsetY) {
    vector<cv::Rect> grids;
    gridSidePercent = max(10, min(100, gridSidePercent));
    int gridSide = static_cast<int>(min(frameWidth, frameHeight) * (gridSidePercent / 100.0));
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

void buildMasks(const cv::Mat& frame, const VisionParams& params, cv::Mat& blueMask, cv::Mat& redMask) {
    cv::Mat blurred, hsv;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::Mat redMask1, redMask2;
    cv::inRange(hsv, cv::Scalar(params.blue_h_min, params.blue_s_min, params.blue_v_min),
                cv::Scalar(params.blue_h_max, 255, params.blue_v_max), blueMask);
    cv::inRange(hsv, cv::Scalar(params.red1_h_min, params.red_s_min, params.red_v_min),
                cv::Scalar(params.red1_h_max, 255, params.red_v_max), redMask1);
    cv::inRange(hsv, cv::Scalar(params.red2_h_min, params.red_s_min, params.red_v_min),
                cv::Scalar(params.red2_h_max, 255, params.red_v_max), redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);

    cv::Mat kernelOpen = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, kernelOpen);
    cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, kernelClose);
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernelOpen);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernelClose);
}

BlockColor judgeOneCell(const cv::Mat& blueMask, const cv::Mat& redMask, const cv::Rect& cellRect,
                        const cv::Size& frameSize, double coreScale, double thresholdRatio, CellDebugInfo& info) {
    cv::Rect validRect = cellRect & cv::Rect(0, 0, frameSize.width, frameSize.height);
    info.valid_rect = validRect;
    if (validRect.area() <= 0) return BlockColor::UNKNOWN;

    cv::Rect coreRect = shrinkRect(validRect, coreScale) & cv::Rect(0, 0, frameSize.width, frameSize.height);
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
        if (minArea > 0 && maxArea < minArea * 1.5) return BlockColor::UNKNOWN;
        return info.blue_largest > info.red_largest ? BlockColor::BLUE : BlockColor::RED;
    }
    if (blueValid) return BlockColor::BLUE;
    if (redValid) return BlockColor::RED;
    return BlockColor::EMPTY;
}

vector<BlockColor> getGridColors(const cv::Mat& frame, const VisionParams& params, vector<cv::Rect>& grids,
                                 vector<CellDebugInfo>& debugInfos, cv::Mat& blueMask, cv::Mat& redMask) {
    vector<BlockColor> colors(9, BlockColor::EMPTY);
    debugInfos.assign(9, CellDebugInfo{});
    if (frame.empty()) return colors;

    buildMasks(frame, params, blueMask, redMask);
    grids = buildGrid(frame.cols, frame.rows, params.grid_percent, params.offset_x, params.offset_y);

    for (int i = 0; i < 9; ++i) {
        colors[i] = judgeOneCell(blueMask, redMask, grids[i], frame.size(),
                                 params.core_scale, params.threshold_ratio, debugInfos[i]);
    }
    return colors;
}

Cell visualIndexToBoardCell(int index) {
    int visualRow = index / 3;
    int col = index % 3;
    int layer;
    if constexpr (IMAGE_TOP_ROW_IS_TOP_LAYER) layer = TOP - visualRow;
    else layer = visualRow;
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
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) board[r][c] = EMPTY;
    for (int i = 0; i < 9 && i < static_cast<int>(colors.size()); ++i) {
        Cell cell = visualIndexToBoardCell(i);
        board[cell.r][cell.c] = blockColorToBoardState(colors[i]);
    }
    return board;
}

bool boardValid(const Board& board) {
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
        int v = board[r][c];
        if (v != EMPTY && v != OWN && v != OPP && v != UNKNOWN) return false;
    }
    return true;
}

bool hasUnknown(const Board& board) {
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) if (board[r][c] == UNKNOWN) return true;
    return false;
}

bool isEmpty(const Board& board, Cell cell) { return board[cell.r][cell.c] == EMPTY; }

bool checkWin(const Board& board, int side) {
    for (const auto& line : LINES) {
        bool ok = true;
        for (const auto& cell : line) {
            if (board[cell.r][cell.c] != side) { ok = false; break; }
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
            for (const auto& old : result) if (old == target) { exists = true; break; }
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
        if (sideCount == 2 && emptyCells.size() == 1 && !blocked) threats.push_back(emptyCells[0]);
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
            if (x == cell) { count++; break; }
        }
    }
    return count;
}

bool r2CanPlace(Cell cell) { return cell.r == MIDDLE; }

vector<Cell> r2LegalMiddleCells(const Board& board) {
    vector<Cell> cells;
    for (int c = 0; c < 3; ++c) {
        Cell cell{MIDDLE, c};
        if (board[MIDDLE][c] == EMPTY) cells.push_back(cell);
    }
    return cells;
}

int countOwnBlocksInLayer(const Board& board, int layer) {
    int count = 0;
    if (layer < 0 || layer >= 3) return 0;
    for (int c = 0; c < 3; ++c) if (board[layer][c] == OWN) count++;
    return count;
}

int getR2MiddleRemainingPlaces(const Board& board, int r2KfsCount) {
    int ownMiddleCount = countOwnBlocksInLayer(board, MIDDLE);
    int remainingByBoard = R2_MIDDLE_TARGET_BLOCKS - ownMiddleCount;
    if (remainingByBoard < 0) remainingByBoard = 0;
    return min(remainingByBoard, max(0, r2KfsCount));
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
    score += 800 * countFutureThreatsAfterPlace(board, OWN, cell);
    score += 700 * countFutureThreatsAfterPlace(board, OPP, cell);
    score += middleCellBaseScore(cell);
    score += 100 * countLinesIncludingCell(cell);

    for (const auto& line : LINES) {
        bool includes = false;
        for (const auto& x : line) if (x == cell) { includes = true; break; }
        if (!includes) continue;

        int ownCount = 0, oppCount = 0, emptyCount = 0;
        for (const auto& x : line) {
            int value = board[x.r][x.c];
            if (value == OWN) ownCount++;
            else if (value == OPP) oppCount++;
            else if (value == EMPTY) emptyCount++;
        }
        if (oppCount == 0) { score += 120 * ownCount; score += 20 * emptyCount; }
        if (ownCount == 0) score += 100 * oppCount;
    }
    return score;
}

Cell chooseBestMiddleCell(Board& board, bool& ok) {
    vector<Cell> candidates = r2LegalMiddleCells(board);
    if (candidates.empty()) { ok = false; return Cell{-1, -1}; }
    Cell bestCell = candidates[0];
    int bestScore = -999999;
    for (const auto& cell : candidates) {
        int score = scoreMiddleCell(board, cell);
        if (score > bestScore) { bestScore = score; bestCell = cell; }
    }
    ok = true;
    return bestCell;
}

Decision decideR2MiddleOnly(Board board, int r2KfsCount) {
    if (!boardValid(board)) return Decision{"R2", "RESCAN", "NONE", Cell{-1, -1}, "INVALID_BOARD", 0};
    if (hasUnknown(board)) return Decision{"R2", "RESCAN", "NONE", Cell{-1, -1}, "BOARD_HAS_UNKNOWN", 0};

    int ownMiddleCount = countOwnBlocksInLayer(board, MIDDLE);
    int r2MiddleRemaining = getR2MiddleRemainingPlaces(board, r2KfsCount);

    if (ownMiddleCount >= R2_MIDDLE_TARGET_BLOCKS) {
        return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "R2_MIDDLE_TARGET_DONE_TOP_RESERVED", 0};
    }
    if (r2MiddleRemaining <= 0) {
        return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "R2_NO_MIDDLE_BLOCK_REMAINING", 0};
    }
    if (checkWin(board, OWN)) return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "OWN_ALREADY_WIN", 0};
    if (checkWin(board, OPP)) return Decision{"R2", "WAIT", "NONE", Cell{-1, -1}, "OPP_ALREADY_WIN", 0};

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
            if (score > bestScore) { bestScore = score; bestBlock = cell; }
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

string makeSerialPacket(const Decision& decision) {
    int actionCode = 1;
    int targetId = 9;

    if (decision.action == "PLACE") {
        actionCode = 0;
        if (decision.target_rc.r == MIDDLE && decision.target_rc.c >= 0 && decision.target_rc.c <= 2) {
            targetId = decision.target_rc.c;
        }
    } else if (decision.action == "RESCAN") {
        actionCode = 2;
        targetId = 9;
    } else {
        actionCode = 1;
        targetId = 9;
    }

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

void drawDebugView(cv::Mat& display, const vector<cv::Rect>& grids, const vector<BlockColor>& colors,
                   const vector<CellDebugInfo>& infos, const Decision& decision, const string& packet) {
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

int main(int argc, char** argv) {
    // D455 版本用法：
    // ./getcubeinfo_d455 /dev/video6 none 115200 blue
    // ./getcubeinfo_d455 /dev/video6 /dev/serial/by-path/xxx 115200 blue
    // ./getcubeinfo_d455 6 none 115200 blue
    // 可选参数：argv[5]=width argv[6]=height argv[7]=fps

    string cameraArg = "/dev/video6";
    string serialDevice = "none";
    int baud = 115200;
    string ownColor = "blue";
    int width = DEFAULT_CAMERA_WIDTH;
    int height = DEFAULT_CAMERA_HEIGHT;
    int fps = DEFAULT_CAMERA_FPS;

    if (argc >= 2) cameraArg = argv[1];
    if (argc >= 3) serialDevice = argv[2];
    if (argc >= 4) baud = atoi(argv[3]);
    if (argc >= 5) {
        ownColor = argv[4];
        transform(ownColor.begin(), ownColor.end(), ownColor.begin(), ::tolower);
    }
    if (argc >= 6) width = atoi(argv[5]);
    if (argc >= 7) height = atoi(argv[6]);
    if (argc >= 8) fps = atoi(argv[7]);

    g_own_is_blue = (ownColor != "red");
    VisionParams params;

    cout << "Camera    : D455 / OpenCV V4L2" << endl;
    cout << "Camera arg: " << cameraArg << endl;
    cout << "Own color : " << (g_own_is_blue ? "BLUE" : "RED") << endl;
    cout << "Serial    : " << serialDevice << " @ " << baud << endl;
    cout << "Grid      : " << params.grid_percent << "%, core=" << params.core_scale
         << ", threshold=" << params.threshold_ratio << endl;

    D455Camera camera;
    if (!camera.openCamera(cameraArg, width, height, fps)) return -1;

    SerialPort serial;
    bool serialEnabled = false;
    if (!serialDevice.empty() && serialDevice != "none") {
        serialEnabled = serial.openPort(serialDevice, baud);
        if (!serialEnabled) cerr << "Serial disabled, only print decision." << endl;
    } else {
        cout << "Serial disabled. Use: ./getcubeinfo_d455 /dev/video6 /dev/ttyUSB0 115200 blue" << endl;
    }

    int r2KfsCount = R2_MIDDLE_TARGET_BLOCKS;
    string lastCandidateKey = "";
    int stableCount = 0;
    string lastSentPacket = "";
    auto lastSendTime = chrono::steady_clock::now() - chrono::milliseconds(5000);

    cv::Mat frame, blueMask, redMask;
    cout << "Started. Press q or ESC to exit." << endl;

    while (true) {
        if (!camera.getFrame(frame) || frame.empty()) {
            cerr << "Warning: empty D455 frame." << endl;
            cv::waitKey(10);
            continue;
        }

        vector<cv::Rect> grids;
        vector<CellDebugInfo> infos;
        vector<BlockColor> colors = getGridColors(frame, params, grids, infos, blueMask, redMask);
        Board board = colorsToBoard(colors);
        Decision decision = decideR2MiddleOnly(board, r2KfsCount);
        string decisionKey = makeDecisionKey(decision);
        string packet = makeSerialPacket(decision);

        if (decisionKey == lastCandidateKey) stableCount++;
        else { lastCandidateKey = decisionKey; stableCount = 1; }

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
        cv::imshow("R2 D455 Decision - OpenCV", display);
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
