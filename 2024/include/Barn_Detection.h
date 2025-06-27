#ifndef BARNDETECTION_H
#define BARNDETECTION_H

#pragma comment(lib, "k4a.lib")
#include <opencv2/opencv.hpp>
#include "Yolov5.h"
#include <cstdio>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <k4a/k4a.hpp>
#include "depthai/depthai.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <sys/types.h>
#include <cstdlib>
#include <omp.h>
#include <vector>
#include <algorithm>
#include <cmath>

class Ball
{
public:
    std::vector<Yolo::Detection> Ball_box;
    void Get_Depth(std::vector<Yolo::Detection> &Ball_box, cv::Mat &src_no_alpha, cv::Mat &transformed_depth_frame);
    void Get_Depth_one(std::vector<Yolo::Detection> &Ball_box, cv::Mat &src_no_alpha, cv::Mat &transformed_depth_frame, Ball* ball);
    struct Point_with_wide
    {
        int x;
        int y;
        int wide;
        int hight;
    };
};
class Barn_Detection{
    public:
        Barn_Detection();
        void process();
    };
#endif //__SERIAL_PORT_THREAD_H
