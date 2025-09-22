#ifndef TOWERDETECTION_H
#define TOWERDETECTION_H

#include <opencv2/opencv.hpp>
#include "Yolo/Yolov5.h"
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <k4a/k4a.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/time.h>
#include <sys/types.h>
#include <cstdlib>
#include <omp.h>

using namespace cv;
using namespace std;
class Tower_Detection
{
public:
        Tower_Detection();
        int process();
};
class Tower
{
public:
        std::vector<Yolo::Detection> Tower_box;

        void Control_method_one(std::vector<Yolo::Detection> &Tower_box, Mat &src_no_alpha, Mat &transformed_depth_frame, Tower *Tower); //控制模式1
        void Find_tower_method_one(std::vector<Yolo::Detection> &Tower_box, Mat &src_no_alpha, Mat &transformed_depth_frame);            //模式一专用，发偏差角度
        void Find_depth_method_one(std::vector<Yolo::Detection> &Tower_box, Mat &transformed_depth_frame, Mat &src_no_alpha);            //模式一专用，发转速和仰角
        void Find_tower_method_one_far(std::vector<Yolo::Detection> &Tower_box, Mat &src_no_alpha, Mat &transformed_depth_frame);        //模式一专用，发偏差角度
        struct Point_with_wide
        {
                int x;
                int y;
                int wide;
                int hight;
        };
};
#endif //__SERIAL_PORT_THREAD_H