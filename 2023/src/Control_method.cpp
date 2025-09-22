#include "include/Tower_Detection.h"
#include "include/serial_port.h"
#include <vector>
#include <algorithm>

extern bool catch_tower, left_tower, right_tower, mocalun;
extern int tower_detection_counter;
extern double catch_moudle;
extern int depth_keep;

void Tower::Control_method_one(std::vector<Yolo::Detection> &Tower_box, Mat &src_no_alpha, Mat &transformed_depth_frame, Tower *Tower)
{
        switch ((int)catch_moudle)
        {
        case 1:
                mocalun = false;
                depth_keep = 0;
                Tower->Find_tower_method_one(Tower_box, src_no_alpha, transformed_depth_frame);
                break;
        case 2:
                Tower->Find_depth_method_one(Tower_box, transformed_depth_frame, src_no_alpha);
                break;
        case 3:
                mocalun = false;
                depth_keep = 0;
                tower_detection_counter = 0;
                send_data(3, 0);
                break;
        case 5:
                mocalun = false;
                depth_keep = 0;
                Tower->Find_tower_method_one_far(Tower_box, src_no_alpha, transformed_depth_frame);
                break;
        }
}
