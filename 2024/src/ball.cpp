#include "include/Barn_Detection.h"
#include "include/serial_port.h"
#include <vector>
#include <algorithm>
#include <cmath>
using namespace cv;
using namespace std;
int ball_detection_counter;
Mat K = (Mat_<double>(3, 3) << 605.719, 0, 639.149, 0, 605.785, 368.511, 0, 0, 1);

float get_depth_pingjun(Mat &transformed_depth_frame, int x, int y) //对深度取平均
{
    float depth[9] = {0};
    depth[0] = transformed_depth_frame.at<ushort>(y - 1, x - 1);
    depth[1] = transformed_depth_frame.at<ushort>(y - 1, x);
    depth[2] = transformed_depth_frame.at<ushort>(y - 1, x + 1);
    depth[3] = transformed_depth_frame.at<ushort>(y, x - 1);
    depth[4] = transformed_depth_frame.at<ushort>(y, x);
    depth[5] = transformed_depth_frame.at<ushort>(y, x + 1);
    depth[6] = transformed_depth_frame.at<ushort>(y + 1, x - 1);
    depth[7] = transformed_depth_frame.at<ushort>(y + 1, x);
    depth[8] = transformed_depth_frame.at<ushort>(y + 1, x + 1);
    vector<float> depth_new;
    for (int i = 0; i < 9; i++)
    {
        if (depth[i] != 0 && depth[i] <= 5200)
            depth_new.push_back(depth[i]);
    }
    float all_depth = 0;
    for (int i = 0; i < depth_new.size(); i++)
    {
        all_depth += depth_new[i];
    }
    if (depth_new.size() > 0)
        all_depth = all_depth / depth_new.size();
    return all_depth;
}


bool Point_sorting(Ball::Point_with_wide a, Ball::Point_with_wide b) //依靠点的x坐标从小到大排序
{
    return a.x < b.x;
};

void Ball::Get_Depth(std::vector<Yolo::Detection> &Ball_box, Mat &src_no_alpha, Mat &transformed_depth_frame)
{
    vector<Ball::Point_with_wide> Ball_box_x;

    for (int i = 0; i < Ball_box.size(); i++) //获得所有柱子的横坐标
    {

        if (get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) <= 3500) //只打sanmiwu米以内的柱子
        {
            Ball_box_x.push_back(Ball::Point_with_wide{(int)Ball_box[i].bbox[0], (int)Ball_box[i].bbox[1], (int)Ball_box[i].bbox[2], (int)Ball_box[i].bbox[3]});
        }
    }
    sort(Ball_box_x.begin(), Ball_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的
    if (Ball_box_x.size() > 0)
    {
        ball_detection_counter=0;
        int j = 0;
        int min = 2000;
        for (int i = 0; i < Ball_box_x.size(); i++) //找到离中心点最近的框
        {
            int ifmin = abs(Ball_box_x[i].x - 640);
            if (ifmin < min)
            {
                min = ifmin;
                j = i; //第几个框
            }
        }

        double u = (float)Ball_box_x[j].x, v = (float)Ball_box_x[j].y;
        float depth_value;
        depth_value = get_depth_pingjun(transformed_depth_frame, Ball_box_x[j].x, Ball_box_x[j].y);   // + 50
        //cout << "depth_value" << depth_value << endl;
        // 转化为相机坐标
        Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
        Mat cameraPoint = K.inv() * pixelPoint;
        double X = cameraPoint.at<double>(0, 0) * depth_value / 10;
        double Y = cameraPoint.at<double>(1, 0) * depth_value / 10;
        double Z = cameraPoint.at<double>(2, 0) * depth_value / 10;

        double angle_h = atan(X / Z);

        //send_data(1, (float)angle_h);
        //cout << "--deviation:" << angle_h << endl;
    }
    else
    {
        ball_detection_counter++;
        if (ball_detection_counter >= 20)
        {
            //send_data(4, 0);
        }
        //cout << "--mode--tower--not--found--" << endl;
    }
}


void Ball::Get_Depth_one(std::vector<Yolo::Detection> &Ball_box, Mat &src_no_alpha, Mat &transformed_depth_frame, Ball *ball)
{
    ball->Get_Depth(Ball_box, src_no_alpha, transformed_depth_frame);
}
