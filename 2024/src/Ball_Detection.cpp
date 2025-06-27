//
// Created by zgh on 24-1-20.
//
#include "Ball_Detection.h"
#include "serial_port.h"
#include <vector>
#include <algorithm>
#include <cmath>
#define pi 3.1415926
/*
double u=0;
double v=0;
double h=0;
double w=0;
int getOne;
*/
int ball_detection_counter;
Mat K = (Mat_<double>(3, 3) << 605.719, 0, 639.149, 0, 605.785, 368.511, 0, 0, 1);

float get_depth_pingjun(Mat &transformed_depth_frame, int x, int y) //对深度取平均
{
    float depth[5] = {0};
    depth[0] = transformed_depth_frame.at<ushort>(y - 1, x - 1);
    depth[1] = transformed_depth_frame.at<ushort>(y - 1, x+1);
    depth[2] = transformed_depth_frame.at<ushort>(y + 1, x + 1);
    depth[3] = transformed_depth_frame.at<ushort>(y+1, x );
    depth[4] = transformed_depth_frame.at<ushort>(y, x);

    vector<float> depth_new;
    for (int i = 0; i < 5; i++)
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

    for (int i = 0; i < Ball_box.size(); i++) //获得所有球的横坐标
    {


        if (get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) <= 4500) //只打sanmiwu米以内的柱子
        {
            Ball_box_x.push_back(Ball::Point_with_wide{(int)Ball_box[i].bbox[0], (int)Ball_box[i].bbox[1], (int)Ball_box[i].bbox[2], (int)Ball_box[i].bbox[3]});
        }
    }
    sort(Ball_box_x.begin(), Ball_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的
    if (Ball_box_x.size() > 0)
    {
        ball_detection_counter=0;
        int j = 0;
        int min = 1000000;
        for (int i = 0; i < Ball_box_x.size(); i++) //找到离中心点最近的框为如果不要太
        {
            //int ifmin = abs(Ball_box_x[i].x - 640);
            int ifmin = (640-Ball_box_x[i].x)*(640-Ball_box_x[i].x)+(720-Ball_box_x[i].y)*(720-Ball_box_x[i].y);
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
        double X = cameraPoint.at<double>(0, 0) * depth_value ;
        double Y = cameraPoint.at<double>(1, 0) * depth_value ;
        double Z = cameraPoint.at<double>(2, 0) * depth_value ;
        cout <<X<<","<<Y<<","<<Z<< endl;
        //double angle_h = atan(X / Z);

        // 旋转变换
        cv::Mat rotationMatrix = rotateX(-14); // 绕X轴旋转11度
        cv::Mat point = (cv::Mat_<double>(3, 1) << X, Y, Z);
        cv::Mat rotatedPoint = rotationMatrix * point;
        double X1=rotatedPoint.at<double>(0, 0);
        double Y1=rotatedPoint.at<double>(1, 0);
        double Z1=rotatedPoint.at<double>(2, 0);
        double angle_h = atan(X1 / Z1);
        // 输出旋转后的坐标
        cout <<X1<<","<<Y1<<","<<Z1<<","<<angle_h*180/pi<< endl;
        //send_data(1, (float)angle_h);

        send_data((float)X1,(float)Y1,(float)Z1,(float)angle_h*180/pi);
        auto current_time = std::chrono::steady_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_send_time).count();
        std::cout << "Time interval since last send_data(): " << time_diff << " milliseconds" << std::endl;
        last_send_time = current_time;
        //cout << "--deviation:" << angle_h << endl;
    }

    else
    {
        ball_detection_counter++;
        if (ball_detection_counter >= 20)
        {
            //send_data(4, 0);
            cout << "not--found--" << endl;
        }

    }
}


vector<Ball::Point_with_wide> GetTheTargetOfTrace(std::vector<Yolo::Detection> &Ball_box, Mat &src_no_alpha, Mat &transformed_depth_frame)
{
    vector<Ball::Point_with_wide> Ball_box_x;

    for (int i = 0; i < Ball_box.size(); i++) //获得所有球的横坐标
    {


        if (get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) <= 4500) //只打sanmiwu米以内的柱子
        {
            Ball_box_x.push_back(Ball::Point_with_wide{(int)Ball_box[i].bbox[0], (int)Ball_box[i].bbox[1], (int)Ball_box[i].bbox[2], (int)Ball_box[i].bbox[3]});
        }
    }
    sort(Ball_box_x.begin(), Ball_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的
    if (Ball_box_x.size() > 0) {
        ball_detection_counter = 0;
        int j = 0;
        int min = 1000000;
        for (int i = 0; i < Ball_box_x.size(); i++) //找到离中心点最近的框为如果不要太
        {
            //int ifmin = abs(Ball_box_x[i].x - 640);
            int ifmin = (640 - Ball_box_x[i].x) * (640 - Ball_box_x[i].x) +
                        (720 - Ball_box_x[i].y) * (720 - Ball_box_x[i].y);
            if (ifmin < min) {
                min = ifmin;
                j = i; //第几个框
            }
        }
    }
    return Ball_box_x[j];
}

/*
void Ball::Get_One_XY(std::vector<Yolo::Detection> &Ball_box, Mat &src_no_alpha, Mat &transformed_depth_frame)
{
    vector<Ball::Point_with_wide> Ball_box_x;

    for (int i = 0; i < Ball_box.size(); i++) //获得所有球的横坐标
    {

        if (get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Ball_box[i].bbox[0], Ball_box[i].bbox[1]) <= 3500) //只打sanmiwu米以内的柱子
        {
            Ball_box_x.push_back(Ball::Point_with_wide{(int)Ball_box[i].bbox[0], (int)Ball_box[i].bbox[1], (int)Ball_box[i].bbox[2], (int)Ball_box[i].bbox[3]});
        }
    }
    sort(Ball_box_x.begin(), Ball_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的
    if (Ball_box_x.size() > 0&& getOne==0)
    {
        ball_detection_counter=0;
        int j = 0;
        int min = 1000000;
        for (int i = 0; i < Ball_box_x.size(); i++) //找到离中心点最近的框
        {
            //int ifmin = abs(Ball_box_x[i].x - 640);
            int ifmin = (640-Ball_box_x[i].x)*(640-Ball_box_x[i].x)+(720-Ball_box_x[i].y)*(720-Ball_box_x[i].y);
            if (ifmin < min)
            {
                min = ifmin;
                j = i; //第几个框
            }
        }

        u = (float)Ball_box_x[j].x;
        v = (float)Ball_box_x[j].y;
        h = (float)Ball_box_x[j].hight;
        w = (float)Ball_box_x[j].wide;
        getOne=1;
        //cout << "--deviation:" << angle_h << endl;
    }

    else
    {
        ball_detection_counter++;
        if (ball_detection_counter >= 20)
        {
            //send_data(4, 0);
            cout << "not--found--" << endl;
        }
        if (getOne==1)
        {
            cout << "already--get--one--" << endl;
        }

    }
}
*/
void Ball::Get_Depth_one(std::vector<Yolo::Detection> &Ball_box, Mat &src_no_alpha, Mat &transformed_depth_frame, Ball *ball)
{
    ball->Get_Depth(Ball_box, src_no_alpha, transformed_depth_frame);
}
/*
void Ball::Get_One_XY_one(std::vector<Yolo::Detection> &Ball_box, Mat &src_no_alpha, Mat &transformed_depth_frame, Ball *ball)
{
    ball->Get_One_XY(Ball_box, src_no_alpha, transformed_depth_frame);
}
 */
// 定义旋转函数
cv::Mat rotateX(double angle) {
    double rad = angle * CV_PI / 180.0;
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    rotation.at<double>(1, 1) = cos(rad);
    rotation.at<double>(1, 2) = -sin(rad);
    rotation.at<double>(2, 1) = sin(rad);
    rotation.at<double>(2, 2) = cos(rad);
    return rotation;
}
