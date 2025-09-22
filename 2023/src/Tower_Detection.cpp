//TODO:取深度的时候有时候取不到，就是因为减5像素的判断太捞了，应该用特征去判断然后取，还有一点就是转过的角度我需要给他加一个非线性的量，让它能在大角度时获取更大的角度
#include "include/Tower_Detection.h"
#include "include/serial_port.h"
#include <opencv2/imgproc/types_c.h>
#include <vector>
#include <algorithm>
#include <cmath>
int src_middle = 640;
extern double catch_moudle;
int depth_keep = 0;          //对深度进行保存
int tower_detection_counter; //建立柱检测计数器，计数器达到一定次数即可判断为柱丢失

bool mocalun = false;

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
double get_angle_a(Point2d A, int C) //用点和相机与转轴距离算出应转过角度
{
        //cout << A.x << "   " << A.y << endl;
        double sita_1 = atan(A.x / A.y); //计算出弧度
        //cout << "sita_1   " << sita_1 << endl;
        double D = sqrt(pow(A.x, 2) + pow(A.y, 2));
        double angle_a = atan(D * sin(sita_1) / (C + D * cos(sita_1))); // * 180 / M_PI; //得到转轴应该转过的角度
        return angle_a;
}

bool Point_sorting(Tower::Point_with_wide a, Tower::Point_with_wide b) //依靠点的x坐标从小到大排序
{
        return a.x < b.x;
};

void Tower::Find_tower_method_one(std::vector<Yolo::Detection> &Tower_box, Mat &src_no_alpha, Mat &transformed_depth_frame) //专为模式一设计，用于检测四米以内最近的柱子，并给出角度。
{
        vector<Tower::Point_with_wide> Tower_box_x; //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序

        for (int i = 0; i < Tower_box.size(); i++) //获得所有柱子的横坐标
        {
                //cout << "made......." << endl;
                //cout << transformed_depth_frame.at<ushort>(Tower_box_x[i].y, Tower_box_x[i].x) << endl;
                if (get_depth_pingjun(transformed_depth_frame, Tower_box[i].bbox[0], Tower_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Tower_box[i].bbox[0], Tower_box[i].bbox[1]) <= 3500) //只打sanmiwu米以内的柱子
                {
                        Tower_box_x.push_back(Tower::Point_with_wide{(int)Tower_box[i].bbox[0], (int)Tower_box[i].bbox[1], (int)Tower_box[i].bbox[2], (int)Tower_box[i].bbox[3]});
                        //cout << Tower_box_x[i].x << endl;
                }
        }
        sort(Tower_box_x.begin(), Tower_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的
        if (Tower_box_x.size() > 0)                                  //若有柱子
        {
                tower_detection_counter = 0;
                // cout << Tower_box.size() << endl;
                int j = 0;
                int min = 2000;
                for (int i = 0; i < Tower_box_x.size(); i++) //找到离中心点最近的框
                {
                        int ifmin = abs(Tower_box_x[i].x - src_middle);
                        if (ifmin < min)
                        {
                                min = ifmin;
                                j = i; //第几个框
                        }
                }

                double u = (float)Tower_box_x[j].x, v = (float)Tower_box_x[j].y;
                float depth_value;
                depth_value = get_depth_pingjun(transformed_depth_frame, Tower_box_x[j].x, Tower_box_x[j].y) + 50; //柱子中心点深度
                //cout << "depth_value" << depth_value << endl;
                // 转化为相机坐标
                Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
                Mat cameraPoint = K.inv() * pixelPoint;
                double X = cameraPoint.at<double>(0, 0) * depth_value / 10;
                double Y = cameraPoint.at<double>(1, 0) * depth_value / 10;
                double Z = cameraPoint.at<double>(2, 0) * depth_value / 10;

                double angle_h = atan(X / Z);

                send_data(1, (float)angle_h);
                cout << "--deviation:" << angle_h << endl;
        }
        else
        {
                tower_detection_counter++;
                if (tower_detection_counter >= 20)
                {
                        send_data(4, 0);
                }
                cout << "--mode--tower--not--found--" << endl;
        }
}
void Tower::Find_tower_method_one_far(std::vector<Yolo::Detection> &Tower_box, Mat &src_no_alpha, Mat &transformed_depth_frame) //专为模式一设计，用于检测四米以内最近的柱子，并给出角度。
{
        vector<Tower::Point_with_wide> Tower_box_x; //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序

        for (int i = 0; i < Tower_box.size(); i++) //获得所有柱子的横坐标
        {
                //cout << "made......." << endl;
                //cout << transformed_depth_frame.at<ushort>(Tower_box_x[i].y, Tower_box_x[i].x) << endl;
                if (get_depth_pingjun(transformed_depth_frame, Tower_box[i].bbox[0], Tower_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Tower_box[i].bbox[0], Tower_box[i].bbox[1]) <= 5200) //只打五米以内的柱子
                {
                        Tower_box_x.push_back(Tower::Point_with_wide{(int)Tower_box[i].bbox[0], (int)Tower_box[i].bbox[1], (int)Tower_box[i].bbox[2], (int)Tower_box[i].bbox[3]});
                        //cout << Tower_box_x[i].x << endl;
                }
        }
        sort(Tower_box_x.begin(), Tower_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的
        if (Tower_box_x.size() > 0)                                  //若有柱子
        {
                tower_detection_counter = 0;
                // cout << Tower_box.size() << endl;
                int j = 0;
                int min = 2000;
                for (int i = 0; i < Tower_box_x.size(); i++) //找到离中心点最近的框
                {
                        int ifmin = abs(Tower_box_x[i].x - src_middle);
                        if (ifmin < min)
                        {
                                min = ifmin;
                                j = i; //第几个框
                        }
                }

                double u = (float)Tower_box_x[j].x, v = (float)Tower_box_x[j].y;
                float depth_value;
                depth_value = get_depth_pingjun(transformed_depth_frame, Tower_box_x[j].x, Tower_box_x[j].y) + 50; //柱子中心点深度
                //cout << "depth_value" << depth_value << endl;
                // 转化为相机坐标
                Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
                Mat cameraPoint = K.inv() * pixelPoint;
                double X = cameraPoint.at<double>(0, 0) * depth_value / 10;
                double Y = cameraPoint.at<double>(1, 0) * depth_value / 10;
                double Z = cameraPoint.at<double>(2, 0) * depth_value / 10;

                double angle_h = atan(X / Z);

                send_data(5, (float)angle_h);
                cout << "--deviation:" << angle_h << endl;
        }
        else
        {
                tower_detection_counter++;
                if (tower_detection_counter >= 20)
                {
                        send_data(4, 0);
                }
                cout << "--mode--tower--not--found--" << endl;
        }
}

void Tower::Find_depth_method_one(std::vector<Yolo::Detection> &Tower_box, Mat &transformed_depth_frame, Mat &src_no_alpha) //模式一专用深度寻找，可以获取柱子高度和深度来给出推荐转速和仰角
{
        vector<Tower::Point_with_wide> Tower_box_x; //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序

        for (int i = 0; i < Tower_box.size(); i++) //获得所有柱子的横坐标
        {
                if (get_depth_pingjun(transformed_depth_frame, Tower_box[i].bbox[0], Tower_box[i].bbox[1]) != 0 && get_depth_pingjun(transformed_depth_frame, Tower_box[i].bbox[0], Tower_box[i].bbox[1]) <= 5000) //    transformed_depth_frame.at<ushort>(Tower_box[i].bbox[1], Tower_box[i].bbox[0]
                        Tower_box_x.push_back(Tower::Point_with_wide{(int)Tower_box[i].bbox[0], (int)Tower_box[i].bbox[1], (int)Tower_box[i].bbox[2], (int)Tower_box[i].bbox[3]});
        }
        sort(Tower_box_x.begin(), Tower_box_x.end(), Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的

        if (Tower_box_x.size() > 0) //若有柱子
        {
                int j = 0;
                int min = abs(Tower_box_x[0].x - src_middle);
                for (int i = 0; i < Tower_box_x.size(); i++) //找到离中心点最近的框
                {
                        int ifmin = abs(Tower_box_x[i].x - src_middle);
                        if (ifmin < min)
                        {
                                min = ifmin;
                                j = i; //第几个框
                        }
                }

                if (mocalun == false)
                {
                        tower_detection_counter = 0;
                        // 像素坐标 正中心坐标
                        double u = (float)Tower_box_x[j].x, v = (float)Tower_box_x[j].y;
                        float depth_value = 0;
                        //cv::Mat src_hsv;
                        //cv::cvtColor(src_no_alpha, src_hsv, CV_BGR2HSV);
                        if (get_depth_pingjun(transformed_depth_frame, Tower_box_x[j].x, Tower_box_x[j].y) != 0) //&& src_hsv.at<Vec3b>((Tower_box_x[j].y - Tower_box_x[j].hight / 2 + i), Tower_box_x[j].x)[0] >= 0 && src_hsv.at<Vec3b>((Tower_box_x[j].y - Tower_box_x[j].hight / 2 + i), Tower_box_x[j].x)[0] <= 180
                        {
                                depth_value = get_depth_pingjun(transformed_depth_frame, Tower_box_x[j].x, Tower_box_x[j].y) + 50; //柱子中心点深度 //transformed_depth_frame.at<ushort>(Tower_box_x[j].y, Tower_box_x[j].x) + 60; //柱子中心点深度
                        }
                        else
                        {
                                depth_value = 0;
                        }

                        //float depth_value = transformed_depth_frame.at<ushort>((Tower_box_x[j].y - Tower_box_x[j].hight / 2 + 5), Tower_box_x[j].x);
                        // 转化为相机坐标
                        Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
                        Mat cameraPoint = K.inv() * pixelPoint;

                        double X = cameraPoint.at<double>(0, 0) * depth_value / 10;
                        double Y = cameraPoint.at<double>(1, 0) * depth_value / 10;
                        double Z = cameraPoint.at<double>(2, 0) * depth_value / 10;
                        if (Z == 0)
                        {
                                tower_detection_counter++;
                                if (tower_detection_counter >= 20)
                                {
                                        send_data(4, 0);
                                        cout << "--mode--tower--not--found--" << endl;
                                }
                        }
                        else
                        {
                                depth_keep = Z;
                                send_data(2, Z); //发深度
                                cout << "--mode--tower--depth--" << Z << " cm" << endl;
                                mocalun = true;
                        }
                }
                else
                {
                        tower_detection_counter = 0;
                        send_data(2, depth_keep); //发维持深度
                        cout << "--mode--tower--depth--continue" << depth_keep << " cm" << endl;
                }
        }
        else
        {
                tower_detection_counter++;
                if (tower_detection_counter >= 20)
                {
                        send_data(4, 0);
                        cout << "--mode--tower--not--found--" << endl;
                }
        }
}
