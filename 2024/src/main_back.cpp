//
// Created by zgh on 24-1-20.
//
#if 1
#include "Ball_Detection.h"
#include "serial_port.h"
#include <thread>
#include <mutex>
#include <iostream>
#include "depthai/depthai.hpp"
#include "Yolov5.h"
using namespace cv;
using namespace std;

/**********************************************************************/
//谷仓识别

float best; //需要传递的数据：最优的谷仓序号
//五个谷仓中的红球数量
int red[5];
//五个谷仓中的蓝球数量
int blue[5];
//记录检测到的所有红球的中心x坐标
int tem_red[15];
//检测到的红球数量
int num_red;
//记录检测到的所有蓝球的中心x坐标
int tem_blue[15];
//检测到的蓝球数量
int num_blue;
//谷仓
struct Barn
{
    int x1; //左侧x坐标
    int x2; //右侧x坐标
};
//记录五个谷仓的左侧x坐标和右侧x坐标
Barn barn[5];
//检测到的谷仓数量
int num_barn;

int max(int arr[5]);
int find(int arr[5], int num);
//读取数据
void read_txt();
//判断每个谷仓中球的个数
void tell();
//策略判断
void strategy();
//刷新每一次检测到的数据
void refresh();
/**********************************************************************/

Ball_Detection::Ball_Detection()
{
    std::string engine_name = "/home/robot/桌面/2024_ball_xioabai/Mask_Detector/engine/best_5_int8.engine";
    engine_init(engine_name);
}
Ball_Detection_2::Ball_Detection_2()
{
    std::string engine_name = "/home/robot/桌面/2024_ball_xioabai/Mask_Detector/engine/best_barn.engine";
    engine_init_2(engine_name);
}
int Ball_Detection::process_AK()
{

    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
    k4a::capture capture;

    const uint32_t device_count = k4a::device::get_installed_count();
    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
    device.start_cameras(&config);
    /*校准深度和颜色*/
    k4a_calibration_t calibration = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P);

    k4a::transformation transformation = k4a::transformation(calibration);

    k4a::image rgbImage;
    k4a::image depthImage;

    k4a::image transformed_depth_image = NULL;
    /*将image转化为Mat类型的接收的容器*/
    cv::Mat transformed_depth_frame;
    Mat src, src_no_alpha;
    Ball ball;
    namedWindow("srcImage", 0);

    while (true)
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            ball.last_send_time = std::chrono::steady_clock::now();
            rgbImage = capture.get_color_image();
            src = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, rgbImage.get_buffer());
            depthImage = capture.get_depth_image();
            transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
            transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());
            cv::cvtColor(src, src_no_alpha, cv::COLOR_BGRA2BGR);
            if (src_no_alpha.empty())
            {
                continue;
            }

            std::vector<Yolo::Detection> Ball_box = yolo_main(src_no_alpha); //放入模型

            ball.Get_Depth_one(Ball_box, src_no_alpha, transformed_depth_frame, &ball);

            imshow("srcImage", src_no_alpha);
            //cout<<src_no_alpha.size()<<endl;
            src.release();
            src_no_alpha.release();
            capture.reset();
            transformed_depth_frame.release();
            if (cv::waitKey(10) == ' ')
            {
                break;
            }
        }
    }
    cv::destroyAllWindows();
    device.close();
}
void runKinectCamera(){
    Ball_Detection BallDet;
    BallDet.process_AK();
}
int Ball_Detection_2::process_OAK(){
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    xoutVideo->setStreamName("video");
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1920, 1080);
    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);
    camRgb->video.link(xoutVideo->input);
    dai::Device device(pipeline);
    auto video = device.getOutputQueue("video");
    cv::Mat img_RGB;
    while(true)
    {
        auto videoIn = video->get<dai::ImgFrame>();
        img_RGB = videoIn->getCvFrame();
        //调整亮度
        img_RGB += 20;
        yolo_main_2(img_RGB);
        //cout<<"aaaaaaaa"<<endl;

        read_txt();
        //五个谷仓都能看到时
        if(num_barn == 5)
        {
            tell();
            //策略判断
            strategy();
            //传递数据
            cout<<best<<endl;
            send_data_oak((float)best);
        }
        //刷新数据
        refresh();

        cv::imshow("OAK", img_RGB);
        if(waitKey(10) == ' ')
        {
            break;
        }
        img_RGB.release();
    }

    cv::destroyAllWindows();
}
void runOAKCamera() {
    Ball_Detection_2 BarnDet;
    BarnDet.process_OAK();
}
int main()
{

    for(int i = 0; i < 5; i++)
    {
        barn[i].x1 = -1;
        barn[i].x2 = -1;
    }
    thread kinectThread(runKinectCamera);

    // Start OAK camera thread
    thread oakThread(runOAKCamera);

    // Wait for the threads to finish

    kinectThread.join();
    oakThread.join();
    return 0;
}

int max(int arr[5])
{
    int m = arr[0];
    for (int i = 0; i < 5; i++)
    {
        if (arr[i] > m)
        {
            m = arr[i];
        }
    }
    return m;
}

int find(int arr[5], int num)
{
    int index = -1;
    for (int i = 0; i < 5; i++)
    {
        if (arr[i] == num)
        {
            index = i;
            break;
        }
    }
    return index;
}

void strategy()
{
    //假设我方为红球，对方为蓝球（瘪谷为紫球）
    //如果有谷仓中已经有3个球，将其设为-1
    for (int i = 0; i < 5; i++)
    {
        if (red[i] + blue[i] == 3)
        {
            red[i] = -1;
            blue[i] = -1;
        }
    }
    //判断
    int red_max = max(red);
    int blue_max = max(blue);
    //增加约束条件，考虑到时间问题，在策略最优的情况下优先往近的谷仓放球
    //若谷仓中均没有球，则放置在中间的谷仓
    if (red_max == 0 && blue_max == 0)
    {
        //cout << find(red, 0) << endl;
        best = float(2);
    }
    //若谷仓中有一个红球、没有蓝球，则优先考虑将球放进最近的空谷仓中，其次考虑放入有红球的谷仓中
    else if (red_max == 1 && blue_max == 0)
    {
        int num123[5] = {0};    //存储空谷仓的序号
        int num122[5] = {0};    //存储有一个红球的谷仓序号
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 0 && blue[i] == 0)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
        }
        else if(num123[1] == 1)
        {
            best = float(1);
        }
        else if(num123[3] == 1)
        {
            best = float(3);
        }
        else if(num123[0] == 1)
        {
            best = float(1);
        }
        else if(num123[4] == 1)
        {
            best = float(4);
        }
        else
        {
            for (int i = 0; i < 5; i++)
            {
                if (red[i] == 1)
                {
                    num122[i] = 1;
                }
            }
            if(num122[2] == 1)
            {
                best = float(2);
            }
            else if(num122[1] == 1)
            {
                best = float(1);
            }
            else if(num122[3] == 1)
            {
                best = float(3);
            }
            else if(num122[0] == 1)
            {
                best = float(1);
            }
            else if(num122[4] == 1)
            {
                best = float(4);
            }
        }
    }
    //若谷仓中有两个红球、没有蓝球，则将球放在此谷仓中
    else if (red_max == 2 && blue_max == 0)
    {
        int num123[5] = {0};    //存储有两个红球的谷仓的序号
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 2 && blue[i] == 0)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
        }
        else if(num123[1] == 1)
        {
            best = float(1);
        }
        else if(num123[3] == 1)
        {
            best = float(3);
        }
        else if(num123[0] == 1)
        {
            best = float(1);
        }
        else if(num123[4] == 1)
        {
            best = float(4);
        }
    }
    //若谷仓中没有红球、有一个蓝球，则优先考虑将球放进最近的空谷仓中，其次考虑放入有蓝球的谷仓中
    else if (red_max == 0 && blue_max == 1)
    {
        int num123[5] = {0};    //存储空谷仓的序号
        int num122[5] = {0};    //存储有一个蓝球的谷仓序号
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 0 && blue[i] == 0)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
        }
        else if(num123[1] == 1)
        {
            best = float(1);
        }
        else if(num123[3] == 1)
        {
            best = float(3);
        }
        else if(num123[0] == 1)
        {
            best = float(1);
        }
        else if(num123[4] == 1)
        {
            best = float(4);
        }
        else
        {
            for (int i = 0; i < 5; i++)
            {
                if (blue[i] == 1)
                {
                    num122[i] = 1;
                }
            }
            if(num122[2] == 1)
            {
                best = float(2);
            }
            else if(num122[1] == 1)
            {
                best = float(1);
            }
            else if(num122[3] == 1)
            {
                best = float(3);
            }
            else if(num122[0] == 1)
            {
                best = float(1);
            }
            else if(num122[4] == 1)
            {
                best = float(4);
            }
        }
    }
    //若谷仓中一个红球、一个蓝球，先判断它们是否在同一个谷仓中，若在，则放在此；若不在，则优先放在近的空谷仓
    else if (red_max == 1 && blue_max == 1)
    {
        int num123[5] = {0};    //存储同时有一个红球、一个蓝球的谷仓的序号
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num123[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num123[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[4] == 1)
        {
            best = float(4);
            return ;
        }
        int index = find(red, 0);
        if (index == -1)
        {
            index = find(blue, 0);
            if (index == -1)
            {
                //cout << find(red, 1) << endl;
                best = float(find(red, 1));
                return ;
            }
            else
            {
                //cout << index << endl;
                best = float(index);
            }
        }
        else
        {
            for(int i = 0; i < 5; i++)
            {
                if(red[i] + blue[i] == 0)
                {
                    //cout << i << endl;
                    best = float(i);
                    return ;
                }
            }
            //cout << find(red, 1) << endl;
            best = float(find(red, 1));
        }
    }
    //若谷仓中两个红球、一个蓝球，则放在此仓中
    else if (red_max == 2 && blue_max == 1)
    {
        int num123[5] = {0};    //存储同时有一个红球、一个蓝球的谷仓的序号
        int num122[5] = {0};    //存储只有两个红球的谷仓 
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num123[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num123[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[4] == 1)
        {
            best = float(4);
            return ;
        }
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 2)
            {
                num122[i] = 1;
            }
        }
        if(num122[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num122[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num122[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num122[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num122[4] == 1)
        {
            best = float(4);
            return ;
        }
    }
    //若谷仓中没有红球、两个蓝球，则放在此仓中
    else if (red_max == 0 && blue_max == 2)
    {
        int num123[5] = {0};    //存储有两个蓝球的谷仓的序号
        for (int i = 0; i < 5; i++)
        {
            if (blue[i] == 2)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num123[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num123[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[4] == 1)
        {
            best = float(4);
            return ;
        }
    }
    //若谷仓中有一个红球、两个蓝球，则放在此仓中
    //问题：若有一个仓中有一个红球和一个蓝球，则这个仓优先级更高
    else if (red_max == 1 && blue_max == 2)
    {
        int num123[5] = {0};    //存储同时有一个红球、一个蓝球的谷仓的序号
        int num122[5] = {0};    //存储只有两个蓝球的谷仓 
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num123[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num123[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[4] == 1)
        {
            best = float(4);
            return ;
        }
        for (int i = 0; i < 5; i++)
        {
            if (blue[i] == 2)
            {
                num122[i] = 1;
            }
        }
        if(num122[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num122[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num122[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num122[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num122[4] == 1)
        {
            best = float(4);
            return ;
        }
    }
    //若谷仓中有两个红球、两个蓝球，则放在红仓中
    else if (red_max == 2 && blue_max == 2)
    {
        int num123[5] = {0};    //存储同时有一个红球、一个蓝球的谷仓的序号
        int num122[5] = {0};    //存储只有两个红球的谷仓 
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                num123[i] = 1;
            }
        }
        if(num123[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num123[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num123[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num123[4] == 1)
        {
            best = float(4);
            return ;
        }
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 2)
            {
                num122[i] = 1;
            }
        }
        if(num122[2] == 1)
        {
            best = float(2);
            return ;
        }
        else if(num122[1] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num122[3] == 1)
        {
            best = float(3);
            return ;
        }
        else if(num122[0] == 1)
        {
            best = float(1);
            return ;
        }
        else if(num122[4] == 1)
        {
            best = float(4);
            return ;
        }
    }
}

void read_txt()
{
    ifstream infile;
    infile.open("/home/robot/桌面/2024_ball_xioabai/Mask_Detector/data.txt");
    char str[50][10];
    int i = 0;
    while(infile.getline(str[i], 10, ' '))
    {
        //redball
        if(str[i][0] == 'r')
        {
            i++;
            infile.getline(str[i], 10, ' ');
            tem_red[num_red] = stoi(str[i]);
            num_red++;
        }
        //blueball
        if(str[i][0] == 'b' && str[i][1] == 'l')
        {
            i++;
            infile.getline(str[i], 10, ' ');
            tem_blue[num_blue] = stoi(str[i]);
            num_blue++;
        }
        //barn
        if(str[i][0] == 'b' && str[i][1] == 'a')
        {
            i++;
            infile.getline(str[i], 10, ' ');
            barn[num_barn].x1 = stoi(str[i]);
            i++;
            infile.getline(str[i], 10, ' ');
            barn[num_barn].x2 = stoi(str[i]);
            num_barn++;
        }
        i++;
    }
}

void tell()
{
    //谷仓排序
    Barn tem;
    for(int i = 0; i < 4; i++)
    {
        for(int j = i+1; j < 5; j++)
        {
            if(barn[i].x1 > barn[j].x1)
            {
                tem = barn[i];
                barn[i] = barn[j];
                barn[j] = tem;
            }
        }
    }
    //遍历检测到的所有红球
    for(int i = 0; i < num_red; i++)
    {
        if(tem_red[i] > barn[0].x1 && tem_red[i] < barn[0].x2)
        {
            red[0]++;
        }
        else if(tem_red[i] > barn[1].x1 && tem_red[i] < barn[1].x2)
        {
            red[1]++;
        }
        else if(tem_red[i] > barn[2].x1 && tem_red[i] < barn[2].x2)
        {
            red[2]++;
        }
        else if(tem_red[i] > barn[3].x1 && tem_red[i] < barn[3].x2)
        {
            red[3]++;
        }
        else if(tem_red[i] > barn[4].x1 && tem_red[i] < barn[4].x2)
        {
            red[4]++;
        }
    }
    //遍历检测到的所有蓝球
    for(int i = 0; i < num_blue; i++)
    {
        if(tem_blue[i] > barn[0].x1 && tem_blue[i] < barn[0].x2)
        {
            blue[0]++;
        }
        else if(tem_blue[i] > barn[1].x1 && tem_blue[i] < barn[1].x2)
        {
            blue[1]++;
        }
        else if(tem_blue[i] > barn[2].x1 && tem_blue[i] < barn[2].x2)
        {
            blue[2]++;
        }
        else if(tem_blue[i] > barn[3].x1 && tem_blue[i] < barn[3].x2)
        {
            blue[3]++;
        }
        else if(tem_blue[i] > barn[4].x1 && tem_blue[i] < barn[4].x2)
        {
            blue[4]++;
        }
    }
}

void refresh()
{
    for(int i = 0; i < 5; i++)
    {
        red[i] = 0;
        blue[i] = 0;
    }
    for(int i = 0; i < num_barn; i++)
    {
        barn[i].x1 = -1;
        barn[i].x2 = -1;
    }
    for(int i = 0; i < num_red; i++)
    {
        tem_red[i] = 0;
    }
    for(int i = 0; i < num_blue; i++)
    {
        tem_blue[i] = 0;
    }
    num_red = 0;
    num_blue = 0;
    num_barn = 0;
}

#endif

#if 0

#include "Ball_Detection.h"
#include "serial_port.h"
#include <thread>
#include <mutex>
#include <iostream>
#include "depthai/depthai.hpp"
#include "Yolov5.h"
using namespace cv;
using namespace std;

/**********************************************************************/
//谷仓识别
void yolo(cv::Mat mat);
float best; //需要传递的数据：最优的谷仓序号
//五个谷仓中的红球数量
int red[5];
//五个谷仓中的蓝球数量
int blue[5];
//记录检测到的所有红球的中心x坐标
int tem_red[15];
//检测到的红球数量
int num_red;
//记录检测到的所有蓝球的中心x坐标
int tem_blue[15];
//检测到的蓝球数量
int num_blue;
//谷仓
struct Barn
{
    int x1; //左侧x坐标
    int x2; //右侧x坐标
};
//记录五个谷仓的左侧x坐标和右侧x坐标
Barn barn[5];
//检测到的谷仓数量
int num_barn;

int max(int arr[5]);
int find(int arr[5], int num);
//读取数据
void read_txt();
//判断每个谷仓中球的个数
void tell();
//策略判断
void strategy();
//刷新每一次检测到的数据
void refresh();
/**********************************************************************/
//std::mutex kinectMutex;
//std::mutex oakMutex;

Ball_Detection::Ball_Detection()
{
    std::string engine_name_1 = "/home/robot/桌面/2024_ball_xioabai/Mask_Detector/engine/best_5_int8.engine";
    std::string engine_name_2 = "/home/robot/桌面/2024_ball_xioabai/Mask_Detector/engine/best_5S_int8.engine";
    engine_init(engine_name_1,engine_name_2);
}
int Ball_Detection::process_AK()
{

    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
    k4a::capture capture;

    const uint32_t device_count = k4a::device::get_installed_count();
    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.synchronized_images_only = true;
    device.start_cameras(&config);
    /*校准深度和颜色*/
    k4a_calibration_t calibration = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P);

    k4a::transformation transformation = k4a::transformation(calibration);

    k4a::image rgbImage;
    k4a::image depthImage;

    k4a::image transformed_depth_image = NULL;
    /*将image转化为Mat类型的接收的容器*/
    cv::Mat transformed_depth_frame;
    Mat src, src_no_alpha;
    Ball ball;
    namedWindow("srcImage", 0);

    while (true)
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(0)))
        {

            rgbImage = capture.get_color_image();
            src = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, rgbImage.get_buffer());
            // 加锁保护共享资源
            //std::lock_guard<std::mutex> lock(kinectMutex);
            depthImage = capture.get_depth_image();
            transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
            transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());
            cv::cvtColor(src, src_no_alpha, cv::COLOR_BGRA2BGR);
            if (src_no_alpha.empty())
            {
                continue;
            }

            std::vector<Yolo::Detection> Ball_box = yolo_main(src_no_alpha); //放入模型

            ball.Get_Depth_one(Ball_box, src_no_alpha, transformed_depth_frame, &ball);

            imshow("srcImage", src_no_alpha);
            //cout<<src_no_alpha.size()<<endl;
            src.release();
            src_no_alpha.release();
            capture.reset();
            transformed_depth_frame.release();
            if (cv::waitKey(10) == ' ')
            {
                break;
            }
        }
    }
    cv::destroyAllWindows();
    device.close();
}
void runKinectCamera(){
    Ball_Detection BallDet;
    BallDet.process_AK();
}
int Ball_Detection::process_OAK(){
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    xoutVideo->setStreamName("video");
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1280, 720);
    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);
    camRgb->video.link(xoutVideo->input);
    dai::Device device(pipeline);
    auto video = device.getOutputQueue("video");
    cv::Mat img_RGB;
    while(true)
    {
        // 加锁保护共享资源
        //std::lock_guard<std::mutex> lock(oakMutex);
        auto videoIn = video->get<dai::ImgFrame>();
        img_RGB = videoIn->getCvFrame();
        //调整亮度
        img_RGB += 20;
        yolo_main(img_RGB);
        //读取数据
        read_txt();
        //五个谷仓都能看到时
        if(num_barn == 5)
        {
            tell();
            //策略判断
            strategy();
            //传递数据
            cout<<best<<endl;
            //send_data(best);
        }
        //刷新数据
        refresh();
        cv::imshow("OAK", img_RGB);
        if(waitKey(10) == ' ')
        {
            break;
        }
    }
    img_RGB.release();
    cv::destroyAllWindows();
}
void runOAKCamera() {
    Ball_Detection BarnDet;
    BarnDet.process_OAK();
}
int main()
{
    for(int i = 0; i < 5; i++)
    {
        barn[i].x1 = -1;
        barn[i].x2 = -1;
    }
    thread kinectThread(runKinectCamera);

    // Start OAK camera thread
    thread oakThread(runOAKCamera);

    // Wait for the threads to finish
    oakThread.join();
    kinectThread.join();
    return 0;
}

int max(int arr[5])
{
    int m = arr[0];
    for (int i = 0; i < 5; i++)
    {
        if (arr[i] > m)
        {
            m = arr[i];
        }
    }
    return m;
}

int find(int arr[5], int num)
{
    int index = -1;
    for (int i = 0; i < 5; i++)
    {
        if (arr[i] == num)
        {
            index = i;
            break;
        }
    }
    return index;
}

void strategy()
{
    //假设我方为红球，对方为蓝球（瘪谷为紫球）
    //如果有谷仓中已经有3个球，将其设为-1
    for (int i = 0; i < 5; i++)
    {
        if (red[i] + blue[i] == 3)
        {
            red[i] = -1;
            blue[i] = -1;
        }
    }
    //判断
    int red_max = max(red);
    int blue_max = max(blue);
    //若谷仓中均没有球，则随机放置
    if (red_max == 0 && blue_max == 0)
    {
        //cout << find(red, 0) << endl;
        best = float(find(red, 0));
    }
        //若谷仓中有一个红球、没有蓝球，则优先考虑将球放进空的谷仓中，其次考虑放入有红球的谷仓中
    else if (red_max == 1 && blue_max == 0)
    {
        int index = find(red, 0);
        if (index != -1)
        {
            //cout << index << endl;
            best = float(index);
        }
        else
        {
            //cout << find(red, 1) << endl;
            best = float(find(red, 1));
        }
    }
        //若谷仓中有两个红球、没有蓝球，则将球放在此谷仓中
    else if (red_max == 2 && blue_max == 0)
    {
        //cout << find(red, 2) << endl;
        best = float(find(red, 2));
    }
        //若谷仓中没有红球、有一个蓝球，则优先考虑将球放进空的谷仓中，其次考虑放入有蓝球的谷仓中
    else if (red_max == 0 && blue_max == 1)
    {
        int index = find(blue, 0);
        if (index != -1)
        {
            //cout << index << endl;
            best = float(index);
        }
        else
        {
            //cout << find(blue, 1) << endl;
            best = float(find(blue, 1));
        }
    }
        //若谷仓中一个红球、一个蓝球，先判断它们是否在同一个谷仓中，若在，则放在此；若不在，则优先放在空仓
    else if (red_max == 1 && blue_max == 1)
    {
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                //cout << i << endl;
                best = float(i);
                return ;
            }
        }
        int index = find(red, 0);
        if (index == -1)
        {
            index = find(blue, 0);
            if (index == -1)
            {
                //cout << find(red, 1) << endl;
                best = float(find(red, 1));
                return ;
            }
            else
            {
                //cout << index << endl;
                best = float(index);
            }
        }
        else
        {
            for(int i = 0; i < 5; i++)
            {
                if(red[i] + blue[i] == 0)
                {
                    //cout << i << endl;
                    best = float(i);
                    return ;
                }
            }
            //cout << find(red, 1) << endl;
            best = float(find(red, 1));
        }
    }
        //若谷仓中两个红球、一个蓝球，则放在此仓中
    else if (red_max == 2 && blue_max == 1)
    {
        for (int i = 1; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                //cout << i << endl;
                best = float(i);
                return ;
            }
        }
        //cout << find(red, 2) << endl;
        best = float(find(red, 2));
    }
        //若谷仓中没有红球、两个蓝球，则放在此仓中
    else if (red_max == 0 && blue_max == 2)
    {
        //cout << find(blue, 2) << endl;
        best = float(find(blue, 2));
    }
        //若谷仓中有一个红球、两个蓝球，则放在此仓中
        //问题：若有一个仓中有一个红球和一个蓝球，则这个仓优先级更高
    else if (red_max == 1 && blue_max == 2)
    {
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                //cout << i << endl;
                best = float(i);
                return ;
            }
        }
        //cout << find(blue, 2) << endl;
        best = float(find(blue, 2));
    }
        //若谷仓中有两个红球、两个蓝球，则放在红仓中
    else if (red_max == 2 && blue_max == 2)
    {
        for (int i = 0; i < 5; i++)
        {
            if (red[i] == 1 && blue[i] == 1)
            {
                //cout << i << endl;
                best = float(i);
                return ;
            }
        }
        //cout << find(red, 2) << endl;
        best = float(find(red, 2));
    }
}

void read_txt()
{
    ifstream infile;
    infile.open("/home/robot/桌面/2024_ball_xioabai/Mask_Detector/data.txt");
    char str[50][10];
    int i = 0;
    while(infile.getline(str[i], 10, ' '))
    {
        //redball
        if(str[i][0] == 'r')
        {
            i++;
            infile.getline(str[i], 10, ' ');
            tem_red[num_red] = stoi(str[i]);
            num_red++;
        }
        //blueball
        if(str[i][0] == 'b' && str[i][1] == 'l')
        {
            i++;
            infile.getline(str[i], 10, ' ');
            tem_blue[num_blue] = stoi(str[i]);
            num_blue++;
        }
        //barn
        if(str[i][0] == 'b' && str[i][1] == 'a')
        {
            i++;
            infile.getline(str[i], 10, ' ');
            barn[num_barn].x1 = stoi(str[i]);
            i++;
            infile.getline(str[i], 10, ' ');
            barn[num_barn].x2 = stoi(str[i]);
            num_barn++;
        }
        i++;
    }
}

void tell()
{
    //谷仓排序
    Barn tem;
    for(int i = 0; i < 4; i++)
    {
        for(int j = i+1; j < 5; j++)
        {
            if(barn[i].x1 > barn[j].x1)
            {
                tem = barn[i];
                barn[i] = barn[j];
                barn[j] = tem;
            }
        }
    }
    //遍历检测到的所有红球
    for(int i = 0; i < num_red; i++)
    {
        if(tem_red[i] > barn[0].x1 && tem_red[i] < barn[0].x2)
        {
            red[0]++;
        }
        else if(tem_red[i] > barn[1].x1 && tem_red[i] < barn[1].x2)
        {
            red[1]++;
        }
        else if(tem_red[i] > barn[2].x1 && tem_red[i] < barn[2].x2)
        {
            red[2]++;
        }
        else if(tem_red[i] > barn[3].x1 && tem_red[i] < barn[3].x2)
        {
            red[3]++;
        }
        else if(tem_red[i] > barn[4].x1 && tem_red[i] < barn[4].x2)
        {
            red[4]++;
        }
    }
    //遍历检测到的所有蓝球
    for(int i = 0; i < num_blue; i++)
    {
        if(tem_blue[i] > barn[0].x1 && tem_blue[i] < barn[0].x2)
        {
            blue[0]++;
        }
        else if(tem_blue[i] > barn[1].x1 && tem_blue[i] < barn[1].x2)
        {
            blue[1]++;
        }
        else if(tem_blue[i] > barn[2].x1 && tem_blue[i] < barn[2].x2)
        {
            blue[2]++;
        }
        else if(tem_blue[i] > barn[3].x1 && tem_blue[i] < barn[3].x2)
        {
            blue[3]++;
        }
        else if(tem_blue[i] > barn[4].x1 && tem_blue[i] < barn[4].x2)
        {
            blue[4]++;
        }
    }
}

void refresh()
{
    for(int i = 0; i < 5; i++)
    {
        red[i] = 0;
        blue[i] = 0;
    }
    for(int i = 0; i < num_barn; i++)
    {
        barn[i].x1 = -1;
        barn[i].x2 = -1;
    }
    for(int i = 0; i < num_red; i++)
    {
        tem_red[i] = 0;
    }
    for(int i = 0; i < num_blue; i++)
    {
        tem_blue[i] = 0;
    }
    num_red = 0;
    num_blue = 0;
    num_barn = 0;
}

void yolo(cv::Mat mat)
{
    yolo_main(mat);
    //读取数据
    read_txt();
    //五个谷仓都能看到时
    if(num_barn == 5)
    {
        tell();
        //策略判断
        strategy();
        //传递数据
        cout<<best<<endl;
        //send_data(best);
    }
    //刷新数据
    refresh();
    cv::imshow("OAK", mat);
}
#endif