//TODO有个内存访问问题，我怀疑是因为取点的时候取到图像外了，需要在取深度时加一个限定！！！！！

#include "include/Tower_Detection.h"
#include "include/serial_port.h"
#include <thread> //管理多线程

using namespace cv;
using namespace std;
double catch_moudle = 0;
extern bool mocalun;

std::mutex mtx;
void proc1() //读获取后的数的线程
{
        while (true)
        {
                int catch_m = catch_moudle;
                mtx.try_lock();
                catch_moudle = read_data();
                if (catch_moudle == 0) //忽视掉收到的0
                        catch_moudle = catch_m;
                mtx.unlock();
        }
}

// void proc2()
// {
//         Mat M1(2, 2, CV_8UC1, Scalar(0));
//         namedWindow("control", 0);
//         while (true)
//         {
//                 imshow("control", M1);
//                 int temp = cv::waitKey(10);
//                 //cout << temp << endl;
//                 switch (temp)
//                 {
//                 case 119: //w
//                         catch_moudle = 2;
//                         break;
//                 case 113: //q
//                         catch_moudle = 1;
//                         break;
//                 case 101: //e
//                         catch_moudle = 3;
//                         break;
//                 case 32: // space
//                         catch_moudle = 1000;
//                         pthread_exit(NULL);
//                         break;
//                 }
//         }
// }

Tower_Detection::Tower_Detection()
{
        std::string engine_name = "/home/robo/Desktop/RC_2023_ER/engine/best_i.engine"; //best1用的是int8量化
        engine_init(engine_name);
}

int Tower_Detection::process()
{
        thread th1(proc1);
        th1.detach();
        // thread th2(proc2);
        // th2.detach();
        /*
		找到并打开 Azure Kinect 设备
	*/
        k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
        k4a::capture capture;

        /**********************摄像头***********************/
        const uint32_t device_count = k4a::device::get_installed_count();
        if (device_count == 0)
        {
                printf("No K4A devices found\n");
                return 0;
        }

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        //config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
        config.synchronized_images_only = true;
        device.start_cameras(&config); //开启摄像头
                                       /*校准深度和颜色*/
        k4a_calibration_t calibration = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P);

        k4a::transformation transformation = k4a::transformation(calibration);

        /*定义image变量，接收相机传过来的原始数据*/
        k4a::image rgbImage;
        k4a::image depthImage;

        k4a::image transformed_depth_image = NULL;
        /*将image转化为Mat类型的接收的容器*/
        //cv::Mat color_frame;
        cv::Mat depth_frame;
        cv::Mat transformed_depth_frame;
        Mat src, src_no_alpha;
        clock_t startTime, endTime; //计算时间
        bool switch_one = false, switch_two = false;
        Tower Tower;
        double time_true = 0;
        namedWindow("srcImage", 0);

        while (true)
        {
                if (device.get_capture(&capture, std::chrono::milliseconds(0)))
                {

                        rgbImage = capture.get_color_image();
                        //depthImage = capture.get_depth_image();
                        /*转换颜色和深度图像*/
                        //transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
                        /*将颜色和深度图像装入Mat容器中*/
                        src = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, rgbImage.get_buffer());
                        //depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());

                        //transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());

                        depthImage = capture.get_depth_image();
                        transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
                        depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());
                        transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());

                        cv::cvtColor(src, src_no_alpha, cv::COLOR_BGRA2BGR);

                        if (src_no_alpha.empty())
                        {
                                continue;
                        }
                        double timeStart = (double)getTickCount();
                        startTime = clock();                                              //计
                        std::vector<Yolo::Detection> Tower_box = yolo_main(src_no_alpha); //放入模型
                        endTime = clock();                                                //计时结束
                        double nTime = ((double)getTickCount() - timeStart) / getTickFrequency();

                        if (catch_moudle != 0)
                        {
                                Tower.Control_method_one(Tower_box, src_no_alpha, transformed_depth_frame, &Tower);
                        }
                        imshow("srcImage", src_no_alpha);
                        //cout<<src_no_alpha.size()<<endl;
                        src.release();
                        src_no_alpha.release();
                        capture.reset();

                        if (cv::waitKey(10) == ' ')
                        {

                                break;
                        }
                }
        }
        cv::destroyAllWindows();
        device.close();
}
int main()
{
        Tower_Detection Tower_Detection_;
        Tower_Detection_.process();

        return 0;
}