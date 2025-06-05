#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

// OpenCV Includes - OpenCV 相关头文件
#include <opencv2/opencv.hpp>

// Your Project Includes - 您的项目头文件 (确保它们在 include/ 目录下)
#include <fstream>
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>
#include "cuda_utils.h"
#include "logging.h"
#include "model.h"
#include "postprocess.h"
#include "preprocess.h"
#include "utils.h"

// Custom Messages - 自定义消息头文件
#include "yolo_realsense_kinect/DetectedObject3D_kinect_circle.h"
#include "yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h"

// TensorRT Includes - TensorRT 头文件
#include "NvInfer.h"

// --- 全局变量和函数声明 (假设这些在您的头文件或链接库中) ---
Logger gLogger; // TensorRT 日志记录器
using namespace nvinfer1;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1; // 输出大小计算

// --- 函数声明 (您需要提供这些函数的实现或链接对应的 .cpp 文件) ---
// IHostMemory* buildEngineYolo11Det(IBuilder* builder, IBuilderConfig* config, DataType dt, const std::string& wts_path, float& gd, float& gw, int& max_channels, std::string& type);
// std::map<std::string, nvinfer1::Weights> loadWeights(const std::string file);
void cuda_decode(float* predict, int num_bboxes, float confidence_threshold, float* parray, int max_objects, cudaStream_t stream);
void cuda_nms(float* parray, float nms_threshold, int max_objects, cudaStream_t stream);
void cuda_batch_preprocess(std::vector<cv::Mat>& img_batch, float* dst, int dst_width, int dst_height, cudaStream_t stream);
void cuda_preprocess_init(int max_image_size);
void cuda_preprocess_destroy();
void batch_nms(std::vector<std::vector<Detection>>& batch_res, float* output, int batch_size, int output_size, float conf_thresh, float nms_thresh);
void batch_process(std::vector<std::vector<Detection>>& res_batch, const float* decode_ptr_host, int batch_size, int bbox_element, const std::vector<cv::Mat>& img_batch);
void draw_bbox(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch);
IRuntime* createInferRuntime(ILogger& logger);
IBuilder* createInferBuilder(ILogger& logger);
// --- 结束 ---


class YoloDetectorNode {
private:
    ros::NodeHandle nh_; // ROS 节点句柄
    image_transport::ImageTransport it_; // ROS 图像传输句柄

    // --- ROS 消息同步器 ---
    // 用于同步彩色图像、深度图像和相机信息，确保处理的是同一帧的数据
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    message_filters::Synchronizer<MySyncPolicy> sync_;

    // --- ROS 发布器 ---
    ros::Publisher pub_detections_3d_; // 发布 3D 检测结果
    ros::Publisher pub_markers_;       // 发布 Rviz 可视化标记
    image_transport::Publisher pub_debug_image_; // 发布带 BBox 的调试图像

    // --- TensorRT 相关变量 ---
    std::string engine_name_;      // TensorRT 引擎文件名
    std::string cuda_post_process_; // 后处理方式 ('c' for CPU, 'g' for GPU)
    float conf_thresh_;            // 置信度阈值
    float nms_thresh_;             // NMS (非极大值抑制) 阈值

    IRuntime* runtime_ = nullptr;     // TensorRT 运行时
    ICudaEngine* engine_ = nullptr;   // TensorRT 引擎
    IExecutionContext* context_ = nullptr; // TensorRT 执行上下文
    cudaStream_t stream_;             // CUDA 流
    int model_bboxes_;                // 模型输出的 BBox 数量

    // --- GPU/CPU 缓冲区 ---
    float* device_buffers_[2];         // GPU 上的输入输出缓冲区指针数组
    float* output_buffer_host_ = nullptr; // CPU 上的输出缓冲区 (用于 CPU NMS)
    float* decode_ptr_host_ = nullptr;    // CPU 上的解码/NMS 结果缓冲区 (用于 GPU NMS)
    float* decode_ptr_device_ = nullptr;  // GPU 上的解码/NMS 结果缓冲区 (用于 GPU NMS)

    // --- 相机模型和状态 ---
    image_geometry::PinholeCameraModel cam_model_; // 针孔相机模型，用于 3D 转换
    bool cam_info_received_ = false;              // 
    
    // --- 常量定义 ---
    float circle_distance_max_ = 6.0; // 最大距离 (米)
    float circle_distance_min_ = 2.0; // 最小距离 (米)

    // --- 随即数引擎 ---
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    /**
     * @brief 反序列化 TensorRT 引擎文件
     * @param engine_name 引擎文件路径
     * @param runtime TensorRT 运行时指针 (输出)
     * @param engine TensorRT 引擎指针 (输出)
     * @param context TensorRT 执行上下文指针 (输出)
     */
    void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
        std::ifstream file(engine_name, std::ios::binary);
        if (!file.good()) {
            ROS_ERROR("Read %s failed!", engine_name.c_str());
            ros::shutdown(); // 如果无法读取引擎，则关闭节点
            return;
        }
        size_t size = 0;
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        char* serialized_engine = new char[size];
        file.read(serialized_engine, size);
        file.close();

        //*runtime = createInferRuntime(gLogger);
        *runtime = nvinfer1::createInferRuntime(gLogger);
        assert(*runtime);
        *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
        assert(*engine);
        *context = (*engine)->createExecutionContext();
        assert(*context);
        delete[] serialized_engine;
        ROS_INFO("The engine file %s was deserialized successfully.", engine_name.c_str());
    }

    /**
     * @brief 准备 GPU 和 CPU 缓冲区
     */
    void prepare_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                        float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                        std::string cuda_post_process) {
        assert(engine->getNbBindings() == 2); // 确认有输入和输出两个绑定
        const int inputIndex = engine->getBindingIndex(kInputTensorName);
        const int outputIndex = engine->getBindingIndex(kOutputTensorName);
        assert(inputIndex == 0);
        assert(outputIndex == 1);

        // 分配 GPU 内存
        CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));

        // 根据后处理方式分配 CPU 或 GPU 内存
        if (cuda_post_process == "c") {
            *output_buffer_host = new float[kBatchSize * kOutputSize];
        } else if (cuda_post_process == "g") {
            if (kBatchSize > 1) {
                 ROS_ERROR("GPU post-processing doesn't yet support batch sizes larger than 1.");
                 ros::shutdown();
            }
            *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
            CUDA_CHECK(cudaMalloc((void**)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
        }
         ROS_INFO("GPU/CPU buffers are ready.");
    }

    /**
     * @brief 执行 TensorRT 推理和后处理
     */
    void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
           float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process) {
        auto start = std::chrono::system_clock::now();
        // 异步执行推理
        context.enqueueV2(buffers, stream, nullptr);

        // 根据后处理方式执行操作
        if (cuda_post_process == "c") {
            // 将 GPU 输出异步复制回 CPU
            CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
        } else if (cuda_post_process == "g") {
            // 在 GPU 上执行解码和 NMS
            CUDA_CHECK(cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
            cuda_decode((float*)buffers[1], model_bboxes, conf_thresh_, decode_ptr_device, kMaxNumOutputBbox, stream);
            cuda_nms(decode_ptr_device, nms_thresh_, kMaxNumOutputBbox, stream);
            // 将 GPU NMS 结果异步复制回 CPU
            CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost, stream));
        }
        // 同步 CUDA 流，等待所有操作完成
        CUDA_CHECK(cudaStreamSynchronize(stream));
        auto end = std::chrono::system_clock::now();
        ROS_DEBUG("Inference and post-processing time: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    }

    /**
     * @brief 将网络输出的 BBox 坐标 (相对于 kInputW x kInputH) 转换回原始图像坐标
     * @param img 原始图像 (用于获取原始尺寸)
     * @param bbox 检测到的 BBox [x1, y1, x2, y2] (相对于 kInputW x kInputH)
     * @return 转换后的 cv::Rect
     */
    cv::Rect get_rect(const cv::Mat& img, const float bbox[4]) {
        int w = img.cols;
        int h = img.rows;
        // 计算 letterbox 缩放比例和填充
        float r_w = kInputW / (w * 1.0f);
        float r_h = kInputH / (h * 1.0f);
        float r = std::min(r_w, r_h);
        int new_w = static_cast<int>(r * w);
        int new_h = static_cast<int>(r * h);
        float x_pad = (kInputW - new_w) / 2.0f;
        float y_pad = (kInputH - new_h) / 2.0f;

        // BBox 坐标 (来自模型输出，假设是 640x640 上的 [x1, y1, x2, y2])
        float x1 = bbox[0];
        float y1 = bbox[1];
        float x2 = bbox[2];
        float y2 = bbox[3];

        // 逆向计算，从 640x640 映射回原始图像
        float img_x1 = (x1 - x_pad) / r;
        float img_y1 = (y1 - y_pad) / r;
        float img_x2 = (x2 - x_pad) / r;
        float img_y2 = (y2 - y_pad) / r;

        // 裁剪并转换为整数 cv::Rect
        int final_x1 = std::max(0, static_cast<int>(img_x1));
        int final_y1 = std::max(0, static_cast<int>(img_y1));
        int final_x2 = std::min(w, static_cast<int>(img_x2));
        int final_y2 = std::min(h, static_cast<int>(img_y2));

        return cv::Rect(final_x1, final_y1, final_x2 - final_x1, final_y2 - final_y1);
    }

    /**
     * @brief 对图像进行滤波
     * @param img  输入图片
     * @param blurSigma 滤波sigma
     */
    void Picture_Filter(cv::Mat& img, int blurSigma) {
        cv::Mat blurred;
        cv::GaussianBlur(img, blurred, cv::Size(0, 0), blurSigma);
        cv::Mat result = img + 1.5 * (img - blurred);
        img = cv::max(cv::min(result, 255), 0);
    }
    /**
     * @brief 深度采样函数
     * @param depth_image 深度图像
     * @param box 目标框
     * @return 采样的深度值
     */
    double depth_sample(const cv::Mat& depth_image, const cv::Rect& box) {
        int x0 = (2*box.x+ box.width)/2;
        int y0 = (2*box.y+ box.height)/2;
        int radius = std::min(box.width , box.height) < 3 ? std::min( box.width , box.height) : 3 ;
        float angle = 0;
        float sin_value = 0;
        float cos_value = 0;
        double depth = 0.0;
        double sum = 0.0;
        int count = 6;
        for (int i = 0; i < count; i++) {
            angle = dis(gen);
            sin_value = std::sin(angle);
            cos_value = std::cos(angle);
            if(depth_image.type() == CV_16UC1){
            depth = static_cast<double>(depth_image.at<uint16_t>(static_cast<int>(y0+radius*sin_value), static_cast<int>(x0+radius*cos_value)));
            if(depth > circle_distance_min_*1000 && depth < circle_distance_max_*1000)
            sum += depth;
            }
            else if(depth_image.type() == CV_32FC1){
            depth = static_cast<double>(depth_image.at<float>(static_cast<int>(y0+radius*sin_value), static_cast<int>(x0+radius*cos_value)));
            if(depth >circle_distance_min_ && depth < circle_distance_max_)
            sum += depth;
            }
        }
        return sum / count;
    }

    /**
     * @brief 图像消息的回调函数
     * @param color_msg 彩色图像消息
     * @param depth_msg 深度图像消息
     * @param info_msg 相机信息消息
     */
    void image_callback(const sensor_msgs::ImageConstPtr& color_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& info_msg) {
        auto start = std::chrono::high_resolution_clock::now();
        // 如果是第一次收到相机信息，则加载相机模型
        if (!cam_info_received_) {
            cam_model_.fromCameraInfo(info_msg);
            cam_info_received_ = true;
            ROS_INFO("The camera information has been received and the model has been loaded.");
        }

        // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV Mat
        cv_bridge::CvImagePtr color_ptr, depth_ptr;
        try {
            color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
            // 确保深度图像格式与 RealSense 发布的一致 (通常是 16UC1 - 毫米)
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge Exceptions:%s", e.what());
            return;
        }

        cv::Mat color_image = color_ptr->image;
        cv::Mat depth_image = depth_ptr->image;
        Picture_Filter(color_image, 1.7);
        std::vector<cv::Mat> img_batch = {color_image};

        // --- 执行 YOLO 推理流程 ---
        // 1. 预处理
        cuda_batch_preprocess(img_batch, device_buffers_[0], kInputW, kInputH, stream_);
        // 2. 推理
        infer(*context_, stream_, (void**)device_buffers_, output_buffer_host_, kBatchSize, decode_ptr_host_,
              decode_ptr_device_, model_bboxes_, cuda_post_process_);
        // 3. 后处理 (NMS)
        std::vector<std::vector<Detection>> res_batch;
        if (cuda_post_process_ == "c") {
            batch_nms(res_batch, output_buffer_host_, img_batch.size(), kOutputSize, conf_thresh_, nms_thresh_);
        } else if (cuda_post_process_ == "g") {
            batch_process(res_batch, decode_ptr_host_, img_batch.size(), bbox_element, img_batch);
        }

        // --- 处理结果并发布 ---
        process_and_publish(res_batch, color_image, depth_image, color_msg->header);
    
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        ROS_INFO_STREAM("[Perf] Image processing time: " << duration.count() << "ms," << "fps: " << 1000.0 / duration.count());
    }

    /**
     * @brief 处理检测结果，计算 3D 坐标并发布消息
     * @param res_batch 检测结果批次
     * @param color_image 彩色图像
     * @param depth_image 深度图像
     * @param header 消息头 (用于时间戳和坐标系)
     */
    void process_and_publish(const std::vector<std::vector<Detection>>& res_batch,
                             const cv::Mat& color_image, const cv::Mat& depth_image,
                             const std_msgs::Header& header) {

        yolo_realsense_kinect::DetectedObject3DArray_kinect_circle detections_msg; // 创建 3D 检测数组消息
        detections_msg.header = header; // 设置消息头

        visualization_msgs::MarkerArray marker_array; // 创建 Rviz 标记数组消息
        cv::Mat debug_image = color_image.clone(); // 创建用于绘制 BBox 的调试图像

        int marker_id = 0; // 用于 Rviz 标记的唯一 ID
        for (const auto& detections : res_batch) { // 遍历批次 (虽然我们通常批次大小为 1)
            for (const auto& det : detections) { // 遍历当前图像的所有检测结果
                // 将 float[4] 转换为 cv::Rect，并进行坐标系转换
                cv::Rect r = get_rect(color_image, det.bbox);

                // --- 1. 计算 3D 坐标 ---
                cv::Point3d point3d_cv;
                bool valid_depth = false;
                // 获取 BBox 中心点
                int cx = r.x + r.width / 2;
                int cy = r.y + r.height / 2;

                // 确保中心点在图像范围内
                if (cx >= 0 && cx < depth_image.cols && cy >= 0 && cy < depth_image.rows && r.area() > 190 && det.conf > 0.50) {
                    float depth_m = 0.0f;
                    // 从深度图中获取深度值 (通常是 uint16_t，单位毫米)
                    if (depth_image.type() == CV_16UC1) {
                       // uint16_t depth_mm = depth_image.at<uint16_t>(cy, cx);
                        depth_m =static_cast<float>(depth_sample(depth_image, r) / 1000.0f); // 转换为米
                    } else if (depth_image.type() == CV_32FC1) { // 如果是 float 类型
                        depth_m =static_cast<float>(depth_sample(depth_image, r));
                    }

                    // 检查深度值是否有效 (例如，大于 0.1 米，小于 3.5 米)
                    if (depth_m > circle_distance_min_ && depth_m < circle_distance_max_) {
                        cv::Point2d pixel_uv(cx, cy);
                        // 使用相机模型将 2D 像素点 + 深度 转换为 3D 坐标
                        point3d_cv = cam_model_.projectPixelTo3dRay(pixel_uv) * depth_m;
                        valid_depth = true;
                    }
                }

                // --- 2. 填充自定义消息 ---
                yolo_realsense_kinect::DetectedObject3D_kinect_circle obj;
                obj.header = header;
                obj.id = static_cast<uint32_t>(det.class_id);
                // **注意**: 您应该使用一个标签映射来获取真实的类名
                obj.class_name = "Class_" + std::to_string(static_cast<int>(det.class_id));
                obj.confidence = det.conf;
                obj.bbox_2d.x_offset = r.x;
                obj.bbox_2d.y_offset = r.y;
                obj.bbox_2d.height = r.height;
                obj.bbox_2d.width = r.width;

                if (valid_depth) {
                    obj.point_3d.x = point3d_cv.x;
                    obj.point_3d.y = point3d_cv.y;
                    obj.point_3d.z = point3d_cv.z;
                } else { // 如果深度无效，则设置为 NaN
                    // obj.point_3d.x = std::numeric_limits<double>::quiet_NaN();
                    // obj.point_3d.y = std::numeric_limits<double>::quiet_NaN();
                    // obj.point_3d.z = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }
                detections_msg.detections.push_back(obj);

                // --- 3. 在调试图像上绘制 BBox 和标签 ---
                cv::rectangle(debug_image, r, cv::Scalar(0, 255, 0), 2);
                cv::putText(debug_image, obj.class_name + ": " + std::to_string(obj.confidence).substr(0, 4),
                            cv::Point(r.x, r.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                cv::putText(debug_image,"box_area"+std::to_string(r.area()),cv::Point(r.x, r.y+r.height+5),
            cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1);
                if (valid_depth) {
                            std::string point_text = "X:" + std::to_string(obj.point_3d.x) + 
                             " Y:" + std::to_string(obj.point_3d.y) + 
                             " Z:" + std::to_string(obj.point_3d.z);
                            cv::putText(debug_image, point_text, cv::Point(r.x, r.y + r.height + 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
                                 }
                // --- 4. 创建 Rviz 可视化标记 (如果 3D 坐标有效) ---
                if (valid_depth) {
                    // 创建球体标记
                    visualization_msgs::Marker marker;
                    marker.header = header;
                    marker.ns = "yolo_kinect_circle_detections";
                    marker.id = marker_id++;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position = obj.point_3d;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
                    marker.color.a = 0.8; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                    marker.lifetime = ros::Duration(0.5); // 标记持续时间
                    marker_array.markers.push_back(marker);

                    // 创建文本标记
                    visualization_msgs::Marker text_marker = marker;
                    text_marker.id = marker_id++;
                    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    text_marker.text = obj.class_name;
                    text_marker.pose.position.z += 0.1; // 将文本放在球体上方
                    text_marker.scale.z = 0.1; // 文本高度
                    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0;
                    marker_array.markers.push_back(text_marker);
                }
            }
        }

        // --- 发布所有消息 ---
        pub_detections_3d_.publish(detections_msg);
        pub_markers_.publish(marker_array);

        // 发布调试图像
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
        pub_debug_image_.publish(msg);
    }

public:
    /**
     * @brief 构造函数：初始化 ROS 节点、参数、TensorRT 和订阅/发布器
     */
    YoloDetectorNode() : nh_("~"), it_(nh_), // 初始化节点句柄和图像传输，"~"表示私有命名空间
      sync_(MySyncPolicy(10), color_sub_, depth_sub_, info_sub_), // 初始化同步器，队列大小为 10
      gen(rd()), dis(0.0f, 2*M_PI)
    {
        cudaSetDevice(kGpuId); // 设置使用的 GPU ID

        // --- 从 ROS 参数服务器获取参数 ---
        nh_.param<std::string>("engine_name", engine_name_, "yolo11n.engine");
        nh_.param<std::string>("cuda_post_process", cuda_post_process_, "g");
        nh_.param<float>("conf_thresh", conf_thresh_, kConfThresh);
        nh_.param<float>("nms_thresh", nms_thresh_, kNmsThresh);
        nh_.param<float>("circle_distance_max", circle_distance_max_, 3.5);
        nh_.param<float>("circle_distance_min", circle_distance_min_, 0.2);

        // 获取包路径并构建引擎文件的完整路径
        ROS_INFO("engine_name: %s", engine_name_.c_str());
        ROS_INFO("CUDA: %s", cuda_post_process_.c_str());
        ROS_INFO("conf_thresh_: %f", conf_thresh_);
        ROS_INFO("nms_thresh_: %f", nms_thresh_);

        // --- 初始化 TensorRT ---
        deserialize_engine(engine_name_, &runtime_, &engine_, &context_);
        if (!engine_) {
            ROS_FATAL("deserialize_engine Failed!");
            return;
        }
        CUDA_CHECK(cudaStreamCreate(&stream_)); // 创建 CUDA 流
        cuda_preprocess_init(kMaxInputImageSize); // 初始化预处理
        auto out_dims = engine_->getBindingDimensions(1); // 获取输出维度
        model_bboxes_ = out_dims.d[0]; // 获取模型输出 BBox 数量
        prepare_buffer(engine_, &device_buffers_[0], &device_buffers_[1], &output_buffer_host_,
                       &decode_ptr_host_, &decode_ptr_device_, cuda_post_process_); // 准备缓冲区

        // --- 设置 ROS 发布器 ---
        pub_detections_3d_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_circle>("/yolo/kincet/circle/detections_3d", 10);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/yolo/kinect/circle/markers", 10);
        pub_debug_image_ = it_.advertise("/yolo/kinect/circle/debug_image", 1);

        // --- 设置 ROS 订阅器 ---
        std::string color_topic, depth_topic, info_topic;
        nh_.param<std::string>("color_topic", color_topic, "/k4a/color/image_raw"); // 默认值也更新
        nh_.param<std::string>("depth_topic", depth_topic, "/k4a/aligned_depth_to_color/image_raw"); // 默认值也更新
        nh_.param<std::string>("info_topic", info_topic, "/k4a/color/camera_info"); // 默认值也更新

        color_sub_.subscribe(nh_, color_topic, 1);
        depth_sub_.subscribe(nh_, depth_topic, 1);
        info_sub_.subscribe(nh_, info_topic, 1);

        // 注册同步回调函数
        sync_.registerCallback(boost::bind(&YoloDetectorNode::image_callback, this, _1, _2, _3));

        ROS_INFO("Realsense ROS node initialized successfully. Subscribed to the following topics:\n\tColor: %s\n\tDepth: %s\n\tInfo: %s",
                 color_topic.c_str(), depth_topic.c_str(), info_topic.c_str());
    }

    /**
     * @brief 析构函数：释放所有分配的资源
     */
    ~YoloDetectorNode() {
        cudaStreamDestroy(stream_);
        CUDA_CHECK(cudaFree(device_buffers_[0]));
        CUDA_CHECK(cudaFree(device_buffers_[1]));
        CUDA_CHECK(cudaFree(decode_ptr_device_));
        delete[] decode_ptr_host_;
        delete[] output_buffer_host_;
        cuda_preprocess_destroy();
        delete context_;
        delete engine_;
        delete runtime_;
        ROS_INFO("Realsnese ROS node has been shut down and resources released.");
    }
};

/**
 * @brief 主函数：初始化 ROS 并启动节点
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_realsense_node"); // 初始化 ROS 节点
    YoloDetectorNode node; // 创建节点对象
    ros::spin(); // 进入 ROS 事件循环
    return 0;
}