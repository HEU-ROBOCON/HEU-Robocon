#ifndef CUBE_CLASSIFIER_HPP
#define CUBE_CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

class CubeClassifier {
private:
    Ort::Env env;
    Ort::Session session;
    vector<const char*> input_node_names = {"input"};
    vector<const char*> output_node_names = {"output"};
    
    // 注意：这里不用写死 input_dims，我们在 predict 里动态生成
    
public:
    CubeClassifier(const string& model_path) 
        : env(ORT_LOGGING_LEVEL_WARNING, "BatchClassifier"), session(nullptr) 
    {
        Ort::SessionOptions session_options;
        // session_options.EnableProfiling("my_model_test"); 
        session_options.SetIntraOpNumThreads(4); // 根据你的CPU核数调整，ROS里通常设为1-2防止占满CPU
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        try {
            session = Ort::Session(env, model_path.c_str(), session_options);
        } catch (const Ort::Exception& e) {
            cerr << "模型加载失败: " << e.what() << endl;
        }
    }

    /**
     * @brief 批量推理
     * @param batch_imgs 一组切下来的小灰度图 (100x100)
     * @return 对应的一组类别ID
     */
    vector<int> predict_batch(const vector<Mat>& batch_imgs) {
        if (batch_imgs.empty()) return {};

        size_t batch_size = batch_imgs.size();
        size_t h = 100;
        size_t w = 100;
        size_t input_tensor_size = batch_size * 1 * h * w;

        // 1. 准备打平的输入数据
        vector<float> input_tensor_values(input_tensor_size);
        float mean = 0.5f;
        float std = 0.5f;

        // 并行填充数据 
        // int64 kaishi = cv::getTickCount();
        cv::parallel_for_(cv::Range(0, batch_size), [&](const cv::Range& r) {
    for (int b = r.start; b < r.end; ++b) {
        const Mat& img = batch_imgs[b];

        Mat gray;
        if (img.channels() == 3)
            cvtColor(img, gray, COLOR_BGR2GRAY);
        else
            gray = img;

        const uchar* ptr = gray.ptr<uchar>(0);

        size_t base = b * (h * w);

        for (int i = 0; i < h; i++) {
            const uchar* row = gray.ptr<uchar>(i);
            size_t row_base = base + i * w;

            for (int j = 0; j < w; j++) {
                float pixel = static_cast<float>(row[j]);
                input_tensor_values[row_base + j] =
                    (pixel / 255.0f - mean) / std;
            }
        }
    }
});

        // int64 jieshu = cv::getTickCount();
        // cout<<jieshu-kaishi<<endl;
        // 2. 动态定义 Input Shape: [N, 1, 100, 100]
        vector<int64_t> input_dims = {static_cast<int64_t>(batch_size), 1, 100, 100};

        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, 
            input_tensor_values.data(), 
            input_tensor_size, 
            input_dims.data(), 
            input_dims.size()
        );

        // 3. 执行一次推理
        auto output_tensors = session.Run(
            Ort::RunOptions{nullptr}, 
            input_node_names.data(), 
            &input_tensor, 
            1, 
            output_node_names.data(), 
            1
        );

        // 4. 解析结果 (输出是 [N, 32])
        float* floatarr = output_tensors[0].GetTensorMutableData<float>();
        vector<int> results;
        results.reserve(batch_size);

        for (size_t b = 0; b < batch_size; ++b) {
            // 指向当前这张图的 32 个分数起始位置
            float* current_logits = floatarr + (b * 32);
            
            int best_id = 0;
            float max_score = -10000.0f;
            for (int k = 0; k < 32; ++k) {
                if (current_logits[k] > max_score) {
                    max_score = current_logits[k];
                    best_id = k;
                }
            }
            results.push_back(best_id);
        }

        return results;
    }
};

#endif