#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

using namespace std;
using namespace cv;

class OnnxClassifier {
private:
    Ort::Env env;
    Ort::Session session;
    vector<const char*> input_node_names = {"input"};
    vector<const char*> output_node_names = {"output"};
    vector<int64_t> input_dims = {1, 1, 100, 100}; // NCHW

public:
    OnnxClassifier(const string& model_path) 
        : env(ORT_LOGGING_LEVEL_WARNING, "GridTester"), session(nullptr) 
    {
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
        try {
            session = Ort::Session(env, model_path.c_str(), session_options);
        } catch (const Ort::Exception& e) {
            cerr << "模型加载失败: " << e.what() << endl;
            exit(-1);
        }
    }

    int predict(const Mat& cell_img) {
        if (cell_img.empty()) return -1;

        // 1. 预处理: 转灰度 -> 归一化
        Mat gray;
        if (cell_img.channels() == 3) {
            cvtColor(cell_img, gray, COLOR_BGR2GRAY);
        } else {
            gray = cell_img;
        }

        // 检查是否是纯黑图 (这是为了跳过拼图里的空白填充区域)
        // 如果平均像素值极低，认为是无效区域，返回 -1
        Scalar mean_val = mean(gray);
        if (mean_val[0] < 5.0) return -1; 

        size_t input_tensor_size = 100 * 100;
        vector<float> input_tensor_values(input_tensor_size);
        float mean = 0.5f;
        float std = 0.5f;

        for (int i = 0; i < 100; i++) {
            for (int j = 0; j < 100; j++) {
                float pixel = static_cast<float>(gray.at<uchar>(i, j));
                input_tensor_values[i * 100 + j] = (pixel / 255.0f - mean) / std;
            }
        }

        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_size, 
            input_dims.data(), input_dims.size()
        );

        auto output_tensors = session.Run(
            Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, 
            output_node_names.data(), 1
        );

        float* floatarr = output_tensors[0].GetTensorMutableData<float>();
        int best_class_id = 0;
        float max_score = -10000.0f;
        
        for (int i = 0; i < 32; i++) {
            if (floatarr[i] > max_score) {
                max_score = floatarr[i];
                best_class_id = i;
            }
        }
        return best_class_id;
    }
};

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "用法: ./test_grid [模型路径.onnx] [拼图路径.jpg]" << endl;
        return -1;
    }

    string model_path = argv[1];
    string img_path = argv[2];

    // 1. 加载模型
    OnnxClassifier classifier(model_path);

    // 2. 读取拼图大图
    Mat grid_img = imread(img_path);
    if (grid_img.empty()) {
        cerr << "无法读取图片: " << img_path << endl;
        return -1;
    }

    // 确保图片尺寸合理 (至少要有 100x100)
    if (grid_img.cols < 100 || grid_img.rows < 100) {
        cerr << "图片尺寸太小，不是拼图文件。" << endl;
        return -1;
    }

    cout << "开始处理拼图: " << img_path << " (" << grid_img.cols << "x" << grid_img.rows << ")" << endl;

    int cell_size = 100;
    Mat result_view = grid_img.clone(); // 复制一份用于画图
    
    int processed_count = 0;

    // 3. 双重循环拆解网格
    for (int y = 0; y <= grid_img.rows - cell_size; y += cell_size) {
        for (int x = 0; x <= grid_img.cols - cell_size; x += cell_size) {
            
            // 定义 ROI (感兴趣区域)
            Rect roi(x, y, cell_size, cell_size);
            
            // 扣下小图
            Mat cell = grid_img(roi);

            // 推理
            int class_id = classifier.predict(cell);

            // 如果返回 -1，说明是纯黑填充区，跳过
            if (class_id == -1) continue;

            // --- 可视化 ---
            // 1. 画个框
            rectangle(result_view, roi, Scalar(0, 255, 0), 1);
            
            // 2. 写上数字 ID
            string label = to_string(class_id);
            
            // 为了字迹清晰，加个黑色描边
            Point text_pos(x + 10, y + 60);
            putText(result_view, label, text_pos, FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 0), 5); // 粗黑描边
            putText(result_view, label, text_pos, FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 255, 255), 2); // 黄色字芯

            processed_count++;
            cout << "Cell (" << x << "," << y << ") -> Class " << class_id << endl;
        }
    }

    cout << "处理完成，共识别 " << processed_count << " 个方块。" << endl;

    // 显示结果
    // 如果拼图太大屏幕放不下，缩放一下
    if (result_view.rows > 800) {
        float scale = 800.0f / result_view.rows;
        resize(result_view, result_view, Size(), scale, scale);
    }

    imshow("Grid Inference Result", result_view);
    waitKey(0);

    return 0;
}