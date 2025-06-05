#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/UInt8.h>
#include <serial/serial.h>
#include <tf/tf.h>
#include <cmath>
// 包含您自定义的Kinect Loop和Circle检测消息类型
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h>

#define RAD2DE 180.0/3.14159265358979
// 定义数据包结构体，确保无填充字节
struct __attribute__((packed)) DataPacket {
    uint8_t header[2] = {0x0A, 0x0D}; //2 bytes
    float kinect_loop_depth = 0; //4 bytes
    float kinect_loop_dx = 0;    //4 bytes
    float kinect_circle_x = 0;   //4 bytes
    float kinect_circle_y = 0;   //4 bytes
    float base_link_x = 0;       //4 bytes
    float base_link_y = 0;       //4 bytes
    float base_link_w = 0;       //4 bytes
    uint8_t tail[2] = {0x0B, 0x0C}; //2 bytes
    // 总大小：4 * 8 = 32 bytes
};

class SerialSender {
public:
    SerialSender(ros::NodeHandle& nh) : ser() {
        //初始化参数
        double rate;
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        nh.param<double>("baud_rate", rate, 115200);
        nh.param<std::string>("base_link_pub", base_link_pub, "/robot_pose_in_world" );
        nh.param<std::string>("kinect_loop_pub", kinect_loop_pub, "/kinect/loop/targets_in_world" );
        nh.param<std::string>("kinect_circle_pub", kinect_circle_pub, "/kinect/circle/targets_in_world" );
        baud_rate = static_cast<size_t>(rate);
        // 初始化订阅者
        sub_base_link = nh.subscribe(base_link_pub, 1, &SerialSender::sub_base_link_Callback, this);
        sub_kinect_loop = nh.subscribe(kinect_loop_pub, 1, &SerialSender::sub_kinect_loop_Callback, this);
        sub_kinect_circle = nh.subscribe(kinect_circle_pub, 1, &SerialSender::sub_kinect_circle_Callback, this);

        // 初始化数据包默认值
        data_packet.kinect_loop_depth = 100;
        data_packet.kinect_loop_dx = 0.0;
        data_packet.kinect_circle_x = 0.0;
        data_packet.kinect_circle_y = 0.0;
        data_packet.base_link_x = 0.0;
        data_packet.base_link_y = 0.0;
        data_packet.base_link_w = 0.0;
        

        //配置并打开串口
        try {
            ser.setPort(port);  // 串口设备，根据实际情况修改
            ser.setBaudrate(baud_rate);      // 波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("无法打开串口: " << e.what());
        }

        if (ser.isOpen()) {
            ROS_INFO("串口成功打开");
        } else {
            ROS_ERROR("串口打开失败");
        }
    }

    // AprilTag检测回调函数
    void sub_base_link_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
            data_packet.base_link_x = msg->pose.position.x;
            data_packet.base_link_y = msg->pose.position.y;

            // 四元数转欧拉角
            double t_roll, t_pitch, t_yaw;
            tf::Quaternion tf_q;
            tf::quaternionMsgToTF(msg->pose.orientation, tf_q);
            tf::Matrix3x3(tf_q).getRPY(t_roll, t_pitch, t_yaw);
            data_packet.base_link_w = t_yaw*180/M_PI;
       
    }

    // TF变换回调函数
    void sub_kinect_loop_Callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg) {
        if(msg->detections.size() != 1){
            data_packet.kinect_loop_depth = 0;
            data_packet.kinect_loop_dx = 0;
            ROS_WARN("loop_detection_num != 1");
        }else{
            data_packet.kinect_loop_depth = msg->detections[0].point_3d.z;
            data_packet.kinect_loop_dx =  msg->detections[0].dx;
        }
        // 若未找到变换，保留上一次值
    }

    // 障碍物检测回调函数
    void sub_kinect_circle_Callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg) {
        if(msg->detections.size() == 0){
            data_packet.kinect_circle_x = 0;
            data_packet.kinect_circle_y = 0;
        }else{
            data_packet.kinect_circle_x = 10000;
            data_packet.kinect_circle_y = 10000;
            for(const auto& detection : msg->detections){
                data_packet.kinect_circle_x = detection.point_3d.x < data_packet.kinect_circle_x ? detection.point_3d.x : data_packet.kinect_circle_x;
                data_packet.kinect_circle_y = detection.point_3d.y < data_packet.kinect_circle_y ? detection.point_3d.y : data_packet.kinect_circle_y;
        }
    }
    }

    // 发送数据函数
    void sendData() {
        if (ser.isOpen()) {
            ser.write(reinterpret_cast<uint8_t*>(&data_packet), sizeof(data_packet));
        } else {
            ROS_WARN("串口未打开");
        }
    }
    
    void print_info(){
        ROS_INFO("kinect_loop_depth: %.3f",this->data_packet.kinect_loop_depth);
        ROS_INFO("kinect_loop_dx: %.3f",this->data_packet.kinect_loop_dx);
        ROS_INFO("kinect_circle_x: %.3f",this->data_packet.kinect_circle_x);
        ROS_INFO("kinect_circle_y: %.3f",this->data_packet.kinect_circle_y);
        ROS_INFO("base_link_x: %.3f",this->data_packet.base_link_x);
        ROS_INFO("base_link_y: %.3f",this->data_packet.base_link_y);
        ROS_INFO("base_link_w: %.3f",this->data_packet.base_link_w);
    }
private:
    ros::Subscriber sub_base_link;
    ros::Subscriber sub_kinect_loop;
    ros::Subscriber sub_kinect_circle;
    std::string base_link_pub;
    std::string kinect_loop_pub;
    std::string kinect_circle_pub;
    serial::Serial ser;
    DataPacket data_packet;
    size_t baud_rate;
    std::string port;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_kinect_lidar");
    ros::NodeHandle nh;
    SerialSender sender(nh);

    ros::Rate rate(30);  // 25Hz循环
    while (ros::ok()) {
        sender.sendData();
        sender.print_info();
        rate.sleep();
        ros::spinOnce();  // 处理回调
    }

    return 0;
}
