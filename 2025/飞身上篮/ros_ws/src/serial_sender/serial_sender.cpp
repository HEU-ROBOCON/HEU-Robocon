/*
    串口代码模板。
*/
#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/UInt8.h>
#include <serial/serial.h>
#include <tf/tf.h>
#include <cmath>

#define RAD2DE 180.0/3.14159265358979
// 定义数据包结构体，确保无填充字节
struct __attribute__((packed)) DataPacket {
    uint8_t header[2] = {0x0A, 0x0D}; //2 bytes
    uint8_t tag_id;          // 1 byte
    float tag_x;             // 4 bytes
    float tag_y;             // 4 bytes
    float tag_z;             // 4 bytes
    float tag_roll;          // 4 bytes
    float tag_pitch;         // 4 bytes
    float tag_yaw;           // 4 bytes
    float tf_x;              // 4 bytes
    float tf_y;              // 4 bytes
    float tf_z;              // 4 bytes
    float tf_roll;           // 4 bytes
    float tf_pitch;          // 4 bytes
    float tf_yaw;            // 4 bytes
    uint8_t obstacle_value;  // 1 byte
    uint8_t tail[2] = {0x0B, 0x0C}; //2 bytes
    // 总大小：2 + 1 + 4 * 12 + 1 + 2 = 54 bytes
};

class SerialSender {
public:
    SerialSender(ros::NodeHandle& nh) : ser() {
        //初始化参数
        double rate;
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        nh.param<double>("baud_rate", rate, 115200);
        baud_rate = static_cast<size_t>(rate);
        // 初始化订阅者
        sub_tag = nh.subscribe("/tag_detections", 1, &SerialSender::tagCallback, this);
        sub_tf = nh.subscribe("/tf", 1, &SerialSender::tfCallback, this);
        sub_obstacle = nh.subscribe("/obstacle_detection", 1, &SerialSender::obstacleCallback, this);

        // 初始化数据包默认值
        data_packet.tag_id = 100;
        data_packet.tag_x = 0.0;
        data_packet.tag_y = 0.0;
        data_packet.tag_z = 0.0;
        data_packet.tag_roll = 0.0;
        data_packet.tag_pitch = 0.0;
        data_packet.tag_yaw = 0.0;
        data_packet.tf_x = 0.0;
        data_packet.tf_y = 0.0;
        data_packet.tf_z = 0.0;
        data_packet.tf_roll = 0.0;
        data_packet.tf_pitch = 0.0;
        data_packet.tf_yaw = 0.0;
        data_packet.obstacle_value = 100;

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
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        if (!msg->detections.empty()) {
            const auto& detection = msg->detections[0];
            data_packet.tag_id = detection.id[0];  // 取第一个ID
            const auto& pose = detection.pose.pose;  // geometry_msgs/Pose
            data_packet.tag_x = static_cast<float>(round(pose.pose.position.x * 1000.0) / 1000.0);
            data_packet.tag_y = static_cast<float>(round(pose.pose.position.y * 1000.0) / 1000.0);
            data_packet.tag_z = static_cast<float>(round(pose.pose.position.z * 1000.0) / 1000.0);

            // 四元数转欧拉角
            double t_roll, t_pitch, t_yaw;
            tf::Quaternion tf_q;
            tf::quaternionMsgToTF(pose.pose.orientation, tf_q);
            tf::Matrix3x3(tf_q).getRPY(t_roll, t_pitch, t_yaw);
            data_packet.tag_roll = static_cast<float>(round(t_roll * 1000.0 * RAD2DE) / 1000.0);
            data_packet.tag_pitch = static_cast<float>(round(t_pitch * 1000.0 * RAD2DE) / 1000.0);
            data_packet.tag_yaw = static_cast<float>(round(t_yaw * 1000.0 * RAD2DE) / 1000.0);
        } else {
            // 无检测时使用默认值
            data_packet.tag_id = -1;
            data_packet.tag_x = 0.0;
            data_packet.tag_y = 0.0;
            data_packet.tag_z = 0.0;
            data_packet.tag_roll = 0.0;
            data_packet.tag_pitch = 0.0;
            data_packet.tag_yaw = 0.0;
        }
    }

    // TF变换回调函数
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        for (const auto& transform : msg->transforms) {
            if (transform.header.frame_id == "camera_init" && transform.child_frame_id == "aft_mapped") {
                data_packet.tf_x = static_cast<float>(round(transform.transform.translation.x * 1000.0) / 1000.0);
                data_packet.tf_y = static_cast<float>(round(transform.transform.translation.y * 1000.0) / 1000.0);
                data_packet.tf_z = static_cast<float>(round(transform.transform.translation.z * 1000.0) / 1000.0);

                // 四元数转欧拉角
                double f_roll, f_pitch, f_yaw;
                tf::Quaternion tf_q;
                tf::quaternionMsgToTF(transform.transform.rotation, tf_q);
                tf::Matrix3x3(tf_q).getRPY(f_roll, f_pitch, f_yaw);
                data_packet.tf_roll = static_cast<float>(round(f_roll * 1000.0 * RAD2DE) / 1000.0);
                data_packet.tf_pitch = static_cast<float>(round(f_pitch * 1000.0 * RAD2DE) / 1000.0);
                data_packet.tf_yaw = static_cast<float>(round(f_yaw * 1000.0 * RAD2DE) / 1000.0);
                break;  // 找到匹配变换后退出循环
            }
        }
        // 若未找到变换，保留上一次值
    }

    // 障碍物检测回调函数
    void obstacleCallback(const std_msgs::UInt8::ConstPtr& msg) {
        data_packet.obstacle_value = msg->data;
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
        ROS_INFO("tag_id: %d",this->data_packet.tag_id);
        ROS_INFO("tag_x: %.3f",this->data_packet.tag_x);
        ROS_INFO("tag_y: %.3f",this->data_packet.tag_y);
        ROS_INFO("tag_z: %.3f",this->data_packet.tag_z);
        ROS_INFO("tag_roll: %.3f",this->data_packet.tag_roll*RAD2DE);
        ROS_INFO("tag_pitch: %.3f",this->data_packet.tag_pitch*RAD2DE);
        ROS_INFO("tag_yaw: %.3f",this->data_packet.tag_yaw*RAD2DE);
        ROS_INFO("tf_x: %.3f",this->data_packet.tf_x);
        ROS_INFO("tf_y: %.3f",this->data_packet.tf_y);
        ROS_INFO("tf_z: %.3f",this->data_packet.tf_z);
        ROS_INFO("tf_roll: %.3f",this->data_packet.tf_roll*RAD2DE);
        ROS_INFO("tf_pitch: %.3f",this->data_packet.tf_pitch*RAD2DE);
        ROS_INFO("tf_yaw: %.3f",this->data_packet.tf_yaw*RAD2DE);
        ROS_INFO("obstacle_value: %d",this->data_packet.obstacle_value);

    }
private:
    ros::Subscriber sub_tag;
    ros::Subscriber sub_tf;
    ros::Subscriber sub_obstacle;
    serial::Serial ser;
    DataPacket data_packet;
    size_t baud_rate;
    std::string port;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_sender");
    ros::NodeHandle nh;
    SerialSender sender(nh);

    ros::Rate rate(25);  // 25Hz循环
    while (ros::ok()) {
        sender.sendData();
        sender.print_info();
        rate.sleep();
        ros::spinOnce();  // 处理回调
    }

    return 0;
}
