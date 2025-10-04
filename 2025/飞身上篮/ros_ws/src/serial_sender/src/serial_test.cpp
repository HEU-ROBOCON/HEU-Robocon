#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/UInt8.h>
#include <serial/serial.h>
#include <tf/tf.h>
#include <cmath>

#define RAD2DE 180.0/3.14159265358979
// 定义数据包结构体，确保无填充字节
struct __attribute__((packed)) DataPacket {
    uint8_t header[2] = {0x0A, 0x0D}; //2 bytes
    float kinect_z;
    float kinect_x;
    float realsense_x;
    float realsense_y;
    float lidar_x;
    float lidar_y;
    float lidar_w;
    uint8_t tail[2] = {0x0B, 0x0C}; //2 bytes
    // 总大小：2 + 1 + 4 * 12 + 1 + 2 = 54 bytes
};

class SerialSender {
public:
    SerialSender(ros::NodeHandle& nh) : ser() {
        //初始化参数*
        double rate;
        // nh.param<std::string>("port", port, "/dev/ttyUSB0");
        // nh.param<double>("baud_rate", rate, 115200);
        this->port = "/dev/ttyUSB0";
        //this->rate = 115200;
        this->baud_rate = static_cast<size_t>(115200);

        //配置并打开串口
        try {
            ser.setPort(this->port);  // 串口设备，根据实际情况修改
            ser.setBaudrate(this->baud_rate);      // 波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("无法打开串口: " << e.what());
        }

        if (ser.isOpen()) {
            ROS_INFO("serial port opened");
        } else {
            ROS_ERROR("serial port not opened");
        }
    }

    // 发送数据函数
    void sendData() {
        if (ser.isOpen()) {
            ser.write(reinterpret_cast<uint8_t*>(&data_packet), sizeof(data_packet));
            ROS_INFO("data sent");
        } else {
            ROS_WARN("串口未打开");
        }
    }
    
    void print_info(){
        ROS_INFO("kinect_x: %.5f",this->data_packet.kinect_x);
        ROS_INFO("kinect_z: %.5f",this->data_packet.kinect_z);
        ROS_INFO("realsense_x: %.5f",this->data_packet.realsense_x);
        ROS_INFO("realsense_y: %.5f",this->data_packet.realsense_y);
        ROS_INFO("lidar_x: %.5f",this->data_packet.lidar_x);
        ROS_INFO("lidar_y: %.5f",this->data_packet.lidar_y);
        ROS_INFO("lidar_w %.5f",this->data_packet.lidar_w);
    }

    void change_data(int i){
        this->data_packet.kinect_z += i ;
        this->data_packet.kinect_x += i ;
        this->data_packet.realsense_x +=i ;
        this->data_packet.realsense_y +=i ;
        this->data_packet.lidar_x +=i ;
        this->data_packet.lidar_y +=i ;
        this->data_packet.lidar_w +=i ;
    }
    void init_data(){ 
        this->data_packet.kinect_z = 0.12345670;
        this->data_packet.kinect_x = 0.12345670;
        this->data_packet.realsense_x = 0.12345670;
        this->data_packet.realsense_y = 0.12345670;
        this->data_packet.lidar_x = 0.12345670;
        this->data_packet.lidar_y = 0.12345670;
        this->data_packet.lidar_w = 0.12345670;
    }
private:
    serial::Serial ser;
    DataPacket data_packet;
    size_t baud_rate;
    std::string port;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_test");
    ros::NodeHandle nh("~");
    SerialSender sender(nh);

    ros::Rate rate(1);  // 25Hz循环
    sender.init_data();
    while (ros::ok()) {
        sender.change_data(1);
        sender.sendData();
        sender.print_info();
        rate.sleep();
        ros::spinOnce();  // 处理回调
    }

    return 0;
}
