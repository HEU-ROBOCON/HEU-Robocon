/*
    代码备份。
*/
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/UInt8.h>
#include <serial/serial.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h>
#include <vector>
#include <utility>

struct __attribute__((packed)) DataPacket
{
    uint8_t header[2] = {0x0A, 0x0D};
    float kinect_loop_depth = 10000;
    float kinect_loop_dx = 10000;
    float kinect_circle_x = 0;
    float kinect_circle_y = 0;
    float base_link_x = 0;
    float base_link_y = 0;
    float base_link_w = 0;
    uint8_t tail[2] = {0x0B, 0x0C};
};

struct __attribute__((packed)) check_point
{
    double x;
    double y;
};

class SerialSender
{
public:
    SerialSender(ros::NodeHandle &nh) : ser()
    {
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        nh.param<int32_t>("baud_rate", baud_rate, 115200);
        nh.param<std::string>("base_link_pub", base_link_pub, "/robot_pose_in_world");
        nh.param<std::string>("kinect_loop_pub", kinect_loop_pub, "/kinect/loop/targets_in_world");
        nh.param<std::string>("kinect_circle_pub", kinect_circle_pub, "/kinect/circle/targets_in_world");
        nh.param<bool>("en_kinect_loop", en_kinect_loop, true);
        nh.param<bool>("en_kinect_circle", en_kinect_circle, true);
        nh.param<bool>("en_base_link", en_base_link, true);
        nh.param<float>("arrive_circle_point_distance_threshold", arrive_circle_point_distance_threshold, 0.5);
        nh.param<float>("arrive_circle_point_x_threshold", arrive_circle_point_x_threshold, 0.1);
        nh.param<float>("arrive_circle_point_y_threshold", arrive_circle_point_y_threshold, 0.1);
        nh.param<float>("similar_circle_point_x_threshold", similar_circle_point_x_threshold, 0.1);
        nh.param<float>("similar_circle_point_y_threshold", similar_circle_point_y_threshold, 0.1);
        nh.param<std::string>("world_frame_id", world_frame_id, "world");
        nh.param<bool>("x_same_direction", x_same_direction, true);
        nh.param<bool>("y_same_direction", y_same_direction, false);
        nh.param<float>("margin", margin, 0.1f);             // 单位为米
        nh.param<float>("changdi_kuan", changdi_kuan, 8.0f); // 单位为米
        nh.param<float>("changdi_chang_zhu", changdi_chang_zhu, 15.0f);
        nh.param<float>("changdi_chang_tiao", changdi_chang_tiao, 6.0f);
        nh.param<bool>("kuan_equal_x", kuan_equal_x, true);
        nh.param<bool>("whether_zhu", whether_zhu, false);
        nh.param<int>("frequency_count", frequency_count, 3);
        nh.param<int>("threshold_to_rectify", threshold_to_rectify, 3);
        nh.param<bool>("debug_mode", debug_mode, false);

        pub_circle_debug_markers_ = nh.advertise<visualization_msgs::MarkerArray>("serial_sender/circle_debug_markers", 1);

        if (en_base_link)
            sub_base_link = nh.subscribe(base_link_pub, 1, &SerialSender::sub_base_link_Callback, this);
        if (en_kinect_loop)
            sub_kinect_loop = nh.subscribe(kinect_loop_pub, 1, &SerialSender::sub_kinect_loop_Callback, this);
        if (en_kinect_circle)
            sub_kinect_circle = nh.subscribe(kinect_circle_pub, 1, &SerialSender::sub_kinect_circle_Callback, this);

        data_packet.kinect_loop_depth = 10000.0;
        data_packet.kinect_loop_dx = 10000.0;
        data_packet.kinect_circle_x = 0.0;
        data_packet.kinect_circle_y = 0.0;
        data_packet.base_link_x = 0.0;
        data_packet.base_link_y = 0.0;
        data_packet.base_link_w = 0.0;
        this->fixed_circle_points_list.clear();
        this->count = 0;
        this->count0 = 5;
        this->counter = 0;
        this->x_count = 0;
        this->y_count = 0;

        try
        {
            ser.setPort(port);
            ser.setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("无法打开串口: " << e.what());
        }

        if (ser.isOpen())
        {
            ROS_INFO("串口成功打开");
        }
        else
        {
            ROS_ERROR("串口打开失败");
        }
    }

    void sub_base_link_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if(this->counter < this->count0){
            this->x_list.push_back(msg->pose.position.x * 1000);
            this->y_list.push_back(msg->pose.position.y * 1000);
            this->x_count += msg->pose.position.x * 1000;
            this->y_count += msg->pose.position.y * 1000;
            this->counter++;   
            return;
        }
        
        
        this->x_count = (this->x_count - this->x_list[counter % count0] + msg->pose.position.x * 1000);
        this->y_count = (this->y_count - this->y_list[counter % count0] + msg->pose.position.y * 1000);
        data_packet.base_link_x = (this->x_count) / count0;
        data_packet.base_link_y = (this->y_count) / count0;
        if(counter % count0 == 0){
            counter = count0;
        }
        this->x_list[counter % count0] = msg->pose.position.x * 1000;
        this->y_list[counter % count0] = msg->pose.position.y * 1000;
        counter++;

        double t_roll, t_pitch, t_yaw;
        tf::Quaternion tf_q;
        tf::quaternionMsgToTF(msg->pose.orientation, tf_q);
        tf::Matrix3x3(tf_q).getRPY(t_roll, t_pitch, t_yaw);
        data_packet.base_link_w = t_yaw * 180 / M_PI;
    }

    void sub_kinect_loop_Callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr &msg)
    {
        if (msg->detections.empty())
        {
            data_packet.kinect_loop_depth = 50000;
            data_packet.kinect_loop_dx = 50000;
            // ROS_WARN("no_loop_target");
        }
        else
        {
            float conf = 0;
            int index = 0;
            for (int i = 0; i < msg->detections.size(); i++)
            {
                if (msg->detections[i].confidence > conf)
                {
                    conf = msg->detections[i].confidence;
                    index = i;
                }
            }
            data_packet.kinect_loop_depth = msg->detections[index].point_3d.z * 1000;
            data_packet.kinect_loop_dx = msg->detections[index].dx;
        }
    }

    void sub_kinect_circle_Callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr &msg)
    {
        if (msg->detections.empty())
        {
            data_packet.kinect_circle_x = 0;
            data_packet.kinect_circle_y = 0;
            return;
        }

        if (this->count < this->frequency_count)
        {
            evaluate_good_circle_point(msg);
            this->count++;
            return;
        }

        this->rectifyed_circle_points_list.clear();

        for (const auto &adjust_point : this->fixed_circle_points_list)
        {
            if (adjust_point.first >= threshold_to_rectify)
            {
                this->rectifyed_circle_points_list.push_back(adjust_point.second);
            }
        }

        this->count = 0;
        this->fixed_circle_points_list.clear();
        // 找出非打卡点中距离底盘最近的点
        double min_dist_sq = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto &detection : this->rectifyed_circle_points_list)
        {
            // ROS_INFO("circle point: %f %f", detection.point_3d.x, detection.point_3d.y);
            if ((!is_in_arrived_list(detection)) && whether_cross_line(detection.point_3d.x, detection.point_3d.y))
            {
                double dx = detection.point_3d.x - data_packet.base_link_x / 1000.0;
                double dy = detection.point_3d.y - data_packet.base_link_y / 1000.0;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < min_dist_sq)
                {
                    min_dist_sq = dist_sq;
                    data_packet.kinect_circle_x = detection.point_3d.x * 1000;
                    data_packet.kinect_circle_y = detection.point_3d.y * 1000;
                    found = true;
                }
            }
        }

        // 发布marker用于调试
        visualization_msgs::Marker marker;
        marker.header.frame_id = world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "sender_circle_debug_markers";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = data_packet.kinect_circle_x / 1000.0;
        marker.pose.position.y = data_packet.kinect_circle_y / 1000.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        pub_circle_debug_markers_.publish(marker_array);

        // 更新打卡点列表
        for (const auto &detection : this->rectifyed_circle_points_list)
        {
            // double dx = detection.point_3d.x - data_packet.base_link_x / 1000.0;
            double dy = detection.point_3d.y - data_packet.base_link_y / 1000.0;
            // double dist_sq = dx * dx + dy * dy;
            if (dy < arrive_circle_point_distance_threshold &&
                !is_in_arrived_list(detection))
            {
                this->arrive_circle_points.push_back(check_point{detection.point_3d.x, detection.point_3d.y});
            }
        }

        if (!found)
        {
            data_packet.kinect_circle_x = 0;
            data_packet.kinect_circle_y = 0;
        }
    }

    void sendData()
    {
        if (ser.isOpen())
        {
            ser.write(reinterpret_cast<uint8_t *>(&data_packet), sizeof(data_packet));
        }
        else
        {
            ROS_WARN("串口未打开");
        }
    }

    void evaluate_good_circle_point(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr &msg)
    {
        bool is_similar = false;
        for (const auto &detection : msg->detections)
        {
            for (int i = 0; i < this->fixed_circle_points_list.size(); i++)
            {
                if (std::abs(detection.point_3d.x - this->fixed_circle_points_list[i].second.point_3d.x) < this->similar_circle_point_x_threshold && std::abs(detection.point_3d.y - this->fixed_circle_points_list[i].second.point_3d.y) < this->similar_circle_point_y_threshold)
                {
                    this->fixed_circle_points_list[i].first += 1;
                    this->fixed_circle_points_list[i].second.point_3d.x = (detection.point_3d.x + this->fixed_circle_points_list[i].second.point_3d.x) / 2;
                    this->fixed_circle_points_list[i].second.point_3d.y = (detection.point_3d.y + this->fixed_circle_points_list[i].second.point_3d.y) / 2;
                    is_similar = true;
                    break;
                }
            }
            if (!is_similar)
            {
                this->fixed_circle_points_list.push_back(std::make_pair(1, detection));
            }

            is_similar = false;
        }
    }

    void print_info()
    {
        ROS_INFO("kinect_loop_depth: %.3f", this->data_packet.kinect_loop_depth);
        ROS_INFO("kinect_loop_dx: %.3f", this->data_packet.kinect_loop_dx);
        ROS_INFO("kinect_circle_x: %.3f", this->data_packet.kinect_circle_x);
        ROS_INFO("kinect_circle_y: %.3f", this->data_packet.kinect_circle_y);
        ROS_INFO("base_link_x: %.3f", this->data_packet.base_link_x);
        ROS_INFO("base_link_y: %.3f", this->data_packet.base_link_y);
        ROS_INFO("base_link_w: %.3f", this->data_packet.base_link_w);
        ROS_INFO("point_to_base_distance: %.3f", std::sqrt(pow(data_packet.base_link_x - data_packet.kinect_circle_x, 2) + pow(data_packet.base_link_y - data_packet.kinect_circle_y, 2)) / 1000);
    }

    void print_params()
    {
        ROS_INFO("当前参数配置:");
        ROS_INFO("串口参数:");
        ROS_INFO("  port: %s", port.c_str());
        ROS_INFO("  baud_rate: %d", baud_rate);

        ROS_INFO("\n话题配置:");
        ROS_INFO("  base_link_pub: %s", base_link_pub.c_str());
        ROS_INFO("  kinect_loop_pub: %s", kinect_loop_pub.c_str());
        ROS_INFO("  kinect_circle_pub: %s", kinect_circle_pub.c_str());

        ROS_INFO("\n使能状态:");
        ROS_INFO("  en_kinect_loop: %s", en_kinect_loop ? "true" : "false");
        ROS_INFO("  en_kinect_circle: %s", en_kinect_circle ? "true" : "false");
        ROS_INFO("  en_base_link: %s", en_base_link ? "true" : "false");

        ROS_INFO("\n阈值设置:");
        ROS_INFO("  arrive_circle_point_distance_threshold: %.3f", arrive_circle_point_distance_threshold);
        ROS_INFO("  arrive_circle_point_x_threshold: %.3f", arrive_circle_point_x_threshold);
        ROS_INFO("  arrive_circle_point_y_threshold: %.3f", arrive_circle_point_y_threshold);
        ROS_INFO("  similar_circle_point_x_threshold: %.3f", similar_circle_point_x_threshold);
        ROS_INFO("  similar_circle_point_y_threshold: %.3f", similar_circle_point_y_threshold);
        ROS_INFO("  threshold_to_rectify: %d", threshold_to_rectify);
        ROS_INFO("  frequency_count: %d", frequency_count);

        ROS_INFO("\n坐标系配置:");
        ROS_INFO("  world_frame_id: %s", world_frame_id.c_str());
        ROS_INFO("  x_same_direction: %s", x_same_direction ? "true" : "false");
        ROS_INFO("  y_same_direction: %s", y_same_direction ? "true" : "false");

        ROS_INFO("\n场地参数:");
        ROS_INFO("  margin: %.3f m", margin);
        ROS_INFO("  changdi_kuan: %.3f m", changdi_kuan);
        ROS_INFO("  changdi_chang_zhu: %.3f m", changdi_chang_zhu);
        ROS_INFO("  changdi_chang_tiao: %.3f m", changdi_chang_tiao);

        ROS_INFO("\n布局配置:");
        ROS_INFO("  kuan_equal_x: %s", kuan_equal_x ? "true" : "false");
        ROS_INFO("  whether_zhu: %s", whether_zhu ? "true" : "false");
    }

    bool is_in_arrived_list(const yolo_realsense_kinect::DetectedObject3D_kinect_circle &detection)
    {
        for (const auto &point : this->arrive_circle_points)
        {
            if (std::abs(point.x - detection.point_3d.x) < arrive_circle_point_x_threshold &&
                std::abs(point.y - detection.point_3d.y) < arrive_circle_point_y_threshold)
            {
                return true;
            }
        }
        return false;
    }

    bool whether_cross_line(float x, float y)
    {
        int case_value1 = (whether_zhu << 1) | (kuan_equal_x);
        int case_value2 = (x_same_direction << 1) | (y_same_direction);
        // ROS_INFO("case_value1: %d", case_value1);
        // ROS_INFO("case_value2: %d", case_value2);

        switch (case_value1)
        {
        case 0:
            /* whether_zhu = false, kuan_equal_x = false */
            switch (case_value2)
            {
            case 0:
                /* x_same_direction = false, y_same_direction = false */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_chang_tiao)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_kuan)))
                    return true;
                break;

            case 1:
                /* x_same_direction = false, y_same_direction = true */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_chang_tiao)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_kuan)))
                    return true;
                break;

            case 2:
                /* x_same_direction = true, y_same_direction = false */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_chang_tiao)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_kuan)))
                    return true;
                break;

            case 3:
                /* x_same_direction = true, y_same_direction = true */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_chang_tiao)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_kuan)))
                    return true;
            default:
                break;
            }
            break;

        case 1:
            /* whether_zhu = false, kuan_equal_x = true */
            switch (case_value2)
            {
            case 0:
                /* x_same_direction = false, y_same_direction = false */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_kuan)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_chang_tiao)))
                    return true;
                break;

            case 1:
                /* x_same_direction = false, y_same_direction = true */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_kuan)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_chang_tiao)))
                    return true;
                break;

            case 2:
                /* x_same_direction = true, y_same_direction = false */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_kuan)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_chang_tiao)))
                    return true;
                break;

            case 3:
                /* x_same_direction = true, y_same_direction = true */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_kuan)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_chang_tiao)))
                    return true;
            default:
                break;
            }
            break;

        case 2:
            /* whether_zhu = true, kuan_equal_x = false */
            switch (case_value2)
            {
            case 0:
                /* x_same_direction = false, y_same_direction = false */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_chang_zhu)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_kuan)))
                    return true;
                break;

            case 1:
                /* x_same_direction = false, y_same_direction = true */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_chang_zhu)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_kuan)))
                    return true;
                break;

            case 2:
                /* x_same_direction = true, y_same_direction = false */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_chang_zhu)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_kuan)))
                    return true;
                break;

            case 3:
                /* x_same_direction = true, y_same_direction = true */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_chang_zhu)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_kuan)))
                    return true;
            default:
                break;
            }
            break;

        case 3:
            /* whether_zhu = true, kuan_equal_x = true */
            switch (case_value2)
            {
            case 0:
                /* x_same_direction = false, y_same_direction = false */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_kuan)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_chang_zhu)))
                    return true;
                break;

            case 1:
                /* x_same_direction = false, y_same_direction = true */
                if ((x - this->margin < 0) && (x + this->margin > (-this->changdi_kuan)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_chang_zhu)))
                    return true;
                break;

            case 2:
                /* x_same_direction = true, y_same_direction = false */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_kuan)) && (y - this->margin < 0) && (y + this->margin > (-this->changdi_chang_zhu)))
                    return true;
                break;

            case 3:
                /* x_same_direction = true, y_same_direction = true */
                if ((x + this->margin > 0) && (x - this->margin < (this->changdi_kuan)) && (y + this->margin > 0) && (y - this->margin < (this->changdi_chang_zhu)))
                    return true;
            default:
                break;
            }
            break;

        default:
            break;
        }
        return false;
    }

    bool debug_mode;

private:
    ros::Subscriber sub_base_link;
    ros::Subscriber sub_kinect_loop;
    ros::Subscriber sub_kinect_circle;
    ros::Publisher pub_circle_debug_markers_;
    std::string base_link_pub;
    std::string kinect_loop_pub;
    std::string kinect_circle_pub;
    bool en_kinect_loop;
    bool en_kinect_circle;
    bool en_base_link;
    serial::Serial ser;
    DataPacket data_packet;
    int32_t baud_rate;
    std::string port;
    std::vector<check_point> arrive_circle_points;
    float arrive_circle_point_distance_threshold;
    float arrive_circle_point_x_threshold;
    float arrive_circle_point_y_threshold;
    float similar_circle_point_x_threshold;
    float similar_circle_point_y_threshold;
    std::string world_frame_id;
    bool x_same_direction;
    bool y_same_direction;
    float margin;
    float changdi_kuan;
    float changdi_chang_zhu;
    float changdi_chang_tiao;
    bool whether_zhu;
    bool kuan_equal_x;
    std::vector<std::pair<int, yolo_realsense_kinect::DetectedObject3D_kinect_circle>> fixed_circle_points_list;
    std::vector<yolo_realsense_kinect::DetectedObject3D_kinect_circle> rectifyed_circle_points_list;
    int frequency_count;
    int threshold_to_rectify;
    int count;
    int count0;
    int counter;
    float x_count;
    float y_count;
    std::vector<float> x_list;
    std::vector<float> y_list;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_kinect_lidar");
    ros::NodeHandle nh("~");
    SerialSender sender(nh);

    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();   // 先处理回调，更新data_packet
        sender.sendData(); // 再发送更新后的数据
        if (sender.debug_mode)
        {
            sender.print_params();
            sender.print_info();
        }
        rate.sleep();
    }

    return 0;
}