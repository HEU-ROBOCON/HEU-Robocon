#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于 tf2::doTransform
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <cmath> // For M_PI

// 用于将 roll, pitch, yaw 转换为四元数
tf2::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // [cite: 2]
    return q;
}

class TargetWorldTransformer {
public:
    TargetWorldTransformer(ros::NodeHandle& nh) :
        nh_(nh),
        tf_listener_(tf_buffer_) // [cite: 16]
    {
        // --- 从参数服务器加载帧 ID ---
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
        nh_.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link"); // [cite: 3]
        nh_.param<std::string>("kinect_loop_camera_optical_frame_id", kinect_loop_camera_optical_frame_id_, "kinect_loop_color_optical_frame");
        nh_.param<std::string>("kinect_circle_camera_optical_frame_id", kinect_circle_camera_optical_frame_id_, "kinect_circle_color_optical_frame");
        nh_.param<std::string>("lidar_sensor_frame_id", lidar_sensor_frame_id_, "lidar_link"); // [cite: 4]
        nh_.param<std::string>("lidar_map_origin_frame_id", lidar_map_origin_frame_id_, "camera_init"); // [cite: 5]
        nh_.param<std::string>("lidar_current_pose_frame_id", lidar_current_pose_frame_id_, "aft_mapped"); // [cite: 6]

        ROS_INFO("帧ID配置: world='%s', base_link='%s', kinect_loop_cam='%s', kinect_circle_cam='%s', lidar_sensor='%s', lidar_map_origin='%s', lidar_current_pose='%s'",
                 world_frame_id_.c_str(), base_link_frame_id_.c_str(), kinect_loop_camera_optical_frame_id_.c_str(),
                 kinect_circle_camera_optical_frame_id_.c_str(), lidar_sensor_frame_id_.c_str(),
                 lidar_map_origin_frame_id_.c_str(), lidar_current_pose_frame_id_.c_str());

        // --- 从参数服务器加载静态变换参数 (x, y, z, roll_deg, pitch_deg, yaw_deg) ---
        load_static_transform_param("world_to_lidar_map_origin", world_frame_id_, lidar_map_origin_frame_id_);
        load_static_transform_param("base_link_to_kinect_loop_camera", base_link_frame_id_, kinect_loop_camera_optical_frame_id_); // [cite: 7]
        load_static_transform_param("base_link_to_kinect_circle_camera", base_link_frame_id_, kinect_circle_camera_optical_frame_id_); // [cite: 8]
        load_static_transform_param("base_link_to_lidar_sensor", base_link_frame_id_, lidar_sensor_frame_id_); // [cite: 9]

        if (!static_transforms_.empty()) {
            static_broadcaster_.sendTransform(static_transforms_); // [cite: 10]
            ROS_INFO("已发布 %zu 个静态TF变换。", static_transforms_.size());
        } else {
            ROS_WARN("没有从参数服务器加载到任何静态TF变换。");
        }

        // --- ROS 订阅器 --- [cite: 11]
        sub_kinect_loop_detections_ = nh_.subscribe("/yolo/kinect/loop/detections_3d", 10, &TargetWorldTransformer::kinect_loop_detections_callback, this); // [cite: 11]
        sub_kinect_circle_detections_ = nh_.subscribe("/yolo/kinect/circle/detections_3d", 10, &TargetWorldTransformer::kinect_circle_detections_callback, this); // [cite: 12]

        // --- ROS 发布器 ---
        pub_base_link_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose_in_world", 10);

        // 发布转换到世界坐标系的目标点数组 (使用原始消息类型，但更新坐标和frame_id)
        pub_kinect_loop_targets_world_array_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_loop>("/kinect/loop/targets_in_world", 10);
        pub_kinect_circle_targets_world_array_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_circle>("/kinect/circle/targets_in_world", 10);

        // 发布用于调试的 MarkerArray
        pub_kinect_loop_debug_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/loop/targets_in_world_marker_debug", 10); // [cite: 17]
        pub_kinect_circle_debug_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/circle/targets_in_world_marker_debug", 10);

        // --- 定时器用于发布机器人位姿 (约30Hz) --- [cite: 14]
        pose_publish_timer_ = nh_.createTimer(ros::Duration(1.0/30.0), &TargetWorldTransformer::publish_robot_pose_callback, this); // [cite: 15]

        ROS_INFO("目标点世界坐标转换节点已初始化。");
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> static_transforms_;

    ros::Subscriber sub_kinect_loop_detections_;
    ros::Subscriber sub_kinect_circle_detections_;

    ros::Publisher pub_base_link_pose_;
    ros::Publisher pub_kinect_loop_targets_world_array_;
    ros::Publisher pub_kinect_circle_targets_world_array_;
    ros::Publisher pub_kinect_loop_debug_markers_;
    ros::Publisher pub_kinect_circle_debug_markers_;

    ros::Timer pose_publish_timer_;

    // 帧 ID
    std::string world_frame_id_;
    std::string base_link_frame_id_;
    std::string kinect_loop_camera_optical_frame_id_;
    std::string kinect_circle_camera_optical_frame_id_;
    std::string lidar_sensor_frame_id_; // [cite: 19]
    std::string lidar_map_origin_frame_id_;
    std::string lidar_current_pose_frame_id_; // [cite: 20]

    // 从 launch 文件加载静态变换参数并存储
    void load_static_transform_param(const std::string& param_name_prefix, const std::string& parent_frame, const std::string& child_frame) {
        std::vector<double> transform_values; // [cite: 21]
        if (nh_.getParam(param_name_prefix, transform_values)) {
            if (transform_values.size() == 6) {
                geometry_msgs::TransformStamped T_stamped;
                T_stamped.header.stamp = ros::Time::now(); // [cite: 22]
                T_stamped.header.frame_id = parent_frame;
                T_stamped.child_frame_id = child_frame;
                T_stamped.transform.translation.x = transform_values[0];
                T_stamped.transform.translation.y = transform_values[1];
                T_stamped.transform.translation.z = transform_values[2];

                tf2::Quaternion q = rpy_to_quaternion( // [cite: 23]
                    transform_values[3] * M_PI / 180.0, // roll in radians
                    transform_values[4] * M_PI / 180.0, // pitch in radians
                    transform_values[5] * M_PI / 180.0  // yaw in radians
                );
                T_stamped.transform.rotation.x = q.x(); // [cite: 24]
                T_stamped.transform.rotation.y = q.y();
                T_stamped.transform.rotation.z = q.z();
                T_stamped.transform.rotation.w = q.w();
                static_transforms_.push_back(T_stamped);
                ROS_INFO("已加载静态变换 %s -> %s: [t: %.2f,%.2f,%.2f], [q: %.2f,%.2f,%.2f,%.2f]",
                         parent_frame.c_str(), child_frame.c_str(),
                         T_stamped.transform.translation.x, T_stamped.transform.translation.y, T_stamped.transform.translation.z,
                         T_stamped.transform.rotation.x, T_stamped.transform.rotation.y, T_stamped.transform.rotation.z, T_stamped.transform.rotation.w);
            } else {
                ROS_ERROR("静态变换参数 '%s' 的值数量不正确 (需要6个，实际为 %zu)。", param_name_prefix.c_str(), transform_values.size());
            }
        } else { // [cite: 25]
            ROS_ERROR("未能从参数服务器加载静态变换 '%s'。将为此变换添加单位矩阵作为备用（可能导致错误行为）。", param_name_prefix.c_str());
            geometry_msgs::TransformStamped T_stamped_identity;
            T_stamped_identity.header.stamp = ros::Time::now();
            T_stamped_identity.header.frame_id = parent_frame; // [cite: 27]
            T_stamped_identity.child_frame_id = child_frame;
            T_stamped_identity.transform.translation.x = 0;
            T_stamped_identity.transform.translation.y = 0;
            T_stamped_identity.transform.translation.z = 0;
            T_stamped_identity.transform.rotation.x = 0;
            T_stamped_identity.transform.rotation.y = 0; // [cite: 28]
            T_stamped_identity.transform.rotation.z = 0;
            T_stamped_identity.transform.rotation.w = 1;
            static_transforms_.push_back(T_stamped_identity); // [cite: 29]
        }
    }

    // 定时回调，发布机器人底盘在世界坐标系下的位姿
    void publish_robot_pose_callback(const ros::TimerEvent&) {
        geometry_msgs::TransformStamped T_world_base_stamped;
        try { // [cite: 30]
            // TF 链预期是: world -> camera_init (static) -> aft_mapped (dynamic) -> lidar_sensor_frame (它就是aft_mapped所代表的帧) <- base_link (via static inverse)
            // 我们发布的静态TF包括:
            // 1. world -> camera_init (lidar_map_origin_frame_id_)
            // 2. base_link -> lidar_sensor_frame_id_
            // 雷达节点发布: camera_init -> aft_mapped (其中 aft_mapped 代表了 lidar_sensor_frame_id_ 在 camera_init 中的位姿)
            // 因此，TF系统可以通过 world -> camera_init -> aft_mapped (即 lidar_sensor_frame_id_) <- base_link 来查找 world 到 base_link。 [cite: 31, 32]
            T_world_base_stamped = tf_buffer_.lookupTransform(world_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(0.1)); // 缩短超时以更快响应
        } catch (tf2::TransformException &ex) { // [cite: 33]
            ROS_WARN_THROTTLE(1.0, "获取 TF 变换 %s -> %s 失败: %s. 请检查TF树是否完整且所有节点运行正常。",
                              world_frame_id_.c_str(), base_link_frame_id_.c_str(), ex.what());
            return; // [cite: 34]
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = T_world_base_stamped.header.stamp; // 使用查找到的变换的时间戳更准确 [cite: 35]
        pose_stamped.header.frame_id = world_frame_id_;
        pose_stamped.pose.position.x = T_world_base_stamped.transform.translation.x;
        pose_stamped.pose.position.y = T_world_base_stamped.transform.translation.y;
        pose_stamped.pose.position.z = T_world_base_stamped.transform.translation.z; // [cite: 36]
        pose_stamped.pose.orientation = T_world_base_stamped.transform.rotation;

        pub_base_link_pose_.publish(pose_stamped);

        tf2::Quaternion q(
            T_world_base_stamped.transform.rotation.x,
            T_world_base_stamped.transform.rotation.y,
            T_world_base_stamped.transform.rotation.z,
            T_world_base_stamped.transform.rotation.w);
        tf2::Matrix3x3 m(q); // [cite: 37]
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_DEBUG_THROTTLE(1.0, "机器人位姿 (World Frame) [RPY deg]: Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI); // [cite: 38]
    }

    // Kinect Loop 目标检测回调
    void kinect_loop_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array_debug; // [cite: 39]
        process_detections_kinect(msg, kinect_loop_camera_optical_frame_id_, pub_kinect_loop_targets_world_array_, marker_array_debug);
        if(!marker_array_debug.markers.empty()){ // [cite: 40]
            pub_kinect_loop_debug_markers_.publish(marker_array_debug); // [cite: 41]
        }
    }

    // Kinect Circle 目标检测回调
    void kinect_circle_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array_debug; // [cite: 42]
        process_detections_realsense(msg, kinect_circle_camera_optical_frame_id_, pub_kinect_circle_targets_world_array_, marker_array_debug);
        if(!marker_array_debug.markers.empty()){ // [cite: 43]
            pub_kinect_circle_debug_markers_.publish(marker_array_debug);
        }
    }

    // 通用处理函数 - kinect circle
    void process_detections_kinect_circle(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg_in,
                                     const std::string& camera_optical_frame,
                                     ros::Publisher& array_publisher,
                                     visualization_msgs::MarkerArray& marker_array_for_debug) { // [cite: 138]
        if (msg_in->detections.empty()) { // [cite: 139]
            return;
        }

        yolo_realsense_kinect::DetectedObject3DArray_kinect_circle msg_out_array; // 使用输入消息类型
        msg_out_array.header.stamp = ros::Time::now(); // [cite: 140]
        msg_out_array.header.frame_id = world_frame_id_; // [cite: 141]
        marker_array_for_debug.markers.clear(); // 清空上次的 markers

        int current_marker_id = 0; // 用于在单个 MarkerArray 中生成唯一 ID

        for (const auto& det_in : msg_in->detections) {
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z)) {
                ROS_DEBUG("RealSense: 检测到无效的输入目标点坐标 (NaN)，跳过。");
                continue; // [cite: 142]
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame; // [cite: 143]
            pt_cam.header.stamp = msg_in->header.stamp; // [cite: 144]
            pt_cam.point = det_in.point_3d;

            try { // [cite: 145]
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.1)); // 缩短超时 [cite: 146]
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "RealSense: 目标点 TF 变换 %s -> %s 失败: %s", camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue; // [cite: 147]
            }

            if (should_publish_target_point_common(pt_world, det_in.class_name, det_in.confidence, marker_array_for_debug, current_marker_id, "kinect_circle_detections")) {
                yolo_realsense_kinect::DetectedObject3D_kinect_circle det_out = det_in; // 复制其他信息
                det_out.point_3d = pt_world.point; // 更新为世界坐标
                msg_out_array.detections.push_back(det_out);
            }
        }

        if (!msg_out_array.detections.empty()) {
            array_publisher.publish(msg_out_array); // [cite: 151]
        }
    }

    // 通用处理函数 - Kinect
    void process_detections_kinect_loop(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg_in,
                                   const std::string& camera_optical_frame,
                                   ros::Publisher& array_publisher,
                                   visualization_msgs::MarkerArray& marker_array_for_debug) {
        if (msg_in->detections.empty()) { // [cite: 152]
            return; // [cite: 153]
        }

        yolo_realsense_kinect::DetectedObject3DArray_kinect_loop msg_out_array; // 使用输入消息类型
        msg_out_array.header.stamp = ros::Time::now(); // [cite: 154]
        msg_out_array.header.frame_id = world_frame_id_; // [cite: 155]
        marker_array_for_debug.markers.clear();

        int current_marker_id = 0;

        for (const auto& det_in : msg_in->detections) {
            // 假设 DetectedObject3D_kinect_loop 也有 point_3d 成员
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z)) {
                ROS_DEBUG("Kinect: 检测到无效的输入目标点坐标 (NaN)，跳过。");
                continue; // [cite: 156]
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame; // [cite: 157]
            pt_cam.header.stamp = msg_in->header.stamp; // [cite: 158]
            pt_cam.point = det_in.point_3d; // 假设与Realsense结构一致

            try { // [cite: 159]
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.1)); // [cite: 160]
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "Kinect: 目标点 TF 变换 %s -> %s 失败: %s", camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue; // [cite: 161]
            }

            // 假设 DetectedObject3D_kinect_loop 有 class_name 和 confidence 成员
            if (should_publish_target_point_common(pt_world, det_in.class_name, det_in.confidence, marker_array_for_debug, current_marker_id, "kinect_loop_detections")) {
                yolo_realsense_kinect::DetectedObject3D_kinect_loop det_out = det_in;
                det_out.point_3d = pt_world.point; // 更新为世界坐标
                msg_out_array.detections.push_back(det_out);
            }
        }

        if (!msg_out_array.detections.empty()) {
            array_publisher.publish(msg_out_array); // [cite: 165]
        }
    }

    // --- 通用判断函数，用于判断是否发布目标点并创建Markers ---
    bool should_publish_target_point_common(const geometry_msgs::PointStamped& point_in_world,
                                            const std::string& class_name,
                                            float confidence,
                                            visualization_msgs::MarkerArray& marker_array, /*传入引用*/
                                            int& marker_id_counter, /*传入引用以递增*/
                                            const std::string& ns_prefix) {
        // 在这里实现您的判断逻辑
        // 例如:
        // if (confidence < 0.7) { return false; }
        // if (point_in_world.point.z > 2.0) { return false; } // 忽略太远的点

        ROS_DEBUG("目标点 (世界坐标系, %s): x=%.2f, y=%.2f, z=%.2f, 类别=%s, 置信度=%.2f",
                  ns_prefix.c_str(), point_in_world.point.x, point_in_world.point.y, point_in_world.point.z,
                  class_name.c_str(), confidence);

        // 创建球体标记
        visualization_msgs::Marker marker; // [cite: 75]
        marker.header = point_in_world.header; // 使用转换后点的时间戳和frame_id (world)
        marker.ns = ns_prefix + "_world";
        marker.id = marker_id_counter++; // 使用传入的计数器并递增
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point_in_world.point; // [cite: 87]
        marker.pose.orientation.w = 1.0; // [cite: 76]
        marker.scale.x = 0.15; marker.scale.y = 0.15; marker.scale.z = 0.15; // 稍微大一点
        marker.color.a = 0.8;
        if (ns_prefix == "kinect_circle_detections") {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Kinect Circle 红色 [cite: 77]
        } else { // Kinect
            marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // Kinect 蓝色 [cite: 88]
        }
        marker.lifetime = ros::Duration(0.5); // [cite: 89]
        marker_array.markers.push_back(marker);

        // 创建文本标记 [cite: 78, 90]
        visualization_msgs::Marker text_marker = marker; // 复制基本属性
        text_marker.id = marker_id_counter++; // 使用传入的计数器并递增
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.text = class_name + " (" + std::to_string(confidence).substr(0,4) + ")"; // [cite: 79, 91]
        text_marker.pose.position.z += 0.15; // 将文本放在球体上方
        text_marker.scale.z = 0.1; // [cite: 80, 92]
        text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; // 白色文本
        marker_array.markers.push_back(text_marker);

        return true; // [cite: 82, 93, 176]
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "target_world_transformer_node");
    ros::NodeHandle nh("~"); // [cite: 94, 188]
    TargetWorldTransformer transformer(nh);
    ros::spin();
    return 0;
}