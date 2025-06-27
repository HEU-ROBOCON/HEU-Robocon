#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于 tf2::doTransform
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
// 包含您自定义的Kinect Loop和Circle检测消息类型
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <cmath>    // For M_PI
#include <iomanip>  // For std::setprecision in stringstream
#include <sstream>  // For std::stringstream

// 用于将 roll, pitch, yaw (弧度) 转换为四元数
tf2::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // [cite: 2, 117]
    return q;
}

class TargetWorldTransformer {
public:
    TargetWorldTransformer(ros::NodeHandle& nh) :
        nh_(nh),
        tf_listener_(tf_buffer_) // [cite: 16, 118, 137]
    {
        // --- 1. 从参数服务器加载帧 ID ---
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
        nh_.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link"); // [cite: 4, 119]
        nh_.param<std::string>("kinect_loop_camera_optical_frame_id", kinect_loop_camera_optical_frame_id_, "kinect_loop_color_optical_frame");
        nh_.param<std::string>("kinect_circle_camera_optical_frame_id", kinect_circle_camera_optical_frame_id_, "kinect_circle_color_optical_frame");
        nh_.param<std::string>("lidar_sensor_frame_id", lidar_sensor_frame_id_, "lidar_link"); // [cite: 5, 120]
        nh_.param<std::string>("lidar_map_origin_frame_id", lidar_map_origin_frame_id_, "camera_init"); // [cite: 6, 121]
        nh_.param<std::string>("lidar_current_pose_frame_id", lidar_current_pose_frame_id_, "aft_mapped"); // [cite: 7, 122]

        ROS_INFO("帧ID配置: world='%s', base_link='%s', kinect_loop_cam='%s', kinect_circle_cam='%s', lidar_sensor='%s', lidar_map_origin='%s', lidar_current_pose='%s'",
                 world_frame_id_.c_str(), base_link_frame_id_.c_str(), kinect_loop_camera_optical_frame_id_.c_str(),
                 kinect_circle_camera_optical_frame_id_.c_str(), lidar_sensor_frame_id_.c_str(),
                 lidar_map_origin_frame_id_.c_str(), lidar_current_pose_frame_id_.c_str());

        // --- 2. 从参数服务器加载并发布静态变换 ---
        load_static_transform_param("world_to_lidar_map_origin", world_frame_id_, lidar_map_origin_frame_id_); // [cite: 8, 123]
        load_static_transform_param("base_link_to_kinect_loop_camera", base_link_frame_id_, kinect_loop_camera_optical_frame_id_); // [cite: 9, 124]
        load_static_transform_param("base_link_to_kinect_circle_camera", base_link_frame_id_, kinect_circle_camera_optical_frame_id_); // [cite: 10, 125]
        load_static_transform_param("base_link_to_lidar_sensor", base_link_frame_id_, lidar_sensor_frame_id_); // [cite: 11, 126]

        if (!static_transforms_.empty()) {
            static_broadcaster_.sendTransform(static_transforms_); // [cite: 12, 127]
            ROS_INFO("已成功发布 %zu 个静态TF变换。", static_transforms_.size()); // [cite: 13, 128]
        } else {
            ROS_WARN("没有从参数服务器加载到任何有效的静态TF变换。TF树可能不完整。"); // [cite: 14, 129]
        }

        // --- 3. 初始化ROS订阅器 ---
        sub_kinect_loop_detections_ = nh_.subscribe("/yolo/kinect/loop/detections_3d", 10, &TargetWorldTransformer::kinect_loop_detections_callback, this); // [cite: 15, 130]
        sub_kinect_circle_detections_ = nh_.subscribe("/yolo/kinect/circle/detections_3d", 10, &TargetWorldTransformer::kinect_circle_detections_callback, this); // [cite: 16, 131]

        // --- 4. 初始化ROS发布器 ---
        pub_base_link_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose_in_world", 10);

        pub_kinect_loop_targets_world_array_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_loop>("/kinect/loop/targets_in_world", 10); // [cite: 17, 132]
        pub_kinect_circle_targets_world_array_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_circle>("/kinect/circle/targets_in_world", 10); // [cite: 17, 132]

        pub_kinect_loop_debug_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/loop/targets_in_world_marker_debug", 10); // [cite: 18, 19, 133, 134]
        pub_kinect_circle_debug_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/circle/targets_in_world_marker_debug", 10); // [cite: 18, 19, 133, 134]

        // --- 5. 创建定时器用于定期发布机器人位姿 ---
        pose_publish_timer_ = nh_.createTimer(ros::Duration(1.0/30.0), &TargetWorldTransformer::publish_robot_pose_callback, this); // [cite: 20, 21, 135, 136] // 目标30Hz

        ROS_INFO("目标点世界坐标转换节点 (transformer_node) 已成功初始化。");
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_; // [cite: 22, 137]
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> static_transforms_;

    ros::Subscriber sub_kinect_loop_detections_;
    ros::Subscriber sub_kinect_circle_detections_;

    ros::Publisher pub_base_link_pose_;
    ros::Publisher pub_kinect_loop_targets_world_array_;
    ros::Publisher pub_kinect_circle_targets_world_array_;
    ros::Publisher pub_kinect_loop_debug_markers_;
    ros::Publisher pub_kinect_circle_debug_markers_;

    ros::Timer pose_publish_timer_; // 确保声明存在 [cite: 23, 138]

    std::string world_frame_id_;
    std::string base_link_frame_id_;
    std::string kinect_loop_camera_optical_frame_id_;
    std::string kinect_circle_camera_optical_frame_id_;
    std::string lidar_sensor_frame_id_; // [cite: 24, 139]
    std::string lidar_map_origin_frame_id_;
    std::string lidar_current_pose_frame_id_; // [cite: 25, 140]

    void load_static_transform_param(const std::string& param_name_prefix, const std::string& parent_frame, const std::string& child_frame) {
        std::vector<double> transform_values; // [cite: 26, 141]
        if (nh_.getParam(param_name_prefix, transform_values)) {
            if (transform_values.size() == 6) {
                geometry_msgs::TransformStamped T_stamped;
                T_stamped.header.stamp = ros::Time::now(); // [cite: 27, 142]
                T_stamped.header.frame_id = parent_frame;
                T_stamped.child_frame_id = child_frame; // [cite: 28, 143]
                T_stamped.transform.translation.x = transform_values[0];
                T_stamped.transform.translation.y = transform_values[1];
                T_stamped.transform.translation.z = transform_values[2];

                tf2::Quaternion q = rpy_to_quaternion( // [cite: 29, 144]
                    transform_values[3] * M_PI / 180.0,
                    transform_values[4] * M_PI / 180.0,
                    transform_values[5] * M_PI / 180.0
                ); // [cite: 30, 145]
                T_stamped.transform.rotation.x = q.x(); // [cite: 31, 146]
                T_stamped.transform.rotation.y = q.y();
                T_stamped.transform.rotation.z = q.z(); // [cite: 32, 147]
                T_stamped.transform.rotation.w = q.w();
                static_transforms_.push_back(T_stamped);
                ROS_INFO("已加载静态变换 %s -> %s: [t: %.3f, %.3f, %.3f], [q: %.3f, %.3f, %.3f, %.3f]",
                         parent_frame.c_str(), child_frame.c_str(),
                         T_stamped.transform.translation.x, T_stamped.transform.translation.y, T_stamped.transform.translation.z,
                         T_stamped.transform.rotation.x, T_stamped.transform.rotation.y, T_stamped.transform.rotation.z, T_stamped.transform.rotation.w);
            } else {
                ROS_ERROR("静态变换参数 '%s' 的值数量不正确 (需要6个，实际为 %zu)。将跳过此变换。", param_name_prefix.c_str(), transform_values.size()); // [cite: 33, 34, 148, 149]
            }
        } else { // [cite: 25, 149]
            ROS_ERROR("未能从参数服务器加载静态变换 '%s'。将为此变换添加单位矩阵作为备用（可能导致TF树不完整或行为错误）。请检查launch文件配置！", param_name_prefix.c_str()); // [cite: 35, 150]
            geometry_msgs::TransformStamped T_stamped_identity;
            T_stamped_identity.header.stamp = ros::Time::now();
            T_stamped_identity.header.frame_id = parent_frame; // [cite: 27, 150]
            T_stamped_identity.child_frame_id = child_frame; // [cite: 150]
            T_stamped_identity.transform.translation.x = 0; T_stamped_identity.transform.translation.y = 0; T_stamped_identity.transform.translation.z = 0; // [cite: 36, 151]
            T_stamped_identity.transform.rotation.x = 0; T_stamped_identity.transform.rotation.y = 0; T_stamped_identity.transform.rotation.z = 0; T_stamped_identity.transform.rotation.w = 1; // [cite: 37, 152]
            static_transforms_.push_back(T_stamped_identity); // [cite: 38, 153]
        }
    }

    void publish_robot_pose_callback(const ros::TimerEvent&) {
        geometry_msgs::TransformStamped T_world_base_stamped;
        try { // [cite: 39, 154]
            T_world_base_stamped = tf_buffer_.lookupTransform(world_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(0.1)); // [cite: 41, 156]
        } catch (tf2::TransformException &ex) { // [cite: 33, 42, 156, 157]
            ROS_WARN_THROTTLE(2.0, "获取TF变换 %s -> %s 失败: %s. 机器人位姿将不会发布。请检查TF树是否完整且所有相关节点（如SLAM）运行正常。",
                              world_frame_id_.c_str(), base_link_frame_id_.c_str(), ex.what());
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = T_world_base_stamped.header.stamp; // [cite: 43, 158]
        pose_stamped.header.frame_id = world_frame_id_;
        pose_stamped.pose.position.x = T_world_base_stamped.transform.translation.x;
        pose_stamped.pose.position.y = T_world_base_stamped.transform.translation.y; // [cite: 44, 159]
        pose_stamped.pose.position.z = T_world_base_stamped.transform.translation.z; // [cite: 36, 44, 159]
        pose_stamped.pose.orientation = T_world_base_stamped.transform.rotation;

        pub_base_link_pose_.publish(pose_stamped);

        tf2::Quaternion q_world_base( // [cite: 45, 160]
            T_world_base_stamped.transform.rotation.x,
            T_world_base_stamped.transform.rotation.y,
            T_world_base_stamped.transform.rotation.z,
            T_world_base_stamped.transform.rotation.w);
        tf2::Matrix3x3 m_world_base(q_world_base); // [cite: 46, 161]
        double roll, pitch, yaw;
        m_world_base.getRPY(roll, pitch, yaw); // [cite: 47, 162]
        ROS_DEBUG_THROTTLE(1.0, "机器人位姿 (World Frame) [x,y,z: %.2f,%.2f,%.2f] [RPY deg: %.1f,%.1f,%.1f]",
            pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z,
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI); // [cite: 47, 48, 162, 163]
    }

    void kinect_loop_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array_debug; // [cite: 49, 164]
        process_detections_kinect_loop(msg, kinect_loop_camera_optical_frame_id_, pub_kinect_loop_targets_world_array_, marker_array_debug);
        if(!marker_array_debug.markers.empty()){ // [cite: 50, 165]
            pub_kinect_loop_debug_markers_.publish(marker_array_debug); // [cite: 51, 166]
        }
    }

    void kinect_circle_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array_debug; // [cite: 52, 167]
        // ***修正笔误：调用 process_detections_kinect_circle***
        process_detections_kinect_circle(msg, kinect_circle_camera_optical_frame_id_, pub_kinect_circle_targets_world_array_, marker_array_debug); // [cite: 53]
        if(!marker_array_debug.markers.empty()){ // [cite: 53, 168]
            pub_kinect_circle_debug_markers_.publish(marker_array_debug); // [cite: 54, 169]
        }
    }

    // 重命名此函数以明确其用于 Kinect Circle
    void process_detections_kinect_circle(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr& msg_in,
                                     const std::string& camera_optical_frame,
                                     ros::Publisher& array_publisher,
                                     visualization_msgs::MarkerArray& marker_array_for_debug) { // [cite: 55, 170]
        if (msg_in->detections.empty()) { // [cite: 56, 171]
            return;
        }

        yolo_realsense_kinect::DetectedObject3DArray_kinect_circle msg_out_array; // [cite: 57, 172]
        msg_out_array.header.stamp = ros::Time::now(); // [cite: 58, 173]
        msg_out_array.header.frame_id = world_frame_id_; // [cite: 59, 174]
        marker_array_for_debug.markers.clear(); // [cite: 60, 175]

        int current_marker_id_for_this_array = 0; // [cite: 61, 176]

        for (const auto& det_in : msg_in->detections) {
            // 假设 DetectedObject3D_kinect_circle 包含 point_3d, class_name, confidence
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z)) {
                ROS_DEBUG("Kinect Circle: 检测到无效的输入目标点坐标 (NaN)，已跳过。"); // [cite: 62]
                continue; // [cite: 62, 177]
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame; // [cite: 63, 178]
            pt_cam.header.stamp = msg_in->header.stamp;    // [cite: 64, 179]
            pt_cam.point = det_in.point_3d;              // [cite: 65, 180]

            try { // [cite: 65, 180]
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.1)); // [cite: 66, 181]
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "Kinect Circle: 目标点TF变换从 '%s' 到 '%s' 失败: %s. 跳过此目标点。", // [cite: 67]
                                  camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue; // [cite: 67, 182]
            }

            if (should_publish_target_point_common(pt_world, det_in.class_name, det_in.confidence, marker_array_for_debug, current_marker_id_for_this_array, "kinect_circle_detections")) {
                yolo_realsense_kinect::DetectedObject3D_kinect_circle det_out = det_in; // [cite: 68, 183]
                det_out.point_3d = pt_world.point; // [cite: 69, 184]
                msg_out_array.detections.push_back(det_out); // [cite: 70, 185]
            }
        }

        if (!msg_out_array.detections.empty()) {
            array_publisher.publish(msg_out_array); // [cite: 71, 186]
        }
    }

    void process_detections_kinect_loop(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr& msg_in,
                                   const std::string& camera_optical_frame,
                                   ros::Publisher& array_publisher, // [cite: 72, 187]
                                   visualization_msgs::MarkerArray& marker_array_for_debug) {
        if (msg_in->detections.empty()) { // [cite: 73, 188]
            return;
        }

        yolo_realsense_kinect::DetectedObject3DArray_kinect_loop msg_out_array; // [cite: 74, 189]
        msg_out_array.header.stamp = ros::Time::now(); // [cite: 75, 190]
        msg_out_array.header.frame_id = world_frame_id_; // [cite: 76, 191]
        marker_array_for_debug.markers.clear();

        int current_marker_id_for_this_array = 0; // [cite: 77, 192]

        for (const auto& det_in : msg_in->detections) {
            // 假设 DetectedObject3D_kinect_loop 包含 point_3d, class_name, confidence
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z)) {
                ROS_DEBUG("Kinect Loop: 检测到无效的输入目标点坐标 (NaN)，已跳过。"); // [cite: 78, 193]
                continue;
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame; // [cite: 79, 194]
            pt_cam.header.stamp = msg_in->header.stamp;    // [cite: 80, 195]
            pt_cam.point = det_in.point_3d;              // [cite: 81, 196]

            try { // [cite: 81, 159, 196]
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.1)); // [cite: 82, 197]
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1.0, "Kinect Loop: 目标点TF变换从 '%s' 到 '%s' 失败: %s. 跳过此目标点。", // [cite: 83, 198]
                                  camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue;
            }

            if (should_publish_target_point_common(pt_world, det_in.class_name, det_in.confidence, marker_array_for_debug, current_marker_id_for_this_array, "kinect_loop_detections")) {
                yolo_realsense_kinect::DetectedObject3D_kinect_loop det_out = det_in;
                det_out.point_3d = pt_world.point; // [cite: 84, 199]
                msg_out_array.detections.push_back(det_out); // [cite: 85, 200]
            }
        }

        if (!msg_out_array.detections.empty()) {
            array_publisher.publish(msg_out_array); // [cite: 86, 201]
        }
    }

    bool should_publish_target_point_common(const geometry_msgs::PointStamped& point_in_world,
                                            const std::string& class_name,
                                            float confidence, // [cite: 87, 202]
                                            visualization_msgs::MarkerArray& marker_array,
                                            int& marker_id_counter, // [cite: 88, 203]
                                            const std::string& ns_prefix) {
        // --- 在此实现您的自定义判断逻辑 ---
        // if (confidence < 0.6) { return false; } [cite: 89, 204]
        // if (point_in_world.point.z < 0.0 || point_in_world.point.z > 5.0) { return false; } [cite: 90, 205]

        ROS_DEBUG("目标点 (World Frame, 来自 %s): [x:%.2f, y:%.2f, z:%.2f], 类别:'%s', 置信度:%.2f",
                  ns_prefix.c_str(), point_in_world.point.x, point_in_world.point.y, point_in_world.point.z,
                  class_name.c_str(), confidence);

        visualization_msgs::Marker marker; // [cite: 91, 92, 206, 207]
        marker.header = point_in_world.header; // [cite: 93, 208]
        marker.ns = ns_prefix + "_world_markers";
        marker.id = marker_id_counter++;      // [cite: 94, 209]
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point_in_world.point; // [cite: 87, 95, 210]
        marker.pose.orientation.w = 1.0; // [cite: 76, 96, 211]
        marker.scale.x = 0.15; marker.scale.y = 0.15; marker.scale.z = 0.15; // [cite: 97, 212]
        marker.color.a = 0.8; // [cite: 98, 213]
        if (ns_prefix == "kinect_circle_detections") {
            marker.color.r = 1.0; marker.color.g = 0.2; marker.color.b = 0.2; // Kinect Circle 红色系 (调整了G分量) // [cite: 99]
        } else { // Kinect Loop (默认)
            marker.color.r = 0.2; marker.color.g = 0.2; marker.color.b = 1.0; // Kinect Loop 蓝色系 (调整了R,G分量) // [cite: 100]
        }
        marker.lifetime = ros::Duration(0.5); // [cite: 101, 216]
        marker_array.markers.push_back(marker);

        visualization_msgs::Marker text_marker = marker; // [cite: 102, 103, 217, 218]
        text_marker.id = marker_id_counter++;    // [cite: 104, 219]
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING; // [cite: 105, 220]
        std::stringstream ss;
        ss << class_name << " (" << std::fixed << std::setprecision(2) << confidence << ")"; // [cite: 106, 221]
        text_marker.text = ss.str();
        text_marker.pose.position.z += 0.15; // [cite: 107, 222]
        text_marker.scale.x = 0.0; text_marker.scale.y = 0.0; text_marker.scale.z = 0.1; // [cite: 108, 223]
        text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; // [cite: 109, 224]
        marker_array.markers.push_back(text_marker);

        return true; // [cite: 110, 225]
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_world_transformer_node");
    ros::NodeHandle nh("~"); // [cite: 94, 111, 188, 226]
    TargetWorldTransformer transformer(nh);
    ros::spin();
    return 0;
}