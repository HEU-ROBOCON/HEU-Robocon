#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

// 自定义的Kinect Loop和Circle检测消息类型
#include <yolo_realsense_kinect/DetectedObject3D_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_loop.h>
#include <yolo_realsense_kinect/DetectedObject3D_kinect_circle.h>
#include <yolo_realsense_kinect/DetectedObject3DArray_kinect_circle.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <cmath>   // For M_PI
#include <iomanip> // For std::setprecision in stringstream
#include <sstream> // For std::stringstream

// 用于将 roll, pitch, yaw (弧度) 转换为四元数
tf2::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // [cite: 2, 117]
    return q;
}

class TargetWorldTransformer
{
public:
    TargetWorldTransformer(ros::NodeHandle &nh) : nh_(nh),
                                                  tf_listener_(tf_buffer_)
    {
        // --- 1. 从参数服务器加载帧 ID ---
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
        nh_.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
        nh_.param<std::string>("kinect_loop_camera_optical_frame_id", kinect_loop_camera_optical_frame_id_, "kinect_loop_color_optical_frame");
        nh_.param<std::string>("kinect_circle_camera_optical_frame_id", kinect_circle_camera_optical_frame_id_, "kinect_circle_color_optical_frame");
        nh_.param<std::string>("lidar_sensor_frame_id", lidar_sensor_frame_id_, "lidar_link");             
        nh_.param<std::string>("lidar_map_origin_frame_id", lidar_map_origin_frame_id_, "camera_init");    
        nh_.param<std::string>("lidar_current_pose_frame_id", lidar_current_pose_frame_id_, "aft_mapped"); 

        ROS_INFO("帧ID配置: world='%s', base_link='%s', kinect_loop_cam='%s', kinect_circle_cam='%s', lidar_sensor='%s', lidar_map_origin='%s', lidar_current_pose='%s'",
                 world_frame_id_.c_str(), base_link_frame_id_.c_str(), kinect_loop_camera_optical_frame_id_.c_str(),
                 kinect_circle_camera_optical_frame_id_.c_str(), lidar_sensor_frame_id_.c_str(),
                 lidar_map_origin_frame_id_.c_str(), lidar_current_pose_frame_id_.c_str());

        // --- 2. 从参数服务器加载并发布静态变换 ---
        load_static_transform_param("world_to_lidar_map_origin", world_frame_id_, lidar_map_origin_frame_id_);                         
        load_static_transform_param("base_link_to_kinect_loop_camera", base_link_frame_id_, kinect_loop_camera_optical_frame_id_);     
        load_static_transform_param("base_link_to_kinect_circle_camera", base_link_frame_id_, kinect_circle_camera_optical_frame_id_); 
        load_static_transform_param("lidar_sensor_to_base_link", lidar_sensor_frame_id_, base_link_frame_id_);                         
        load_static_transform_param("lidar_current_pose_frame_to_lidar_link", lidar_current_pose_frame_id_, lidar_sensor_frame_id_);

        if (!static_transforms_.empty())
        {
            static_broadcaster_.sendTransform(static_transforms_);                
            ROS_INFO("已成功发布 %zu 个静态TF变换。", static_transforms_.size()); 
            ROS_INFO("sender TF %li", static_transforms_.size());
        }
        else
        {
            ROS_WARN("没有从参数服务器加载到任何有效的静态TF变换。TF树可能不完整。");
        }

        // --- 3. 初始化ROS订阅器 ---
        sub_kinect_loop_detections_ = nh_.subscribe("/yolo/kinect/loop/detections_3d", 10, &TargetWorldTransformer::kinect_loop_detections_callback, this);       
        sub_kinect_circle_detections_ = nh_.subscribe("/yolo/kinect/circle/detections_3d", 10, &TargetWorldTransformer::kinect_circle_detections_callback, this); 

        // --- 4. 初始化ROS发布器 ---
        pub_base_link_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose_in_world", 10);

        // 发布篮筐和圈在世界坐标系下的位置
        pub_kinect_loop_targets_world_array_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_loop>("/kinect/loop/targets_in_world", 10);      
        pub_kinect_circle_targets_world_array_ = nh_.advertise<yolo_realsense_kinect::DetectedObject3DArray_kinect_circle>("/kinect/circle/targets_in_world", 10);

        // 发布debug mark点，用于调试
        pub_kinect_loop_debug_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/loop/targets_in_world_marker_debug", 10);     
        pub_kinect_circle_debug_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/kinect/circle/targets_in_world_marker_debug", 10);

        // --- 5. 创建定时器用于定期发布机器人位姿 ---
        pose_publish_timer_ = nh_.createTimer(ros::Duration(1.0 / 30.0), &TargetWorldTransformer::publish_robot_pose_callback, this); 

        ROS_INFO("目标点世界坐标转换节点 (transformer_node) 已成功初始化。");
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> static_transforms_;

    // 订阅篮筐和圈的检测结果，在相机坐标系下
    ros::Subscriber sub_kinect_loop_detections_;
    ros::Subscriber sub_kinect_circle_detections_;

    ros::Publisher pub_base_link_pose_;
    ros::Publisher pub_kinect_loop_targets_world_array_;
    ros::Publisher pub_kinect_circle_targets_world_array_;
    ros::Publisher pub_kinect_loop_debug_markers_;
    ros::Publisher pub_kinect_circle_debug_markers_;

    ros::Timer pose_publish_timer_;

    std::string world_frame_id_;
    std::string base_link_frame_id_;
    std::string kinect_loop_camera_optical_frame_id_;
    std::string kinect_circle_camera_optical_frame_id_;
    std::string lidar_sensor_frame_id_;
    std::string lidar_map_origin_frame_id_;
    std::string lidar_current_pose_frame_id_;

    // 静态变换广播函数，用于广播相机坐标系和底盘坐标系在世界坐标系下的位置
    void load_static_transform_param(const std::string &param_name_prefix, const std::string &parent_frame, const std::string &child_frame)
    {
        std::vector<double> transform_values;
        if (nh_.getParam(param_name_prefix, transform_values))
        {
            if (transform_values.size() == 6)
            {
                geometry_msgs::TransformStamped T_stamped;
                T_stamped.header.stamp = ros::Time::now();
                T_stamped.header.frame_id = parent_frame;
                T_stamped.child_frame_id = child_frame;
                T_stamped.transform.translation.x = transform_values[0];
                T_stamped.transform.translation.y = transform_values[1];
                T_stamped.transform.translation.z = transform_values[2];

                tf2::Quaternion q = rpy_to_quaternion(
                    transform_values[3] * M_PI / 180.0,
                    transform_values[4] * M_PI / 180.0,
                    transform_values[5] * M_PI / 180.0);
                T_stamped.transform.rotation.x = q.x();
                T_stamped.transform.rotation.y = q.y();
                T_stamped.transform.rotation.z = q.z();
                T_stamped.transform.rotation.w = q.w();
                static_transforms_.push_back(T_stamped);
                ROS_INFO("已加载静态变换 %s -> %s: [t: %.3f, %.3f, %.3f], [q: %.3f, %.3f, %.3f, %.3f]",
                         parent_frame.c_str(), child_frame.c_str(),
                         T_stamped.transform.translation.x, T_stamped.transform.translation.y, T_stamped.transform.translation.z,
                         T_stamped.transform.rotation.x, T_stamped.transform.rotation.y, T_stamped.transform.rotation.z, T_stamped.transform.rotation.w);
            }
            else
            {
                ROS_ERROR("静态变换参数 '%s' 的值数量不正确 (需要6个，实际为 %zu)。将跳过此变换。", param_name_prefix.c_str(), transform_values.size()); 
            }
        }
        else
        {
            ROS_ERROR("未能从参数服务器加载静态变换 '%s'。将为此变换添加单位矩阵作为备用（可能导致TF树不完整或行为错误）。请检查launch文件配置！", param_name_prefix.c_str()); 
            geometry_msgs::TransformStamped T_stamped_identity;
            T_stamped_identity.header.stamp = ros::Time::now();
            T_stamped_identity.header.frame_id = parent_frame;
            T_stamped_identity.child_frame_id = child_frame;
            T_stamped_identity.transform.translation.x = 0;
            T_stamped_identity.transform.translation.y = 0;
            T_stamped_identity.transform.translation.z = 0;
            T_stamped_identity.transform.rotation.x = 0;
            T_stamped_identity.transform.rotation.y = 0;
            T_stamped_identity.transform.rotation.z = 0;
            T_stamped_identity.transform.rotation.w = 1;
            static_transforms_.push_back(T_stamped_identity);
        }
    }

    // 发布底盘在世界坐标系下的位姿
    void publish_robot_pose_callback(const ros::TimerEvent &)
    {
        geometry_msgs::TransformStamped T_world_base_stamped;
        try
        {
            T_world_base_stamped = tf_buffer_.lookupTransform(world_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "获取TF变换 %s -> %s 失败: %s. 机器人位姿将不会发布。请检查TF树是否完整且所有相关节点（如SLAM）运行正常。",
                              world_frame_id_.c_str(), base_link_frame_id_.c_str(), ex.what());
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = T_world_base_stamped.header.stamp;
        pose_stamped.header.frame_id = world_frame_id_;
        pose_stamped.pose.position.x = T_world_base_stamped.transform.translation.x;
        pose_stamped.pose.position.y = T_world_base_stamped.transform.translation.y;
        pose_stamped.pose.position.z = T_world_base_stamped.transform.translation.z;
        pose_stamped.pose.orientation = T_world_base_stamped.transform.rotation;

        pub_base_link_pose_.publish(pose_stamped);

        tf2::Quaternion q_world_base(
            T_world_base_stamped.transform.rotation.x,
            T_world_base_stamped.transform.rotation.y,
            T_world_base_stamped.transform.rotation.z,
            T_world_base_stamped.transform.rotation.w);
        tf2::Matrix3x3 m_world_base(q_world_base);
        double roll, pitch, yaw;
        m_world_base.getRPY(roll, pitch, yaw);
        ROS_DEBUG_THROTTLE(1.0, "机器人位姿 (World Frame) [x,y,z: %.2f,%.2f,%.2f] [RPY deg: %.1f,%.1f,%.1f]",
                           pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z,
                           roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }

    // 订阅篮筐在相机坐标系下的检测结果，并发布到世界坐标系下
    void kinect_loop_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr &msg)
    {
        // visualization_msgs::MarkerArray marker_array_debug;

        // 处理篮筐在相机坐标系下的检测结果，并发布到世界坐标系下
        // process_detections_kinect_loop(msg, kinect_loop_camera_optical_frame_id_, pub_kinect_loop_targets_world_array_, marker_array_debug);

        // if(!marker_array_debug.markers.empty()){
        //     pub_kinect_loop_debug_markers_.publish(marker_array_debug);
        // }
        pub_kinect_loop_targets_world_array_.publish(msg); // 直接发布消息
    }

    void kinect_circle_detections_callback(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr &msg)
    {
        visualization_msgs::MarkerArray marker_array_debug;

        // 处理圆在相机坐标系下的检测结果，并发布到世界坐标系下
        process_detections_kinect_circle(msg, kinect_circle_camera_optical_frame_id_, pub_kinect_circle_targets_world_array_, marker_array_debug);

        if (!marker_array_debug.markers.empty())
        {
            pub_kinect_circle_debug_markers_.publish(marker_array_debug);
        }
    }

    // 处理圆在kinect相机坐标系下的检测结果
    void process_detections_kinect_circle(const yolo_realsense_kinect::DetectedObject3DArray_kinect_circle::ConstPtr &msg_in,
                                          const std::string &camera_optical_frame,
                                          ros::Publisher &array_publisher,
                                          visualization_msgs::MarkerArray &marker_array_for_debug)
    {
        if (msg_in->detections.empty())
        {

            array_publisher.publish(msg_in);
            return;
        }

        yolo_realsense_kinect::DetectedObject3DArray_kinect_circle msg_out_array;
        msg_out_array.header.stamp = ros::Time::now();
        msg_out_array.header.frame_id = world_frame_id_;
        marker_array_for_debug.markers.clear();

        int current_marker_id_for_this_array = 0;

        for (const auto &det_in : msg_in->detections)
        {
            // 判断圆的位置在合理范围内
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z))
            {
                ROS_DEBUG("Kinect Circle: 检测到无效的输入目标点坐标 (NaN)，已跳过。");
                continue;
            }

            geometry_msgs::PointStamped pt_cam, pt_world; // pt_cam：圆在相机坐标系下的位置；pt_world：圆在世界坐标系下的位置
            pt_cam.header.frame_id = camera_optical_frame;
            pt_cam.header.stamp = msg_in->header.stamp;
            pt_cam.point = det_in.point_3d;

            try
            {
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.1));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "Kinect Circle: 目标点TF变换从 '%s' 到 '%s' 失败: %s. 跳过此目标点。",
                                  camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue;
            }

            if (should_publish_target_point_common(pt_world, det_in.class_name, marker_array_for_debug, current_marker_id_for_this_array, "kinect_circle_detections"))
            {
                yolo_realsense_kinect::DetectedObject3D_kinect_circle det_out = det_in;
                det_out.point_3d = pt_world.point;
                msg_out_array.detections.push_back(det_out);
            }
        }

        if (!msg_out_array.detections.empty())
        {
            array_publisher.publish(msg_out_array);
        }
    }

    // 处理圆在kinect相机坐标系下的检测结果
    void process_detections_kinect_loop(const yolo_realsense_kinect::DetectedObject3DArray_kinect_loop::ConstPtr &msg_in,
                                        const std::string &camera_optical_frame,
                                        ros::Publisher &array_publisher,
                                        visualization_msgs::MarkerArray &marker_array_for_debug)
    {
        if (msg_in->detections.empty())
        {

            array_publisher.publish(msg_in);
            return;
        }

        yolo_realsense_kinect::DetectedObject3DArray_kinect_loop msg_out_array;
        msg_out_array.header.stamp = ros::Time::now();
        msg_out_array.header.frame_id = world_frame_id_;
        marker_array_for_debug.markers.clear();

        int current_marker_id_for_this_array = 0;

        for (const auto &det_in : msg_in->detections)
        {
            // 假设 DetectedObject3D_kinect_loop 包含 point_3d, class_name, confidence
            if (std::isnan(det_in.point_3d.x) || std::isnan(det_in.point_3d.y) || std::isnan(det_in.point_3d.z))
            {
                ROS_DEBUG("Kinect Loop: 检测到无效的输入目标点坐标 (NaN)，已跳过。"); 
                continue;
            }

            geometry_msgs::PointStamped pt_cam, pt_world;
            pt_cam.header.frame_id = camera_optical_frame;
            pt_cam.header.stamp = msg_in->header.stamp;
            pt_cam.point = det_in.point_3d;

            try
            {
                tf_buffer_.transform(pt_cam, pt_world, world_frame_id_, ros::Duration(0.1));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "Kinect Loop: 目标点TF变换从 '%s' 到 '%s' 失败: %s. 跳过此目标点。",
                                  camera_optical_frame.c_str(), world_frame_id_.c_str(), ex.what());
                continue;
            }

            if (should_publish_target_point_common(pt_world, det_in.class_name, marker_array_for_debug, current_marker_id_for_this_array, "kinect_loop_detections"))
            {
                yolo_realsense_kinect::DetectedObject3D_kinect_loop det_out = det_in;
                det_out.point_3d = pt_world.point;
                msg_out_array.detections.push_back(det_out);
            }
        }

        if (!msg_out_array.detections.empty())
        {
            array_publisher.publish(msg_out_array);
        }
    }

    bool should_publish_target_point_common(const geometry_msgs::PointStamped &point_in_world,
                                            const std::string &class_name,
                                            visualization_msgs::MarkerArray &marker_array,
                                            int &marker_id_counter,
                                            const std::string &ns_prefix)
    {

        ROS_DEBUG("目标点 (World Frame, 来自 %s): [x:%.2f, y:%.2f, z:%.2f], 类别:'%s'",
                  ns_prefix.c_str(), point_in_world.point.x, point_in_world.point.y, point_in_world.point.z,
                  class_name.c_str());

        visualization_msgs::Marker marker;
        marker.header = point_in_world.header;
        marker.ns = ns_prefix + "_world_markers";
        marker.id = marker_id_counter++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point_in_world.point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 0.8;
        if (ns_prefix == "kinect_circle_detections")
        {
            marker.color.r = 1.0;
            marker.color.g = 0.2;
            marker.color.b = 0.2; // Kinect Circle 红色系 (调整了G分量) 
        }
        else
        { // Kinect Loop (默认)
            marker.color.r = 0.2;
            marker.color.g = 0.2;
            marker.color.b = 1.0; // Kinect Loop 蓝色系 (调整了R,G分量) 
        }
        marker.lifetime = ros::Duration(0.5);
        marker_array.markers.push_back(marker);

        visualization_msgs::Marker text_marker = marker;
        text_marker.id = marker_id_counter++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        std::stringstream ss;
        ss << class_name << " (" << std::fixed << std::setprecision(2) << ")";
        text_marker.text = ss.str();
        text_marker.pose.position.z += 0.15;
        text_marker.scale.x = 0.0;
        text_marker.scale.y = 0.0;
        text_marker.scale.z = 0.1;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        marker_array.markers.push_back(text_marker);

        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_world_transformer_node");
    ros::NodeHandle nh("~");
    TargetWorldTransformer transformer(nh);
    ros::spin();
    return 0;
}