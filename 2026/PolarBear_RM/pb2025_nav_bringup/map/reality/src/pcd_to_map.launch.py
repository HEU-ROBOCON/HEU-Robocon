#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 第一步：把 PCD 文件发布成 PointCloud2 话题
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_to_pointcloud',
            output='screen',
            parameters=[{'frame_id': 'map'}],
            arguments=[
                '/home/why/fast_ws/src/FAST_LIO_ROS2/PCD/scans.pcd',
                '0.05',   # 每 0.05 秒发布一次（20Hz），足够快
            ],
            remappings=[
                ('cloud_pcd', '/my_pcd_points')
            ]
        ),

        # 第二步：用 pointcloud_to_laserscan 把点云投成 2D 激光（可选，后面用不到但防止冲突）
        # （这里我们直接用 depthimage_to_laserscan 更稳，但最简单还是直接用 map_saver_cli 监听点云转地图）

        # 第三步：直接用 nav2 的 map_saver_cli 监听点云并生成地图（最新版支持）
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver_cli',
            output='screen',
            arguments=[
                '-f', '/home/why/fast_ws/src/FAST_LIO_ROS2/PCD/scans/scans',
                '--ros-args',
                '-p', 'topic:=/projected_map',           # 如果你 PCD 转出来的就是 /projected_map，用这个
                '-p', 'save_map_timeout:=120000',        # 等 120 秒，绝对够
            ]
        )
    ])