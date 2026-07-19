#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="MAP",
        description="先验PCD名称，不包含.pcd后缀",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir,
            "config",
            "reality",
            "nav2_params.yaml",
        ),
        description="定位参数文件",
    )

    declare_prior_pcd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    bringup_dir,
                    "pcd",
                    "reality",
                    "",
                )
            ),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="先验点云文件",
    )

    declare_use_robot_state_pub = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="是否启动robot_state_publisher",
    )

    # 迷你主机建议默认不启动RViz，等定位稳定后手动打开
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="是否启动RViz",
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "robot_state_publisher_launch.py",
            )
        ),
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": "",
            "use_sim_time": "False",
        }.items(),
    )

    # 1. MID-360驱动
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        parameters=[params_file],
    )

    # 2. Point-LIO
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        parameters=[
            params_file,
            {
                "prior_pcd.prior_pcd_map_path": prior_pcd_file,
            },
        ],
    )

    # 3. 将Point-LIO输出转换到odom坐标系
    loam_interface = Node(
        package="loam_interface",
        executable="loam_interface_node",
        name="loam_interface",
        output="screen",
        parameters=[params_file],
    )

    # 4. 发布odom→base_footprint以及车体里程计
    sensor_scan_generation = Node(
        package="sensor_scan_generation",
        executable="sensor_scan_generation_node",
        name="sensor_scan_generation",
        output="screen",
        parameters=[params_file],
    )

    # 5. GICP先验地图重定位，发布map→odom
    small_gicp = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        name="small_gicp_relocalization",
        output="screen",
        parameters=[
            params_file,
            {
                "prior_pcd_file": prior_pcd_file,
                "input_cloud_topic": "registered_scan",
            },
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": "",
            "use_sim_time": "False",
            "rviz_config": os.path.join(
                bringup_dir,
                "rviz",
                "nav2_default_view.rviz",
            ),
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_world)
    ld.add_action(declare_params_file)
    ld.add_action(declare_prior_pcd)
    ld.add_action(declare_use_robot_state_pub)
    ld.add_action(declare_use_rviz)

    # 先建立静态外参TF并启动雷达
    ld.add_action(robot_state_publisher)
    ld.add_action(livox_driver)

    # 延迟启动，降低启动竞态
    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[point_lio],
        )
    )

    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[
                loam_interface,
                sensor_scan_generation,
            ],
        )
    )

    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[small_gicp],
        )
    )

    ld.add_action(
        TimerAction(
            period=12.0,
            actions=[rviz],
        )
    )

    return ld
