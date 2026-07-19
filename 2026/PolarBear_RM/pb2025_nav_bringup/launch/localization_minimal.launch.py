#!/usr/bin/env python3
"""
Minimal MID-360 global localization launch.

Core localization nodes:
  1. livox_ros_driver2
  2. point_lio
  3. small_gicp_relocalization

Optional visualization nodes:
  4. nav2_map_server
  5. nav2_lifecycle_manager
  6. rviz2

TF chain:
  map -> <lio_frame>        : small_gicp_relocalization
  <lio_frame> -> aft_mapped : Point-LIO

Radar pose:
  ros2 run tf2_ros tf2_echo map aft_mapped
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    lio_frame = LaunchConfiguration("lio_frame")

    use_map = LaunchConfiguration("use_map")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # 使用与原项目相同的参数解析方式。
    # 它会展开配置文件中的 $(find-pkg-share ...)。
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace",
    )

    # PCD 和 YAML 建议使用同一个 world 名称
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="red_11",
        description="Map filename without extension",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir,
            "config",
            "reality",
            "nav2_params.yaml",
        ),
        description="Reality parameter file",
    )

    declare_prior_pcd_file = DeclareLaunchArgument(
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
        description="Full path to prior PCD map",
    )

    declare_map_yaml_file = DeclareLaunchArgument(
        "map_yaml_file",
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    bringup_dir,
                    "map",
                    "reality",
                    "",
                )
            ),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to occupancy map YAML",
    )

    declare_lio_frame = DeclareLaunchArgument(
        "lio_frame",
        default_value="camera_init",
        description="Frame ID of /cloud_registered",
    )

    # 只需要坐标时关闭，降低资源占用
    declare_use_map = DeclareLaunchArgument(
        "use_map",
        default_value="False",
        description="Start map_server for RViz map visualization",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Start RViz",
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.expanduser(
            "~/RC/PolarBear_RM/rviz/localization_minimal.rviz"
        ),
        description="Full path to RViz configuration file",
    )

    # 二维栅格地图，仅用于可视化
    map_server = Node(
        condition=IfCondition(use_map),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "yaml_filename": map_yaml_file,
                "topic_name": "map",
                "frame_id": "map",
                "use_sim_time": False,
            }
        ],
    )

    # 自动激活 map_server
    map_lifecycle_manager = Node(
        condition=IfCondition(use_map),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    # MID-360 驱动
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        namespace=namespace,
        output="screen",
        parameters=[configured_params],
    )

    # Point-LIO
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        namespace=namespace,
        output="screen",
        parameters=[
            configured_params,
            {
                "prior_pcd.enable": False,
                "runtime_pos_log_enable": False,

                "publish.path_en": False,
                "publish.scan_publish_en": True,
                "publish.scan_bodyframe_pub_en": False,
                "publish.tf_send_en": True,

                "pcd_save.pcd_save_en": False,
            },
        ],
    )

    # 全局点云重定位
    small_gicp = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        name="small_gicp_relocalization",
        namespace=namespace,
        output="screen",
        parameters=[
            configured_params,
            {
                "prior_pcd_file": prior_pcd_file,
                "input_cloud_topic": "/cloud_registered",

                "map_frame": "map",
                "odom_frame": lio_frame,

                # 最小架构中不使用 base_footprint 外参修正
                "base_frame": lio_frame,
                "lidar_frame": lio_frame,
                "robot_base_frame": lio_frame,

                # 降低 CPU 占用
                "num_threads": 2,
            },
        ],
    )

    # 使用用户保存的 RViz 配置
    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="screen",
        arguments=[
            "-d",
            rviz_config_file,
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_world,
            declare_params_file,
            declare_prior_pcd_file,
            declare_map_yaml_file,
            declare_lio_frame,
            declare_use_map,
            declare_use_rviz,
            declare_rviz_config_file,

            livox_driver,

            TimerAction(
                period=2.0,
                actions=[point_lio],
            ),

            TimerAction(
                period=7.0,
                actions=[small_gicp],
            ),

            # 可选地图显示
            TimerAction(
                period=8.0,
                actions=[
                    map_server,
                    map_lifecycle_manager,
                ],
            ),

            # RViz 最后启动，减少启动竞争
            TimerAction(
                period=10.0,
                actions=[rviz],
            ),
        ]
    )