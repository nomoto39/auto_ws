#!/usr/bin/env python3
# config:utf-8
"""
事前地図なし自律走行システムを起動するLaunchファイル。
SLAMエンジンとしてFAST-LIOを使用し、nav2と連携させる。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument,EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # --- 1. センサーデータ供給 (Livox Driver) ---
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'rviz_MID360_launch.py')
        ),
        # launch_arguments={'rviz': 'false'}.items()
    )

    # --- 2. SLAM (Fast-LIO) ---
    # これにより動的なTF (lidar_map -> sensor) をFast-LIOが配信
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping_mid360.launch.py')
        ),
        # launch_arguments={'rviz': 'false'}.items()
    )
    
    # 1. map -> odom の静的TF（ロボット足元からlidarまでの座標変換）
    map_to_lidar_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )

    # 2. body -> base_link の静的TF （lidarからロボット足元までの座標変換）
    sensor_to_map_body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_link',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'body', '--child-frame-id', 'base_link'],
        output='screen'
    )

    # 3. base_link -> livox_frame の静的TF（ロボット足元からLiDARまでの座標変換）
    # MID360を下向きに取り付けた場合: roll='3.14159'（180度回転）
    static_tf_base_link_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_livox',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        output='screen'
    )

    pkg_robot_controller = get_package_share_directory('robot_controller')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(pkg_robot_controller, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # PointCloud to LaserScan
    # Converts 3D PointCloud2 from Livox to 2D LaserScan for Nav2 Costmap
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': -0.2,
            'max_height': 1.0,
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[('cloud_in', '/livox/lidar')],
        output='screen'
    )

    # Include Nav2 Navigation Launch (no map server, no amcl)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )

    # RViz2 with Nav2 config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ros2とマイコンの通信ノード
    ros_serial_node = Node(
        package='ros2serial_arduino',
        executable='serial_send_node',
        name='serial_send_node',
        output='screen',
        # parameters=[{'teleop_button_index': 3}] # Bボタン (ELECOM)
                                                # 使用するコントローラーとボタンに合わせて値を設定
    )

    # Nav2を5秒遅延起動
    nav2_delayed_launch = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    # --- LaunchDescriptionの構築 ---
    ld = LaunchDescription()
    ld.add_action(livox_driver_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(map_to_lidar_map_tf)
    ld.add_action(sensor_to_map_body_tf)
    ld.add_action(static_tf_base_link_to_livox)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(nav2_delayed_launch)
    ld.add_action(rviz_node)
    # ld.add_action(ros_serial_node)
    return ld
