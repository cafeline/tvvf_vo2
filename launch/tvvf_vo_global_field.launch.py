#!/usr/bin/env python3
"""
TVVF-VO Global Field Navigation Launch File

This launch file starts the TVVF-VO navigation system with global field generation.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('tvvf_vo_c')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_viz_arg = DeclareLaunchArgument(
        'enable_vector_field_viz',
        default_value='true',
        description='Enable vector field visualization'
    )
    
    # TVVF-VO node with global field generator
    tvvf_vo_node = Node(
        package='tvvf_vo_c',
        executable='tvvf_vo_c_node',
        name='tvvf_vo_global_field',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # TVVF基本パラメータ
            'k_attraction': 1.0,
            'k_repulsion': 2.0,
            'influence_radius': 3.0,
            
            # 経路統合パラメータ
            'k_path_attraction': 2.0,
            'path_influence_radius': 2.0,
            'lookahead_distance': 1.5,
            
            # 障害物回避関連パラメータ
            'safety_margin': 0.2,
            
            # ロボット仕様パラメータ
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 2.0,
            'robot_radius': 0.25,
            
            # フレーム名
            'base_frame': 'base_footprint',
            'global_frame': 'map',
            
            # 制御関連
            'goal_tolerance': 0.2,
            
            # デバッグ/テスト用（TFが利用できない場合）
            'use_fixed_robot_pose': True,  # TFの代わりに固定位置を使用
            'fixed_robot_x': 1.0,          # ロボットの初期X座標
            'fixed_robot_y': 1.0,          # ロボットの初期Y座標
            'fixed_robot_theta': 0.0,      # ロボットの初期角度
            
            # 可視化設定
            'enable_vector_field_viz': LaunchConfiguration('enable_vector_field_viz'),
            'vector_field_resolution': 0.5,
            'vector_field_range': 5.0,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('map', '/map'),
            ('clicked_point', '/clicked_point'),
            ('planned_path', '/planned_path'),
        ]
    )
    
    # RViz2 for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'tvvf_vo_global.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=LaunchConfiguration('enable_vector_field_viz')
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_viz_arg,
        tvvf_vo_node,
        # rviz_node,  # コメントアウト（RViz設定ファイルがまだない）
    ])
