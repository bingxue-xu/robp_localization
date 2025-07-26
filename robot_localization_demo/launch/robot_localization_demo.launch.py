#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Positioning system node
        Node(
            package='robot_localization_demo',
            executable='positioning_system_node',
            name='turtle1_positioning_system_node',
            arguments=['-f', '1.0', '-x', '0.2', '-y', '0.2', '-t', '0.2', '-v'],
            output='screen'
        ),
        
        # Odometry node  
        Node(
            package='robot_localization_demo',
            executable='odometry_node',
            name='turtle1_odometry_node',
            arguments=['-f', '20.0', '-x', '0.05', '-X', '0.0', '-t', '0.0', '-T', '0.02', '-v'],
            output='screen'
        ),
        
        # EKF localization node for odom frame
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_localization_ekf_node_odom',
            parameters=[{
                'frequency': 10.0,
                'sensor_timeout': 0.2,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'print_diagnostics': True,
                'twist0': 'turtle1/sensors/twist',
                'twist0_differential': False,
                'twist0_config': [False, False, False, False, False, False,
                                 True,  True,  False, False, False, True,
                                 False, False, False]
            }],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_twist')
            ]
        ),
        
        # EKF localization node for map frame
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_localization_ekf_node_map',
            parameters=[{
                'frequency': 10.0,
                'sensor_timeout': 0.2,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'map',
                'twist0': 'turtle1/sensors/twist',
                'twist0_config': [False, False, False, False, False, False,
                                 True,  True,  False, False, False, True,
                                 False, False, False],
                'pose0': 'turtle1/sensors/pose',
                'pose0_config': [True,  True,  False, False, False, True,
                                False, False, False, False, False, False,
                                False, False, False]
            }],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_map')
            ]
        ),
        
        # Transformation visualization node
        Node(
            package='robot_localization_demo',
            executable='transformation_visualization_node',
            name='transformation_visualization_node'
        )
    ])
