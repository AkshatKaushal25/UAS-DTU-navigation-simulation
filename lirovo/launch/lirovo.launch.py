from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namePackage = 'lirovo'
    slam_params_path = os.path.join(get_package_share_directory(namePackage), 'config','slam_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'autostart': 'True',
            'map': 'map',  
        }.items()
    )

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'lidar',
                'transform_tolerance': 0.1,
                'min_height': -1.0,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max': +3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.8,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50
            }],
            remappings=[
                ('cloud_in', '/bf_lidar/point_cloud_out'),
                ('scan', '/scan'),
            ],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_node',
            output='screen',
            parameters=[slam_params_path]
        ),
    
        Node(
            package='lirovo',
            executable='mavros_bridge',
            name='mavros_bridge',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar'],
            name='static_tf_lidar'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        #     name='static_tf_odom'
        # ),
        
        nav2_launch
    ])
