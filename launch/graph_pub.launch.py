#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('graph_pub')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        #default_value='mapaTransformSoloLab',
        default_value='complete_lab',
        description='Name of the map file (without extension)'
    )
    
    xml_file_arg = DeclareLaunchArgument(
        'xml_file',
        #default_value='wireless_data_transform_lab.xml',
        default_value='wireless_data_amcl.xml',
        description='Name of the wireless data XML file'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the map and pointcloud'
    )
    
    # Define nodes
    map_publisher_node = Node(
        package='graph_pub',
        executable='map_publisher',
        name='map_publisher',
        parameters=[{
            'map_file': LaunchConfiguration('map_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_frequency': 1.0,
        }],
        output='screen'
    )
    
    wireless_data_publisher_node = Node(
        package='graph_pub',
        executable='wireless_data_publisher',
        name='wireless_data_publisher',
        parameters=[{
            'xml_file': LaunchConfiguration('xml_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_frequency': 1.0,
            'map_file': LaunchConfiguration('map_file'),
        }],
        output='screen'
    )
    
    # RViz node
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'graph_pub.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        xml_file_arg,
        frame_id_arg,
        map_publisher_node,
        wireless_data_publisher_node,
        rviz_node,
    ])
