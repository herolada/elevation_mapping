import os
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('elevation_mapping')
    config_dir = os.path.join(share_dir, 'config')
    
    config_file = LaunchConfiguration('config_file').perform(context)
    map_topic_name = LaunchConfiguration('map_topic_name').perform(context)

    list_params = [
        os.path.join(config_dir, config_file),
        os.path.join(config_dir, "postprocessing/postprocessor_pipeline.yaml")
    ]

    remappings = [
        ('elevation_map', map_topic_name),
    ]

    return [
        Node(
            package='elevation_mapping',
            executable='gridmap_to_pointcloud_node',
            name='gridmap_to_pointcloud_node',
            output='screen',
            parameters=list_params,
            remappings=remappings,
        ),
        Node(
            package='elevation_mapping',
            executable='elevation_mapping',
            name='elevation_mapping',
            output='screen',
            parameters=list_params,
            remappings=remappings,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('elevation_mapping'), 'config'),
            description='Config file.'
        ),
        DeclareLaunchArgument(
            'map_topic_name',
            default_value="elevation_map",
            description='Name of the elevation map topic published by elevation_mapping.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
