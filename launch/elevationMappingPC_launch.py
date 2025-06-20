import os
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('elevation_mapping')
    config_dir = os.path.join(share_dir, 'config')
    xacro_path = os.path.join(config_dir, 'robot.urdf.xacro')
    list_params = []
    # for filee in ["robots/ground_truth_demo.yaml","elevation_maps/long_range.yaml","sensor_processors/realsense_d435.yaml","postprocessing/postprocessor_pipeline.yaml"]:
    for filee in ["robots/marv.yaml","postprocessing/postprocessor_pipeline.yaml"]:
        list_params.append(os.path.join(config_dir, filee))
        
    return launch.LaunchDescription(
        [
            Node(
                package='elevation_mapping',
                executable='pose_publisher_node',
                name='pose_publisher_node',
                output='screen',
                parameters=list_params,
            ),
            Node(
                package='elevation_mapping',
                executable='gridmap_to_pointcloud_node',
                name='gridmap_to_pointcloud_node',
                output='screen',
                parameters=list_params,
            ),
            Node(
                package='elevation_mapping',
                executable='elevation_mapping',
                name='elevation_mapping',
                output='screen',
                parameters=list_params,
                # arguments=['--ros-args', '--log-level', 'DEBUG']
            ),
            #launch_ros.actions.Node(
            #package='robot_state_publisher',
            #executable='robot_state_publisher',
            #name='robot_state_publisher',
            #output='screen',
            #parameters=[{
            #    'robot_description': launch.substitutions.Command(['xacro', ' ', xacro_path])
            #}]
        #)
            # launch_ros.actions.Node(
            #     package= 'rviz2',
            #     executable= 'rviz2',
            #     name= 'rviz',
            #     arguments= ['--display-config', os.path.join(share_dir, 'rviz2', 'custom_rviz2.rviz')],
            #     output= 'screen'
            # )
        ]
    )
