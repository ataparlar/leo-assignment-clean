from email.policy import default
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
import pathlib
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pcd_file1 = LaunchConfiguration('pcd_file1')
    pcd_file2 = LaunchConfiguration('pcd_file2')

    pcd_file1_launch_argument = DeclareLaunchArgument(
        'pcd_file1',
        default_value=""
    )
    pcd_file2_launch_argument = DeclareLaunchArgument(
        'pcd_file2',
        default_value=""
    )

    param_file_name = "pcd_config.yaml"

    param_file_path = str(pathlib.Path(__file__).parents[5])
    # /home/ataparlar/leo_ws
    param_file_path += '/src/' + "leo-assignment-clean" + "/config/" + param_file_name

    point_cloud_registering_node = Node(
        package='point_cloud_registering',
        namespace="point_cloud_registering",
        executable='point_cloud_registration',
        name='point_cloud_registering',
        parameters=[
            param_file_path
        ]
        
        # Also we have a chance like below
        #     parameters=[
        #         {"param1": "value1"},
        #         {"param2": "value2"}
        #     ]
        
    )

    return LaunchDescription([
        pcd_file1_launch_argument,
        pcd_file2_launch_argument,
        point_cloud_registering_node
    ])
