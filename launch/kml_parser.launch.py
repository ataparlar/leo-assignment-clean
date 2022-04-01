from email.policy import default
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
import pathlib
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    kml_file_launch_argument = DeclareLaunchArgument(
        'kml_file',
        default_value=""
    )

    param_file_name = "kml_config.yaml"

    param_file_path = str(pathlib.Path(__file__).parents[5])
    # /home/ataparlar/leo_ws
    param_file_path += '/src/' + "point_cloud_registering" + "/config/" + param_file_name

    kml_parser_node = Node(
        package='point_cloud_registering',
        namespace="kml_parser",
        executable='kml_parser',
        name='kml_parser',
        parameters=[
            #kml_file_path
            param_file_path
        ]
        
        # Also we have a chance like below
        #     parameters=[
        #         
        #     ]
        
    )

    return LaunchDescription([
        kml_file_launch_argument,
        kml_parser_node
    ])
