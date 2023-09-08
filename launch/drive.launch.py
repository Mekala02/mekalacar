from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os

def generate_launch_description():
    cwd = os.getcwd()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('description'),
                    'launch',
                    'description.launch.py'
                ])
            ]),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zed_wrapper'),
                    'launch',
                    'zedm.launch.py'
                ])
            ]),
            launch_arguments={
                "config_path": os.path.join(cwd, 'config', 'zed_common.yaml')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('arduino'),
                    'launch',
                    'arduino.launch.py'
                ])
            ]),
        ),
    ])