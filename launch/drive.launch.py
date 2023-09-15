from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():

    rtabmap_args = {
        'new_map': 'False',
        'localization': 'False',
        'map_name': 'home',
    }

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
                "config_path": os.path.join(cwd, 'config', 'zed_common.yaml'),
                "cam_pose": "[0.0814,0.00875,0.11015,0.0,0.1745329,0.0]"
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    cwd,
                    'include',
                    'rtabmap.launch.py'
                ])
            ]),
            launch_arguments=rtabmap_args.items()
        ),
    ])
