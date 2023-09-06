from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    cwd = os.getcwd()

    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(cwd, 'config', 'config_file.rviz')]]
        )
    ])