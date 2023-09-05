import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('arduino'),
      'config',
      'arduino_config.yaml'
      )

   return LaunchDescription([
      Node(
         package='arduino',
         executable='arduino_node',
         name='arduino',
         parameters=[config]
      )
   ])