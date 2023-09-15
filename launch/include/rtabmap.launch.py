from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    # new_map = LaunchConfiguration('new_map')
    map_name = LaunchConfiguration('map_name')
    localization = LaunchConfiguration('localization')

    use_sim_time = LaunchConfiguration('use_sim_time')

    cwd = os.getcwd()

    parameters={
        'frame_id':'base_link',
        'use_sim_time':use_sim_time,
        'subscribe_depth':False,
        'subscribe_rgbd':True,
        'use_action_for_goal':True,
        "queue_size": 10,
        # Custom
        "Grid/RangeMax": "3",
        "Grid/RayTracing": "true",
        "Grid/3D": "true",
        "Grid/MaxObstacleHeight": "2",

        # "RGBD/StartAtOrigin": "true",
        "database_path": f"{os.path.dirname(cwd)}/maps/{map_name.perform(context)}.db",
    }

    remappings=[
          ('rgb/image', '/zedm/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zedm/zed_node/rgb/camera_info'),
          ('depth/image', '/zedm/zed_node/depth/depth_registered'),
          ("odom", "/zedm/zed_node/odom")
          ]

    # Nodes to launch
    rtabmap_sync_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        # parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time, 'qos':qos}],
        parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
        remappings=remappings)

    if localization.perform(context) == 'True':
        # Localization mode:
        rtabmap_slam_node = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings)
    else:
        # SLAM Mode:
        rtabmap_slam_node = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'])

    return [
        rtabmap_sync_node,
        rtabmap_slam_node
    ]  

def generate_launch_description():
    return LaunchDescription([

        # If you not defined those arguments on your main launch file defaults will be used.
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map_name', default_value='map',
            description='Map files name.'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        OpaqueFunction(function=launch_setup)
    ])
