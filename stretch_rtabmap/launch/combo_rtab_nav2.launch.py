import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace



def generate_launch_description():

    stretch_rtabmap_dir = get_package_share_directory('stretch_rtabmap')
    stretch_nav2_dir = get_package_share_directory('stretch_nav2')
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_nav2_launch_dir = os.path.join(stretch_nav2_dir, 'launch')


    declare_nav2_param = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(stretch_rtabmap_dir, 'config', 'nav2_voxel_localonly_param.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))


    nav2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([stretch_nav2_dir, '/launch/navigation_launch.py']),
            launch_arguments={'namespace': '',
                              'params_file': nav2_params_file,
                              'use_lifecycle_mgr': 'true',
                              'map_subscribe_transient_local': 'true'}.items()
            )

    # ld = LaunchDescription()
    # ld.add_action(rplidar_launch)
    # ld.add_action(nav2_include)
    
    # return ld
    return LaunchDescription([
        declare_nav2_param,
        rplidar_launch,
        nav2_include,
    ])
