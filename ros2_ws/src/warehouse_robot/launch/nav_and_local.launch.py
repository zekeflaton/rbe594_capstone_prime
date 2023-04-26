import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='warehouse_robot' #<--- CHANGE ME
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    map_config_path = os.path.join(pkg_share,'maps/empty_warehouse2.yaml')
    rviz_config_path = os.path.join(pkg_share, 'config/main.rviz')

    localization_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','localization_launch.py'
                )]), launch_arguments={'map':LaunchConfiguration('map'), 'use_sim_time': 'true'}.items()
    )

    navigation_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'map_subscribe_transient_local':'true'}.items()
    )

    rviz_config = DeclareLaunchArgument(name='rvizconfig',
                          default_value=rviz_config_path,
                          description='Absolute path to rviz config file')
    map_config_config = DeclareLaunchArgument(name='map',
                          default_value=map_config_path,
                          description='Absolute path to map config file')


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    return LaunchDescription([
        rviz_config,
        map_config_config,
        localization_launch,
        navigation_launch,
        rviz_node,

    ])