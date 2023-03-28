import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro


def generate_launch_description():
    pkg_name = 'robot_description'
    pkg_share = launch_ros.substitutions.FindPackageShare(package=pkg_name).find(pkg_name)
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robotmodel.rviz')
    doc = xacro.process_file(default_model_path, mappings={'radius': '0.9'})
    world_path=os.path.join(pkg_share, 'world/warehouse.world')
    pkg_gazebo_ros = launch_ros.substitutions.FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot',
                   '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',
                    '-Y', '0.00'],
        output='screen'
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ',
                                                   LaunchConfiguration('urdf_model'),
                                                   ' sensor_rplidar:=true',
                                                   ' sensor_lidar:=false',
                                                   ' sensor_imu:=true',
                                                   ' sensor_camera:=true'])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    start_gazebo_server_cmd = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    launch_arguments={'world': LaunchConfiguration('world')}.items())

    # Start Gazebo client
    start_gazebo_client_cmd = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    declare_world_cmd = launch.actions.DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    declare_urdf_model_path_cmd = launch.actions.DeclareLaunchArgument(
    name='urdf_model',
    default_value=default_model_path,
    description='Absolute path to robot urdf file')

    # Run these beforehand
    # . /usr/share/gazebo/setup.sh
    # export GAZEBO_PLUGIN_PATH=~/git/rbe594_capstone_prime/ros2_ws/install/robot_description/lib:${GAZEBO_PLUGIN_PATH}
    # export GAZEBO_MODEL_PATH=~/git/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/models:${GAZEBO_MODEL_PATH}
    # export GAZEBO_RESOURCE_PATH=~/git/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/world:${GAZEBO_RESOURCE_PATH}

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='urdf_model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        declare_world_cmd,
        declare_urdf_model_path_cmd,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        spawn_entity
    ])