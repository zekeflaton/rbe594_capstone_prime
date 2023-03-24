import launch
from launch.substitutions import Command, LaunchConfiguration
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

# path_to_urdf = get_package_share_path('pr2_description') / 'robots' / 'pr2.urdf.xacro'
    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{
    #         'robot_description': launch_ros.descriptions.ParameterValue(
    #             Command(['xacro ',
    #                      str(default_model_path),
    #                      ' sensor_rplidar:=true',
    #                      ' sensor_lidar:=false',
    #                      ' sensor_imu:=true',
    #                     ' sensor_camera:=true']),
    #                 value_type=str
    #         )
    #     }]
    # )

    # spawn_entity = launch_ros.actions.Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
    #     output='screen'
    # )
    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    # )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    # )
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
# export GAZEBO_PLUGIN_PATH=~/<path>/my_package_example/lib:${GAZEBO_PLUGIN_PATH}

# export GAZEBO_MODEL_PATH=~/<path>/my_package_example/models:${GAZEBO_MODEL_PATH}

# export GAZEBO_RESOURCE_PATH=~/<path>/my_package_example/models:${GAZEBO_RESOURCE_PATH}

# export GAZEBO_PLUGIN_PATH=~/git/rbe594_capstone_prime/ros2_ws/install/robot_description/lib:${GAZEBO_PLUGIN_PATH}
# export GAZEBO_MODEL_PATH=~/git/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/models:${GAZEBO_MODEL_PATH}
# export GAZEBO_RESOURCE_PATH=~/git/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/world:${GAZEBO_RESOURCE_PATH}

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                                      output='screen'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
                                      output='screen'),
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        # robot_state_publisher_node,
        rviz_node,
        # spawn_entity
    ])