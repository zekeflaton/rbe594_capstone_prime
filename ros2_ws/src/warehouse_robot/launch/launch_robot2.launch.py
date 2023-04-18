import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

# Does warehouse_robot change?
    package_name='warehouse_robot' #<--- CHANGE ME
# Update Robot Name
    robot_name = LaunchConfiguration('robot_name2')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


# Update robot name and controller
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true', 'robot_name2': robot_name}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')],
            namespace=robot_name
        )




    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

# Need upate controller?
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                #    '-robot_namespace', robot_name
                                   ],
                        # namespace=robot_name,
                        remappings=remappings,
                        output='screen')

# Controller update?
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
        # namespace=robot_name
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

# Controller update?
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_2/diff_cont", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
        # namespace=robot_name
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

# Controller update?
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_2/joint_broad", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
        # namespace=robot_name
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

# Controller update?
    joint_piston_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_2/piston_cont", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
        # namespace=robot_name
    )

    delayed_joint_piston_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_piston_spawner],
        )
    )

    robot_namespace = DeclareLaunchArgument(
            'robot_name2',
            default_value='robot',
            description='Namespace of robot to spawn')

    # Code for delaying a node (I haven't tested how effective it is)
    #
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        # delayed_controller_manager,
        # delayed_diff_drive_spawner,
        diff_drive_spawner,
        # delayed_joint_broad_spawner,
        joint_broad_spawner,
        # delayed_joint_piston_spawner,
        joint_piston_spawner,
        robot_namespace,
        spawn_entity
    ])