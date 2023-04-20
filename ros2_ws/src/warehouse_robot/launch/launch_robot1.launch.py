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
    robot_name = LaunchConfiguration('robot_name')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


# Update robot name and controller
    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true', 'robot_name': robot_name}.items()
    # )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')
    # robot_name = LaunchConfiguration('robot_name')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('warehouse_robot'))
    xacro_file = os.path.join(pkg_path,'urdf','robot_1.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', 'true', ' sim_mode:=', 'true', ' namespace:=', 'robot_1'])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        # namespace=robot_name
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
            # namespace=robot_name
        )




    robot_description = Command(['ros2 param get --hide-type /robot_1/robot_state_publisher robot_1/robot_description'])

# Need upate controller?
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot_1',
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
        namespace=robot_name
    )
#     "--remap", "_target_node_name:__node:=dst_node_name",
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

# Controller update?
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_1_diff_cont", "--controller-manager-timeout", "30"],
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
        arguments=["robot_1_joint_broad", "--controller-manager-timeout", "30"],
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
        arguments=["robot_1_piston_cont", "--controller-manager-timeout", "30"],
        # namespace=robot_name
    )

    delayed_joint_piston_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_piston_spawner],
        )
    )

    robot_namespace = DeclareLaunchArgument(
            'robot_name',
            default_value='robot_1',
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
        # rsp,
        node_robot_state_publisher,
        joystick,
        twist_mux,
        # delayed_controller_manager,
        # controller_manager,
        # delayed_diff_drive_spawner,
        diff_drive_spawner,
        # delayed_joint_broad_spawner,
        joint_broad_spawner,
        # delayed_joint_piston_spawner,
        joint_piston_spawner,
        robot_namespace,
        spawn_entity
    ])
