import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('huron_description')
    world_path = os.path.join(
        pkg_dir,
        'worlds',
        'default.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
                launch_arguments={'world': world_path}.items(),
    )

    simulation_description_path = os.path.join(pkg_dir)
    simulation_urdf_path = os.path.join(simulation_description_path,'urdf','huron.xacro')
    robot_description_config = ParameterValue(
            Command(['xacro ', str(simulation_urdf_path)]), value_type=str
        )
    robot_description = {'robot_description': robot_description_config}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # use_sim_time_decl = DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='true',
    #         description='Use simulation (Gazebo) clock if true'),

    robot_controllers = os.path.join(
        get_package_share_directory("huron_control"),
        "config",
        "gazebo_controllers.yaml")

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #             {"robot_description": robot_description_config}, robot_controllers],
    #     output="both",
    # )

    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'huron',
                                   '-z', '1.1227'],
                        output='both')

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster",
    #                "--controller-manager", "/controller_manager"],
    #     remappings=[('/joint_states', '/huron/joint_states')],
    #     output="both",
    # )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_group_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_group_effort_controller'],
        output='screen'
    )
    nodes = [
        # use_sim_time_decl,
        gazebo,
        spawn_entity,
        # controller_manager,
        node_robot_state_publisher,
        load_joint_state_controller,
        load_joint_group_effort_controller,
        # joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)
