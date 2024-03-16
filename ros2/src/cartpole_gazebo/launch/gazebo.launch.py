import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, \
        DeclareLaunchArgument
# from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pause_gz = LaunchConfiguration('pause_gz')
    # pause_gz_arg = DeclareLaunchArgument(
    #         'pause_gz',
    #         default_value='False',
    #         description='Pause Gazebo at launch'),

    pkg_dir = get_package_share_directory('cartpole_gazebo')
    world_path = os.path.join(
        pkg_dir,
        'worlds',
        'default.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'),
                                               '/gazebo.launch.py']),
                launch_arguments={'world': world_path,
                                  'pause': pause_gz}.items(),
    )

    simulation_description_path = os.path.join(pkg_dir)
    simulation_urdf_path = os.path.join(simulation_description_path,
                                        'urdf', 'cartpole_gazebo.xacro')
    robot_description_config = ParameterValue(
            Command(['xacro ', str(simulation_urdf_path)]), value_type=str
        )
    robot_description = {'robot_description': robot_description_config}

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # use_sim_time_decl = DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='true',
    #         description='Use simulation (Gazebo) clock if true'),

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cartpole',],
                        output='both')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--use-sim-time',
             '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_group_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--use-sim-time',
             '--set-state', 'active', 'joint_group_effort_controller'],
        output='screen'
    )
    nodes = [
        # pause_gz_arg,
        DeclareLaunchArgument(
            'pause_gz',
            default_value='true',
            description='Pause Gazebo at launch'),

        gazebo,
        spawn_entity,
        node_robot_state_publisher,
        load_joint_state_controller,
        load_joint_group_effort_controller,
    ]

    return LaunchDescription(nodes)
