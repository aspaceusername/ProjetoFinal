import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ROBOT_ENTITY_NAME = 'robo_camuflado'  # Nome do teu robô

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='roboauto_world')

    launch_file_dir = os.path.join(get_package_share_directory('roboauto'), 'launch')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join("/opt/ros/humble", "share") +
            ":" +
            os.path.join(get_package_share_directory('roboauto'), "models")
        ]
    )

    # Spawn do robô
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', ROBOT_ENTITY_NAME,
            '-name', ROBOT_ENTITY_NAME,
            '-file', PathJoinSubstitution([
                get_package_share_directory('roboauto'),
                "models", "robo_camuflado", "robo_camuflado.sdf"
            ]),
            '-allow_renaming', 'false',
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01'
        ],
    )

    # Spawn do mundo
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-file', PathJoinSubstitution([
                get_package_share_directory('roboauto'),
                "models", "worlds", "model.sdf"
            ]),
            '-allow_renaming', 'false'
        ],
    )

    world_only = os.path.join(get_package_share_directory('roboauto'), "models", "mundos", "mundo_realista.sdf")

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]
            ),
            launch_arguments=[('ign_args', [' -r -v 3 ' + world_only])],
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir + '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir + '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir + '/navigation2.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
