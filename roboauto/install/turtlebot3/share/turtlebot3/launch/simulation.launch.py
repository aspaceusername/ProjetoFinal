import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = 'robo_camuflado'   # Nome do seu modelo

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')

    # Diretoria launch do pacote turtlebot3
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3'), 'launch')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory('turtlebot3'), 'models')
    )

    # Caminho para o ficheiro SDF do modelo personalizado
    model_sdf_path = os.path.join(
        get_package_share_directory('turtlebot3'),
        'models',
        'robo_camuflado',
        'robo_camuflado.sdf'
    )

    # Spawn do robot com o modelo personalizado
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-name', TURTLEBOT3_MODEL,
            '-file', model_sdf_path,
            '-allow_renaming', 'true',
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01'
        ],
    )

    # Spawn do mundo padrão (pode ajustar se desejar outro mundo)
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-file', os.path.join(get_package_share_directory('turtlebot3'), 'models', 'worlds', 'model.sdf'),
            '-allow_renaming', 'false'
        ],
    )

    world_only = os.path.join(get_package_share_directory('turtlebot3'), "models", "worlds", "world_only.sdf")

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
            launch_arguments=[('ign_args', [' -r -v 3 ' + world_only])]
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'ros_ign_bridge.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'navigation2.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
