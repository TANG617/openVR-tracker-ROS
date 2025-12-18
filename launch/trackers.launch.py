from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('tracker')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'tracker_params.yaml']),
        description='Path to tracker configuration YAML file'
    )

    steamvr_delay_arg = DeclareLaunchArgument(
        'steamvr_delay',
        default_value='3.0',
        description='Delay before starting trackers_node (seconds)'
    )

    launch_steamvr_arg = DeclareLaunchArgument(
        'launch_steamvr',
        default_value='false',
        description='Whether to launch SteamVR node'
    )

    steamvr_node = Node(
        package='tracker',
        executable='steamvr_node',
        name='steamvr',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('launch_steamvr')),
    )

    trackers_node = TimerAction(
        period=LaunchConfiguration('steamvr_delay'),
        actions=[
            Node(
                package='tracker',
                executable='trackers_node',
                name='trackers_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        config_file_arg,
        steamvr_delay_arg,
        launch_steamvr_arg,
        steamvr_node,
        trackers_node,
    ])

