from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    steamvr_node = Node(
        package='tracker',
        executable='steamvr_node',
        name='steamvr',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([steamvr_node])
