from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters_file = PathJoinSubstitution([
        FindPackageShare('okblab_api'),
        'config',
        'move_cont_cart_vel.yaml',
    ])

    return LaunchDescription([
        Node(
            package='okblab_api',
            executable='move_cont_cart_vel',
            name='move_cont_cart_vel',
            output='screen',
            parameters=[parameters_file],
        ),
    ])
