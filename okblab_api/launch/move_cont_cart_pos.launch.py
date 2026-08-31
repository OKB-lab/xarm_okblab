from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xarm6_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('okblab_api'),
                'launch',
                'xarm6_activate.launch.py',
            ])
        ),
        launch_arguments={
            # set_position uses position-control mode.
            'mode': '0',
        }.items(),
    )

    parameters_file = PathJoinSubstitution([
        FindPackageShare('okblab_api'),
        'config',
        'move_cont_cart_pos.yaml',
    ])

    return LaunchDescription([
        xarm6_activate,
        Node(
            package='okblab_api',
            executable='move_cont_cart_pos',
            name='move_cont_cart_pos',
            output='screen',
            parameters=[parameters_file],
        ),
    ])
