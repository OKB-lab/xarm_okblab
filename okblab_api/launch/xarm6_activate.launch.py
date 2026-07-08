from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')

    xarm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_api'),
                'launch',
                'xarm6_driver.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': '192.168.1.203',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='0',
            description='xArm operation mode passed to xarm/set_mode',
        ),
        xarm_driver,
        Node(
            package='okblab_api',
            executable='xarm6_activate',
            name='xarm6_activate',
            output='screen',
            parameters=[{
                'mode': ParameterValue(mode, value_type=int),
            }],
        ),
    ])
