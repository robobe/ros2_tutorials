from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    new_background_r = LaunchConfiguration('new_background_r')

    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='255'
    )


    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )

    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set '
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        new_background_r_launch_arg,
        turtlesim_node,
        change_background_r
    ])