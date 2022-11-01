from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

PACKAGE_NAME = "py_tutorial_pkg"

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='sim'
        )

    goto_node =  Node(
            package='py_tutorial_pkg',
            namespace='',
            executable='turtle_goto_tf',
            name='goto'
        )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'rviz_turtlesim_tf.rviz')]
        )

    ld.add_action(sim_node)
    ld.add_action(goto_node)
    ld.add_action(rviz_node)
    return ld