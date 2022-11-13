from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_pub'
        )

    sub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_sub',
            remappings=[
                ('/simple1', '/simple'),
            ]
        )

    ld.add_action(pub_node)
    ld.add_action(sub_node)
    return ld