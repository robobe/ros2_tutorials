from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_pub',
            namespace="/demo",
            remappings=[
                ('/demo/simple', '/other_demo/simple1'),
            ]
        )

    sub_node =  Node(
            package='py_tutorial_pkg',
            executable='minimal_sub',
            namespace="/other_demo"
            
        )

    ld.add_action(pub_node)
    ld.add_action(sub_node)
    return ld