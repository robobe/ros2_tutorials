from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sim_node =  Node(
            package='demo_pkg',
            namespace='/demo',
            executable='simple'
        )


    ld.add_action(sim_node)
    return ld