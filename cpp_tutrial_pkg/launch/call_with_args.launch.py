from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node=Node(
        name="simple_param",
        package = 'cpp_tutrial_pkg',
        executable = 'param_hello',
        parameters = [
            {"param_name": "param value from launch"}
        ]
    )
    ld.add_action(node)
    return ld
