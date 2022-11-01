from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    param_name = LaunchConfiguration('param_name')

    arg_cmd = DeclareLaunchArgument("param_name", default_value="hello world", description="msg simple description")

    node=Node(
        name="simple_param",
        package = 'cpp_tutrial_pkg',
        executable = 'param_hello',
        parameters = [
            {"param_name": param_name}
        ]
    )

    ld.add_action(arg_cmd)
    ld.add_action(node)
    return ld
