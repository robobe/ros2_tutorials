import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

PACKAGE_NAME = "ign_tutorial"

def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE_NAME)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    paths = [
     os.path.join(pkg, "worlds"),
    #  os.path.join(pkg, "models")
    ]
    env = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[":".join(paths)])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -v 4 box.sdf'
        }.items(),
    )

    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/double_pendulum_with_base0/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
            ],
            remappings=[
                ('/model/double_pendulum_with_base0/pose', '/tf')
            ]
            # ,
            # parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        )

    node = Node(package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "laser"]
    )
    return LaunchDescription([
        env,
        gz_sim,
        bridge,
        #node
    ])