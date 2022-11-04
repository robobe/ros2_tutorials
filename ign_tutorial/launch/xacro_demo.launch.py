import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = "ign_tutorial"

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_demo = get_package_share_directory(PACKAGE_NAME)

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_demo, 'models', 'basic.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = robot_description_config.toxml()
    params = {'use_sim_time': True, 'robot_description': robot_description}
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[params],
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

     # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'basic',
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'rviz.rviz')]
        )

    return LaunchDescription(
        [
            gazebo,
            spawn,
            robot_state_publisher,
            rviz_node
        ]
    )
