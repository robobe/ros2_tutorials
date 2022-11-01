from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '0'
    }

    ld =  LaunchDescription()
    
    include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'child.launch.py'
                ])
            ]),
            launch_arguments={
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
        
    ld.add_action(include)
    return ld