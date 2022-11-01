from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

additional_plugin_path="/home/user/git/ardupilot_gazebo/build"
additional_resource_path="/home/user/git/ardupilot_gazebo/models:/home/user/git/ardupilot_gazebo/worlds"
current_plugin_path = environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default='')
current_resource_path = environ.get('IGN_GAZEBO_RESOURCE_PATH', default='')
exec = "$(which ign) gazebo"
env = {
    'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join([additional_plugin_path, current_plugin_path]),
    'IGN_GAZEBO_RESOURCE_PATH': ':'.join([additional_resource_path, current_resource_path])
    }

exec_args = "iris_arducopter_runway.world"
ign_process = ExecuteProcess(
    cmd=[exec, exec_args],
    additional_env=env,
    shell=True
)

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(ign_process)
    return ld