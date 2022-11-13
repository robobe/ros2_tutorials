import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

PACKAGE_NAME = "ign_tutorial"

def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE_NAME)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    paths = [
     os.path.join(pkg, "worlds"),
     "/home/user/wasp_ws/src/tutorials/ign_tutorial/models"
    ]
    env = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[":".join(paths)])

    sdf_file = os.path.join(pkg, 'models', 'vehicle', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -v 4 my_cart.sdf'
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/Moving_robot/model/vehicle/model/vehicle_blue/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/world/Moving_robot/model/vehicle/model/vehicle_blue/joint_state', 'joint_states')
        ],
        output='screen'
    )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(pkg, 'config', 'rviz.rviz')]
        )

    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="world2chassis",
        arguments = ["0", "0", "0", "0", "0", "0", "world", "chassis"]
    )

    ign_model_prefix="/model/vehicle_blue"
    odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
			name = 'odom_base_tf_bridge',
			output = 'screen',
			parameters=[{
			'use_sim_time': True
			}],
			arguments = [
				ign_model_prefix + '/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
			],
			remappings = [
				(ign_model_prefix + '/tf', '/tf')
			])

    odometry_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
			name = 'odometry_bridge',
			output='screen',
			parameters=[{
				'use_sim_time': True
			}],
			arguments = [
				 ign_model_prefix + '/odometry' + '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
			],
			remappings = [
				(ign_model_prefix + '/odometry', '/odom')
			])

    cmd_vel_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
			name = 'cmd_vel_bridge',
			output='screen',
			parameters=[{
				'use_sim_time': True
			}],
			arguments = [
				ign_model_prefix + '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
			],
			remappings = [
				(ign_model_prefix + '/cmd_vel', '/cmd_vel')
			])

    return LaunchDescription([
        env,
        gz_sim,
        bridge,
        cmd_vel_bridge,
        odometry_bridge,
        odom_base_tf_bridge,
        robot_state_publisher,
        rviz_node  
    ])


#  ros2 topic pub cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}" --qos-reliability reliable
# ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 2.0}, angular: {z: 0.4}"