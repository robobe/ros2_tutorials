from turtle import distance
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from typing import NamedTuple
from math import sqrt, pow, atan2, cos, sin, radians
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Point(NamedTuple):
    x: int
    y: int

TURTLE_NAME = "turtle1"
CMD_VEL_TOPIC = f"/{TURTLE_NAME}/cmd_vel"
POSE_TOPIC = f"/{TURTLE_NAME}/pose"
GOTO_TEST = Point(3, 10)
PERIOD = 1
LINEAR_KP = 0.1
ANGLE_KP = 0.5



class TurtleGoToNode(Node):
    def __init__(self):
        super().__init__("turtle_goto")
        self.__pub_twist = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.__subscription = self.create_subscription(Pose, POSE_TOPIC, self.__pose_handler, 10)
        self.__subscription  # prevent unused variable warning
        self.__pose = Pose()
        self.__timer = self.create_timer(PERIOD, self.__timer_callback)
        self.__timer
        self.__tf_broadcaster = TransformBroadcaster(self)

    def __calc_euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.__pose.x), 2) +
                    pow((goal_pose.y - self.__pose.y), 2))
    
    

    def __calc_angle(self, goal_pose):
        delta = atan2(goal_pose.y - self.__pose.y, goal_pose.x - self.__pose.x)
        return delta - self.__pose.theta

    def __pose_handler(self, msg:Pose):
        self.__pose.x = msg.x
        self.__pose.y = msg.y
        self.__pose.theta = msg.theta

        self.__send_tf(msg)

    def __send_twist(self, x, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(z)
        # self.get_logger().info(f"send x: {x} and angle: {z}")
        self.__pub_twist.publish(msg)

    def __send_tf(self, msg: Pose):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = TURTLE_NAME
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.__tf_broadcaster.sendTransform(t)

    def __timer_callback(self):
        """
        calc distance
        calc angle
        send twist
        """
        x_distance = LINEAR_KP * self.__calc_euclidean_distance(GOTO_TEST)
        z_angle = ANGLE_KP * self.__calc_angle(GOTO_TEST)
        self.__send_twist(x_distance, z_angle)
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoToNode()
    rclpy.spin(node)
    rclpy.shutdown()

def calc_transform_cw(x, y, alpha):
    M = np.array([[cos(alpha), sin(alpha)], [-sin(alpha), cos(alpha)]])
    v = np.array([x, y])
    result = M.dot(v)
    return result[0], result[1]

def translate(x, y, x_offset, y_offset):
    T = np.array([[1, 0, x_offset], [0, 1, y_offset], [0, 0, 1]])
    v = np.array([x, y, 1])
    result = T.dot(v)
    return result[0], result[1]

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

if __name__ == '__main__':
    main()