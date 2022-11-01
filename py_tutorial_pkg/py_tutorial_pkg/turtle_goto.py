from turtle import distance
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from typing import NamedTuple
from math import sqrt, pow, atan2, cos, sin, radians
import numpy as np

class Point(NamedTuple):
    x: int
    y: int

CMD_VEL_TOPIC = "/turtle1/cmd_vel"
POSE_TOPIC = "/turtle1/pose"
GOTO_TEST = Point(3, 10)
PERIOD = 1
LINEAR_KP = 0.1
ANGLE_KP = 0.5



class TurtleGoToNode(Node):
    def __init__(self):
        super().__init__("turtle_goto")
        self.__pub_twist = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Pose, POSE_TOPIC, self.__pose_handler, 10)
        self.__pose = Pose()
        self.__timer = self.create_timer(PERIOD, self.__timer_callback)

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



    def __timer_callback(self):
        """
        calc distance
        calc angle
        send twist
        """
        x_distance = LINEAR_KP * self.__calc_euclidean_distance(GOTO_TEST)
        z_angle = ANGLE_KP * self.__calc_angle(GOTO_TEST)
        self.__send_twist(x_distance, z_angle)
        self.get_logger().info(f"pose {self.__pose.x},{self.__pose.y}")

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

if __name__ == '__main__':
    main()