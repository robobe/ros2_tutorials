import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "simple"
PERIOD = 1

class MyNode(Node):
    def __init__(self):
        node_name="minimal_pub"
        super().__init__(node_name)
        self.__pub = self.create_publisher(String, TOPIC, 10)
        self.__timer = self.create_timer(PERIOD, self.__timer_handler)
        self.__timer
        self.__counter = 0
        self.get_logger().info("run simple pub")

    def __timer_handler(self):
        self.__counter += 1
        msg = String(data="pub counter: {}".format(self.__counter))
        self.__pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()