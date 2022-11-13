import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "simple1"

class MyNode(Node):
    def __init__(self):
        node_name="minimal_sub"
        super().__init__(node_name)
        self.__sub = self.create_subscription(String, TOPIC, self.__sub_handler, 10)
        self.__sub
        self.get_logger().info("start minimal sub")

    def __sub_handler(self, msg: String):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()