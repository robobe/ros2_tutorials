import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.get_logger().info("-----------Hello ROS2-----------------")
        self.get_logger().warning("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()