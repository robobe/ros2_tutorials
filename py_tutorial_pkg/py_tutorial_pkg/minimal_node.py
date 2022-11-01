import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        self.get_logger().info("Hello ROS2")
        super().__init__(node_name)

    def __func(self, data: str):
        self.get_logger().info(data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()