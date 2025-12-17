import rclpy
from rclpy.node import Node

class EmptyNode(Node):
    def __init__(self):
        super().__init__('empty_node')
        self.get_logger().info('Empty node started')

def main(args=None):
    rclpy.init(args=args)
    node = EmptyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

