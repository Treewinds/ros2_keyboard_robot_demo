import rclpy
from rclpy.node import Node

class KeyboardCarNode(Node):
    def __init__(self):
        super().__init__('keyboard_car_node')
        self.get_logger().info('KeyboardCar skeleton node started')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

