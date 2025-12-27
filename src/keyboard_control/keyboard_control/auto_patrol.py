import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoPatrol(Node):
    """
    最小自动巡航：前进 T 秒 -> 左转 T 秒 -> 循环
    默认发布 /cmd_vel（由 launch/bridge 转到 Gazebo 的 /model/simple_car/cmd_vel）
    """

    def __init__(self):
        super().__init__('auto_patrol')

        self.declare_parameter('linear_x', 0.5)
        self.declare_parameter('angular_z', 0.6)
        self.declare_parameter('forward_sec', 3.0)
        self.declare_parameter('turn_sec', 1.6)
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.linear_x = float(self.get_parameter('linear_x').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.forward_sec = float(self.get_parameter('forward_sec').value)
        self.turn_sec = float(self.get_parameter('turn_sec').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(0.05, self._tick)

        self.state = 'forward'
        self.t0 = self.get_clock().now()

        self.get_logger().info(
            f'AutoPatrol started: topic={self.cmd_topic}, '
            f'v={self.linear_x}, w={self.angular_z}, '
            f'fwd={self.forward_sec}s, turn={self.turn_sec}s'
        )

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9

        msg = Twist()

        if self.state == 'forward':
            msg.linear.x = self.linear_x
            if dt >= self.forward_sec:
                self.state = 'turn'
                self.t0 = now

        else:  # turn
            msg.angular.z = self.angular_z
            if dt >= self.turn_sec:
                self.state = 'forward'
                self.t0 = now

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AutoPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

