import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    # q: geometry_msgs/Quaternion
    # yaw (z) from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(a):
    # wrap to [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class AutoSquareOdom(Node):
    """
    用里程计走标准正方形：直行固定边长 -> 左转精确90° -> 循环
    发布 /cmd_vel，订阅 /model/simple_car/odometry
    """

    def __init__(self):
        super().__init__('auto_square_odom')

        # 可调参数：更快的默认速度
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/model/simple_car/odometry')
        self.declare_parameter('side_length', 4.0)      # 每条边长度(米)
        self.declare_parameter('linear_speed', 1.2)     # 直行速度(米/秒)  <- 更快
        self.declare_parameter('angular_speed', 1.4)    # 转弯速度(弧度/秒) <- 更快
        self.declare_parameter('yaw_tolerance_deg', 2.0)   # 角度容差(度)
        self.declare_parameter('dist_tolerance', 0.05)     # 距离容差(米)
        self.declare_parameter('rate_hz', 30.0)

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.side = float(self.get_parameter('side_length').value)
        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        self.yaw_tol = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self.dist_tol = float(self.get_parameter('dist_tolerance').value)
        self.rate = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.tick)

        # odom state
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # square state machine
        self.mode = 'WAIT_ODOM'  # FORWARD / TURN
        self.edge_idx = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.turn_target = 0.0

        self.get_logger().info(
            f'AutoSquareOdom started. cmd={self.cmd_topic}, odom={self.odom_topic}, '
            f'side={self.side}m, v={self.v}, w={self.w}'
        )

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        if not self.have_odom:
            self.have_odom = True
            # 初始化起点
            self.start_x, self.start_y = self.x, self.y
            self.start_yaw = self.yaw
            self.mode = 'FORWARD'
            self.get_logger().info('Got first odom. Start square patrol.')

    def publish_stop(self):
        self.pub.publish(Twist())

    def tick(self):
        if not self.have_odom:
            self.publish_stop()
            return

        cmd = Twist()

        if self.mode == 'FORWARD':
            # 走直线：根据当前位置与起点的距离判断
            dx = self.x - self.start_x
            dy = self.y - self.start_y
            dist = math.hypot(dx, dy)

            if dist >= (self.side - self.dist_tol):
                # 到边长：停一下，准备转 90°
                self.publish_stop()
                self.mode = 'TURN'
                self.start_yaw = self.yaw
                self.turn_target = angle_wrap(self.start_yaw + math.pi / 2.0)  # 左转90°
                return

            cmd.linear.x = self.v
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        if self.mode == 'TURN':
            # 转弯：用 yaw 精确到目标
            err = angle_wrap(self.turn_target - self.yaw)

            if abs(err) <= self.yaw_tol:
                # 转到位：停一下，开始下一条边
                self.publish_stop()
                self.edge_idx = (self.edge_idx + 1) % 4
                self.mode = 'FORWARD'
                self.start_x, self.start_y = self.x, self.y
                return

            # 按误差方向转（这里默认一直左转；如有漂移也能收敛）
            cmd.angular.z = self.w if err > 0.0 else -self.w
            cmd.linear.x = 0.0
            self.pub.publish(cmd)
            return


def main():
    rclpy.init()
    node = AutoSquareOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

