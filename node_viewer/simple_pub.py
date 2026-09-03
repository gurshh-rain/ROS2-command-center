import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Simple Publisher node started, publishing to /cmd_vel')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        msg.angular.z = 0.1  # Slight turn
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Linear X={msg.linear.x}, Angular Z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
