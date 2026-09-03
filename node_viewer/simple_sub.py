import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('Simple Subscriber node started, listening to /cmd_vel')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: Linear X={msg.linear.x}, Angular Z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
