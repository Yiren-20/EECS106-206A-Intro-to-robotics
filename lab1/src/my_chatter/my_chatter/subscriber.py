import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString

class Mylistener(Node):

    def __init__(self):
        super().__init__('my_listener')

        self.subscription = self.create_subscription(
            TimestampString,
            '/user_messages',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.text}" at {msg.timestamp}')


def main(args=None):
    rclpy.init(args=args)

    node_sub = Mylistener()

    rclpy.spin(node_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
