import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString

class Mytalker(Node):

    # Here, we define the constructor
    def __init__(self):
        # We call the Node class's constructor and call it "minimal_publisher"
        super().__init__('my_talker')
        
         # Here, we set that the node publishes message of type String (where did this type come from?), over a topic called "chatter_talk", and with queue size 10. The queue size limits the amount of queued messages if a subscriber doesn't receive them quickly enough.
        self.publisher_ = self.create_publisher(TimestampString, '/user_messages', 10)
        timer_period=0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    # Here we create a message with the counter value appended and publish it
    def timer_callback(self):
        user_input = input("Please enter a line of text and press <Enter>: ")
        msg = TimestampString()
        msg.text = user_input
        msg.timestamp = self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent: "{msg.text}" at {msg.timestamp}')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    node_pub=Mytalker()
   
    # Spin the node so its callbacks are called
    rclpy.spin(node_pub)

    node_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
