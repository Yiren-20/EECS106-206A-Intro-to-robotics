import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class Control(Node):

  
    def __init__(self,turtle_name):
        
        super().__init__('turtle_controller')
        topic=f'/{turtle_name}/cmd_vel'
    
        self.publisher_ = self.create_publisher(Twist, topic, 10)
        
    # Here we create a message with the counter value appended and publish it
    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent: linear={linear}, angular={angular}')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    if len(sys.argv) > 1:
        turtle_name=sys.argv[1]
    else:
        turtle_name="turtle1"
        

    node_control=Control(turtle_name)
    try:
        while rclpy.ok():
            cmd = input("Enter command (w=forward, s=backward, a=left, d=right, q=quit): ")
            if cmd == 'w':
                node_control.send_cmd(linear=2.0, angular=0.0)
            elif cmd == 's':
                node_control.send_cmd(linear=-2.0, angular=0.0)
            elif cmd == 'a':
                node_control.send_cmd(linear=0.0, angular=2.0)
            elif cmd == 'd':
                node_control.send_cmd(linear=0.0, angular=-2.0)
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
 
        node_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
