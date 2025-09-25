#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException

class TurtleBotController(Node):
    def __init__(self, frame1, frame2):
        super().__init__('turtlebot_controller')

        self.turtle_frame = frame1
        self.ar_frame = frame2

        # Note: these constants might not work for your turtlebot, be willing to tune them if it isn't reaching the goal!
        self.K1 = 0.3
        self.K2 = 1.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f"TurtleBotController: frame1 (robot)='{self.turtle_frame}', frame2 (target)='{self.ar_frame}', "
            f"K1={self.K1}, K2={self.K2}"
        )

    def loop(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.turtle_frame, self.ar_frame, Time())
            self.get_logger().info(f"Transform: {tf}")
            # print(tf)
            x_err = tf.transform.translation.x
            y_err = tf.transform.translation.y

            #Twist
            control_cmd = Twist()
            control_cmd.linear.x = self.K1 * x_err
            control_cmd.angular.z = self.K2 * y_err
            # self.get_logger().info(
            #     f"x_err={x_err:.3f}, y_err={y_err:.3f}, "
            #     f"v={control_cmd.linear.x:.3f}, w={control_cmd.angular.z:.3f}"
            # )

            self.pub.publish(control_cmd)

        except (TransformException, LookupException, ConnectivityException, ExtrapolationException):
            self.pub.publish(Twist())
    
    def destroy_node(self):
        try:
            self.pub.publish(Twist())
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: python3 turtlebot_control.py frame1 frame2")
        rclpy.shutdown()
        return

    frame1 = sys.argv[1]
    frame2 = sys.argv[2]

    node = TurtleBotController(frame1, frame2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
