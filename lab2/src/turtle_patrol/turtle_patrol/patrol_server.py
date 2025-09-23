import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class Turtle1PatrolServer(Node):
    def __init__(self,turtle_name: str):
        super().__init__(f'{turtle_name}_patrol_server')

        # Publisher: actually drives turtle1
        self._turtle_name = turtle_name
        self._cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self._srv = self.create_service(Patrol, f'/{turtle_name}/patrol', self.patrol_callback)
        # Current commanded speeds (what timer publishes)
        self._lin = 0.0
        self._ang = 0.0

        # Timer: publish current speeds at 10 Hz
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

        self.get_logger().info(f"{turtle_name}PatrolServer ready (continuous publish mode).")

    # -------------------------------------------------------
    # Timer publishes current Twist
    # -------------------------------------------------------
    def _publish_current_cmd(self):
        msg = Twist()
        msg.linear.x = self._lin
        msg.angular.z = self._ang
        self._cmd_pub.publish(msg)

    # -------------------------------------------------------
    # Service callback: update speeds
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        self.get_logger().info(
            f"[{self._turtle_name}] Patrol request: vel={request.vel:.2f}, omega={request.omega:.2f}"
        
        )

        # Update the speeds that the timer publishes
        self._lin = float(request.vel)
        self._ang = float(request.omega)

        # Prepare response Twist reflecting current command
        cmd = Twist()
        cmd.linear.x = self._lin
        cmd.angular.z = self._ang
        response.cmd = cmd

        self.get_logger().info(
            f"[{self._turtle_name}] Streaming cmd_vel: lin.x={self._lin:.2f}, ang.z={self._ang:.2f} (10 Hz)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node1 = Turtle1PatrolServer("turtle1")
    node2 = Turtle1PatrolServer("turtle2")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node1.destroy_node()
    node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
