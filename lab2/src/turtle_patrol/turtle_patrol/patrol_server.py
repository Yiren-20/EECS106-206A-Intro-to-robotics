# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from std_srvs.srv import Empty
# from turtlesim.srv import TeleportAbsolute
# from turtle_patrol_interface.srv import Patrol


# class Turtle1PatrolServer(Node):
#     def __init__(self,turtle_name: str):
#         super().__init__(f'{turtle_name}_patrol_server')

#         # Publisher: actually drives turtle1
#         self._turtle_name = turtle_name
#         self._cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
#         self._srv = self.create_service(Patrol, f'/{turtle_name}/patrol', self.patrol_callback)
#         # Current commanded speeds (what timer publishes)
#         self._lin = 0.0
#         self._ang = 0.0

#         # Timer: publish current speeds at 10 Hz
#         self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

#         self.get_logger().info(f"{turtle_name}PatrolServer ready (continuous publish mode).")

#     # -------------------------------------------------------
#     # Timer publishes current Twist
#     # -------------------------------------------------------
#     def _publish_current_cmd(self):
#         msg = Twist()
#         msg.linear.x = self._lin
#         msg.angular.z = self._ang
#         self._cmd_pub.publish(msg)

#     # -------------------------------------------------------
#     # Service callback: update speeds
#     # -------------------------------------------------------
#     def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
#         self.get_logger().info(
#             f"[{self._turtle_name}] Patrol request: vel={request.vel:.2f}, omega={request.omega:.2f}"
        
#         )

#         # Update the speeds that the timer publishes
#         self._lin = float(request.vel)
#         self._ang = float(request.omega)

#         # Prepare response Twist reflecting current command
#         cmd = Twist()
#         cmd.linear.x = self._lin
#         cmd.angular.z = self._ang
#         response.cmd = cmd

#         self.get_logger().info(
#             f"[{self._turtle_name}] Streaming cmd_vel: lin.x={self._lin:.2f}, ang.z={self._ang:.2f} (10 Hz)"
#         )
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node1 = Turtle1PatrolServer("turtle1")
#     node2 = Turtle1PatrolServer("turtle2")

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(node1)
#     executor.add_node(node2)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     node1.destroy_node()
#     node2.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class MultiTurtlePatrolServer(Node):
    def __init__(self):
        super().__init__('multi_turtle_patrol_server')

        # Store publishers and velocities for each turtle
        self._publishers = {}         # turtle_name -> publisher
        self._lin = {}                # turtle_name -> linear velocity
        self._ang = {}                # turtle_name -> angular velocity

        # Store teleport service clients for each turtle
        self._teleport_clients = {}

        # Provide the Patrol service
        self._srv = self.create_service(Patrol, 'patrol', self.patrol_callback)

        # Timer to periodically publish Twist commands (10 Hz)
        self._pub_timer = self.create_timer(0.1, self._publish_all_cmds)

        self.get_logger().info("MultiTurtlePatrolServer ready.")

    # Periodically publish Twist messages for all turtles
    def _publish_all_cmds(self):
        for name, pub in self._publishers.items():
            msg = Twist()
            msg.linear.x = self._lin.get(name, 0.0)
            msg.angular.z = self._ang.get(name, 0.0)
            pub.publish(msg)

    # Service callback: handle patrol requests from clients
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        turtle_name = request.turtle_name
        self.get_logger().info(
            f"Patrol request for {turtle_name}: "
            f"x={request.x:.2f}, y={request.y:.2f}, theta={request.theta:.2f}, "
            f"vel={request.vel:.2f}, omega={request.omega:.2f}"
        )

        # If this turtle has no publisher yet, create one
        if turtle_name not in self._publishers:
            topic = f'/{turtle_name}/cmd_vel'
            self._publishers[turtle_name] = self.create_publisher(Twist, topic, 10)
            self._lin[turtle_name] = 0.0
            self._ang[turtle_name] = 0.0
            self.get_logger().info(f"Created publisher for {turtle_name}")

        # Create teleport client if not already created
        if turtle_name not in self._teleport_clients:
            self._teleport_clients[turtle_name] = self.create_client(
                TeleportAbsolute, f'/{turtle_name}/teleport_absolute'
            )

        # Teleport turtle to the given initial pose
        teleport_client = self._teleport_clients[turtle_name]
        if teleport_client.wait_for_service(timeout_sec=2.0):
            req = TeleportAbsolute.Request()
            req.x = request.x
            req.y = request.y
            req.theta = request.theta
            teleport_client.call_async(req)
        else:
            self.get_logger().warn(f"Teleport service not available for {turtle_name}")

        # Update stored velocities for this turtle
        self._lin[turtle_name] = float(request.vel)
        self._ang[turtle_name] = float(request.omega)

        # Fill the response with the Twist command being applied
        cmd = Twist()
        cmd.linear.x = self._lin[turtle_name]
        cmd.angular.z = self._ang[turtle_name]
        response.cmd = cmd

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MultiTurtlePatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
