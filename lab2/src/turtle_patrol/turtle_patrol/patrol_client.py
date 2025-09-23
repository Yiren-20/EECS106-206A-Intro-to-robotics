# import rclpy
# from rclpy.node import Node
# from turtle_patrol_interface.srv import Patrol

# # Import our custom service
# from turtle_patrol_interface.srv import Patrol


# class TurtlePatrolClient(Node):

#     def __init__(self):
#         super().__init__('turtle1_patrol_client')


#         self._service_name = '/turtle1/patrol'

#         # Create a client for our Patrol service type
#         self._client = self.create_client(Patrol, self._service_name)

#         # Wait until the server is up (polling loop; logs once per second)
#         self.get_logger().info(f"Waiting for service {self._service_name} ...")
#         while not self._client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info(f"Service {self._service_name} not available, waiting...")

#         # Hard-coded request values 
#         vel = 2.0
#         omega = 1.0
#         self.get_logger().info(f"Requesting patrol: vel={vel}, omega={omega}")

#         # Build request
#         req = Patrol.Request()
#         req.vel = vel
#         req.omega = omega

#         # Send request (async under the hood)
#         self._future = self._client.call_async(req)


# def main(args=None):
#     rclpy.init(args=args)
#     node = TurtlePatrolClient()

#     # Block here until the service responds (simple for teaching)
#     rclpy.spin_until_future_complete(node, node._future)

#     if node._future.done():
#         result = node._future.result()
#         if result is not None:
#             # Print the Twist returned by the server
#             cmd = result.cmd
#             node.get_logger().info(
#                 f"Service response Twist: lin.x={cmd.linear.x:.2f}, ang.z={cmd.angular.z:.2f}"
#             )
#         else:
#             node.get_logger().error("Service call failed: no result returned.")
#     else:
#         node.get_logger().error("Service call did not complete.")

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from turtle_patrol_interface.srv import Patrol
import sys

class TurtlePatrolClient(Node):
    def __init__(self, args):
        super().__init__('turtle_patrol_client')
        
        # 从命令行参数读取
        if len(args) < 7:
            self.get_logger().error("Usage: ros2 run turtle_patrol patrol_client turtle_name x y theta vel omega")
            rclpy.shutdown()
            return
        
        self._futures = []

        for i in range(1, len(args), 6):
            turtle_name, x, y, theta, vel, omega = args[i:i+6]

            req = Patrol.Request()
            req.turtle_name = turtle_name
            req.x = float(x)
            req.y = float(y)
            req.theta = float(theta)
            req.vel = float(vel)
            req.omega = float(omega)

            self._client = self.create_client(Patrol, f'/{turtle_name}/patrol')

            while not self._client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Service not available, waiting...")


            future = self._client.call_async(req)
            self._futures.append((turtle_name, future))

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePatrolClient(sys.argv)

    if hasattr(node, "_futures"):
        for turtle_name, future in node._futures:
            rclpy.spin_until_future_complete(node, future)
            if future.done():
                result = future.result()
                if result:
                    cmd = result.cmd
                    node.get_logger().info(
                        f"Turtle {turtle_name} -> lin.x={cmd.linear.x}, ang.z={cmd.angular.z}"
                    )

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

