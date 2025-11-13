# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# import tf2_ros
# import numpy as np
# import transforms3d.euler as euler
# from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped, Pose
# from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
# from plannedcntrl.trajectory import plan_curved_trajectory  # Your existing Bezier planner
# import time

# class TurtleBotController(Node):
#     def __init__(self):
#         super().__init__('turtlebot_controller')

#         # Publisher and TF setup
#         self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Controller gains
#         self.Kp = np.diag([0.6, 0.01])
#         self.Kd = np.diag([0.001, 0.0001])
#         self.Ki = np.diag([0.001, 0.0001])

#         # Subscriber
#         self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

#         self.get_logger().info('TurtleBot controller node initialized.')

#     # ------------------------------------------------------------------
#     # Main waypoint controller
#     # ------------------------------------------------------------------
#     def controller(self, waypoint):
#         # initialize an error record
#         self.prev_error= np.array([0.0, 0.0])
#         self.intergral_error=np.array([0.0, 0.0])
        
#         rate_hz = 10.0
#         dt = 1.0 / rate_hz

        
#         while rclpy.ok():
#             try:
#                 # spin once to process TF updates
#                 rclpy.spin_once(self, timeout_sec=0.0)

#                 # get transform from baser_footprint to odom
#                 trans= self.tf_buffer.lookup_transform('base_footprint', 'odom',rclpy.time.Time())
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 rclpy.spin_once(self, timeout_sec=0.1)
#                 continue


#             # TODO: Transform the waypoint from the odom/world frame into the robot's base_link frame 
#             # before computing errors — you'll need this so x_err and yaw_err are in the robot's coordinate system.
#             # trans= self.tf_buffer.lookup_transform('base_footprint', 'odom',rclpy.time.Time())
#             way_pose= Pose()
          
#             way_pose.position.x= waypoint[0]
#             way_pose.position.y= waypoint[1]

#             try:
#                 transformed_pose = do_transform_pose(way_pose, trans)
#             except Exception as e:
#                 self.get_logger().error(f"Transform error: {e}")
#                 continue


#             #TODO: Calculate x and yaw errors! 
#             x_err = transformed_pose.position.x
#             y_err= transformed_pose.position.y
#             yaw_err = math.atan2(y_err, x_err)
#             # TODO: Replace with actual error calculation
#             errors = np.array([x_err,yaw_err])
#             self.get_logger().info(f"x_err: {x_err:.3f}, yaw_err: {yaw_err:.3f}")  
#             d_error = (errors-self.prev_error)/dt
#             self.intergral_error += errors*dt

#             # Control law
#             control_input = (self.Kp @ errors) + (self.Kd @ d_error) + (self.Ki @ self.intergral_error)
#             cmd= Twist()
#             cmd.linear.x = float(np.clip(control_input[0],-0.1,0.1))
#             cmd.angular.z = float(np.clip(control_input[1],-0.1,0.1))
#             # cmd.linear.x = 0.1
#             # cmd.angular.z = 0.0
#             self.pub.publish(cmd)
#             self.prev_error= errors
            

#             if abs(x_err) < 0.03 and abs(yaw_err) < 0.2:
#                 self.get_logger().info("Waypoint reached, moving to next.")
#                 return

#             time.sleep(dt)

#     # ------------------------------------------------------------------
#     # Callback when goal point is published
#     # ------------------------------------------------------------------
#     def planning_callback(self, msg: PointStamped):
#         trajectory = plan_curved_trajectory((msg.point.x, msg.point.y))

#         for waypoint in trajectory:
#             self.controller(waypoint)

#     # ------------------------------------------------------------------
#     # Utility
#     # ------------------------------------------------------------------
#     @staticmethod
#     def _quat_from_yaw(yaw):
#         """Return quaternion (x, y, z, w) from yaw angle."""
#         return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# # ----------------------------------------------------------------------
# # Main
# # ----------------------------------------------------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = TurtleBotController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped, Pose
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from plannedcntrl.trajectory import plan_curved_trajectory  # Your existing Bezier planner
import time


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher and TF setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Controller gains
        self.Kp = np.diag([0.4, 1.2])
        self.Kd = np.diag([0.0, 0.0])
        self.Ki = np.diag([0.0, 0.0])

        self.prev_error = np.zeros(2)
        self.integral_error = np.zeros(2)

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

        self.is_executing = False

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ------------------------------------------------------------------
    def controller(self, waypoint):
        """Drive the TurtleBot to a single (x, y, theta) waypoint"""
        x_goal, y_goal, theta_goal = waypoint
        rate_hz = 10.0
        dt = 1.0 / rate_hz
    
        while rclpy.ok():
            try:
                # Spin once to process TF updates
                rclpy.spin_once(self, timeout_sec=0.0)
    
                # Get transform from base_footprint -> odom
                trans = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
    
            # ----------------------------------------------------------------------
            # Create PoseStamped for the waypoint (in odom frame)
            # ----------------------------------------------------------------------
            waypoint_pose = PoseStamped()
            waypoint_pose.header.frame_id = 'odom'
            waypoint_pose.header.stamp = self.get_clock().now().to_msg()
            waypoint_pose.pose.position.x = x_goal
            waypoint_pose.pose.position.y = y_goal
            waypoint_pose.pose.position.z = 0.0
            # Orientation can be left as identity since we only use position

            pose_in = Pose()
            pose_in.position.x = x_goal
            pose_in.position.y = y_goal
            pose_in.position.z = 0.0
            pose_in.orientation.w = 1.0
    
            # ----------------------------------------------------------------------
            # Transform the waypoint pose into the robot's base frame
            # ----------------------------------------------------------------------
            try:
                transformed_pose = do_transform_pose(pose_in, trans)
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                continue
    
            # Extract errors directly in the robot frame
            x_err_robot = transformed_pose.position.x
            y_err_robot = transformed_pose.position.y
            # yaw_err = math.atan2(y_err_robot, x_err_robot)
    
            # # Log for debugging
            # self.get_logger().info(
            #     f"Errors -> x: {x_err_robot:.3f}, yaw: {yaw_err:.3f}; waypoint ({x_goal:.2f}, {y_goal:.2f})"
            # )
    
            # ----------------------------------------------------------------------
            # PID control law
            # ----------------------------------------------------------------------
            error_vec = np.array([x_err_robot, y_err_robot])
            d_error = (error_vec - self.prev_error) / dt
            self.integral_error += error_vec * dt
    
            control_signal = self.Kp @ error_vec + self.Kd @ d_error + self.Ki @ self.integral_error
    
            v = float(np.clip(control_signal[0], -0.5, 0.5))
            omega = float(np.clip(control_signal[1], -1.0, 1.0))
    
            # Publish Twist command
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.pub.publish(cmd)
    
            self.prev_error = error_vec
    
            # Stop if near waypoint
            if abs(x_err_robot) < 0.008:
                self.pub.publish(Twist())
                self.get_logger().info(f"Waypoint ({x_goal:.2f}, {y_goal:.2f}) reached.")
                return
    
            time.sleep(dt)

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        if self.is_executing:
            return
        
        self.is_executing = True

        trajectory = plan_curved_trajectory((msg.point.x, msg.point.y))

        for waypoint in trajectory:
            self.controller(waypoint)
        
        self.pub.publish(Twist())
        self.get_logger().info('Trajectory finished')
        
        # self.is_executing = False

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw):
        """Return quaternion (x, y, z, w) from yaw angle."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()