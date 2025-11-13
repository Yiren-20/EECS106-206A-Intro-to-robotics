# import rclpy
# from rclpy.node import Node
# from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import PointStamped 

# class TransformCubePose(Node):
#     def __init__(self):
#         super().__init__('transform_cube_pose')

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.cube_pose_sub = self.create_subscription(
#             PointStamped,
#             '/cube_pose',
#             self.cube_pose_callback,
#             10
#         )

#         self.cube_pose_pub = ... # Please ensure this is filled

#         rclpy.spin_once(self, timeout_sec=2)
#         self.cube_pose = None

#     def cube_pose_callback(self, msg: PointStamped):
#         if self.cube_pose is None:
#             self.cube_pose = self.transform_cube_pose(msg)

#     def transform_cube_pose(self, msg: PointStamped):
#         """ 
#         Transform point into base_link frame
#         Args: 
#             - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
#         Returns:
#             Point: point in base_link_frame in form [x, y, z]
#         """

#         return

# def main(args=None):
#     rclpy.init(args=args)
#     node = TransformCubePose()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 
        

# import necessary packages
import numpy as np
from scipy.spatial.transform import Rotation as R


class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose_base', 1) # Please ensure this is filled

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        if self.cube_pose is None:
            self.cube_pose = self.transform_cube_pose(msg)
            self.get_logger().info('Received cube pose')
        self.cube_pose_pub.publish(self.cube_pose)


    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """

        from tf2_geometry_msgs import do_transform_point

        from_frame = msg.header.frame_id
        to_frame = 'base_link'

        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time()
            )

            transformed_point = do_transform_point(msg, transform)
            self.get_logger().info(
                f'Cube in base_link: '
                f'x={transformed_point.point.x:.3f}'
                f'y={transformed_point.point.y:.3f}'
                f'z={transformed_point.point.z:.3f}'
            )
            return transformed_point
        
        except Exception as e:
            self.get_logger().warn(f'Could not transform point: {e}')
            return None



        # source_frame = msg.header.frame_id
        # target_frame = 'base_link'
        # try:
        #     trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time()) # T_base_depth
        # except Exception as e:
        #     self.get_logger().info('Transform not available')
        #     # Writes an error message to the ROS log but does not raise an exception
        #     self.get_logger().error(f"TF lookup failed: {e}")
        #     return None

        # # translation from tf transfor
        # p = np.array([0,0,0,1])
        # p[0] = trans.transform.translation.x
        # p[1] = trans.transform.translation.y
        # p[2] = trans.transform.translation.z
        # # rotation from tf transformation


        # q = np.array([0,0,0,1])
        # q[0] = trans.transform.rotation.x
        # q[1] = trans.transform.rotation.y
        # q[2] = trans.transform.rotation.z
        # q[3] = trans.transform.rotation.w
        # self.get_logger().info('quaternion values: {}, {}, {}, {}'.format(q[0], q[1], q[2], q[3]))
        # self.get_logger().info('translation values: {}, {}, {}, {}'.format(p[0], p[1], p[2], p[3]))
        # if np.linalg.norm(q) == 0:
        #     q = np.array([0,0,0,1])
        # R_mat = R.from_quat(q).as_matrix()  # rotation matrix
 

        # cube_pt = np.array([0,0,0]).reshape(-1,1)
        # cube_pt[0] = msg.point.x
        # cube_pt[1] = msg.point.y
        # cube_pt[2] = msg.point.z

        # # transform point from camera_depth_optical_frame to base_link
        # p_base = R_mat @ cube_pt + p[0:3].reshape(-1,1)
        
        # self.get_logger().info('quaternion values: {}, {}, {}, {}'.format(q[0], q[1], q[2], q[3]))
        
        # pt_msg = PointStamped()
        # pt_msg.point.x = float(p_base[0])
        # pt_msg.point.y = float(p_base[1])
        # pt_msg.point.z = float(p_base[2])
        # pt_msg.header.frame_id = 'base_link'
        # pt_msg.header.stamp = self.get_clock().now().to_msg()


        # return pt_msg

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
