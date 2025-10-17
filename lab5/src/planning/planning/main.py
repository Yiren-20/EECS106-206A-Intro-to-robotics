# ROS Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped 
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_sub = self.create_subscription(PointStamped, '/cube_pose', self.cube_callback, 1)
        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose_base', 1)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rclpy.spin_once(self, timeout_sec=2)

        self.cube_pose = None

    def cube_callback(self, cube_pose):
        if self.cube_pose is None:
            self.cube_pose = self.transform_cube_pose(cube_pose)
        
            self.cube_pose_pub.publish(self.cube_pose)

            self.get_logger().info('Received cube pose')

            self.cube_pose = None  # Reset to receive new cube pose


    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            PointStamped: point in base_link_frame in form [x, y, z]
        """

        # ------------------------
        #TODO: Add your code here!
        # ------------------------
        source_frame = 'camera_depth_optical_frame'
        target_frame = 'base_link'
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time()) # T_base_depth
        except Exception as e:
            self.get_logger().info('Transform not available')

        # translation from tf transfor
        p = np.array([0,0,0,1])
        p[0] = trans.transform.translation.x
        p[1] = trans.transform.translation.y
        p[2] = trans.transform.translation.z
        # rotation from tf transformation


        q = np.array([0,0,0,1])
        q[0] = trans.transform.rotation.x
        q[1] = trans.transform.rotation.y
        q[2] = trans.transform.rotation.z
        q[3] = trans.transform.rotation.w
        self.get_logger().info('quaternion values: {}, {}, {}, {}'.format(q[0], q[1], q[2], q[3]))
        if np.linalg.norm(q) == 0:
            q = np.array([0,0,0,1])
        R_mat = R.from_quat(q).as_matrix()  # rotation matrix

        cube_pt = np.array([0,0,0])
        cube_pt[0] = msg.point.x
        cube_pt[1] = msg.point.y
        cube_pt[2] = msg.point.z

        # transform point from camera_depth_optical_frame to base_link
        p_base = R_mat @ cube_pt + p[0:3]

        pt_msg = PointStamped()
        pt_msg.point.x = p_base[0]
        pt_msg.point.y = p_base[1]
        pt_msg.point.z = p_base[2]
        pt_msg.header.frame_id = 'base_link'
        pt_msg.header.stamp = self.get_clock().now().to_msg()
    
        return pt_msg

def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
