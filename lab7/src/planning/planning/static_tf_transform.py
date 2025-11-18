#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        self.declare_parameter('ar_marker', 'ar_marker_8')
        marker = self.get_parameter('ar_marker').get_parameter_value().string_value

        # Homogeneous transform G_ar->base_link
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])

        rot = G[0:3, 0:3]
        quat = R.from_matrix(rot).as_quat()   # 返回 [x, y, z, w]
        qx, qy, qz, qw = quat
      
      # Create TransformStamped
        self.transform = TransformStamped()
        self.transform.transform.translation.x = G[0, 3]
        self.transform.transform.translation.y = G[1, 3]
        self.transform.transform.translation.z = G[2, 3]
        self.transform.transform.rotation.x = qx
        self.transform.transform.rotation.y = qy
        self.transform.transform.rotation.z = qz
        self.transform.transform.rotation.w = qw
        self.transform.header.frame_id = "ar_marker_8"  # [NOTE]: this is hard-coded for this lab station
        self.transform.child_frame_id = "base_link"
        # ---------------------------
        # TODO: Fill out TransformStamped message
        # --------------------------




        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()














#         # Create TransformStamped
#         self.transform = TransformStamped()
#         # ---------------------------
#         # TODO: Fill out TransformStamped message
#         # --------------------------
#         # Extract rotation (3x3) and translation (3x1)

#         self.timer = self.create_timer(0.05, self.broadcast_tf)

#     def broadcast_tf(self):
#         self.transform.header.stamp = self.get_clock().now().to_msg()
#         self.br.sendTransform(self.transform)

# def main():
#     rclpy.init()
#     node = ConstantTransformPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
