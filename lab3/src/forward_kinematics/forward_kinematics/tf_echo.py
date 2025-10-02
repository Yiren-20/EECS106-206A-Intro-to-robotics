import tf2_ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import forward_kinematics.kin_func_skeleton as kfs
from forward_kinematics.forward_kinematics import ur7e_forward_kinematics_from_joint_state

class Tfecho(Node):
    def __init__(self, target, source):
        super().__init__('tf_echo')
       
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.target_frame = target
        self.source_frame = source

        self.timer = self.create_timer(0.5, self.listener_callback)
    
    def listener_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
            self.get_logger().info(f"Got transform:\n{trans}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info('Could not transform %s to %s: %s' % (self.source_frame, self.target_frame, e))
      



def main(args=None):
    import sys
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Usage: ros2 run forward_kinematics tf_echo target_frame source_frame")
        return
    target = sys.argv[1]
    source = sys.argv[2]

    node = Tfecho(target, source)
    rclpy.spin(node)  
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()