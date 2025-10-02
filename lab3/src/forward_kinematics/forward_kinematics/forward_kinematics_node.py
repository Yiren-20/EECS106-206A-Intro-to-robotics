import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# import kin_func_skeleton as kfs
from forward_kinematics.forward_kinematics import ur7e_forward_kinematics_from_joint_state



class Forwardkinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
       
        g = ur7e_forward_kinematics_from_joint_state(msg)

        self.get_logger().info("Forward Kinematics Result (4x4):\n{}".format(g))
    




def main(args=None):
    rclpy.init(args=args)

    node = Forwardkinematics()
    rclpy.spin(node)  
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()