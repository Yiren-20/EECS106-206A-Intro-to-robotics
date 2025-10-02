#!usr/bin/env python

import numpy as np
import scipy as sp
import forward_kinematics.kin_func_skeleton as kfs 
# import kin_func_skeleton as kfs 


def ur7e_foward_kinematics_from_angles(joint_angles):
    """
    Calculate the orientation of the ur7e's end-effector tool given
    the joint angles of each joint in radians

    Parameters:
    ------------
    joint_angles ((6x) np.ndarray): 6 joint angles (s0, s1, e0, w1, w2, w3)

    Returns: 
    ------------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    q0 = np.ndarray((3, 6)) # Points on each joint axis in the zero config
    w0 = np.ndarray((3, 6)) # Axis vector of each joint axis in the zero config


    q0[:, 0] = [0., 0., 0.1625] # shoulder pan joint - shoulder_link
    q0[:, 1] = [0., 0., 0.1625] # shoulder lift joint - upper_arm_link
    q0[:, 2] = [0.425, 0., 0.1625] # elbow_joint - forearm_link
    q0[:, 3] = [0.817, 0.1333, 0.1625] # wrist 1 - wrist_1_link
    q0[:, 4] = [0.817, 0.1333, 0.06285] # wrist 2 - wrist_2_link
    q0[:, 5] = [0.817, 0.233, 0.06285] # wrist 3 - wrist_3_link

    w0[:, 0] = [0., 0., 1] # shoulder pan joint
    w0[:, 1] = [0, 1., 0] # shoulder lift joint
    w0[:, 2] = [0., 1., 0] # elbow_joint
    w0[:, 3] = [0., 1., 0] # wrist 1
    w0[:, 4] = [0., 0., -1] # wrist 2 
    w0[:, 5] = [0., 1., 0] # wrist 3

    # Rotation matrix from base_link to wrist_3_link in zero config
    R = np.array([[-1., 0., 0.],
                  [0., 0., 1.], 
                  [0., 1., 0.]])

    # YOUR CODE HERE (Task 1)
    q = [0.817, 0.233, 0.06285]

              
    gst0 = np.eye(4)
    gst0[:3, :3] = R
    gst0[:3, 3]  = q
    xi_array = np.zeros((6, 6))

    for i in range(6):
            v = -np.cross(w0[:, i], q0[:, i])
            xi_array[:,i] = np.hstack((v, w0[:, i])) 
    
    g = np.matmul(kfs.prod_exp(xi_array, joint_angles), gst0)
    return g
 



def ur7e_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the ur7e's end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of ur7e robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(6)
    # YOUR CODE HERE (Task 2
    angles[0] = joint_state.position[5]
    angles[1] = joint_state.position[0]
    angles[2] = joint_state.position[1]
    angles[3] = joint_state.position[2]
    angles[4] = joint_state.position[3]
    angles[5] = joint_state.position[4]
    # angles = np.array(joint_state.position[:6])
    return ur7e_foward_kinematics_from_angles(angles)

    # END YOUR CODE HERE
