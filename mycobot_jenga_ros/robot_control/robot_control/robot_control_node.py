import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from mycobot_interfaces.msg import DetectedBlockArray



def rotation_matrix_x(theta):
    """
    Compute the 3D rotation matrix around the x-axis.
    
    Parameters:
    theta (float): Rotation angle in radians.
    
    Returns:
    np.array: 3x3 rotation matrix.
    """
    return np.array([
        [1, 0, 0],
        [0, math.cos(theta), -math.sin(theta)],
        [0, math.sin(theta), math.cos(theta)]
    ])

def rotation_matrix_y(theta):
    """
    Compute the 3D rotation matrix around the y-axis.
    
    Parameters:
    theta (float): Rotation angle in radians.
    
    Returns:
    np.array: 3x3 rotation matrix.
    """
    return np.array([
        [math.cos(theta), 0, math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0, math.cos(theta)]
    ])

def rotation_matrix_z(theta):
    """
    Compute the 3D rotation matrix around the z-axis.
    
    Parameters:
    theta (float): Rotation angle in radians.
    
    Returns:
    np.array: 3x3 rotation matrix.
    """
    return np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])

def forward_kinematics_mycobot(theta):
    T01 = transformation_matrix(rotation_matrix_z(theta[0]), [0, 0, 0.13956])
    T12 = transformation_matrix(rotation_matrix_z(theta[1]), [0, 0, -0.001]) @ \
          transformation_matrix(rotation_matrix_y(1.5708) @ rotation_matrix_x(-1.5708), [0, 0, 0])
    T23 = transformation_matrix(rotation_matrix_z(theta[2]), [-0.1104, 0, 0])
    T34 = transformation_matrix(rotation_matrix_z(theta[3]), [-0.096, 0, 0.06462]) @ \
          transformation_matrix(rotation_matrix_z(-1.5708), [0, 0, 0])
    T45 = transformation_matrix(rotation_matrix_z(theta[4]), [0, -0.07318, 0]) @ \
          transformation_matrix(rotation_matrix_x(1.5708) @ rotation_matrix_y(-1.5708), [0, 0, 0])
    T56 = transformation_matrix(rotation_matrix_z(theta[5]), [0, 0.0456, 0]) @ \
          transformation_matrix(rotation_matrix_x(-1.5708), [0, 0, 0])

    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    return T


def rotation_matrix_to_axis_angle(R):
    """
    Convert a 3x3 rotation matrix to axis-angle representation
    
    Args:
        R: 3x3 rotation matrix (numpy array)
    
    Returns:
        (axis, angle) where:
        - axis is a unit vector (numpy array)
        - angle is in radians
    """
    # Ensure the matrix is a valid rotation matrix
    assert R.shape == (3, 3), "Input must be a 3x3 matrix"
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError("Matrix is not a valid rotation matrix")
    
    # TODOne: Compute the rotation angle
    cos_angle = (np.trace(R) - 1) / 2
    cos_angle = max(min(cos_angle, 1.0), -1.0)
    angle = math.acos(cos_angle)

    
    # Handle special cases
    if abs(angle) < 1e-6:
        # No rotation (identity matrix)
        return np.array([1, 0, 0]), 0.0
    elif abs(angle - math.pi) < 1e-6:
        # 180 degree rotation - special handling needed
        # Axis is the normalized vector from non-diagonal elements
        axis = np.array([
            math.sqrt((R[0, 0] + 1)/2),
            math.sqrt((R[1, 1] + 1)/2),
            math.sqrt((R[2, 2] + 1)/2)
        ])
        # Determine signs of axis components
        if R[0, 2] - R[2, 0] < 0:
            axis[1] = -axis[1]
        if R[1, 0] - R[0, 1] < 0:
            axis[2] = -axis[2]
        if R[2, 1] - R[1, 2] < 0:
            axis[0] = -axis[0]
    else:
        # General case
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * math.sin(angle))
    
    # Normalize the axis (should already be unit vector, but just in case)
    axis = axis / np.linalg.norm(axis)
    
    return axis, angle


def transformation_matrix(R, t):
    # Ensure the inputs are numpy arrays
    rotation_matrix = np.array(R)
    translation_vector = np.array(t)

    
    # Create the transformation matrix
    transformation_matrix = np.eye(4)  # Start with a 4x4 identity matrix
    transformation_matrix[:3, :3] = rotation_matrix  # Set the top-left 3x3 block to the rotation matrix
    transformation_matrix[:3, 3] = translation_vector  # Set the top-right 3x1 block to the translation vector
    
    return transformation_matrix

def compute_error(current_T, ref_T):
    """
    Compute the difference between two homogeneous transformation matrices.
    
    Parameters:
    current_T (np.array): 4 x 4 transformation matrix.
    ref_T (np.array): 4 x 4 transformation matrix.
    
    Returns:
    np.array: 6x1 error term, first three term for rotational error, last three term for translational error.
    """
    axis, angle = rotation_matrix_to_axis_angle(np.transpose(current_T[:3, :3]) @ ref_T[:3, :3])
    rot_error = axis * angle * 0.05  # scale rotation error down
    vec_error = current_T[:3, 3] - ref_T[:3, 3]
    return np.concatenate((rot_error, vec_error))

def jacobian_fd(angles, ref_T):
    #     """
    #     Compute the Jacobian matrix numerically using finite difference
    #     Parameters:
    #     angles (list): joint angles

    #     Returns:
    #     np.array: 6xlen(angles) jacobian matrix
    #     """

    # TODOne: compute jacobian using finite difference. You only need to
    # complete one jacobian function, either jacobian_fd or jacobian.

    n_joints = len(angles)
    epsilon = 0.001
    J = np.zeros((6, n_joints))

    # Get current end-effector pose (needed to compute Jacobian contributions)
    base_T = forward_kinematics_mycobot(angles)
    
    # Extract rotation and position from the transformation matrix
    base_R = base_T[:3, :3]
    base_p = base_T[:3, 3]

    for j in range(n_joints):
        # Perturb joint j by +/-epsilon
        P = angles.copy()
        M = angles.copy()

        P[j] += epsilon
        M[j] -= epsilon

        # Compute forward kinematics for perturbed angles
        f_P = compute_error(forward_kinematics_mycobot(P), ref_T)
        f_M = compute_error(forward_kinematics_mycobot(M), ref_T)
        
        # DEBUG: REMOVE ABOVE?    f_P = compute_error(forward_kinematics_mycobot(P), base_T)
        # DEBUG: REMOVE ABOVE?    f_M = compute_error(forward_kinematics_mycobot(M), base_T)

        J[:, j] = (f_P - f_M) / (2.0 * epsilon)

    return J

def shift_angle(angles):
    # shift angles to between 0 and 2*pi
    return np.mod(angles + np.pi, 2*np.pi) - np.pi
   

def inverse_kinematics_mycobot(ref_T):
    """
    Compute IK using Gauss-Newton Algorithm for MyCobot 280.
    
    Parameters:
    ref_T (np.array): 4x4 target transformation matrix.
    
    Returns:
    np.array: 6x1 joint angles
    """
    # MyCobot 280 joint limits (educatedly guessed numbers)
    joint_limits = [
        (-2.931, 2.931),     # Joint 1
        (-2.355, 2.355),     # Joint 2
        (-2.616, 2.616),     # Joint 3
        (-2.529, 2.519),     # Joint 4
        (-2.878, 2.878),     # Joint 5
        (-3.13, 3.13),      # Joint 6
    ]


    # Random initial guess within joint limits
    angles = np.array([np.random.uniform(low, high) for (low, high) in joint_limits])
    
    max_iter = 300
    tol = 1e-3

    for i in range(max_iter):
        current_T = forward_kinematics_mycobot(angles)
        cur_error = compute_error(current_T, ref_T)

        if np.linalg.norm(cur_error) < tol:
            print(f"Converged after {i} iterations")
            return shift_angle(angles)

        J = jacobian_fd(angles, ref_T)
        # DEBUG: IF PROBLEMS WITH JACOBIAN:  print("Jacobian:\n", J)

        max_step = 0.2 #DEBUG
        delta_th = np.linalg.pinv(J) @ cur_error
        delta_th = np.clip(delta_th, -max_step, max_step) #DEBUG

        
        angles += delta_th

    print("Warning: IK did not converge.")
    print("Final error norm:", np.linalg.norm(cur_error), "Error vector:", cur_error)
    return None

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Subscribe to detected blocks from vision node
        self.subscription = self.create_subscription(
            DetectedBlockArray,
            '/detected_blocks',
            self.blocks_callback,  # Match this with method name below
            10)

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10)

        self.get_logger().info("RobotControlNode initialized and listening to /detected_blocks.")


    def blocks_callback(self, msg):
        print("Callback triggered!")
        if not msg.blocks:
            self.get_logger().info("No blocks detected.")
            return

        # pick the first block
        block = msg.blocks[0]
        pos = block.position
        ori = block.orientation

        # Now print
        print(f"Target position: {pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}")
        print(f"Target orientation (quat): {ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}")

        rot = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_matrix()
        
        target_T = transformation_matrix(rot, [pos.x, pos.y, pos.z])
        joint_angles = inverse_kinematics_mycobot(target_T)

        if joint_angles is not None:
            js = JointState()
            js.name = [f'joint_{i+1}' for i in range(6)]
            js.position = [float(angle) for angle in joint_angles]
            print(f"Publisher is valid: {self.joint_pub is not None}")
            self.joint_pub.publish(js)
            print("Published joint command!")
        else:
            self.get_logger().warn("IK solution not found or did not converge.")




def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

