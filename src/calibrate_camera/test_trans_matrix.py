import numpy as np
from scipy.spatial.transform import Rotation as R

# Function to compute the rotation matrix from a quaternion (x, y, z, w)
def quaternion_to_rotation_matrix(q):
    # Extract the components of the quaternion
    x, y, z, w = q

    # Compute the rotation matrix components (3x3 matrix)
    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])
    return R

# Function to create a transformation matrix from translation and rotation
def create_transformation_matrix(translation, rotation):
    # Create a translation vector
    t = np.array([translation['x'], translation['y'], translation['z']])
    
    # Compute the rotation matrix from the quaternion
    R = quaternion_to_rotation_matrix([rotation['x'], rotation['y'], rotation['z'], rotation['w']])
    
    # Create the full 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = t
    
    return transformation_matrix

# Transformation data from ROS2 /tf_static topic
transforms = [
    # flange -> camera_adapter_link
    {
        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 6.123233995736766e-17}
    },
    # camera_adapter_link -> camera_adapter_tool0
    {
        "translation": {"x": 0.0, "y": 0.0, "z": 0.007},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    # camera_adapter_link -> d435i_link
    {
        "translation": {"x": 0.0, "y": 0.06598, "z": 0.01686},
        "rotation": {"x": -0.4639037882578495, "y": -0.46390378825784956, "z": -0.5336602620019747, "w": 0.5336602620019748}
    },
    # d435i_link -> camera_bottom_screw_frame
    {
        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    # camera_bottom_screw_frame -> camera_link
    {
        "translation": {"x": 0.010600000000000002, "y": 0.0175, "z": 0.0125},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
]

# Initialize the transformation matrix as identity
transformation_matrix = np.eye(4)

# Multiply the individual transformation matrices
for tf in transforms:
    current_transform = create_transformation_matrix(tf["translation"], tf["rotation"])
    transformation_matrix = np.dot(transformation_matrix, current_transform)

# The resulting transformation matrix should now be the transformation from flange to camera_link
print("Computed Transformation matrix from flange to camera_link:")
print(transformation_matrix)

# Example hand-eye calibration matrix (this would be your reference matrix)
hand_eye_calibration_matrix = np.array([
[0.98005266, 0.19229605, 0.0501897, -0.12216612],
 [-0.19873419, 0.94671125, 0.2534611, -0.18043967],
 [0.00122441, -0.25837964, 0.96604268, -0.01745616],
 [0., 0., 0., 1.]
])

def decompose_transform(matrix):
    """Extract rotation matrix and translation vector from a 4x4 transformation matrix."""
    rotation_matrix = matrix[:3, :3]
    translation_vector = matrix[:3, 3]
    return rotation_matrix, translation_vector

def compute_transform_error(gt_matrix, est_matrix):
    # Decompose both matrices
    R_gt, t_gt = decompose_transform(gt_matrix)
    R_est, t_est = decompose_transform(est_matrix)

    # Translation error (Euclidean distance)
    translation_error = np.linalg.norm(t_gt - t_est)

    # Rotation error (angle in degrees between rotation matrices)
    R_diff = R.from_matrix(R_gt.T @ R_est)  # Relative rotation
    rotation_error_deg = R_diff.magnitude() * (180.0 / np.pi)

    return translation_error, rotation_error_deg

# Example usage
translation_error, rotation_error = compute_transform_error(transformation_matrix, hand_eye_calibration_matrix)
print(f"Translation error: {translation_error:.6f} meters")
print(f"Rotation error: {rotation_error:.6f} degrees")

