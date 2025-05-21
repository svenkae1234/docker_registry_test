import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from elite_msgs.msg import RobotState  # Replace with actual package and message name
import numpy as np
import cv2
import base64
import os

class HandEyeCalibration(Node):
    def __init__(self):
        super().__init__('handeye_calibration')

        # === Parameters ===
        self.image_topic = '/realsense/aligned_images'
        self.checkerboard_size = (10, 7)  # Inner corners
        self.square_size = 0.023  # 2.5 cm
        self.output_dir = './calibration_data'
        os.makedirs(self.output_dir, exist_ok=True)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        self.robot_state_received = False

        self.robot_poses = []
        self.camera_poses = []
        self.image_counter = 0

        # === ROS Subscription ===
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense/camera_info',
            self.camera_info_callback,
            10
        )

        self.latest_robot_pose = None
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        self.robot_pose_to_save = None

        self.get_logger().info('HandEyeCalibration node started. Waiting for images...')

    def ensure_base64_padding(self, b64_data):
        """Ensure base64 input has proper padding."""
        b64_string = base64.b64encode(b64_data).decode('utf-8')
        padding = len(b64_string) % 4
        if padding != 0:
            b64_string += '=' * (4 - padding)
        return b64_string
    
    def wait_for_user(self):
        input("Move the robot to a new position and press Enter to capture the next image...")

    def save_robot_pose_to_file(self, pose, filename='robot_poses.txt'):
        with open(filename, 'a') as f:
            line = str(pose) + '\n'
            f.write(line)
        self.get_logger().info(f'Robot pose saved to {filename}')


    def capture_image(self):
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

    def get_camera_info(self):
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense/camera_info',
            self.camera_info_callback,
            10
        )

    
    def get_robot_state(self):
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )


    def robot_state_callback(self, msg):
        if not self.robot_state_received:
            if len(msg.machine_flange_pose) >= 6:
                pos = np.array(msg.machine_flange_pose[:3], dtype=np.float64)/1000
                rot = np.array(msg.machine_flange_pose[3:], dtype=np.float64)
                print(pos, rot)
                self.robot_pose_to_save = [pose/1000 for pose in msg.machine_flange_pose]


                angle = np.linalg.norm(rot)
                if angle > 1e-6:
                    axis = rot / angle
                    K = np.array([
                        [0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]
                    ])
                    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
                else:
                    R = np.eye(3)

                self.latest_robot_pose = (R, pos)
                self.robot_state_received = True
                self.get_logger().info('Robot state received.')

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics received.")


    def image_callback(self, msg):
        self.get_logger().info('Receiving image...')

        # === Decode the image ===
        img_data = self.ensure_base64_padding(msg.data)
        img_data = base64.b64decode(img_data)

        np_arr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().error('Failed to decode image!')
            return

        # === Split into color and depth ===
        height, width, _ = img.shape
        mid_point = width // 2

        color_image = img[:, :mid_point]   # Left half
        depth_image = img[:, mid_point:]   # Right half (not used here)

        # === Find checkerboard ===
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

        if ret:
            self.get_logger().info('Checkerboard detected.')

            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            objp = np.zeros((np.prod(self.checkerboard_size), 3), np.float32)
            objp[:, :2] = np.indices(self.checkerboard_size).T.reshape(-1, 2)
            objp *= self.square_size

            # === Camera intrinsics (FAKE) ===
            if not self.camera_info_received:
                self.get_logger().warn('Camera info not yet received. Skipping frame.')
                return
            else:
                self.get_logger().info('Camera info received.')
                

            camera_matrix = self.camera_matrix
            dist_coeffs = self.dist_coeffs

            success, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)

            if success:
                R_target2cam, _ = cv2.Rodrigues(rvec)
                t_target2cam = tvec.reshape(3)

                self.camera_poses.append((R_target2cam, t_target2cam))

                # === Get (fake) robot pose ===
                if not self.robot_state_received:
                    self.get_logger().warn('Robot pose not yet received. Skipping frame.')
                    return
                else:
                    self.get_logger().info('Robot pose received.')
                    

                R_gripper2base, t_gripper2base = self.latest_robot_pose


                self.robot_poses.append((R_gripper2base, t_gripper2base))

                self.save_robot_pose_to_file(self.robot_pose_to_save)

                # === Save the image ===
                img_path = os.path.join(self.output_dir, f'image_{self.image_counter:03d}.png')
                cv2.imwrite(img_path, color_image)

                self.image_counter += 1
                self.get_logger().info(f'Saved pose #{self.image_counter}')
                self.robot_state_received = False

                if self.image_counter >= 15:
                    self.get_logger().info('Collected enough samples. Performing calibration...')
                    self.perform_calibration()
                    rclpy.shutdown()

            else:
                self.get_logger().warn('solvePnP failed.')
        else:
            self.get_logger().info('Checkerboard not detected.')

        # === Show the color image ===
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Depth Image', depth_image)
        cv2.waitKey(1)

    def fake_robot_pose(self):
        """Generate fake robot poses for testing."""
        angle = np.random.uniform(-0.2, 0.2)
        axis = np.random.randn(3)
        axis /= np.linalg.norm(axis)

        # Create rotation matrix manually
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

        t = np.random.uniform(-0.05, 0.05, size=3)

        return R, t

    def perform_calibration(self):
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for (Rg, tg), (Rc, tc) in zip(self.robot_poses, self.camera_poses):
            R_gripper2base.append(Rg)
            t_gripper2base.append(tg)
            R_target2cam.append(Rc)
            t_target2cam.append(tc)

        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )

        T = np.eye(4)
        T[:3, :3] = R_cam2gripper
        T[:3, 3] = t_cam2gripper.flatten()

        self.get_logger().info('Hand-eye calibration finished.')
        print('Transformation (camera to end-effector):')
        print(T)

        np.save(os.path.join(self.output_dir, 'handeye_calibration.npy'), T)

def main(args=None):
    rclpy.init(args=args)

    node = HandEyeCalibration()

    while rclpy.ok() and node.image_counter < 15:
        node.wait_for_user()
        node.get_robot_state()
        node.capture_image()
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
