import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import base64
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/realsense/aligned_images',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.frame_counter = 0
        self.save_dir = 'saved_images'
        os.makedirs(self.save_dir, exist_ok=True)

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image...')

        # Step 1: Decode base64 back to bytes
        img_data = self.ensure_base64_padding(msg.data)
        img_data = base64.b64decode(img_data)

        # Step 2: Decode bytes into OpenCV image
        np_arr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().error('Failed to decode image!')
            return

        # Step 3: Split into color and depth images
        height, width, _ = img.shape
        mid_point = width // 2

        color_image = img[:, :mid_point]  # Left half
        depth_image = img[:, mid_point:]  # Right half

        # Step 4: Show the images
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Depth Image (Colormap)', depth_image)

        # Step 5: Save images every 50 frames
        self.frame_counter += 1
        if self.frame_counter % 50 == 0:
            self.save_images(color_image, depth_image, self.frame_counter)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def save_images(self, color_img, depth_img, frame_num):
        color_filename = os.path.join(self.save_dir, f'color_{frame_num:06d}.png')
        depth_filename = os.path.join(self.save_dir, f'depth_{frame_num:06d}.png')
        cv2.imwrite(color_filename, color_img)
        cv2.imwrite(depth_filename, depth_img)
        self.get_logger().info(f'Saved color and depth images for frame {frame_num}')

    def ensure_base64_padding(self, b64_string):
        """Ensure the base64 string is properly padded."""
        # Convert the binary array to base64 string
        b64_string = base64.b64encode(b64_string).decode('utf-8')

        # Apply padding if necessary
        padding = len(b64_string) % 4
        if padding != 0:
            b64_string += "=" * (4 - padding)
        return b64_string

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
