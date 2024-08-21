import rclpy
from rclpy.node import Node
import requests
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient
import numpy as np

class PepsiCanDetector(Node):
    def __init__(self):
        super().__init__('pepsi_can_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Initialize the InferenceHTTPClient
        self.client = InferenceHTTPClient(
            api_url="https://detect.roboflow.com",
            api_key="bwZdRPnoYYRFnrcscwxp"  # Replace with your actual API key
        )
        self.model_id = "pepsi-can-detection-krjyn/1"

        self.can_detected = False

    def laser_callback(self, msg):
        # Check if an object is detected in front of the robot
        if min(msg.ranges) < 1.0:  # Example threshold, adjust as needed
            self.get_logger().info("Object detected, analyzing...")
            self.can_detected = True
        else:
            self.get_logger().info("No object detected, searching...")
            self.can_detected = False

    def image_callback(self, msg):
        if not self.can_detected:
            return

        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert image to JPEG format
        _, img_encoded = cv2.imencode('.jpg', frame)
        img_bytes = img_encoded.tobytes()

        # Use the inference client to perform detection
        try:
            result = self.client.infer(img_bytes, model_id=self.model_id)
            self.process_detections(result)
        except Exception as e:
            self.get_logger().error(f"Error communicating with inference server: {e}")

    def process_detections(self, detections):
        # Check if Pepsi can is detected
        detected = False
        for detection in detections.get('predictions', []):
            if detection['class'] == 'pepsi_can':
                self.get_logger().info("Pepsi can detected!")
                detected = True
                break

        if not detected:
            self.get_logger().info("No Pepsi can detected.")
            

def main(args=None):
    rclpy.init(args=args)
    node = PepsiCanDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
