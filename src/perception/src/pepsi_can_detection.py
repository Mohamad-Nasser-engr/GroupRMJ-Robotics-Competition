import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient
import numpy as np
import logging

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('pepsi_can_detector')

class PepsiCanDetector(Node):
    def __init__(self):
        super().__init__('pepsi_can_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Initialize the InferenceHTTPClient
        self.client = InferenceHTTPClient(
            api_url="https://detect.roboflow.com",
            api_key="bwZdRPnoYYRFnrcscwxp"  
        )
        self.model_id = "pepsi-can-detection-krjyn/1"

        # Set initial state
        self.can_detected = False

        logger.info('PepsiCanDetector node has been started.')

    def laser_callback(self, msg):
        # Check if an object is detected in front of the robot
        if min(msg.ranges) < 1.0:  # Example threshold, adjust as needed
            logger.info("Object detected, analyzing...")
            self.can_detected = True
        else:
            logger.info("No object detected, searching...")
            self.can_detected = False

    def image_callback(self, msg):
        if not self.can_detected:
            return

        logger.debug('Received image message.')
        # Convert ROS Image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            logger.debug('Converted image to OpenCV format.')
            
            # Convert image to JPEG format
            _, img_encoded = cv2.imencode('.jpg', frame)
            img_bytes = img_encoded.tobytes()

            # Use the inference client to perform detection
            try:
                result = self.client.infer(img_bytes, model_id=self.model_id)
                self.process_detections(result)
            except Exception as e:
                logger.error(f"Error communicating with inference server: {e}")

        except Exception as e:
            logger.error(f'Failed to convert image: {e}')

    def process_detections(self, detections):
        logger.debug('Processing detections.')
        # Check if Pepsi can is detected
        detected = False
        for detection in detections.get('predictions', []):
            if detection['class'] == 'pepsi_can':
                logger.info("Pepsi can detected!")
                detected = True
                break

        if not detected:
            logger.info("No Pepsi can detected.")
            # Optionally, handle case when no can is detected
            self.handle_no_detection()

    def handle_no_detection(self):
        # Implement actions to take when no Pepsi can is detected
        logger.info('No action needed as Pepsi can is not detected.')

def main(args=None):
    rclpy.init(args=args)
    node = PepsiCanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

