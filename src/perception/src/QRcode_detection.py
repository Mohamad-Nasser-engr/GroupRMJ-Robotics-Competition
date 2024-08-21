import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.qr_code_detector = cv2.QRCodeDetector()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect and decode the QR code
            data, bbox, _ = self.qr_code_detector.detectAndDecode(frame)

            if bbox is not None and data:
                # If a QR code was detected and successfully decoded
                self.get_logger().info(f"QR Code detected: {data}")
                self.draw_bbox(frame, bbox)

            else:
                self.get_logger().info("No QR Code detected.")

            # Optionally, display the frame for debugging
            self.display_frame(frame)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def draw_bbox(self, frame, bbox):
        if bbox is not None:
            # Draw bounding box around the detected QR code
            n = len(bbox)
            for j in range(n):
                point1 = tuple(bbox[j][0])
                point2 = tuple(bbox[(j + 1) % n][0])
                cv2.line(frame, point1, point2, (255, 0, 0), 3)

    def display_frame(self, frame):
        # Display the frame with bounding box in a window
        cv2.imshow("QR Code Detection", frame)
        cv2.waitKey(1)  # Display the image for 1 ms

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
