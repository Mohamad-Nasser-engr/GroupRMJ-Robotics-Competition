import rclpy
from rclpy.node import Node
import requests
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import String
from gipper_interface.action import GripControl  

class PepsiCanDetector(Node):
    def __init__(self):
        super().__init__('pepsi_can_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_control_client = self.create_client(GripControl, '/gripper_control')
        self.detection_url = 'http://localhost:9001/predict'

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert image to JPEG format
        _, img_encoded = cv2.imencode('.jpg', frame)
        img_bytes = img_encoded.tobytes()

        # Send the image to the inference server
        try:
            response = requests.post(self.detection_url, files={'file': img_bytes})
            if response.status_code == 200:
                detections = response.json()
                self.process_detections(detections)
            else:
                self.get_logger().warn(f"Inference server returned status code {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error communicating with inference server: {e}")

    def process_detections(self, detections):
        # Check if Pepsi can is detected
        for detection in detections.get('predictions', []):
            if detection['class'] == 'pepsi_can':
                self.get_logger().info("Pepsi can detected!")
                # Control the robot to approach the can
                self.move_robot_towards_can()
                # Control the gripper to grip the can
                self.call_gripper_service('grip')

    def move_robot_towards_can(self):
        # Create a Twist message to move the robot forward
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        twist.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot moving towards the can.")

        # Assuming you want to move for a certain amount of time and then stop
        self.create_timer(2.0, self.stop_robot)  # Stop after 2 seconds

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")

    def call_gripper_service(self, command):
        while not self.gripper_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for gripper control service...')
        request = GripControl.Request()
        request.command = command
        self.gripper_control_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = PepsiCanDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
