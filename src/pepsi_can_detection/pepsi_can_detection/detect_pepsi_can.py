import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import requests
import cv2
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from control_msgs.action import GripperCommand

class PepsiCanDetector(Node):
    def __init__(self):
        super().__init__('pepsi_can_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_control')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.detection_url = 'http://localhost:9001/predict'

        self.can_detected = False

    def laser_callback(self, msg):
        # Check if an object is detected in front of the robot
        if min(msg.ranges) < 1.0:  # Example threshold, adjust as needed
            self.get_logger().info("Object detected, analyzing...")
            self.can_detected = True
        else:
            self.get_logger().info("No object detected, searching...")
            self.search_for_can()

    def search_for_can(self):
        # Rotate the robot to search for the can
        twist = Twist()
        twist.angular.z = 0.5  # Rotate in place
        self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        if not self.can_detected:
            return

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
                self.can_detected = False
                self.move_robot_towards_can()
                return

        self.get_logger().info("No Pepsi can detected.")
        self.can_detected = False
        self.search_for_can()

    def move_robot_towards_can(self):
        # Create a Twist message to move the robot forward
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        twist.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot moving towards the can.")
        self.create_timer(2.0, self.stop_robot)  # Stop after 2 seconds

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped.")

        # Call gripper to close
        self.gripper_control('close')

    def gripper_control(self, command):
        goal_msg = GripperCommand.Goal()
        if command == 'close':
            goal_msg.command.position = 0.0  # Close gripper
        elif command == 'open':
            goal_msg.command.position = 1.0  # Open gripper

        self.gripper_action_client.wait_for_server()
        self.get_logger().info("Sending gripper command.")
        self.gripper_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PepsiCanDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

