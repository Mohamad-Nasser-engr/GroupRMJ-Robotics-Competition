import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BlackBandDetector(Node):
    def __init__(self):
        super().__init__('black_band_detector')
        self.sensor_sub = self.create_subscription(Float32, '/light_sensor', self.sensor_callback, 10)
        self.black_band_detected = False
        self.threshold = 0.2  
    def sensor_callback(self, msg):
        light_intensity = msg.data
        
        # Check if the detected light intensity corresponds to a black band
        if light_intensity < self.threshold:
            if not self.black_band_detected:
                self.get_logger().info("Black band detected!")
                self.black_band_detected = True
        else:
            if self.black_band_detected:
                self.get_logger().info("Still no black band detected!")
                self.black_band_detected = False

def main(args=None):
    rclpy.init(args=args)
    node = BlackBandDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
