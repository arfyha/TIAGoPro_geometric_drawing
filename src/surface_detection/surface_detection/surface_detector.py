import rclpy
from rclpy import Node


class SurfaceDetector(Node):
    def __init__(self):
        super().__init__('surface_detector')
        self.get_logger().info('Surface Detector Node has been started.')

    def detect_surface(self):
        # Placeholder for surface detection logic
        self.get_logger().info('Detecting surface...')

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()