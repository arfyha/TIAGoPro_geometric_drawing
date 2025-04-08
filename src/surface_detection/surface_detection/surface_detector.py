import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import Pose, PoseArray
import math
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion


class SurfaceDetector(Node):
    def __init__(self):
        super().__init__('surface_detector')
        self.get_logger().info('Surface Detector Node has been started.')
        self.set_parameters([{'use_sim_time': True}])
        

        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match the publisher's QoS
            history=HistoryPolicy.KEEP_LAST,
            durability = DurabilityPolicy.VOLATILE,
            depth=300
        )

        #self.create_subscription(PointCloud2, '/head_front_camera/depth/color/points', self.surcafe_callback, qos_profile_best_effort)

        self.drawing_points_pub = self.create_publisher(PoseArray, '/drawing_points', 10)
        self.timer = self.create_timer(1.0, self.calculate_drawing_points)

    def surcafe_callback(self, msg):
        # Placeholder for surface detection logic
        self.get_logger().info('Detecting surface...')

    def calculate_drawing_points(self):
        
        RADIUS = 0.2 # Radius of the circle
        CENTER_X = 0.5 # X-coordinate of the circle's center
        CENTER_Y = -0.17  # Y-coordinate of the circle's center
        CENTER_Z = 0.75 # Z-coordinate of the circle's center 
        NUM_POINTS = 10 # Number of points to generate on the circle
        q = quaternion_from_euler(0, math.pi / 2, 0)
        q_msg = Quaternion()
        q_msg.x = q[0]
        q_msg.y = q[1]
        q_msg.z = q[2]
        q_msg.w = q[3]

        drawing_points = PoseArray()
        header = Header()
        header.frame_id = "base_footprint"
        header.stamp = self.get_clock().now().to_msg()

        for i in range(NUM_POINTS + 1):
            pose = Pose()
            angle = i * 2 * math.pi / NUM_POINTS
            pose.position.x = CENTER_X + RADIUS * math.cos(angle)
            pose.position.y = CENTER_Y + RADIUS * math.sin(angle)
            pose.position.z = CENTER_Z
            pose.orientation = q_msg
            # Append the points to the drawing_points array
            drawing_points.poses.append(pose)
        
        self.drawing_points_pub.publish(drawing_points)
        self.get_logger().info('Drawing points calculated and published.')


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()