import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math

class ScanPublisher(Node):
    def __init__(self):
        super().__init__('laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.marker_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/marker_pose',
            self.marker_callback,
            10
        )
        self.marker_position = None  # Store marker position
        self.timer = self.create_timer(0.5, self.publish_scan)

    def marker_callback(self, msg):
        """Callback to update the marker position."""
        self.marker_position = msg.pose.position

    def publish_scan(self):
        """Publish a fake LaserScan considering marker position as an obstacle."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        # Laser scan parameters
        scan.angle_min = -math.pi / 2  # -90 degrees
        scan.angle_max = math.pi / 2   # +90 degrees
        scan.angle_increment = math.pi / 180  # 1 degree increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0

        # Generate default ranges (no obstacles)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [float('inf')] * num_readings
        scan.intensities = [1.0] * num_readings

        # Simulate marker as an obstacle if available
        if self.marker_position:
            # Calculate distance and angle to the marker
            distance = math.sqrt(self.marker_position.x**2 + self.marker_position.y**2)
            angle = math.atan2(self.marker_position.y, self.marker_position.x)

            # Find the corresponding index in the scan array
            scan_index = int((angle - scan.angle_min) / scan.angle_increment)

            if 0 <= scan_index < num_readings:
                scan.ranges[scan_index] = min(distance, scan.range_max)

        # Publish the scan
        self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
