import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanPublisher(Node):
    def __init__(self):
        super().__init__('laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.5, self.publish_scan)

    def publish_scan(self):
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

        # Generate dummy ranges (simulating obstacles at 2m and 3m)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [float('inf')] * num_readings

        # Simulate obstacles
        scan.ranges[num_readings // 3] = 2.0  # Obstacle at -30 degrees
        scan.ranges[2 * num_readings // 3] = 3.0  # Obstacle at +30 degrees

        scan.intensities = [1.0] * num_readings

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
