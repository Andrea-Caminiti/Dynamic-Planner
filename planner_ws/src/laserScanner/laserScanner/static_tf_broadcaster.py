import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')

        # Create the TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms
        self.publish_static_transforms()

    def publish_static_transforms(self):
        # global_map → base_link
        gmap_to_base = TransformStamped()
        gmap_to_base.header.stamp = self.get_clock().now().to_msg()
        gmap_to_base.header.frame_id = 'map'
        gmap_to_base.child_frame_id = 'base_link'
        gmap_to_base.transform.translation.x = 0.0
        gmap_to_base.transform.translation.y = 0.0
        gmap_to_base.transform.translation.z = 0.0
        gmap_to_base.transform.rotation.w = 1.0

        # base_link → laser_frame
        base_to_laser = TransformStamped()
        base_to_laser.header.stamp = self.get_clock().now().to_msg()
        base_to_laser.header.frame_id = 'base_link'
        base_to_laser.child_frame_id = 'laser_frame'
        base_to_laser.transform.translation.x = 0.2  # 20 cm forward from base
        base_to_laser.transform.translation.y = 0.0
        base_to_laser.transform.translation.z = 0.3  # 30 cm above ground
        base_to_laser.transform.rotation.w = 1.0

        # Broadcast both transforms
        self.tf_broadcaster.sendTransform([gmap_to_base, base_to_laser])
        self.get_logger().info('Published static transforms.')


def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
