import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_scan_time = None  # Przechowuj czas ostatniego skanu

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom,
            10
        )
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.handle_scan,
            10
        )

    def handle_scan(self, msg):
        # Zapisz czas ostatniego skanu
        self.last_scan_time = msg.header.stamp

    def handle_odom(self, msg):
        t = TransformStamped()
        # Użyj czasu z ostatniego skanu (jeśli jest)
        if self.last_scan_time is not None:
            t.header.stamp = self.last_scan_time
        else:
            t.header.stamp = msg.header.stamp  # fallback na czas z odometrii
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()