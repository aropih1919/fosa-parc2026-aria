import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsNavNode(Node):
    """J1 : lecture du GPS simulé, affichage des coordonnées."""

    def __init__(self):
        super().__init__('gps_nav_node')
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)

    def gps_callback(self, msg):
        self.get_logger().info(
            f'GPS reçu -> lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}, alt: {msg.altitude:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()