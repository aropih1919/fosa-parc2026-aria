import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class GpsNavNode(Node):
    """J2 : lecture GPS + envoi d'un premier goal à Nav2."""

    def __init__(self):
        super().__init__('gps_nav_node')

        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)

        # abonnement à la position déjà convertie en local par navsat_transform_node
        self.local_sub = self.create_subscription(
            PoseStamped, '/gps/local_pose', self.local_pose_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_envoye = False

    def gps_callback(self, msg):
        self.get_logger().info(
            f'GPS reçu -> lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}')

    def local_pose_callback(self, msg):
        # dès qu'on a une position locale valide, on envoie un goal une seule fois (test J2)
        if not self.goal_envoye:
            self.envoyer_goal(msg.pose.position.x + 2.0, msg.pose.position.y)
            self.goal_envoye = True

    def envoyer_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.get_logger().info(f'Envoi du goal -> x: {x:.2f}, y: {y:.2f}')
        self.nav_client.send_goal_async(goal_msg)


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