import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose


class FakeNav2Node(Node):
    """Faux serveur Nav2, juste pour tester mission_manager_node sans Gazebo."""

    def __init__(self):
        super().__init__('fake_nav2_node')
        self._server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
        )
        self.get_logger().info('[FAKE NAV2] prêt à recevoir des buts de test.')

    def execute_callback(self, goal_handle):
        pos = goal_handle.request.pose.pose.position
        self.get_logger().info(f'[FAKE NAV2] But reçu : x={pos.x}, y={pos.y}. Simulation 3s...')
        time.sleep(3.0)
        goal_handle.succeed()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = FakeNav2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()