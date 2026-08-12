import time
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from aria_msgs.action import SearchAndReport


class MissionState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    SEARCHING = auto()
    CONFIRMING = auto()
    DONE = auto()


class MissionManagerNode(Node):

    ZONES = {
        'nord': (10.0, 0.0),
        'sud': (-10.0, 0.0),
        'est': (0.0, 10.0),
        'ouest': (0.0, -10.0),
    }

    def __init__(self):
        super().__init__('mission_manager_node')

        self.state = MissionState.IDLE
        self.CONFIDENCE_THRESHOLD = 0.7

        # Groupe de callbacks "réentrant" : autorise plusieurs callbacks
        # de ce groupe à s'exécuter en même temps, sur des threads différents.
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            SearchAndReport,
            'search_and_report',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group,
        )

        self.get_logger().info('mission_manager_node démarré, en attente de missions (IDLE).')

    def set_state(self, new_state: MissionState):
        self.get_logger().info(f'Transition : {self.state.name} -> {new_state.name}')
        self.state = new_state

    def navigate_to_zone(self, target_zone: str) -> bool:
        if target_zone not in self.ZONES:
            self.get_logger().error(f'Zone inconnue : "{target_zone}"')
            return False

        x, y = self.ZONES[target_zone]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Envoi du but Nav2 -> zone={target_zone} (x={x}, y={y})')
        self.nav_client.wait_for_server()

        nav_done_event = threading.Event()
        nav_result = {'success': False}

        def goal_response_callback(future):
            goal_handle_nav = future.result()
            if not goal_handle_nav.accepted:
                self.get_logger().error('But GPS refusé par Nav2.')
                nav_done_event.set()
                return
            self.get_logger().info('But GPS accepté par Nav2, navigation en cours...')
            result_future = goal_handle_nav.get_result_async()
            result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            status = future.result().status
            nav_result['success'] = (status == GoalStatus.STATUS_SUCCEEDED)
            self.get_logger().info(
                f'Résultat Nav2 reçu : {"succès" if nav_result["success"] else "échec"}'
            )
            nav_done_event.set()

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)

        nav_done_event.wait()
        return nav_result['success']

    def simulate_search(self, target_color: str, target_shape: str):
        self.get_logger().info(f'[SIMULATION] Recherche de "{target_color} {target_shape}"...')
        time.sleep(2.0)
        confidence = 0.92
        position = {'x': 3.5, 'y': 1.2, 'z': 0.0}
        return confidence, position

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f'Nouvelle mission reçue : zone={goal.target_zone}, '
            f'couleur={goal.target_color}, forme={goal.target_shape}'
        )

        feedback_msg = SearchAndReport.Feedback()

        self.set_state(MissionState.NAVIGATING)
        feedback_msg.current_state = self.state.name
        feedback_msg.info = f'Navigation vers {goal.target_zone}'
        goal_handle.publish_feedback(feedback_msg)

        nav_success = self.navigate_to_zone(goal.target_zone)

        if not nav_success:
            goal_handle.abort()
            result = SearchAndReport.Result()
            result.success = False
            result.message = 'Échec de la navigation.'
            self.set_state(MissionState.IDLE)
            return result

        self.set_state(MissionState.SEARCHING)
        feedback_msg.current_state = self.state.name
        feedback_msg.info = f'Recherche de {goal.target_color} {goal.target_shape}'
        goal_handle.publish_feedback(feedback_msg)

        confidence, position = self.simulate_search(goal.target_color, goal.target_shape)

        self.set_state(MissionState.CONFIRMING)
        feedback_msg.current_state = self.state.name
        feedback_msg.info = f'Vérification (confiance={confidence:.2f})'
        goal_handle.publish_feedback(feedback_msg)

        if confidence < self.CONFIDENCE_THRESHOLD:
            goal_handle.abort()
            result = SearchAndReport.Result()
            result.success = False
            result.message = f'Confiance insuffisante ({confidence:.2f}).'
            self.set_state(MissionState.IDLE)
            return result

        self.set_state(MissionState.DONE)

        result = SearchAndReport.Result()
        result.success = True
        result.position.x = position['x']
        result.position.y = position['y']
        result.position.z = position['z']
        result.distance = (position['x'] ** 2 + position['y'] ** 2) ** 0.5
        result.confidence = confidence
        result.message = 'Mission accomplie.'

        goal_handle.succeed()
        self.set_state(MissionState.IDLE)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()

    # MultiThreadedExecutor : plusieurs threads disponibles, pour éviter
    # le blocage mutuel entre execute_callback et les callbacks du nav_client.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()