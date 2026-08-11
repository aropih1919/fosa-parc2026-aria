import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from aria_msgs.action import SearchAndReport


class MissionState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    SEARCHING = auto()
    CONFIRMING = auto()
    DONE = auto()


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.state = MissionState.IDLE

        # Seuil de confiance minimum pour valider une détection (utilisé en CONFIRMING)
        self.CONFIDENCE_THRESHOLD = 0.7

        # Le serveur d'action : porte d'entrée officielle de vos missions.
        # execute_callback est appelé automatiquement dès qu'un goal arrive.
        self._action_server = ActionServer(
            self,
            SearchAndReport,
            'search_and_report',
            execute_callback=self.execute_callback,
        )

        self.get_logger().info('mission_manager_node démarré, en attente de missions (IDLE).')

    # ---------- Gestion des états ----------

    def set_state(self, new_state: MissionState):
        """Point unique de changement d'état : logue toujours la transition."""
        self.get_logger().info(f'Transition : {self.state.name} -> {new_state.name}')
        self.state = new_state

    # ---------- Fonctions simulées (à remplacer par la vraie intégration en J3) ----------

    def simulate_navigation(self, target_zone: str) -> bool:
        """
        Simule l'appel à Edinah (Nav2). En J3, ceci sera remplacé par un vrai
        échange (ex: envoi d'un but Nav2 + attente du résultat réel).
        """
        self.get_logger().info(f'[SIMULATION] Navigation vers "{target_zone}"...')
        time.sleep(2.0)  # simule le temps de trajet
        return True  # succès simulé

    def simulate_search(self, target_color: str, target_shape: str):
        """
        Simule l'appel à Liantsoa/Faneva (détection + position 3D).
        Retourne (confidence, position_xyz) simulés.
        """
        self.get_logger().info(f'[SIMULATION] Recherche de "{target_color} {target_shape}"...')
        time.sleep(2.0)  # simule le temps de recherche
        confidence = 0.92
        position = {'x': 3.5, 'y': 1.2, 'z': 0.0}
        return confidence, position

    # ---------- Le cœur : exécution d'une mission ----------

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f'Nouvelle mission reçue : zone={goal.target_zone}, '
            f'couleur={goal.target_color}, forme={goal.target_shape}'
        )

        feedback_msg = SearchAndReport.Feedback()

        # --- Étape NAVIGATING ---
        self.set_state(MissionState.NAVIGATING)
        feedback_msg.current_state = self.state.name
        feedback_msg.info = f'Navigation vers {goal.target_zone}'
        goal_handle.publish_feedback(feedback_msg)

        nav_success = self.simulate_navigation(goal.target_zone)

        if not nav_success:
            goal_handle.abort()
            result = SearchAndReport.Result()
            result.success = False
            result.message = 'Échec de la navigation.'
            return result

        # --- Étape SEARCHING ---
        self.set_state(MissionState.SEARCHING)
        feedback_msg.current_state = self.state.name
        feedback_msg.info = f'Recherche de {goal.target_color} {goal.target_shape}'
        goal_handle.publish_feedback(feedback_msg)

        confidence, position = self.simulate_search(goal.target_color, goal.target_shape)

        # --- Étape CONFIRMING ---
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

        # --- Étape DONE ---
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

        # On redevient disponible pour une prochaine mission
        self.set_state(MissionState.IDLE)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()