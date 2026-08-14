import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

import numpy as np


class ZedFusionNode(Node):

    def __init__(self):
        super().__init__('zed_fusion_node')

        # État interne

        # Dernière carte de profondeur reçue
        self._last_depth = None

        # Paramètres caméra
        # Résolution utilisée par le stub ZED
        self.width = 640
        self.height = 360

        # Intrinsèques caméra simulés
        self.fx = 700.0
        self.fy = 700.0
        self.cx = 320.0
        self.cy = 180.0

        # Frame de la caméra
        self.camera_frame = 'zed2i_left_camera_optical_frame'

        # Subscribers
        # Profondeur ZED
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/depth',
            self.on_depth,
            10
        )

        # Nuage de points ZED
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/zed/point_cloud',
            self.on_cloud,
            10
        )

        # Détections 2D de Liantsoa
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/vision/detections_2d',
            self.on_detections,
            10
        )

        # Publisher
        # Position 3D de la cible
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/vision/target_pose',
            10
        )

        self.get_logger().info(
            'zed_fusion_node J3 démarré'
        )

        self.get_logger().info(
            'En attente de /zed/depth et /vision/detections_2d'
        )

    # J2 — Réception de la profondeur

    def on_depth(self, msg):
        """
        Convertit le message ROS Image 32FC1
        en tableau NumPy contenant les distances en mètres.
        """

        try:
            self._last_depth = np.frombuffer(
                msg.data,
                dtype=np.float32
            ).reshape(msg.height, msg.width)

            self.get_logger().debug(
                f'Depth reçue : {msg.width}x{msg.height}'
            )

        except Exception as e:
            self._last_depth = None

            self.get_logger().error(
                f'Erreur lors de la conversion de la depth : {e}'
            )

    # J2 — Distance à un pixel
    def get_distance_at_pixel(self, u: int, v: int):
        """
        Retourne la distance en mètres au pixel (u, v).

        Retourne None si :
        - aucune depth n'est disponible ;
        - le pixel est hors image ;
        - la profondeur est NaN ;
        - la profondeur est infinie ;
        - la profondeur est <= 0.
        """

        if self._last_depth is None:
            return None

        height, width = self._last_depth.shape

        if not (0 <= u < width and 0 <= v < height):
            return None

        distance = float(self._last_depth[v, u])

        if not np.isfinite(distance):
            return None

        if distance <= 0.0:
            return None

        return distance

    # J1 — Réception du nuage de points
    def on_cloud(self, msg):
        """
        Lecture simple du nuage de points pour conserver
        le livrable J1.
        """

        points = list(
            point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )
        )

        self.get_logger().debug(
            f'Point cloud reçu : {len(points)} points'
        )

    # J3 — Fusion détection 2D + profondeur
    def on_detections(self, msg):
        """
        Reçoit les détections 2D produites par Liantsoa.
        Pour chaque détection :
        1. récupère le centre de la bounding box ;
        2. récupère la profondeur au centre ;
        3. reprojette le pixel en coordonnées 3D ;
        4. publie une PoseStamped sur /vision/target_pose.
        """

        if self._last_depth is None:
            self.get_logger().warn(
                'Détection reçue mais aucune depth disponible'
            )
            return

        for detection in msg.detections:

            # Centre de la bounding box
            u = int(
                round(
                    detection.bbox.center.position.x
                )
            )

            v = int(
                round(
                    detection.bbox.center.position.y
                )
            )

            # Récupération de la profondeur
            z = self.get_distance_at_pixel(u, v)

            if z is None:
                self.get_logger().warn(
                    f'Distance invalide au pixel ({u}, {v})'
                )
                continue

            # Informations de détection
            label = 'unknown'
            confidence = 0.0

            if detection.results:

                result = detection.results[0]

                label = result.hypothesis.class_id
                confidence = float(
                    result.hypothesis.score
                )
            # Reprojection pixel -> 3D
            #
            # Modèle sténopé :
            #
            # X = (u - cx) * Z / fx
            # Y = (v - cy) * Z / fy
            # Z = profondeur

            x = (
                (float(u) - self.cx)
                * z
                / self.fx
            )

            y = (
                (float(v) - self.cy)
                * z
                / self.fy
            )

            # Création de la pose 3D
            pose = PoseStamped()

            pose.header = msg.header
            pose.header.frame_id = self.camera_frame

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            # Orientation neutre
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            # Publication

            self.target_pub.publish(pose)

            # Log J3

            self.get_logger().info(
                f'[J3] {label} | '
                f'pixel=({u},{v}) | '
                f'3D=({x:.2f}, {y:.2f}, {z:.2f}) m | '
                f'confidence={confidence:.3f}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ZedFusionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()