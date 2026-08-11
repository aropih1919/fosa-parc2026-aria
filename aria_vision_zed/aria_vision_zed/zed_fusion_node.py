import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

class ZedFusionNode(Node):
    def __init__(self):
        super().__init__('zed_fusion_node')
        self.create_subscription(Image, '/zed/depth', self.on_depth, 10)
        self.create_subscription(PointCloud2, '/zed/point_cloud', self.on_cloud, 10)

    def on_depth(self, msg):
        self.get_logger().info(f'Depth reçue : {msg.width}x{msg.height}, encoding={msg.encoding}')

    def on_cloud(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        self.get_logger().info(f'Point cloud reçu : {len(points)} points, premier = {points[0] if points else None}')
    
    def get_distance_at_pixel(self, u: int, v: int):
        """Retourne la distance en mètres au pixel (u,v), ou None si donnée invalide (NaN/Inf)."""
        if self._last_depth is None:
            return None
        if not (0 <= v < self._last_depth.shape[0] and 0 <= u < self._last_depth.shape[1]):
            return None
        d = float(self._last_depth[v, u])
        if not np.isfinite(d) or d <= 0.0:
            return None
        return d

def main():
    rclpy.init()
    rclpy.spin(ZedFusionNode())
    rclpy.shutdown()