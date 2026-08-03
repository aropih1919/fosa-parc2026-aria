import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np

class ZedStubPublisher(Node):
    def __init__(self):
        super().__init__('zed_stub_publisher')
        self.depth_pub = self.create_publisher(Image, '/zed/depth', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/zed/point_cloud', 10)
        self.timer = self.create_timer(0.2, self.publish_fake_data)  # 5 Hz
        self.width, self.height = 640, 360

    def publish_fake_data(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'zed2i_left_camera_optical_frame'

        # --- Carte de profondeur synthétique (mètres) ---
        depth = np.full((self.height, self.width), 5.0, dtype=np.float32)
        # On simule un "objet" plus proche au centre de l'image (ex: le baril bleu)
        depth[150:210, 280:360] = 2.3

        depth_msg = Image()
        depth_msg.header = header
        depth_msg.height, depth_msg.width = self.height, self.width
        depth_msg.encoding = '32FC1'
        depth_msg.step = self.width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)

        # --- Nuage de points synthétique (échantillonné, pas pixel par pixel) ---
        points = []
        for v in range(0, self.height, 10):
            for u in range(0, self.width, 10):
                z = float(depth[v, u])
                x = (u - self.width / 2) * z / 500.0
                y = (v - self.height / 2) * z / 500.0
                points.append([x, y, z])

        pc_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(pc_msg)

def main():
    rclpy.init()
    node = ZedStubPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
