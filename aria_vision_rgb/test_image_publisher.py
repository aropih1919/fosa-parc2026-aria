import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0, self.publish_image)
        self.get_logger().info('Test image publisher démarré')

    def publish_image(self):
        # Créer une image noire 640x480
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Cercle bleu
        cv2.circle(img, (100, 100), 60, (255, 0, 0), -1)

        # Rectangle rouge
        cv2.rectangle(img, (250, 50), (400, 150), (0, 0, 255), -1)

        # Triangle jaune
        pts = np.array([[500, 150], [440, 50], [560, 50]], np.int32)
        cv2.fillPoly(img, [pts], (0, 255, 255))

        # Carré vert
        cv2.rectangle(img, (50, 300), (150, 400), (0, 255, 0), -1)

        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.pub.publish(msg)
        self.get_logger().info('Image test publiée')

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
