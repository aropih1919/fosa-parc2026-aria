import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

# Plages HSV pour les couleurs cibles
COLOR_RANGES = {
    'blue':  ([100, 150, 50], [130, 255, 255]),
    'red':   ([0,   120, 70], [10,  255, 255]),
    'yellow':([20,  100, 100],[35,  255, 255]),
}

class RGBVisionNode(Node):
    def __init__(self):
        super().__init__('rgb_vision_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pub_detections = self.create_publisher(
            Detection2DArray, '/vision/detections_2d', 10)

        self.get_logger().info('rgb_vision_node démarré — en attente d image...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(x + w // 2)
                det.bbox.center.position.y = float(y + h // 2)
                det.bbox.size_x = float(w)
                det.bbox.size_y = float(h)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = color_name
                hyp.hypothesis.score    = float(area) / (frame.shape[0] * frame.shape[1])
                det.results.append(hyp)

                det_array.detections.append(det)
                self.get_logger().info(
                    f'Détecté : {color_name} | bbox=({x},{y},{w},{h}) | score={hyp.hypothesis.score:.3f}')

        self.pub_detections.publish(det_array)

def main(args=None):
    rclpy.init(args=args)
    node = RGBVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
