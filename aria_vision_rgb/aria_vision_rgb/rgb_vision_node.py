import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

COLOR_RANGES = {
    'blue':   ([100, 150,  50], [130, 255, 255]),
    'red':    ([0,   120,  70], [10,  255, 255]),
    'yellow': ([20,  100, 100], [35,  255, 255]),
    'green':  ([40,   70,  70], [80,  255, 255]),
}

KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def detect_shape(contour):
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    vertices = len(approx)
    if vertices == 3:
        return 'triangle'
    elif vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        ratio = w / float(h)
        return 'square' if 0.9 <= ratio <= 1.1 else 'rectangle'
    else:
        area = cv2.contourArea(contour)
        circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
        return 'circle' if circularity > 0.75 else 'ellipse'

class RGBVisionNode(Node):
    def __init__(self):
        super().__init__('rgb_vision_node')
        self.bridge = CvBridge()
        self.declare_parameter('min_area', 500)
        self.declare_parameter('confidence_threshold', 0.001)
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/vision/detections_2d', 10)
        self.get_logger().info('rgb_vision_node J2 démarré — détection robuste active')

    def image_callback(self, msg):
        min_area = self.get_parameter('min_area').value
        conf_threshold = self.get_parameter('confidence_threshold').value
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_area = frame.shape[0] * frame.shape[1]
        det_array = Detection2DArray()
        det_array.header = msg.header

        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                shape = detect_shape(cnt)
                score = float(area) / img_area
                if score < conf_threshold:
                    continue
                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(x + w // 2)
                det.bbox.center.position.y = float(y + h // 2)
                det.bbox.size_x = float(w)
                det.bbox.size_y = float(h)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = f'{color_name}_{shape}'
                hyp.hypothesis.score = score
                det.results.append(hyp)
                det_array.detections.append(det)
                self.get_logger().info(
                    f'[{color_name}_{shape}] bbox=({x},{y},{w},{h}) score={score:.4f}')

        self.pub_detections.publish(det_array)

def main(args=None):
    rclpy.init(args=args)
    node = RGBVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
