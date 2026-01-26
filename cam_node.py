import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        device = int(self.get_parameter('device').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        period = 1.0 / max(1.0, fps)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f'Camera started: dev={device}, {width}x{height}@{fps:.1f}')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('No frame from camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
