import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import ultralytics
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class DecisionEngine(Node):
    def __init__(self):
        super().__init__('decision_engine')

        self.model = ultralytics.YOLO("yolo11m.pt")
        self.bridge = CvBridge()

        self.cam_sub = super().create_subscription(
            Image,
            "/camera/image_raw",
            self.process_image,
            qos_profile_sensor_data
        )

        self.angles_pub = super().create_publisher(
            Float32,
            "/angle_stream",
            1
        )
        self.speed_pub = super().create_publisher(
            Float32,
            "/speed_stream",
            1
        )

    def process_image(self, msg: Image):
        print("img new")
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        res = self.model(img)[0]
        stop_sign_detected = False

        for box, cls, conf in zip(res.boxes.xyxy, res.boxes.cls, res.boxes.conf):
            if int(cls) == 11 and float(conf) > 0.4:
                stop_sign_detected = True

        speed = 0.0 if stop_sign_detected else 3.0

        msg = Float32()
        msg.data = speed
        self.speed_pub.publish(msg)

def main():
    rclpy.init()
    node = DecisionEngine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
