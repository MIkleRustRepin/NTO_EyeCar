import time
import ultralytics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import threading
import torch
from ultralytics import YOLO

class DecisionEngine(Node):
    def __init__(self):
        super().__init__('decision_engine')
        self.get_logger().warn("INIT")

        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # YOLO
        self.ncnn_model = YOLO("./yolo26n_ncnn_model")
        self.slow_coef = 1.0
        self.last_frame = None

        self.yolo_stop = threading.Event()

        self.lock = threading.Lock()

        self.yolo_thread = threading.Thread(
            target=self.yolo_loop,
            daemon=True,
            name="yolo_thread"
        )
        self.yolo_thread.start()
        # YOLO

        self.last_angle = 0
        self.default_speed = 15.0

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
        self.create_timer(0.02, self.tick)

    def tick(self):
        for _ in range(2):
            self.cam.grab()
        ok, frame = self.cam.read()

        if not ok:
            self.get_logger().warn('No frame from camera')
            return

        with self.lock:
            self.last_frame = frame

        direction = self.get_direction(frame)

        angle_msg = Float32()
        angle_msg.data = float(direction)
        self.angles_pub.publish(angle_msg)

        speed_msg = Float32()
        speed_msg.data = self.default_speed * (1 - self.slow_coef)
        self.speed_pub.publish(speed_msg)

    def get_direction(self, img):
        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 40, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)

            h, w = mask.shape[:2]

            roi = mask[int(h * 0.97):h, w // 2:w]
            roi = cv2.resize(roi, (64, 64))

            ys, xs = np.where(roi > 0)

            if len(xs) == 0:
                return 0

            top_idx = np.argmin(ys)
            bottom_idx = np.argmax(ys)

            x_top = xs[top_idx]
            x_bottom = xs[bottom_idx]

            x_mean = (x_top + x_bottom) / 2.0

            return (x_mean - 32) * -3
        except Exception as e:
            return 0

    def yolo_loop(self):
        self.get_logger().warn("YOLO thread started")

        while not self.yolo_stop.is_set():
            frame = None

            with self.lock:
                if self.last_frame is not None:
                    frame = self.last_frame.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                with torch.no_grad():
                    stop = self.detect_people(frame)
                self.slow_coef = 1.0 if stop else 0.0
            except Exception as e:
                self.get_logger().error(f"YOLO error: {e}")

            time.sleep(0.02)

    def detect_people(self, img) -> bool:
        results = self.ncnn_model(img, imgsz=320)
        people = 0 in list(results[0].boxes.cls.cpu().numpy())
        self.get_logger().warn("P") if people else self.get_logger().warn("NP")
        return people

    def destroy_yolo_loop(self):
        self.get_logger().warn("Stopping YOLO thread...")
        self.yolo_stop.set()
        self.yolo_thread.join(timeout=1.0)
        self.get_logger().warn("YOLO thread stopped")

def main():
    rclpy.init()
    node = DecisionEngine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_yolo_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
