import time
import signal

import pigpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def lerp(x, x0, x1, y0, y1):
    if x1 == x0:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)

def angle_to_pulse_us(angle_deg: float) -> int:
    angle_deg = clamp(angle_deg, -45.0, 45.0)
    return int(round(lerp(angle_deg, -45.0, 45.0, 1100.0, 1700.0)))

def setAngle(pi: pigpio.pi, servo_pin: int, angle_deg: float) -> int:
    pulse = angle_to_pulse_us(angle_deg)
    pi.set_servo_pulsewidth(servo_pin, pulse)
    return pulse

def speed_to_pulse_us(speed: float) -> int:
    speed = clamp(speed, 0.0, 10.0)
    # как договорились: 0 -> 1500 (нейтраль), 10 -> 1600
    return int(round(lerp(speed, 0.0, 10.0, 1500.0, 1600.0)))

def setSpeed(pi: pigpio.pi, esc_pin: int, speed: float) -> int:
    pulse = speed_to_pulse_us(speed)
    pi.set_servo_pulsewidth(esc_pin, pulse)
    return pulse


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        self.servo_pin = 18
        self.esc_pin = 19

        # Safety/loop
        self.control_period_s = 0.02   # 50 Hz
        self.cmd_timeout_s = 2       # если дольше нет команд -> safe

        # Последние команды
        self.angle_deg = 0.0
        self.speed = 0.0
        self.last_angle_t = time.monotonic()
        self.last_speed_t = time.monotonic()

        # pigpio
        self.pi = pigpio.pi('127.0.0.1', 8888)
        if not self.pi.connected:
            raise RuntimeError("pigpio не подключился. Запусти: sudo pigpiod")

        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.esc_pin, pigpio.OUTPUT)

        # Арминг ESC: держим нейтраль
        self.pi.set_servo_pulsewidth(self.esc_pin, 1500)
        self.pi.set_servo_pulsewidth(self.servo_pin, angle_to_pulse_us(0.0))
        time.sleep(2.0)

        # ROS subs
        self.create_subscription(Float32, '/angle_stream', self.on_angle, 1)
        self.create_subscription(Float32, '/speed_stream', self.on_speed, 1)

        # Таймер управления
        self.timer = self.create_timer(self.control_period_s, self.control_loop)

        # SIGTERM
        signal.signal(signal.SIGTERM, self._on_sigterm)

        self.get_logger().info("DriveNode started.")

    def on_angle(self, msg: Float32):
        self.angle_deg = float(msg.data)
        self.last_angle_t = time.monotonic()

    def on_speed(self, msg: Float32):
        self.speed = float(msg.data)
        self.last_speed_t = time.monotonic()

    def control_loop(self):
        now = time.monotonic()

        angle_ok = (now - self.last_angle_t) <= self.cmd_timeout_s
        speed_ok = (now - self.last_speed_t) <= self.cmd_timeout_s

        angle = self.angle_deg if angle_ok else 0.0
        speed = self.speed if speed_ok else 0.0

        setAngle(self.pi, self.servo_pin, angle)
        setSpeed(self.pi, self.esc_pin, speed)

    def _on_sigterm(self, *_):
        self.get_logger().warn("SIGTERM -> stopping")
        self.stop()

    def stop(self):
        # безопасно: нейтраль и центр, затем выключить PWM
        try:
            self.pi.set_servo_pulsewidth(self.esc_pin, 1500)
            self.pi.set_servo_pulsewidth(self.servo_pin, angle_to_pulse_us(0.0))
            time.sleep(0.2)
            self.pi.set_servo_pulsewidth(self.esc_pin, 0)
            self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        finally:
            self.pi.stop()


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

