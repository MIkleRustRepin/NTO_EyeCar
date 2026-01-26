import RPi.GPIO as GPIO
import pigpio
import time
import os
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MotorNode(Node):
    def __init__(self):
        print("INIT")
        super().__init__('motor_node')

        self.servo_pin = 18
        self.motors_pin = 19

        try:
            self.pi = pigpio.pi('127.0.0.1', 8888)
        except:
            print("Надо sudo pigpiog")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        self.init_motors()

        self.angles_sub = self.create_subscription(
            Float32,
            '/angle_stream',
            self.set_angle,
            1
        )

        self.speed_sub = self.create_subscription(
            Float32,
            '/speed_stream',
            self.set_motors_speed,
            1

        )

    def init_motors(self):
        signal.signal(signal.SIGTERM, self.shutdown)
        self.pi.set_mode(self.motors_pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.motors_pin, 0)

    def release_motors(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.shutdown()

    def set_angle(self, msg: Float32):
        self.get_logger().info(self.angle_to_pulse(msg.data))
        angle = msg.data

        if angle >= 90:
            angle = angle - 90
        else:
            angle = angle + 90

        duty = 2 + (angle / 18)
        self.pwm.ChangeDutyCycle(duty)
        self.pwm.ChangeDutyCycle(0)

    def set_motors_speed(self, msg: Float32):
        print("SS")
        speed = msg.data
        self.pi.set_servo_pulsewidth(self.motors_pin, self.angle_to_pulse(speed))

    def angle_to_pulse(self, angle):
        angle = angle + 75
        angle = min(angle, 110)
        angle = max(75, angle)
        return int(500 + (angle / 180.0) * 2000)

    def shutdown(self, *_):
        self.pi.set_servo_pulsewidth(self.motors_pin, 0)
        self.pi.stop()
        self.set_angle(0)

def main():
    rclpy.init()
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.release_motors()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()