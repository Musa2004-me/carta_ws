#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class KeyboardThread(threading.Thread):
    def __init__(self, callback):
        super().__init__()
        self.callback = callback
        self.daemon = True
        self.keep_running = True

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.keep_running:
                key = sys.stdin.read(1)
                self.callback(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class WheelKeyControl(Node):
    def __init__(self):
        super().__init__('wheel_key_control')

        # Publishers
        self.left_pub = self.create_publisher(Float64, '/left_wheel_controller/commands', 10)
        self.right_pub = self.create_publisher(Float64, '/right_wheel_controller/commands', 10)

        # Wheel speeds
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.step = 0.1  # increment per key press

        # 🔑 KEY MAPPING (edit this section to change controls)
        self.keymap = {
            'j': ('left', +1),   # increase left
            'n': ('left', -1),   # decrease left
            'k': ('right', +1),  # increase right
            'm': ('right', -1),  # decrease right
        }

        # Start keyboard thread
        self.kb_thread = KeyboardThread(self.key_callback)
        self.kb_thread.start()

        # Timer to publish
        self.timer = self.create_timer(0.1, self.publish_cmds)

    def key_callback(self, key):
        if key == '\x03':  # Ctrl+C
            self.kb_thread.keep_running = False
            rclpy.shutdown()
            return
        elif key == ' ':
            self.left_speed = 0.0
            self.right_speed = 0.0
            self.get_logger().info("STOP both wheels")
        elif key == '+':
            self.step += 0.05
            self.get_logger().info(f"Step increased to {self.step:.2f}")
        elif key == '-':
            self.step = max(0.01, self.step - 0.05)
            self.get_logger().info(f"Step decreased to {self.step:.2f}")
        elif key in self.keymap:
            wheel, direction = self.keymap[key]
            if wheel == 'left':
                self.left_speed += direction * self.step
            elif wheel == 'right':
                self.right_speed += direction * self.step
            self.get_logger().info(
                f"{wheel.capitalize()} wheel -> {self.left_speed:.2f}, {self.right_speed:.2f}"
            )

    def publish_cmds(self):
        # Publish wheel speeds
        lmsg = Float64()
        rmsg = Float64()
        lmsg.data = self.left_speed
        rmsg.data = self.right_speed
        self.left_pub.publish(lmsg)
        self.right_pub.publish(rmsg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelKeyControl()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
