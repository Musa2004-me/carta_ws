#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


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


class WheelTeleop(Node):
    def __init__(self):
        super().__init__('wheel_teleop')

        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        self.current_keys = set()
        self.linear_vel = 0.2
        self.angular_vel = 0.5

        # Start keyboard listener thread
        self.kb_thread = KeyboardThread(self.key_callback)
        self.kb_thread.start()

        self.timer = self.create_timer(0.1, self.update)

    def key_callback(self, key):
        if key == '\x03':  # Ctrl+C
            self.kb_thread.keep_running = False
            rclpy.shutdown()
            return
        elif key in ['b', 'v']:
            self.current_keys.add(key)
        elif key == ' ':
            self.current_keys.clear()
        elif key == '+':
            self.linear_vel += 0.05
            self.angular_vel += 0.05
            self.get_logger().info(f"Velocity increased: lin={self.linear_vel:.2f}, ang={self.angular_vel:.2f}")
        elif key == '-':
            self.linear_vel = max(0.05, self.linear_vel - 0.05)
            self.angular_vel = max(0.05, self.angular_vel - 0.05)
            self.get_logger().info(f"Velocity decreased: lin={self.linear_vel:.2f}, ang={self.angular_vel:.2f}")

    def update(self):
        msg = Twist()
        if 'b' in self.current_keys and 'v' in self.current_keys:
            msg.linear.x = self.linear_vel
        elif 'b' in self.current_keys:
            msg.angular.z = -self.angular_vel
        elif 'v' in self.current_keys:
            msg.angular.z = self.angular_vel

        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing cmd_vel: lin.x={msg.linear.x:.2f}, ang.z={msg.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelTeleop()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
