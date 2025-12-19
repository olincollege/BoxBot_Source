#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class KeyboardTeleopNode(Node):
    def __init__(self):
        ### ROS2 Node Setup
        super().__init__('keyboard_teleop')
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_squeeze = self.create_publisher(String, 'squeeze', 10)
        self.pub_lift = self.create_publisher(String, 'lift', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)

        # Start robot in teleop state
        msg = String()
        msg.data = "teleop"
        self.pub_status.publish(msg)

        self.get_logger().info("""
Keyboard teleop started. Controls:
WASD --> Drive robot
Space bar --> Stop drive
1 --> Hold squeeze
2 --> Squeeze open
3 --> Squeeze close
8 --> Raise scissor lift
9 --> Lower scissor lift
0 --> Hold scissor lift
q --> Switch status to teleop
e --> Switch status to autonomous""")
        self.run()

    def get_key(self):
        """Capture a single keypress without blocking"""
        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()

            if key == '\x03': # Ctrl-C
                self.get_logger().info(f"Exiting...")
                break
            elif key == 'w':
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -0.2
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = -0.1
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = 0.1
            elif key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == '1':
                msg = String()
                msg.data = "Hold"
                self.pub_squeeze.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == '2':
                msg = String()
                msg.data = "Close"
                self.pub_squeeze.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == '3':
                msg = String()
                msg.data = "Open"
                self.pub_squeeze.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == '8':
                msg = String()
                msg.data = "Lift"
                self.pub_lift.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == '9':
                msg = String()
                msg.data = "Lower"
                self.pub_lift.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == '0':
                msg = String()
                msg.data = "Stop"
                self.pub_lift.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == 'q':
                msg = String()
                msg.data = "teleop"
                self.pub_status.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            elif key == 'e':
                msg = String()
                msg.data = "a_approach_pick_up"
                self.pub_status.publish(msg)
                self.get_logger().info(f"Publishing command: {msg.data}")
                continue
            else:
                continue

            self.pub_twist.publish(twist)
            self.get_logger().info(f"Publishing twist: {twist}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
