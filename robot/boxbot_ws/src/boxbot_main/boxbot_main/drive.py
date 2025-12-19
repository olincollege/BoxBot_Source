#!/usr/bin/env python3
from std_msgs.msg import String
import serial
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from boxbot_main.motor_utils import StepperL298N4Pin, ContinousServo

class Drive(Node):
    def __init__(self, port='/dev/ttyS0', baudrate=9600):
        super().__init__('drive')

        # --- Subscriptions ---
        self.sub_twist = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_squeeze = self.create_subscription(String, '/squeeze', self.squeeze_callback, 10)
        self.sub_lift = self.create_subscription(String, '/lift', self.lift_callback, 10)

        # --- DC Motor Driver Setup ---
        # Initialize serial port
        try:
            self.ser = serial.Serial(port, baudrate)
            self.get_logger().info(f"Connected to Sabertooth on {port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # --- Stepper Motor Driver Setup ---
        self.stepper = StepperL298N4Pin(pin1=17, pin2=23, pin3=27, pin4=22)
        self.stepper_command = None
        self.stepper_lock = threading.Lock()

        # --- Start Stepper Thread ---
        self.stepper_thread = threading.Thread(target=self._stepper_loop)
        self.stepper_thread.daemon = True
        self.stepper_thread.start()

        # --- Servo Setup ---        
        self.lift_servos = ContinousServo(pin1=16, pin2=21)
        
        self.get_logger().info("Drive node initialized")

    def cmd_vel_callback(self, msg: Twist):
        """Send commands to DC motor driver. Called everytime a new message is published to cmd_vel."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive
        left_speed = linear - angular
        right_speed = linear + angular

        # Convert to motor bytes
        motor1_byte = self.speed_to_motor1(left_speed)
        motor2_byte = self.speed_to_motor2(right_speed)

        # Send each byte separately to S1
        try:
            self.ser.write(bytes([motor1_byte]))
            self.ser.write(bytes([motor2_byte]))
            self.get_logger().debug(f"Sent Motor1={motor1_byte}, Motor2={motor2_byte}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    @staticmethod
    def speed_to_motor1(speed):
        """Generate a serial message for motor 1 to send DC motor driver"""
        # Motor 1: 1-127
        speed = max(min(speed, 1.0), -1.0)
        return int((speed + 1) * 63.5) or 1  # 1–127

    @staticmethod
    def speed_to_motor2(speed):
        """Generate a serial message for motor 2 to send DC motor driver"""
        # Motor 2: 128-255
        speed = max(min(speed, 1.0), -1.0)
        return int((speed + 1) * 63.5) + 128  # 128–255
    
    def squeeze_callback(self, msg: String):
        """Update stepper command. Called everytime a new command is published to squeeze"""
        with self.stepper_lock:
            self.stepper_command = msg.data.lower()

    def _stepper_loop(self):
        while rclpy.ok():
            with self.stepper_lock:
                cmd = self.stepper_command

            if cmd in ["open", "close"]:
                # Active movement needs fast timing
                if cmd == "open": self.stepper.step("forward")
                else: self.stepper.step("backward")
                time.sleep(0.002) 
            else:
                # IDLE: Stop hogging the CPU!
                self.stepper.hold()
                time.sleep(0.05) # 50ms sleep instead of 1ms

    def lift_callback(self, msg: String):
        """Control two continuous rotation servo motors, one of which is reversed, at max speed."""
        
        if not self.lift_servos:
            self.get_logger().warn("Lift command ignored: Servo objects not initialized (hardware error).")
            return

        command = msg.data
        log_msg = ""
        
        if command == "Lift":
            self.lift_servos.lift()
            log_msg = f"Move up"
            
        elif command == "Lower":
            self.lift_servos.lower()
            log_msg = f"Move down"
            
        else: # Handles 'Stop' or any other command
            self.lift_servos.stop()
            log_msg = "Stop"
            
        self.get_logger().info(f"Lift command received: {log_msg}")


    def destroy_node(self):
        """Destroy node upon completion or interruption"""
        self.get_logger().info("SDestroying drive node")
        try:
            self.ser.write(bytes([0]))  # stop motors
            self.ser.close()
        except:
            pass
        self.stepper.cleanup()
        self.lift_servos.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    
    # Reverting to the standard single-threaded behavior
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down drive node")
    finally:
        # Ensure cleanup happens
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
