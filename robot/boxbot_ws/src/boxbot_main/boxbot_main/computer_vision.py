import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time 

from boxbot_main.vision_utils import AprilTagTracker 

# --- CONFIGURATION ---
PICK_UP_TAG_ID = 32
DROP_OFF_TAG_ID = 0
DISTANCE_PICK_UP = 0.3 # Distance away from pick up tag in meters before squeeze starts
DISTANCE_DROP_OFF = 0.5 # Distance away from drop up tag in meters before open starts

class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')

        # --- PUBS/SUBS ---
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)
        self.pub_squeeze = self.create_publisher(String, 'squeeze', 10)
        self.pub_lift = self.create_publisher(String, 'lift', 10)

        self.sub_status = self.create_subscription(String, '/status', self.status_callback, 10)
        
        self.current_status = "stop" # Default safe state
        self.action_start_time = 0.0 # To track delays without blocking

        # --- VISION ---
        # Initialize the tracker. This handles the camera connection.
        self.tracker = AprilTagTracker(target_dist=0.2)
        
        # --- MAIN LOOP ---
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("Camera node initialized")

    def lift_lower_box(self, lift):
        msg = String()
        msg.data = "Lift" if lift else "Lower"
        self.pub_lift.publish(msg)
        self.get_logger().info(f"Action: {msg.data}")

    def squeeze_open_box(self, squeeze):
        msg = String()
        msg.data = "Squeeze" if squeeze else "Close"
        self.pub_squeeze.publish(msg)
        self.get_logger().info(f"Action: {msg.data}")
        
    def stop_mechanism(self, mechanism):
        msg = String()
        msg.data = "Stop" if mechanism == "lift" else "Hold"
        
        if mechanism == "lift":
            self.pub_lift.publish(msg)
        else:
            self.pub_squeeze.publish(msg)
            
        self.get_logger().info(f"Stopping {mechanism}")

    def timer_callback(self):
        # Check Status
        if self.current_status in [None, "stop", "teleop"]:
            return

        new_status = None
        current_time = time.time()

        # --- STATE MACHINE ---

        # --- Pick up sequence ---
        if self.current_status == "a_approach_pick_up": 
            cmd_vel, distance = self.tracker.get_tracking_data(PICK_UP_TAG_ID)

            if distance < 0:
                # Tag not seen, initiate a search turn
                self.get_logger().info("Tag not found. Searching left...")
                search_twist = Twist()
                search_twist.angular.z = 0.2
                self.pub_cmd_vel.publish(search_twist)
                
            elif distance <= DISTANCE_PICK_UP:
                # Reached transition distance. Stop and move to next state.
                self.pub_cmd_vel.publish(Twist()) # Stop command
                new_status = "a_lower1"
                self.get_logger().info(f"Pick up tag reached, moving state to {new_status}, distance: {distance}")
            else:
                # Tag seen, drive forward using the tracker's command
                self.pub_cmd_vel.publish(cmd_vel)
                self.get_logger().info(f"Moving towards tag, distance (m):{distance}")
      
        elif self.current_status == "a_lower1":
            self.lift_lower_box(lift=False)
            self.action_start_time = current_time
            # Transition to a "waiting" state so we don't spam the command
            self.current_status = "waiting_lower1" 

        elif self.current_status == "waiting_lower1":
            # Check if 5 seconds have passed
            if current_time - self.action_start_time > 5.0:
                self.stop_mechanism("lift")
                new_status = "a_squeeze1"

        elif self.current_status == "a_squeeze1":
            self.squeeze_open_box(squeeze=True)
            self.action_start_time = current_time
            self.current_status = "waiting_squeeze1"

        elif self.current_status == "waiting_squeeze1":
            if current_time - self.action_start_time > 1.0:
                self.stop_mechanism("squeeze")
                new_status = "a_lift1"

        elif self.current_status == "a_lift1":
            self.lift_lower_box(lift=True)
            self.action_start_time = current_time
            self.current_status = "waiting_lift1"

        elif self.current_status == "waiting_lift1":
            if current_time - self.action_start_time > 5.0:
                self.stop_mechanism("lift")
                new_status = "a_approach_drop_off"

        # --- Drop off sequence ---
        elif self.current_status == "a_approach_drop_off": 
            cmd_vel, distance = self.tracker.get_tracking_data(DROP_OFF_TAG_ID)

            if distance < 0:
                # Tag not seen, initiate a search turn
                self.get_logger().info("Tag not found. Searching left...")
                search_twist = Twist()
                search_twist.angular.z = 0.2
                self.pub_cmd_vel.publish(search_twist)
                
            elif distance <= DISTANCE_DROP_OFF:
                # Reached transition distance. Stop and move to next state.
                self.pub_cmd_vel.publish(Twist()) # Stop command
                new_status = "a_lower2"
                self.get_logger().info(f"Drop off tag reached, moving state to {new_status}")
            else:
                # Tag seen, drive forward using the tracker's command
                self.pub_cmd_vel.publish(cmd_vel)

        elif self.current_status == "a_lower2":
            self.lift_lower_box(lift=False)
            self.action_start_time = current_time
            # Transition to a "waiting" state so we don't spam the command
            self.current_status = "waiting_lower2" 

        elif self.current_status == "waiting_lower2":
            # Check if 5 seconds have passed
            if current_time - self.action_start_time > 5.0:
                self.stop_mechanism("lift")
                new_status = "a_open1"

        elif self.current_status == "a_open1":
            self.squeeze_open_box(squeeze=False)
            self.action_start_time = current_time
            self.current_status = "waiting_open1"

        elif self.current_status == "waiting_open1":
            if current_time - self.action_start_time > 1.0:
                self.stop_mechanism("squeeze")
                new_status = "teleop"

        # --- STATE UPDATE ---
        if new_status:
            msg = String()
            msg.data = new_status
            self.current_status = new_status # Update internal immediately
            self.pub_status.publish(msg)     # Inform network
            self.get_logger().info(f"State transitioned to: {new_status}")

    def status_callback(self, msg: String):
        self.current_status = msg.data
        self.get_logger().info(f"External Status Command: {self.current_status}")

    def destroy_node(self):
        self.tracker.stop() # Stops the RealSense pipeline inside the tracker
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()