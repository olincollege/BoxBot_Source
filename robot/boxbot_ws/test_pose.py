import pyrealsense2 as rs
import numpy as np
import cv2
import math
import sys
import os

# --- CRITICAL: Add the directory containing your AprilTagTracker class 
# --- to the Python path so it can be imported directly.
# Assuming AprilTagTracker is in a file named 'vision_utils.py' 
# inside a package folder like 'boxbot_main' in your workspace's 'src' directory.
try:
    # Adjust this path based on where 'vision_utils.py' actually lives
    # For a typical ROS setup: ~/boxbot_ws/src/boxbot_main/boxbot_main
    sys.path.append(os.path.abspath(os.path.join(os.getcwd(), 'src/boxbot_main/boxbot_main')))
    from vision_utils import AprilTagTracker
except ImportError as e:
    print("------------------------------------------------------------------")
    print(f"ERROR: Could not import AprilTagTracker. Please check the path.")
    print(f"Error details: {e}")
    print("You might need to adjust the sys.path.append() line.")
    print("------------------------------------------------------------------")
    sys.exit(1)


# --- CONFIGURATION ---
TARGET_TAG_ID = 32 # Use the ID of the tag you are testing
TAG_SIZE = 0.045   # 7.5 cm (in meters)

class PoseTestRunner:
    def __init__(self, tag_id):
        self.target_tag_id = tag_id
        
        # Instantiate the tracker class directly. 
        # This will run the Realsense and detector setup.
        # Note: We can pass any default parameters here, the rest are from the class definition.
        self.tracker = AprilTagTracker(target_dist=0.2, k_yaw=0.5, k_x=0.3, k_z=0.8)

        # Wait for auto-exposure to settle
        print("Waiting for RealSense pipeline to settle...")
        for i in range(30):
            self.tracker.pipeline.wait_for_frames()
        print("Ready. Place tag in view.")

    def run_test(self):
        """Continuously calls the tracking function and prints data."""
        print("\n--- Starting Test ---")
        print(f"Assumed Tag Size: {self.tracker.tag_size} meters")
        print(f"Target Tag ID: {self.target_tag_id}")
        
        # Print header
        print("-" * 60)
        print("Distance (m) | Linear X (cmd) | Angular Z (cmd) | Status")
        print("-" * 60)

        while True:
            # 1. Call the method directly from the class instance
            cmd, distance_m = self.tracker.get_tracking_data(self.target_tag_id)
            
            # --- REMOVED: cv2.imshow and related calls ---
            # --- Only print the results ---

            if distance_m > 0:
                print(f"{distance_m:12.4f} | {cmd.linear.x:14.4f} | {cmd.angular.z:15.4f} | Found", end='\r')
            else:
                print(" " * 60, end='\r') # Clear the line if tag is not found

            # Add a small delay so the terminal isn't overwhelmed
            # You'll need to add time.sleep() if you don't already have it
            import time
            time.sleep(0.1) # Wait 100ms
            
            # Since we removed cv2.waitKey(1), you need a different way to exit
            # We will rely on Ctrl+C to exit, which is less graceful but functional.


    def stop(self):
        self.tracker.stop()
        # cv2.destroyAllWindows() <-- REMOVED
        print("\nTest stopped.")


if __name__ == '__main__':
    tester = None
    try:
        # Before running, ensure you fix the display errors from the previous session 
        # (e.g., run this on a desktop environment or use an X-forwarding setup)
        tester = PoseTestRunner(TARGET_TAG_ID)
        tester.run_test()

    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
    finally:
        if tester:
            tester.stop()