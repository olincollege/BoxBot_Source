import pyrealsense2 as rs
import numpy as np
import cv2
import math
from pupil_apriltags import Detector
from geometry_msgs.msg import Twist

class AprilTagTracker:
    def __init__(self, target_dist=0.175, k_yaw=0.5, k_x=0.3, k_z=0.8):
        """
        Initializes the Camera and Detector, adjusted for 90-degree CCW rotation.
        """
        self.target_dist = target_dist
        self.k_yaw = k_yaw
        self.k_x = k_x
        self.k_z = k_z
        
        # --- CONFIGURATION ---
        self.tag_family = 'tag25h9'
        self.tag_size = 0.045 # Set the physically measured size in meters
        
        # --- REALSENSE SETUP ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Assume original streams are 640x480
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) 
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start Pipeline
        profile = self.pipeline.start(config)
        
        # Get Original Intrinsics (640x480 frame)
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        orig_fx = intrinsics.fx
        orig_fy = intrinsics.fy
        orig_ppx = intrinsics.ppx
        orig_ppy = intrinsics.ppy

        # The new image size is 480 (width) x 640 (height).
        # New fx = Old fy
        # New fy = Old fx
        # New ppx = Old PPY
        # New ppy = Old width (640) - Old PPX
        
        new_fx = orig_fy
        new_fy = orig_fx
        new_ppx = orig_ppy
        new_ppy = 640 - orig_ppx # Width of the original frame is 640

        # Create the new Camera Matrix (K)
        self.K = np.array([
            [new_fx, 0.0, new_ppx],
            [0.0, new_fy, new_ppy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        
        # --- DETECTOR SETUP (Now uses the swapped Intrinsics) ---
        self.detector = Detector(
            families=self.tag_family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        # Other variables needed for consistency (though not used in final pose)
        self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
        half = self.tag_size / 2
        self.object_points = np.array([
            [-half, -half, 0], [ half, -half, 0],
            [ half,  half, 0], [-half,  half, 0]
        ], dtype=np.float32)


    def get_tracking_data(self, target_tag_id):
        """
        Processes image frames, computes the Twist command for the target tag, 
        using the 90-degree CCW image and pose adjustment.
        """
        cmd = Twist()
        distance_m = -1.0 

        frames = self.pipeline.poll_for_frames()
        if not frames:
            return cmd, distance_m

        color_frame = frames.get_color_frame()
        if not color_frame:
            return cmd, distance_m

        # Process Image
        color_image = np.asanyarray(color_frame.get_data())

        rotated_image = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        gray_image = cv2.cvtColor(rotated_image, cv2.COLOR_BGR2GRAY)
        
        # Detector computes pose_t and pose_R based on the rotated image
        detections = self.detector.detect(
            gray_image,
            estimate_tag_pose=True,
            tag_size=self.tag_size,
            camera_params=[self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]]
        )

        target_det = None
        for det in detections:
            if det.tag_id == target_tag_id:
                target_det = det
                break
        
        if target_det:
            if target_det.pose_t is not None and target_det.pose_R is not None:
                
                tvec = target_det.pose_t
                R = target_det.pose_R
                
                # Pose estimation result based on the rotated image:
                # tvec[0][0] (X): Vertical offset in the new frame.
                # tvec[1][0] (Y): Lateral offset (left/right error) in the new frame.
                # tvec[2][0] (Z): Depth (Distance).
                
                # Z-distance is correct
                z = tvec[2][0] 
                distance_m = abs(z)

                # Lateral Error (X_error): Use tvec[1][0] (the new horizontal axis)
                lateral_error = tvec[1][0]
                
                # Orientation (Yaw) Extraction: Remains complex due to rotation, 
                # but we will rely on the translational error (lateral_error) 
                # as the dominant steering factor for robustness.
                
                # Control Logic
                
                # Forward/Backward control uses Z-distance
                cmd.linear.x = self.k_z * (distance_m - self.target_dist)

                cmd.angular.z = self.k_x * lateral_error

        return cmd, distance_m

    def stop(self):
        """Call this in your node's cleanup/destroy method"""
        self.pipeline.stop()