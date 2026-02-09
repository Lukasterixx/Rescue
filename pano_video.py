import cv2
import numpy as np
import os
import sys
from tqdm import tqdm

# ================= CONFIGURATION =================
# Directory containing the data
DATA_DIR = "./photos/1/"
VIDEO_FILENAME = "video.mp4"

# Output filename
OUTPUT_NAME = "solar_panorama_video.jpg"

# Stabilization: 0.0 = No smoothing, 0.95 = Very heavy smoothing
# 0.85 is a good balance to remove camera shake (heave)
SMOOTHING_FACTOR = 0.85

# Minimum speed to record data (prevents recording when robot is still)
MIN_SPEED_THRESHOLD = 0.5
# =================================================

def create_pushbroom():
    # Construct full path
    video_path = os.path.join(DATA_DIR, VIDEO_FILENAME)
    
    if not os.path.exists(video_path):
        print(f"Error: File not found at {video_path}")
        print("Please ensure 'video.mp4' is inside the folder.")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Could not open video file.")
        return

    # Read first frame to initialize
    ret, prev_frame = cap.read()
    if not ret: 
        print("Video appears to be empty.")
        return
    
    h, w = prev_frame.shape[:2]
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    
    # Storage for the vertical strips
    strips = []
    
    # Variable to track vertical 'heave' (up/down drift)
    accumulated_dy = 0.0
    
    # Get total frame count for the progress bar
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Processing {total_frames} frames from {video_path}...")
    
    # Create a mask to focus Optical Flow on the center of the image
    # (The edges of the lens often have distortion, center is best)
    feature_mask = np.zeros_like(prev_gray)
    # Focus on the middle 40% of the screen
    feature_mask[int(h*0.3):int(h*0.7), int(w*0.3):int(w*0.7)] = 255

    for _ in tqdm(range(total_frames - 1)):
        ret, curr_frame = cap.read()
        if not ret: break
        
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
        
        # 1. Optical Flow: Calculate how many pixels the world moved
        # We use Farneback Dense Optical Flow
        flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 
                                            0.5, 3, 15, 3, 5, 1.2, 0)
        
        # apply mask to ignore edge noise
        valid_flow = flow[int(h*0.3):int(h*0.7), int(w*0.3):int(w*0.7)]
        
        # Get the median movement (robust to outliers)
        dx = np.median(valid_flow[..., 0]) # Horizontal movement
        dy = np.median(valid_flow[..., 1]) # Vertical movement (heave)
        
        # 2. Software Stabilization (Heave Compensation)
        # Smooth out the vertical jitter using an exponential moving average
        accumulated_dy = (accumulated_dy * SMOOTHING_FACTOR) + (dy * (1.0 - SMOOTHING_FACTOR))
        shift_y = int(accumulated_dy)
        
        # Shift the current frame vertically to cancel out the jitter
        M_stab = np.float32([[1, 0, 0], [0, 1, -shift_y]])
        stabilized_frame = cv2.warpAffine(curr_frame, M_stab, (w, h))
        
        # 3. Dynamic Slit Extraction
        # If the robot moves 10 pixels, we take a 10-pixel strip.
        # If it moves 2 pixels, we take a 2-pixel strip.
        speed = abs(dx)
        
        if speed > MIN_SPEED_THRESHOLD:
            # We extract from the dead center of the frame (least lens distortion)
            center_x = w // 2
            
            # The width of the strip matches the speed
            strip_width = max(1, int(round(speed)))
            
            # Ensure we don't grab outside image bounds
            start_col = center_x
            end_col = min(w, center_x + strip_width)
            
            if end_col > start_col:
                strip = stabilized_frame[:, start_col:end_col]
                strips.append(strip)
            
        prev_gray = curr_gray

    cap.release()

    if not strips:
        print("No movement detected (or video was static).")
        return

    print("Stitching strips together...")
    # Concatenate all strips horizontally
    result = np.hstack(strips)
    
    # Save result
    save_path = os.path.join(DATA_DIR, OUTPUT_NAME)
    cv2.imwrite(save_path, result)
    print(f"Success! Panorama saved to: {save_path}")
    print(f"Final Width: {result.shape[1]} pixels")

if __name__ == "__main__":
    create_pushbroom()