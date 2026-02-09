import cv2
import numpy as np
import os
import json
import sys
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from ament_index_python.packages import get_package_share_directory

# ================= CONSTANTS =================
PACKAGE_NAME = 'go2_control_cpp'

# Camera Parameters
FOCAL_LENGTH_MM = 13.0
SENSOR_WIDTH_MM = 7.68
IMAGE_WIDTH_PX  = 640
IMAGE_HEIGHT_PX = 512

# TF Frames
FRAME_ODOM   = 'odom'
FRAME_LIVOX  = 'livox_frame'
FRAME_CAMERA = 'camera'

# Stabilization Parameters
MAX_VERTICAL_JUMP_PX = 15.0 
DRIFT_DECAY = 0.98 
# =============================================

def get_ppm(dist_m):
    if dist_m <= 0: return 1.0
    field_of_view_m = dist_m * (SENSOR_WIDTH_MM / FOCAL_LENGTH_MM)
    return IMAGE_WIDTH_PX / field_of_view_m

class TFBuffer:
    def __init__(self): self.transforms = {} 
    def add_msg(self, msg):
        for t in msg.transforms:
            key = f"{t.header.frame_id}->{t.child_frame_id}"
            if key not in self.transforms: self.transforms[key] = []
            ts = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
            trans = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            rot = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
            self.transforms[key].append((ts, trans, rot))

    def lookup(self, parent, child, query_time_ns):
        key = f"{parent}->{child}"
        if key not in self.transforms: return None
        data = self.transforms[key]
        times = [d[0] for d in data]
        idx = np.searchsorted(times, query_time_ns)
        if idx == 0: return data[0][1], data[0][2]
        if idx >= len(data): return data[-1][1], data[-1][2]
        
        t1, p1, q1 = data[idx-1]
        t2, p2, q2 = data[idx]
        dt = t2 - t1
        if dt == 0: return p1, q1
        ratio = (query_time_ns - t1) / dt
        
        pos = p1 + (p2 - p1) * ratio
        q_mix = q1 + (q2 - q1) * ratio
        q_mix = q_mix / np.linalg.norm(q_mix)
        return pos, q_mix

def calculate_constrained_shift(img_curr, img_prev):
    h, w = img_curr.shape
    SEARCH_H = 80   
    SEARCH_W = 200  
    
    cx, cy = w // 2, h // 2
    t_x = cx - (SEARCH_W // 2)
    t_y = cy - (SEARCH_H // 2)
    
    template = img_prev[t_y:t_y+SEARCH_H, t_x:t_x+SEARCH_W]
    
    margin = int(MAX_VERTICAL_JUMP_PX + 5) 
    
    s_y_start = max(0, t_y - margin)
    s_y_end   = min(h, t_y + SEARCH_H + margin)
    s_x_start = t_x
    s_x_end   = t_x + SEARCH_W
    
    search_region = img_curr[s_y_start:s_y_end, s_x_start:s_x_end]
    
    try:
        res = cv2.matchTemplate(search_region, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        
        if max_val < 0.6: return 0.0 
            
        match_y_local = max_loc[1]
        match_y_global = s_y_start + match_y_local
        dy = match_y_global - t_y
        
        if abs(dy) > MAX_VERTICAL_JUMP_PX:
            return 0.0
            
        return dy
    except:
        return 0.0

def process_folder(photos_root, folder_name):
    """
    Runs the stitcher for a single folder (e.g., '1', '0.1', '2')
    """
    row_id = folder_name
    base_dir = os.path.join(photos_root, folder_name)

    print(f"\n{'='*40}")
    print(f"Processing Folder: {folder_name} | Path: {base_dir}")
    print(f"{'='*40}")

    # --- UPDATED NAMING CONVENTION ---
    # Video: video_0.mp4, video_0.1.mp4, etc.
    # JSON: pano0.json, pano0.1.json, etc.
    # Bag: panel_bag_0, panel_bag_0.1, etc.
    video_filename = f"video_{row_id}.mp4"
    json_filename  = f"pano{row_id}.json"
    bag_filename   = f"panel_bag_{row_id}"

    video_path = os.path.join(base_dir, video_filename)
    bag_path   = os.path.join(base_dir, bag_filename)
    json_path  = os.path.join(base_dir, json_filename)

    # 1. Load JSON Metadata
    if not os.path.exists(json_path): 
        print(f"  [SKIP] JSON not found: {json_filename}")
        return
        
    try:
        with open(json_path, 'r') as f: meta = json.load(f)
        bag_end_time_ns = meta["end_timestamp_nanos"]
        initial_dist_m  = meta["initial_lateral_distance"]
        ppm = get_ppm(initial_dist_m)
        print(f"  Meta Loaded. Scale: {ppm:.1f} px/m")
    except Exception as e:
        print(f"  [ERROR] Failed to read JSON: {e}")
        return

    # 2. Load ROS Bag
    tf_buffer = TFBuffer()
    if not os.path.exists(bag_path):
        print(f"  [SKIP] Bag file not found: {bag_filename}")
        return

    print("  Loading Bag data...")
    try:
        with Reader(bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in ['/tf', '/tf_static']:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    tf_buffer.add_msg(msg)
    except Exception as e:
        print(f"  [ERROR] Failed to read Bag: {e}")
        return

    # 3. Load Video
    if not os.path.exists(video_path):
        print(f"  [SKIP] Video file not found: {video_filename}")
        return

    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    if fps <= 0 or total_frames <= 0:
        print("  [ERROR] Invalid video metadata.")
        return

    video_duration_ns = int((total_frames / fps) * 1_000_000_000)
    start_time_ns = bag_end_time_ns - video_duration_ns

    strips = []
    last_pos = None
    prev_gray = None
    current_y_offset = 0.0 

    print(f"  Stitching {total_frames} frames...")
    
    pbar = tqdm(range(total_frames), desc=f"  Folder {folder_name}", unit="frame")
    
    for i in pbar:
        ret, frame = cap.read()
        if not ret: break
        
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        t = start_time_ns + int((i / fps) * 1_000_000_000)
        odom_tf = tf_buffer.lookup(FRAME_ODOM, FRAME_LIVOX, t)
        if odom_tf is None: continue
        
        curr_pos, _ = odom_tf
        
        if last_pos is None:
            last_pos = curr_pos
            prev_gray = curr_gray
            continue
            
        step_dist = np.linalg.norm(curr_pos - last_pos)
        last_pos = curr_pos
        
        slice_width = int(round(step_dist * ppm))
        
        dy = 0.0
        if prev_gray is not None:
            dy = calculate_constrained_shift(curr_gray, prev_gray)
        
        current_y_offset = (current_y_offset + dy) * DRIFT_DECAY
        prev_gray = curr_gray
        
        if slice_width < 1: continue

        M = np.float32([[1, 0, 0], [0, 1, -current_y_offset]])
        frame_stab = cv2.warpAffine(frame, M, (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX))
        
        center_x = IMAGE_WIDTH_PX // 2
        start_x = center_x - (slice_width // 2)
        end_x = start_x + slice_width
        
        if start_x < 0: start_x = 0
        if end_x > IMAGE_WIDTH_PX: end_x = IMAGE_WIDTH_PX
        
        if end_x > start_x:
            strip = frame_stab[:, start_x:end_x]
            strips.append(strip)

    cap.release()

    if strips:
        full_pano = np.hstack(strips)
        # Save output inside the specific folder
        out_path = os.path.join(base_dir, f"final_damped_row_{row_id}.jpg")
        cv2.imwrite(out_path, full_pano)
        print(f"  [SUCCESS] Saved: {out_path}")
    else:
        print("  [WARN] No strips generated.")

def main():
    # 1. Determine Root Directory
    try:
        pkg_path = get_package_share_directory(PACKAGE_NAME)
        photos_root = os.path.join(pkg_path, 'photos')
    except Exception as e:
        print(f"Warning: Could not find package '{PACKAGE_NAME}'. Using local fallback.")
        photos_root = "./photos"

    if not os.path.exists(photos_root):
        print(f"Error: Photos root directory not found at: {photos_root}")
        return

    print(f"Scanning for numeric folders in: {photos_root}")

    # 2. Get all subdirectories
    try:
        all_items = os.listdir(photos_root)
        folders = [item for item in all_items if os.path.isdir(os.path.join(photos_root, item))]
    except Exception as e:
        print(f"Error reading directory: {e}")
        return

    # 3. Filter and Sort folders numerically
    numeric_folders = []
    for f in folders:
        try:
            val = float(f)
            numeric_folders.append((val, f))
        except ValueError:
            continue

    # Sort based on the float value
    numeric_folders.sort(key=lambda x: x[0])

    if not numeric_folders:
        print("No numbered folders found.")
        return

    # 4. Iterate and Process
    print(f"Found {len(numeric_folders)} folders to process: {[f[1] for f in numeric_folders]}")
    
    for _, folder_name in numeric_folders:
        process_folder(photos_root, folder_name)

    print("\nAll folders processed.")

if __name__ == "__main__":
    main()
