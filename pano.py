import cv2
import numpy as np
import glob
import os
from tqdm import tqdm

# ================= CONFIGURATION =================
PHOTOS_PATH = "./photos/1/"
OUTPUT_NAME = "solar_panorama_sift.jpg"

# SIFT Parameters
# Solar panels have high contrast, so we can be strict with contrast threshold
SIFT_CONTRAST_THRESH = 0.04
SIFT_EDGE_THRESH = 10

# RANSAC Parameters
MIN_MATCH_COUNT = 10       # Need at least this many good matches to accept a stitch
REPROJ_THRESH = 4.0        # Pixel tolerance for RANSAC (lower = stricter)

# Constraint: The robot moves roughly horizontally.
# We reject matches that suggest vertical jumps larger than this.
MAX_VERTICAL_DRIFT = 100   
# =================================================

def load_images(folder):
    # Load images and sort them alphabetically/numerically
    files = sorted(
        glob.glob(os.path.join(folder, "*.*")),
        key=lambda f: os.path.basename(f)
    )
    
    images = []
    print(f"Loading {len(files)} images...")
    for f in files:
        img = cv2.imread(f)
        if img is not None:
            images.append(img)
    return images

def compute_pairwise_homography(img1, img2):
    """
    Finds the transformation to align img2 onto img1 using SIFT features.
    Returns: (Homography Matrix M, (dx, dy))
    """
    # 1. Detect SIFT features
    sift = cv2.SIFT_create(contrastThreshold=SIFT_CONTRAST_THRESH, edgeThreshold=SIFT_EDGE_THRESH)
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    if des1 is None or des2 is None or len(des1) < MIN_MATCH_COUNT or len(des2) < MIN_MATCH_COUNT:
        print("  ! Not enough features found.")
        return None, (0, 0)

    # 2. Match features (FlannBasedMatcher is faster than BruteForce for SIFT)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    matches = flann.knnMatch(des1, des2, k=2)

    # 3. Ratio Test (Lowe's paper) - Filter out weak matches
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) < MIN_MATCH_COUNT:
        print(f"  ! Not enough good matches ({len(good)}/{MIN_MATCH_COUNT}).")
        return None, (0, 0)

    # 4. Extract coordinates
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    # 5. Compute Homography with RANSAC
    # We map "dst" (img2) to "src" (img1)
    M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, REPROJ_THRESH)
    
    if M is None:
        return None, (0, 0)

    # 6. Sanity Check: Translation
    # Extract translation components from the matrix
    dx = M[0, 2]
    dy = M[1, 2]

    # If the robot thinks it moved backwards (negative dx) or 
    # jumped huge vertically (dy), ignore this match.
    if dx < 0: 
        print(f"  ! Rejected: Detected backward movement (dx={dx:.1f})")
        return None, (0, 0)
    
    if abs(dy) > MAX_VERTICAL_DRIFT:
        print(f"  ! Rejected: Vertical drift too high (dy={dy:.1f})")
        return None, (0, 0)

    return M, (dx, dy)

def stitch_mosaic(images):
    if not images:
        return

    # Initialize canvas with the first image
    h, w, c = images[0].shape
    
    # Estimate massive canvas width (safety buffer)
    est_width = int(w * len(images) * 0.6) + 1000
    est_height = h + 600 # Buffer for vertical drift (heave)
    
    # Create black canvas
    canvas = np.zeros((est_height, est_width, 3), dtype=np.uint8)
    
    # Place first image in the vertical center of the left side
    current_x, current_y = 0, 300
    
    # We maintain a transformation matrix "T_global" that accumulates shifts
    T_global = np.identity(3)
    
    # Paste first image
    canvas[current_y:current_y+h, current_x:current_x+w] = images[0]
    
    print("Stitching with SIFT...")
    
    for i in tqdm(range(1, len(images))):
        img_prev = images[i-1]
        img_curr = images[i]
        
        # Find how img_curr relates to img_prev
        M_local, (dx, dy) = compute_pairwise_homography(img_prev, img_curr)
        
        if M_local is None:
            # Fallback: If SIFT fails, assume a clean 50% shift based on user input
            # (Coverage * 0.5) roughly means shift is Width * 0.5
            dx = w * 0.5
            dy = 0
            print("  (Using fallback blind stitch)")
        
        # Accumulate global position
        # We simplify the homography to just translation to prevent "curving" distortion 
        # over long solar rows. Full warping accumulates perspective errors rapidly.
        current_x += int(dx)
        current_y += int(dy)
        
        # Check Canvas Bounds
        if current_y < 0: current_y = 0
        if current_y + h > canvas.shape[0]:
            print("  ! Canvas too short vertically, cropping bottom.")
            current_y = canvas.shape[0] - h
        if current_x + w > canvas.shape[1]:
            print("  ! Canvas too narrow, cropping right.")
            current_x = canvas.shape[1] - w

        # --- BLENDING ---
        # We blend the overlap region using a linear gradient (alpha blending)
        # to hide the seam line.
        
        # 1. Define the overlap width
        overlap = w - int(dx)
        if overlap > 0:
            # 2. Paste the non-overlapping part (Right side of new image)
            # The part of img_curr that is NEW is from x=overlap to w
            new_part = img_curr[:, overlap:]
            
            start_x_canvas = current_x + overlap
            end_x_canvas = start_x_canvas + new_part.shape[1]
            
            canvas[current_y:current_y+h, start_x_canvas:end_x_canvas] = new_part
            
            # 3. Blend the overlapping part
            # Overlap region on canvas:
            # from (current_x) to (current_x + overlap)
            
            # Slice the existing canvas (which contains the Right of Prev Image)
            roi_existing = canvas[current_y:current_y+h, current_x:current_x+overlap]
            
            # Slice the incoming image (Left of Curr Image)
            roi_new = img_curr[:, :overlap]
            
            # Ensure sizes match (sometimes rounding errors occur)
            min_w = min(roi_existing.shape[1], roi_new.shape[1])
            roi_existing = roi_existing[:, :min_w]
            roi_new = roi_new[:, :min_w]
            
            # Create alpha mask (0.0 to 1.0)
            alpha_mask = np.linspace(0, 1, min_w).reshape(1, min_w, 1)
            alpha_mask = np.tile(alpha_mask, (h, 1, 3))
            
            # Blend: Old * (1-alpha) + New * alpha
            blended = (roi_existing * (1 - alpha_mask)) + (roi_new * alpha_mask)
            
            canvas[current_y:current_y+h, current_x:current_x+min_w] = blended.astype(np.uint8)
            
        else:
            # If shift was huge (no overlap), just paste
            canvas[current_y:current_y+h, current_x:current_x+w] = img_curr

    # Post-processing: Crop black borders
    print("Cropping result...")
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        x, y, w_rect, h_rect = cv2.boundingRect(contours[0])
        final_mosaic = canvas[y:y+h_rect, x:x+w_rect]
        cv2.imwrite(OUTPUT_NAME, final_mosaic)
        print(f"Saved {OUTPUT_NAME}")
    else:
        cv2.imwrite(OUTPUT_NAME, canvas)

if __name__ == "__main__":
    imgs = load_images(PHOTOS_PATH)
    stitch_mosaic(imgs)