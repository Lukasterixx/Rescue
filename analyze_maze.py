import cv2
import numpy as np
import yaml
import math
import os
import random

def get_random_color():
    h = random.randint(0, 179)
    s = random.randint(150, 255)
    v = random.randint(150, 255)
    color = np.uint8([[[h, s, v]]])
    bgr = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)[0][0]
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))

def generate_dynamic_topological_map(yaml_path='my_map.yaml'):
    if not os.path.exists(yaml_path):
        print(f"Error: {yaml_path} not found.")
        return

    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    image_path = map_data.get('image', 'my_map.pgm')
    resolution = map_data.get('resolution', 0.05)
    
    if not os.path.exists(image_path):
        return
        
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None: return

    h, w = img.shape
    start_cx, start_cy = w // 2, h // 2

    # Extract walls and find dominant angle
    walls = np.where(img < 100, 255, 0).astype(np.uint8)
    lines = cv2.HoughLinesP(walls, 1, np.pi/180, threshold=20, minLineLength=10, maxLineGap=5)
    
    angles, lengths = [], []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 90 
            angles.append(angle)
            lengths.append(math.hypot(x2 - x1, y2 - y1))

    dominant_angle = 0.0
    if angles:
        hist, bins = np.histogram(angles, bins=90, range=(0, 90), weights=lengths)
        dominant_angle = bins[np.argmax(hist)]

    # Rotate the map ONCE
    M = cv2.getRotationMatrix2D((start_cx, start_cy), dominant_angle, 1.0)
    rotated_img = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_NEAREST, borderValue=205)

    obstacle_thresh = 100
    stop_ratio = 0.35 
    
    # Noise filter: gap must be at least 0.5m to be considered a real doorway, not an artifact.
    min_door_width_px = int(0.5 / resolution) 
    max_room_dimension_px = int(25.0 / resolution) 
    brush_thickness = 3
    search_radius = int(0.25 / resolution)

    queue = [(start_cx, start_cy)]
    discovered_rooms = [] 
    all_doorways = []     

    # DYNAMIC VARIABLE: Tracks the smallest legitimate door found so far
    global_min_door_len_px = float('inf')

    def is_point_in_mapped_room(px, py):
        for (l, t, r, b, color) in discovered_rooms:
            if l <= px <= r and t <= py <= b: return True
        return False

    def hits_room_x(x, y1, y2):
        for (l, t, r, b, color) in discovered_rooms:
            if l <= x <= r and max(y1, t) <= min(y2, b): return True
        return False

    def hits_room_y(y, x1, x2):
        for (l, t, r, b, color) in discovered_rooms:
            if t <= y <= b and max(x1, l) <= min(x2, r): return True
        return False

    def extract_doorways_brush(region_2d, axis):
        if region_2d.size == 0: return []
        
        # FIXED: Using np.min instead of np.mean.
        # If any pixel across the brush depth is a wall, it will pull the min value 
        # below the obstacle_thresh and correctly invalidate the doorway.
        line_array = np.min(region_2d, axis=axis)
        
        is_free = line_array > obstacle_thresh
        diff = np.diff(is_free.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends = np.where(diff == -1)[0]
        
        if is_free[0]: starts = np.insert(starts, 0, 0)
        if is_free[-1]: ends = np.append(ends, len(line_array) - 1)
            
        segments = []
        for s, e in zip(starts, ends):
            if e - s >= min_door_width_px:
                segments.append((s, e))
        return segments

    print("Starting Dynamic Colorized Topological mapping...")
    
    while queue:
        cx, cy = queue.pop(0)

        if not (0 <= cx < w and 0 <= cy < h): continue
        if is_point_in_mapped_room(cx, cy): continue

        if rotated_img[cy, cx] < 200:
            found_free = False
            for dy in range(-search_radius, search_radius + 1):
                for dx in range(-search_radius, search_radius + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h and rotated_img[ny, nx] >= 200:
                        if not is_point_in_mapped_room(nx, ny):
                            cx, cy = nx, ny
                            found_free = True
                            break
                if found_free: break
            if not found_free: continue 

        box_size = 2 
        left, right = max(0, cx - box_size), min(w - 1, cx + box_size)
        top, bottom = max(0, cy - box_size), min(h - 1, cy + box_size)
        
        expand_left = expand_right = expand_top = expand_bottom = True

        while expand_left or expand_right or expand_top or expand_bottom:
            if expand_left and left > 0:
                if hits_room_x(left - 1, top, bottom): expand_left = False
                elif np.sum(rotated_img[top:bottom+1, left-1] < obstacle_thresh) / (bottom - top + 1) > stop_ratio: expand_left = False
                else: left -= 1
            else: expand_left = False

            if expand_right and right < w - 1:
                if hits_room_x(right + 1, top, bottom): expand_right = False
                elif np.sum(rotated_img[top:bottom+1, right+1] < obstacle_thresh) / (bottom - top + 1) > stop_ratio: expand_right = False
                else: right += 1
            else: expand_right = False

            if expand_top and top > 0:
                if hits_room_y(top - 1, left, right): expand_top = False
                elif np.sum(rotated_img[top-1, left:right+1] < obstacle_thresh) / (right - left + 1) > stop_ratio: expand_top = False
                else: top -= 1
            else: expand_top = False

            if expand_bottom and bottom < h - 1:
                if hits_room_y(bottom + 1, left, right): expand_bottom = False
                elif np.sum(rotated_img[bottom+1, left:right+1] < obstacle_thresh) / (right - left + 1) > stop_ratio: expand_bottom = False
                else: bottom += 1
            else: expand_bottom = False

        if (right - left) > max_room_dimension_px or (bottom - top) > max_room_dimension_px: continue
        if left == 0 or right == w - 1 or top == 0 or bottom == h - 1: continue

        room_color = get_random_color()
        discovered_rooms.append((left, top, right, bottom, room_color))

        # --- TWO-PASS DOORWAY PROCESSING ---
        current_room_doors = []
        
        brush_outside = 3
        brush_inside = 2

        # Pass 1: Collect all doorways using the 4-pixel offset brush
        
        # Left Wall Brush: 3 outside, 1 inside -> [left-3 : left+1]
        l_start = max(0, left - brush_outside)
        l_end = min(w, left + brush_inside)
        if l_end > l_start:
            for s, e in extract_doorways_brush(rotated_img[top:bottom+1, l_start:l_end], axis=1):
                current_room_doors.append(('left', s, e, left, top + s, left, top + e))

        # Right Wall Brush: 1 inside, 3 outside -> [right : right+4]
        r_start = max(0, right - brush_inside + 1)
        r_end = min(w, right + 1 + brush_outside)
        if r_end > r_start:
            for s, e in extract_doorways_brush(rotated_img[top:bottom+1, r_start:r_end], axis=1):
                current_room_doors.append(('right', s, e, right, top + s, right, top + e))

        # Top Wall Brush: 3 outside, 1 inside -> [top-3 : top+1]
        t_start = max(0, top - brush_outside)
        t_end = min(h, top + brush_inside)
        if t_end > t_start:
            for s, e in extract_doorways_brush(rotated_img[t_start:t_end, left:right+1], axis=0):
                current_room_doors.append(('top', s, e, left + s, top, left + e, top))

        # Bottom Wall Brush: 1 inside, 3 outside -> [bottom : bottom+4]
        b_start = max(0, bottom - brush_inside + 1)
        b_end = min(h, bottom + 1 + brush_outside)
        if b_end > b_start:
            for s, e in extract_doorways_brush(rotated_img[b_start:b_end, left:right+1], axis=0):
                current_room_doors.append(('bottom', s, e, left + s, bottom, left + e, bottom))
        # Pass 2: Update the global smallest door length dynamically
        for door in current_room_doors:
            side, s, e, p1x, p1y, p2x, p2y = door
            door_len = e - s
            if door_len < global_min_door_len_px:
                global_min_door_len_px = door_len

        # If it's the very first room and no doors were found, fallback so it doesn't crash
        L = global_min_door_len_px if global_min_door_len_px != float('inf') else min_door_width_px
        
        # Dynamic Jump Distance: Exactly L/2 (half the smallest door length)
        jump_px = max(1, int(L / 2))

        # Pass 3: Drop seeds based dynamically on L
        for door in current_room_doors:
            side, s, e, p1x, p1y, p2x, p2y = door
            all_doorways.append(((p1x, p1y), (p2x, p2y)))
            
            current_door_len = e - s
            
            # Scale seeds based on how many 'standard' doors L fit into this gap
            num_seeds = max(1, int(round(current_door_len / L)))
            step = current_door_len / (num_seeds + 1)

            for i in range(1, num_seeds + 1):
                if side == 'left':
                    queue.append((int(left - jump_px), int(top + s + step * i)))
                elif side == 'right':
                    queue.append((int(right + jump_px), int(top + s + step * i)))
                elif side == 'top':
                    queue.append((int(left + s + step * i), int(top - jump_px)))
                elif side == 'bottom':
                    queue.append((int(left + s + step * i), int(bottom + jump_px)))

    # Inverse Rotation & Visualization
    M_inv = cv2.getRotationMatrix2D((start_cx, start_cy), -dominant_angle, 1.0)
    
    def inv_transform(x, y):
        nx = M_inv[0, 0] * x + M_inv[0, 1] * y + M_inv[0, 2]
        ny = M_inv[1, 0] * x + M_inv[1, 1] * y + M_inv[1, 2]
        return int(nx), int(ny)

    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    overlay = color_img.copy()

    for (l, t, r, b, color) in discovered_rooms:
        corners = np.array([[l, t], [r, t], [r, b], [l, b]], dtype=np.float32)
        corners_homo = np.hstack([corners, np.ones((4, 1))])
        orig_corners = np.int32(M_inv.dot(corners_homo.T).T)
        cv2.fillPoly(overlay, [orig_corners], color) 

    alpha = 0.55
    cv2.addWeighted(overlay, alpha, color_img, 1 - alpha, 0, color_img)

    for pt1, pt2 in all_doorways:
        orig_pt1 = inv_transform(pt1[0], pt1[1])
        orig_pt2 = inv_transform(pt2[0], pt2[1])
        cv2.line(color_img, orig_pt1, orig_pt2, (0, 255, 255), 4)

    cv2.circle(color_img, (start_cx, start_cy), 5, (0, 0, 255), -1) 

    output_filename = 'dynamic_maze_overlay.png'
    cv2.imwrite(output_filename, color_img)
    
    print(f"--- Maze Exploration Complete ---")
    print(f"Smallest Standard Door (L) : {global_min_door_len_px * resolution:.2f} meters")
    print(f"Calculated Jump (L/2)      : {(jump_px * resolution):.2f} meters")
    print(f"Rooms Discovered           : {len(discovered_rooms)}")
    print(f"Total Doorways             : {len(all_doorways)}")
    print(f"Saved to                   : {output_filename}")

if __name__ == '__main__':
    generate_dynamic_topological_map('my_map.yaml')