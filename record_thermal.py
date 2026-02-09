#!/usr/bin/env python3
import socket
import struct
import time
import requests
import os
import re

# --- CONFIGURATION ---
TARGET_IP = "192.168.144.25"
TARGET_PORT = 37260
WEB_PORT = 82
RECORD_DURATION = 10  # Seconds

# THIS IS THE PATH YOU CONFIRMED WORKS
VIDEO_URL_PATH = f"http://{TARGET_IP}:{WEB_PORT}/photo/100SIYI_VID/"

# ===================================================================
# CRC16 Implementation
# ===================================================================
crc16_tab = [0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0]
def crc16(data):
    crc = 0
    for b in data:
        temp = (crc >> 8) & 0xFF
        crc = ((crc << 8) & 0xFFFF) ^ crc16_tab[(b ^ temp) & 0xFF]
    return crc & 0xFFFF

# ===================================================================
# Protocol Helpers
# ===================================================================
seq = 0
def send_cmd(sock, cmd_id, payload=b'', wait_ack=True):
    global seq
    seq = (seq + 1) & 0xFFFF
    frame = bytearray()
    frame += b'\x55\x66' + b'\x01'
    frame += struct.pack("<H", len(payload))
    frame += struct.pack("<H", seq) + bytes([cmd_id]) + payload
    frame += struct.pack("<H", crc16(frame))

    for attempt in range(3):
        sock.sendto(frame, (TARGET_IP, TARGET_PORT))
        if not wait_ack: return True
        try:
            data, _ = sock.recvfrom(256)
            if len(data) >= 8 and data[7] == cmd_id:
                return data
        except socket.timeout:
            continue
    return None

def is_recording(sock):
    data = send_cmd(sock, 0x0A)
    if data and len(data) > 11:
        return data[11] == 1
    return False

# ===================================================================
# Smart File Selection
# ===================================================================
def find_latest_video_url():
    print(f"   > Browsing {VIDEO_URL_PATH}...")
    try:
        r = requests.get(VIDEO_URL_PATH, timeout=5)
        if r.status_code != 200:
            print(f"   > Error: HTTP {r.status_code}")
            return None
        
        # Regex to find .mp4 files
        links = re.findall(r'href="([^"]+)"', r.text)
        mp4s = [f for f in links if f.lower().endswith('.mp4')]
        
        if not mp4s:
            print("   > No MP4 files found.")
            return None
            
        # --- NEW LOGIC: Only look at the last 3 files ---
        candidates = mp4s[-3:] if len(mp4s) > 3 else mp4s
        print(f"   > Candidates (last {len(candidates)}): {candidates}")
        
        largest_size = -1
        best_file_url = None
        
        for vid in candidates:
            full_url = VIDEO_URL_PATH + vid
            try:
                head_resp = requests.head(full_url, timeout=2)
                size = int(head_resp.headers.get('Content-Length', 0))
                
                status = "VALID" if size > 1024*1024 else "INVALID"
                print(f"     - {vid}: {size / 1024 / 1024:.2f} MB [{status}]")
                
                # UPDATE: Use >= to ensure we pick the LATEST file if sizes are equal
                if size >= largest_size:
                    largest_size = size
                    best_file_url = full_url
            except:
                pass
        
        if largest_size < 1024 * 1024:
            print("   > Warning: All recent files seem too small.")
            
        return best_file_url

    except Exception as e:
        print(f"   > Scraping error: {e}")
        return None

# ===================================================================
# Main Logic
# ===================================================================
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    print(f"--- SIYI ZT6 Smart Recorder (Select Newest) ---")
    
    print("Checking camera status...")
    if is_recording(sock):
        print(">> Camera is busy. Stopping first...")
        send_cmd(sock, 0x0C, b'\x02') 
        time.sleep(3) 
    else:
        print(">> Camera is IDLE.")

    print("Setting Split Screen Mode (RGB + Thermal)...")
    send_cmd(sock, 0x11, b'\x00')
    time.sleep(2)

    print("Moving Gimbal to Pitch -45°...")
    move_payload = struct.pack('<hh', 0, -450) 
    send_cmd(sock, 0x0E, move_payload)
    time.sleep(3)

    print("Starting Recording...")
    send_cmd(sock, 0x0C, b'\x02') 
    
    print(f">> Recording started! Waiting {RECORD_DURATION} seconds...")
    time.sleep(RECORD_DURATION)

    print("Stopping Recording...")
    send_cmd(sock, 0x0C, b'\x02') 
    
    print(">> Waiting 5 seconds for file to finalize...")
    time.sleep(5) 

    print("Finding video file...")
    
    # Auto-increment filename
    i = 1
    while os.path.exists(f"thermal{i}.mp4"): i += 1
    local_name = f"thermal{i}.mp4"

    remote_url = find_latest_video_url()

    if remote_url:
        print(f"Downloading {remote_url} -> {local_name}")
        try:
            with requests.get(remote_url, stream=True) as r:
                r.raise_for_status()
                with open(local_name, 'wb') as f:
                    for chunk in r.iter_content(chunk_size=8192):
                        f.write(chunk)
            print(f"\nSUCCESS: Video saved as {local_name}")
        except Exception as e:
            print(f"XX Download error: {e}")
    else:
        print("XX Could not find a valid video URL.")

    sock.close()

if __name__ == "__main__":
    main()