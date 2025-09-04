# RPI Controller - Vision Processing and Decision Making for WRO Future Engineers
# Sends commands to ESP32 via serial communication

import os
import cv2
import numpy as np
import serial
import time
import threading
import json
from collections import deque

# Set OpenCV thread count for better Pi performance
cv2.setNumThreads(2)

# ================== CONFIGURATION ==================
# Speed constants
SPEED_K = 0.9
PARKING_SPEED = int(60 * SPEED_K)
TURNING_SPEED = int(80 * SPEED_K)
SPEED = int(100 * SPEED_K)
SPEED_NO_AURA_FARM = 90

# Debugging
DEBUG = False

# Serial communication
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
COMMAND_TIMEOUT = 2.0  # seconds before ESP32 switches to autonomous

# Logic constants
KP = 1.0
COUNTER = 0
COUNTER_MAX = 13
target_angle = 0
BLOCK_MULITPLIER_GREEN_ANTI = 1.5

# Failsafe timings
BACK_AFTER_TURN_TIME = 0.8
BACK_BEFORE_TURN_TIME = 1.5
COOLDOWN = 3

# Multi-frame reverse confirmation
AFTER_BLOCK_PID_MS = 500
AFTER_BLOCK_PID_SCALE = 0.6
REVERSE_CONFIRM_N = 5
REVERSE_CONFIRM_M = 3

# State tracking
last_turn = time.time()
last_cooldown = time.time()
last_block_pass = time.time()
DIRECTION = None
InnerStuckFailsafeFlag = False
OuterStuckFailsafeFlag = False

# Camera settings
BRIGHTNESS = 0
CONTRAST = 3.0
GAMMA = 0.7
inv_gamma = 1.0 / GAMMA
table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")

# ================== COLOR DETECTION ==================
# Default color ranges (will be overridden by calibration file if available)
COLOR_RANGES = {
    "red":    ([0, 184, 108], [255, 255, 255]),
    "green":  ([0, 0, 0],     [255, 101, 255]),
    "blue":   ([0, 153, 0],   [255, 186, 90]),
    "orange": ([0, 146, 144], [255, 255, 255])
}

MIN_AREA = {
    "red": 500,
    "green": 500,
    "blue": 300,
    "orange": 300
}

def load_camera_calibration(filename="camera_calibration.json"):
    """Load camera calibration from JSON file"""
    global COLOR_RANGES, MIN_AREA, BRIGHTNESS, CONTRAST, GAMMA
    
    try:
        with open(filename, 'r') as f:
            calibration_data = json.load(f)
        
        if "color_ranges" in calibration_data:
            # Convert calibration format to COLOR_RANGES format
            for color, values in calibration_data["color_ranges"].items():
                if color in COLOR_RANGES:
                    COLOR_RANGES[color] = (values["lower"], values["upper"])
                    MIN_AREA[color] = values["min_area"]
        
        if "image_processing" in calibration_data:
            img_proc = calibration_data["image_processing"]
            BRIGHTNESS = img_proc.get("brightness", BRIGHTNESS)
            CONTRAST = img_proc.get("contrast", CONTRAST) 
            GAMMA = img_proc.get("gamma", GAMMA)
            
            # Update gamma table
            global table
            inv_gamma = 1.0 / GAMMA
            table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")
        
        print(f"Camera calibration loaded from {filename}")
        return True
        
    except FileNotFoundError:
        print(f"Calibration file {filename} not found, using defaults")
        return False
    except Exception as e:
        print(f"Error loading calibration: {e}")
        return False

# Load calibration on import
load_camera_calibration()

colors = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "orange": (255, 165, 0)
}

# Turning matrices
red_turning_values = [0, 0, 0, 0, 0, 0, 0, 5, 10, 15, 0, 10, 15, 25, 30, 5, 10, 15, 25, 35, 0, 0, -1, -1, -1]
red_turning_values_4th_turn_anti = [-5, 0, 0, 0, 0, -5, 0, 5, 10, 15, -5, 10, 15, 25, 30, -5, 10, 15, 25, 35, -5, 0, -1, -1, -1]
red_turning_values_less_than_3000 = [0, 0, 0, 0, 0, 0, 0, 5, 20, 25, -10, 10, 15, 25, 30, -10, 10, 15, 25, 35, 0, 0, -1, -1, -1]
green_turning_values = [0, 0, 0, 0, 0, -15, -10, -5, 0, 0, -30, -25, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]
green_turning_values_4th_turn_clock = [0, 0, 0, 0, 0, -15, -10, -5, 0, 0, -30, -25, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]
green_turning_values_less_than_3000 = [0, 0, 0, 0, 0, -25, -20, -5, 0, 0, -35, -30, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]

# ================== CLASSES ==================
class ThreadedVideoCapture:
    """Drop-in replacement for cv2.VideoCapture with threading for lower latency"""
    def __init__(self, device="/dev/video0"):
        w = int(os.getenv("CAM_W", "1280"))
        h = int(os.getenv("CAM_H", "720"))
        fps = int(os.getenv("CAM_FPS", "30"))

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("Error: Cannot open camera")
            raise SystemExit(1)

        # Use MJPG for better Pi performance
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

        # Warmup
        t0 = time.time()
        while self.frame is None and (time.time() - t0) < 2.0:
            time.sleep(0.01)

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                continue
            with self.lock:
                self.frame = f

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        try:
            self.t.join(timeout=0.5)
        except Exception:
            pass
        self.cap.release()

class ReverseTrigger:
    """Multi-frame confirmation for reverse decisions"""
    def __init__(self, n=REVERSE_CONFIRM_N, m=REVERSE_CONFIRM_M):
        self.n = n
        self.m = m
        self.buf = deque(maxlen=n)

    def push(self, cond: bool):
        self.buf.append(1 if cond else 0)

    def should_reverse(self):
        return len(self.buf) == self.n and sum(self.buf) >= self.m

    def clear(self):
        self.buf.clear()

class SerialCommunicator:
    """Handles structured communication with ESP32"""
    def __init__(self, port, baudrate, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.last_command_time = time.time()
        time.sleep(2)  # Let ESP32 initialize
        
    def send_command(self, cmd_type, *args):
        """Send structured command to ESP32"""
        try:
            if cmd_type == "MOVE":
                speed, direction, steer = args
                cmd = f"CMD:MOVE,{speed},{direction},{steer}\n"
            elif cmd_type == "STOP":
                cmd = "CMD:STOP\n"
            elif cmd_type == "AUTONOMOUS":
                cmd = "CMD:AUTONOMOUS\n"
            else:
                print(f"Unknown command type: {cmd_type}")
                return False
                
            self.ser.write(cmd.encode())
            self.last_command_time = time.time()
            return True
        except Exception as e:
            print(f"Serial send error: {e}")
            return False
    
    def read_sensor_data(self):
        """Read sensor data from ESP32"""
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return None
            
            parts = line.split(",")
            if len(parts) != 4:  # angle,left,right,front
                return None
                
            angle = normalize_angle(int(parts[0]))
            left = int(parts[1])
            right = int(parts[2])
            front = int(parts[3])
            
            return {
                'angle': angle,
                'left': left,
                'right': right,
                'front': front,
                'timestamp': time.time()
            }
        except Exception as e:
            if DEBUG:
                print(f"Serial read error: {e}")
            return None
    
    def flush(self):
        """Flush serial buffers"""
        while self.ser.in_waiting:
            self.ser.readline()
    
    def wait_for_ok(self):
        """Wait for ESP32 handshake"""
        while True:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                print("ESP32:", line)
                if line.upper() == "OK":
                    print("Sending back OK...")
                    self.ser.write(b"OK\n")
                    break
    
    def close(self):
        """Close serial connection"""
        if hasattr(self, 'ser'):
            self.ser.close()

# ================== HELPER FUNCTIONS ==================
def normalize_angle(angle):
    """Normalize angle to [-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle

def angle_diff(target, current):
    """Calculate shortest angular difference"""
    diff = target - current
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

def dampen_after_block(error):
    """Reduce PID aggressiveness after block detection"""
    if time.time() - last_block_pass < (AFTER_BLOCK_PID_MS / 1000.0):
        return int(error * AFTER_BLOCK_PID_SCALE)
    return error

def adjust_frame(frame):
    """Apply brightness, contrast, and gamma correction"""
    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)
    adjusted = cv2.LUT(adjusted, table)
    return adjusted

def detect_parking_marker(frame):
    """Detect ArUco parking markers"""
    try:
        aruco = cv2.aruco
        dict_ = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        params = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(frame, dict_, parameters=params)
        if ids is not None and 0 in ids.flatten():
            return True
    except Exception:
        pass
    return False

# ================== VISION PROCESSING ==================
def detect_biggest_block(frame, show=False):
    """Detect and analyze blocks in the frame"""
    global last_turn, DIRECTION, last_cooldown
    
    full_h, full_w, _ = frame.shape
    roi_start = int(full_h * 0.2)
    frame = frame[roi_start:full_h, 0:full_w]

    # Image preprocessing
    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)
    adjusted = cv2.LUT(adjusted, table)

    lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_clahe = clahe.apply(l)
    lab_clahe = cv2.merge((l_clahe, a, b))
    adjusted_frame = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

    lab_frame = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2LAB)
    lab_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    detections = {"red": [], "green": [], "blue": [], "orange": []}
    blue_y_values, orange_y_values = [], []

    # Color detection
    for color, (lower, upper) in COLOR_RANGES.items():
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        source_lab = lab_raw if color == "orange" else lab_frame
        mask = cv2.inRange(source_lab, lower_bound, upper_bound)
        
        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA.get(color, 0):
                continue
                
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            detections[color].append((cx, cy, area, color))
            
            if color == "blue":
                blue_y_values.append(cy)
            elif color == "orange":
                orange_y_values.append(cy)
                
            if show:
                cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), colors[color], 2)
                cv2.circle(frame, (cx, cy), 4, colors[color], -1)
                cv2.putText(frame, f"{color} ({area})", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[color], 2)

    max_blue_y = max(blue_y_values) if blue_y_values else None
    max_orange_y = max(orange_y_values) if orange_y_values else None

    # Filter valid blocks
    valid_blocks = []
    for color in ["red", "green"]:
        for (cx, cy, area, col) in detections[color]:
            if time.time() - last_cooldown > COOLDOWN:
                if max_blue_y and cy <= max_blue_y:
                    continue
                if max_orange_y and cy <= max_orange_y:
                    continue

            # Cell position calculation
            cy_full = cy + roi_start
            cell_w = full_w // 5
            cell_h = full_h // 5
            col_idx = cx // cell_w
            row_idx = cy_full // cell_h
            cell_num = row_idx * 5 + col_idx + 1

            # CODENAME PENDING FAILSAFE
            if DIRECTION == "anticlockwise" and cell_num in [1, 6, 11, 16, 21]:
                if time.time() - last_turn <= 1:
                    if DEBUG:
                        print("CODENAMING PENDING FAILSAFE")
                    continue

            valid_blocks.append((col, cy, cx, area))

    # Find closest block
    closest_block = None
    if valid_blocks:
        closest_block = max(valid_blocks, key=lambda b: b[1])
        color, cy, cx, area = closest_block
        cy_full = cy + roi_start
        x_percent = (cx / full_w) * 100
        y_percent = ((full_h - cy_full) / full_h) * 100
        cell_w = full_w // 5
        cell_h = full_h // 5
        col_idx = cx // cell_w
        row_idx = cy_full // cell_h
        cell_number = row_idx * 5 + col_idx + 1
    else:
        color, cy, cx, area, cell_number, x_percent, y_percent = (None, None, None, None, None, None, None)

    # Special case detection
    special_case = 0
    if DIRECTION == "anticlockwise" and detections["green"]:
        cell_w = full_w // 5
        cell_h = full_h // 5
        green_in_right = False
        green_in_left_front = False
        
        for (cx_g, cy_g, area_g, _) in detections["green"]:
            cy_full_g = cy_g + roi_start
            col_idx = cx_g // cell_w
            row_idx = cy_full_g // cell_h
            cell_num = row_idx * 5 + col_idx + 1
            
            if cell_num in [5, 10, 15, 20, 25] and area_g > 30000:
                green_in_right = True
            if cell_num % 5 in [1, 2, 3, 4]:
                if ((max_blue_y and cy_g > max_blue_y) or (max_orange_y and cy_g > max_orange_y)):
                    if area_g > 7000:
                        green_in_left_front = True
        
        if green_in_right and green_in_left_front:
            special_case = 1
            
    elif DIRECTION == "clockwise" and detections["red"]:
        cell_w = full_w // 5
        cell_h = full_h // 5
        red_in_left = False
        red_in_right_front = False
        
        for (cx_r, cy_r, area_r, _) in detections["red"]:
            cy_full_r = cy_r + roi_start
            col_idx = cx_r // cell_w
            row_idx = cy_full_r // cell_h
            cell_num = row_idx * 5 + col_idx + 1
            
            if cell_num in [1, 6, 11, 16, 21] and area_r > 20000:
                red_in_left = True
            if cell_num % 5 in [4, 0]:
                if ((max_blue_y and cy_r > max_blue_y) or (max_orange_y and cy_r > max_orange_y)):
                    if area_r > 7000:
                        red_in_right_front = True
        
        if red_in_left and red_in_right_front:
            special_case = 1

    if show and DEBUG:
        # Draw grid and visualization
        cell_w = full_w // 5
        cell_h = full_h // 5
        display = frame.copy()
        
        for i in range(1, 5):
            cv2.line(display, (i * cell_w, 0), (i * cell_w, frame.shape[0]), (200, 200, 200), 1)
        for j in range(1, 5):
            y_line = j * cell_h - roi_start
            if 0 <= y_line < frame.shape[0]:
                cv2.line(display, (0, y_line), (frame.shape[1], y_line), (200, 200, 200), 1)
        
        num = 1
        for r in range(5):
            for c in range(5):
                y_pos = r * cell_h + 20 - roi_start
                if 0 <= y_pos < frame.shape[0]:
                    x_pos = c * cell_w + 10
                    cv2.putText(display, str(num), (x_pos, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                num += 1
        
        if closest_block:
            cv2.rectangle(display, (cx - 20, cy - 20), (cx + 20, cy + 20), (0, 255, 255), 3)
            cv2.putText(display, f"Closest: {color} Cell:{cell_number}",
                       (cx, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("Block Detection", display)
        cv2.waitKey(1)

    return (color if closest_block else None,
            y_percent if closest_block else None,
            x_percent if closest_block else None,
            area if closest_block else None,
            cell_number if closest_block else None,
            special_case)

# ================== MAIN LOGIC ==================
class RPiController:
    """Main controller class for RPI-based robot control"""
    
    def __init__(self):
        self.comm = SerialCommunicator(PORT, BAUDRATE)
        self.cap = ThreadedVideoCapture("/dev/video0")
        self.reverse_trigger = ReverseTrigger()
        self.running = True
        
    def wait_for_sensor_data(self, timeout=2.0):
        """Wait for valid sensor data from ESP32"""
        start_time = time.time()
        attempts = 0
        while time.time() - start_time < timeout:
            data = self.comm.read_sensor_data()
            if data and all(key in data for key in ['angle', 'left', 'right', 'front']):
                if data['front'] is not None and data['left'] is not None and data['right'] is not None:
                    return data
            attempts += 1
            if attempts > 20:  # Too many failed attempts
                print("Warning: Multiple failed sensor data attempts")
                break
            time.sleep(0.01)
        print(f"Timeout waiting for sensor data after {timeout}s")
        return None
    
    def exit_parking_lot(self):
        """Determine direction and exit parking area"""
        global DIRECTION
        
        print("Exiting parking lot...")
        data = self.wait_for_sensor_data()
        if not data:
            print("Failed to get sensor data for parking exit")
            return False
            
        if data['left'] > data['right']:
            DIRECTION = "anticlockwise"
        else:
            DIRECTION = "clockwise"
        
        print(f"Direction determined: {DIRECTION}")
        
        # Execute exit sequence based on direction
        if DIRECTION == "clockwise":
            self.steer_until_angle(0, 10, PARKING_SPEED, 0, 50)
            self.steer_until_angle(10, 25, PARKING_SPEED, 1, -50)
            self.steer_until_angle(25, 45, PARKING_SPEED, 0, 50)
        elif DIRECTION == "anticlockwise":
            self.steer_until_angle(0, -10, PARKING_SPEED, 0, -50)
            self.steer_until_angle(-10, -25, PARKING_SPEED, 1, 50)
            self.steer_until_angle(-25, -45, PARKING_SPEED, 0, -50)
            
        return True
    
    def steer_until_angle(self, current_angle, target_angle, speed, direction, steer):
        """Steer until reaching target angle"""
        sign = ">" if target_angle > current_angle else "<"
        
        data = self.wait_for_sensor_data()
        if not data:
            return False
            
        angle = data['angle']
        
        if sign == "<":
            while angle > target_angle:
                self.comm.send_command("MOVE", speed, direction, steer)
                time.sleep(0.05)
                data = self.wait_for_sensor_data()
                if not data:
                    break
                angle = data['angle']
        elif sign == ">":
            while angle < target_angle:
                self.comm.send_command("MOVE", speed, direction, steer)
                time.sleep(0.05)
                data = self.wait_for_sensor_data()
                if not data:
                    break
                angle = data['angle']
        
        self.comm.send_command("STOP")
        return True
    
    def first_block_sequence(self):
        """Handle first block detection and navigation"""
        global DIRECTION
        
        print("Starting first block sequence...")
        detected_blocks = []
        
        for attempt in range(20):
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            # Simplified block detection for first sequence
            masks = {
                "red": (np.array([0, 167, 120]), np.array([255, 255, 255])),
                "green": (np.array([0, 0, 0]), np.array([255, 87, 255])),
                "blue": (np.array([0, 137, 0]), np.array([255, 177, 92]))
            }
            
            detected_blocks = self.process_frame_simple(frame, masks, DIRECTION)
            
            # Remove blue blocks and check for red/green
            detected_blocks = [block for block in detected_blocks if block[0] != "blue"]
            if detected_blocks:
                break
        
        if detected_blocks:
            block_color = detected_blocks[0][0]
            print(f"First block detected: {block_color}")
            
            if block_color == "red" and DIRECTION == "anticlockwise":
                self.steer_until_angle(-45, -30, PARKING_SPEED, 0, 10)
                self.steer_until_angle(-30, 0, PARKING_SPEED, 0, 20)
            elif block_color == "green" and DIRECTION == "anticlockwise":
                self.steer_until_angle(-45, -90, PARKING_SPEED, 0, -35)
                self.drive_until_distance(25, PARKING_SPEED)
                self.steer_until_angle(-90, 0, PARKING_SPEED, 0, 50)
            elif block_color == "green" and DIRECTION == "clockwise":
                self.steer_until_angle(45, 30, PARKING_SPEED, 0, -20)
                self.steer_until_angle(30, 0, PARKING_SPEED, 0, -30)
            elif block_color == "red" and DIRECTION == "clockwise":
                self.steer_until_angle(45, 70, PARKING_SPEED, 0, 35)
                self.drive_until_distance(25, PARKING_SPEED)
                self.steer_until_angle(90, 0, PARKING_SPEED, 0, -50)
        else:
            # Default behavior if no blocks detected
            if DIRECTION == "anticlockwise":
                self.steer_until_angle(-45, -30, PARKING_SPEED, 0, 10)
                self.steer_until_angle(-30, 0, PARKING_SPEED, 0, 20)
            elif DIRECTION == "clockwise":
                self.steer_until_angle(45, 30, PARKING_SPEED, 0, -20)
                self.steer_until_angle(30, 0, PARKING_SPEED, 0, -30)
    
    def process_frame_simple(self, frame, masks, direction):
        """Simplified frame processing for first block sequence"""
        results = []
        frame = adjust_frame(frame)
        
        height = frame.shape[0]
        third_line = height // 3 if direction == "clockwise" else height // 2
        
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        
        for name, (lower, upper) in masks.items():
            mask = cv2.inRange(lab, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    if y + h > third_line:
                        results.append((name, area))
        
        results.sort(key=lambda t: t[1], reverse=True)
        return results
    
    def drive_until_distance(self, target_distance, speed):
        """Drive forward until reaching target distance"""
        while True:
            data = self.wait_for_sensor_data()
            if not data:
                break
            if data['front'] <= target_distance:
                break
            self.comm.send_command("MOVE", speed, 0, 0)
            time.sleep(0.1)
        self.comm.send_command("STOP")
    
    def main_logic(self):
        """Main control loop with full state machine"""
        global DIRECTION, COUNTER, target_angle, last_turn, last_block_pass, last_cooldown
        global InnerStuckFailsafeFlag, OuterStuckFailsafeFlag
        
        print(f"Starting main logic - Direction: {DIRECTION}")
        
        if DIRECTION == "anticlockwise":
            self.main_logic_anticlockwise()
        elif DIRECTION == "clockwise":
            self.main_logic_clockwise()
    
    def main_logic_anticlockwise(self):
        """Main logic for anticlockwise direction"""
        global COUNTER, target_angle, last_turn, last_block_pass, last_cooldown
        global InnerStuckFailsafeFlag, OuterStuckFailsafeFlag
        
        while COUNTER < COUNTER_MAX and self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # Get block detection
            block = detect_biggest_block(frame, show=DEBUG)
            
            # Get sensor data
            sensor_data = self.comm.read_sensor_data()
            if not sensor_data:
                continue
                
            angle = sensor_data['angle']
            left = sensor_data['left']
            right = sensor_data['right']
            front = sensor_data['front']
            
            if DEBUG:
                print(f"Sensor: {sensor_data}, Block: {block}")
            
            # Wrong direction failsafe
            if (angle_diff(target_angle, angle) > 45 and time.time() - last_turn < 5) or (angle_diff(target_angle, angle) > 70):
                print("GOING WRONG DIRECTION FAILSAFE")
                self.comm.send_command("MOVE", SPEED, 0, 40)
                time.sleep(0.5)
                self.comm.send_command("STOP")
                continue
            
            # Speed adjustment based on recent activity
            current_speed = SPEED if (not block[0] and time.time() - last_turn > 5 and time.time() - last_block_pass > 2) else SPEED_NO_AURA_FARM
            
            # Multi-frame reverse confirmation
            reverse_cond = (block[1] and block[1] < 20 and 40 < block[2] < 60)
            self.reverse_trigger.push(reverse_cond)
            
            if self.reverse_trigger.should_reverse():
                print("REVERSE TRIGGERED")
                self.comm.send_command("MOVE", 80, 1, 0)
                time.sleep(1)
                self.comm.send_command("STOP")
                self.comm.flush()
                self.reverse_trigger.clear()
                # Refresh camera frames
                for _ in range(3):
                    ret, frame = self.cap.read()
                continue
            
            # Inner/Outer stuck failsafes
            if left < 5 and right > 60:
                if not InnerStuckFailsafeFlag:
                    InnerStuckFailsafeFlag = True
                    last_inner_stuck = time.time()
                elif time.time() - last_inner_stuck > 5:
                    print("INNER STUCK FAILSAFE")
                    self.comm.send_command("MOVE", 80, 1, -10)
                    time.sleep(1)
                    self.comm.send_command("MOVE", 80, 0, 10)
                    time.sleep(1)
                    self.comm.send_command("STOP")
                    InnerStuckFailsafeFlag = False
                    continue
            
            if right < 5 and left > 60:
                if not OuterStuckFailsafeFlag:
                    OuterStuckFailsafeFlag = True
                    last_outer_stuck = time.time()
                elif time.time() - last_outer_stuck > 5:
                    print("OUTER STUCK FAILSAFE")
                    self.comm.send_command("MOVE", 80, 1, 10)
                    time.sleep(1)
                    self.comm.send_command("MOVE", 80, 0, -10)
                    time.sleep(1)
                    self.comm.send_command("STOP")
                    OuterStuckFailsafeFlag = False
                    continue
            
            # Block avoidance logic
            if block[0] and block[3] > 4000:
                color, y, x, area, cell, special_case = block
                
                if special_case:
                    print("DOUBLE TROUBLE FAILSAFE ACTIVATED")
                    self.comm.send_command("MOVE", 80, 0, 0)
                    time.sleep(2)
                    self.comm.send_command("STOP")
                    self.comm.flush()
                    continue
                
                if color in ["red", "green"]:
                    last_block_pass = time.time()
                
                if color == "red":
                    if red_turning_values[cell-1] != -1:
                        steer_val = red_turning_values[cell-1]
                        if COUNTER % 4 == 0:
                            steer_val = red_turning_values_4th_turn_anti[cell-1]
                        elif area < 3000:
                            steer_val = red_turning_values_less_than_3000[cell-1]
                        self.comm.send_command("MOVE", current_speed, 0, int(steer_val))
                        continue
                    else:
                        self.comm.send_command("MOVE", 80, 1, 0)
                        time.sleep(1)
                        self.comm.send_command("STOP")
                        continue
                        
                elif color == "green":
                    if green_turning_values[cell-1] != -1:
                        steer_val = int(green_turning_values[cell-1] * BLOCK_MULITPLIER_GREEN_ANTI)
                        self.comm.send_command("MOVE", current_speed, 0, steer_val)
                        continue
                    else:
                        self.comm.send_command("STOP")
                        time.sleep(0.1)
                        if abs(angle_diff(target_angle, angle)) < 5 and left > 100:
                            self.comm.send_command("MOVE", 80, 0, 0)
                            time.sleep(1)
                        self.comm.send_command("MOVE", 80, 1, 0)
                        time.sleep(1)
                        self.comm.send_command("STOP")
                        continue
            
            # Parking check
            if (COUNTER == COUNTER_MAX - 1 and front < 15 and not block[0] and 
                time.time() - last_turn > 7.5 and left > 100 and abs(angle_diff(0, angle)) < 15):
                print("PARKING SEQUENCE")
                self.parking()
                return
            
            # Turn logic
            if (front < 15 and block[0] is None and 
                (left > 100 or ((COUNTER + 1) % 4 == 0 and left > 50)) and 
                COUNTER != COUNTER_MAX - 1):
                
                # False turn failsafe
                if abs(angle_diff(normalize_angle(target_angle + 5), angle)) > 20:
                    self.comm.send_command("MOVE", 80, 1, 0)
                    time.sleep(1)
                    self.comm.send_command("STOP")
                    continue
                
                # Execute turn
                target_angle = normalize_angle(target_angle - 90)
                COUNTER += 1
                print(f"TURN {COUNTER}, TARGET={target_angle}")
                
                if right >= 25:
                    while True:
                        self.comm.send_command("MOVE", TURNING_SPEED, 1, 40)
                        data = self.wait_for_sensor_data()
                        if not data:
                            break
                        
                        err = angle_diff(normalize_angle(target_angle + 10), data['angle'])
                        if err >= -10:
                            self.comm.send_command("MOVE", 80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            self.comm.send_command("STOP")
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                else:
                    self.comm.send_command("MOVE", 80, 1, 0)
                    time.sleep(BACK_BEFORE_TURN_TIME)
                    self.comm.send_command("STOP")
                    
                    while True:
                        self.comm.send_command("MOVE", TURNING_SPEED, 0, -40)
                        data = self.wait_for_sensor_data()
                        if not data:
                            break
                        
                        err = angle_diff(normalize_angle(target_angle + 10), data['angle'])
                        if err >= -10:
                            self.comm.send_command("MOVE", 80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            self.comm.send_command("STOP")
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                
                self.comm.flush()
                continue
            
            # PID control
            if COUNTER % 4 == 0:
                error = angle_diff(normalize_angle(target_angle - 4), angle)
            else:
                error = angle_diff(target_angle, angle)
            
            error = dampen_after_block(error)
            self.comm.send_command("MOVE", current_speed, 0, error)
    
    def main_logic_clockwise(self):
        """Main logic for clockwise direction"""
        global COUNTER, target_angle, last_turn, last_block_pass, last_cooldown
        global InnerStuckFailsafeFlag, OuterStuckFailsafeFlag
        
        while COUNTER < COUNTER_MAX and self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # Get block detection
            block = detect_biggest_block(frame, show=DEBUG)
            
            # Get sensor data
            sensor_data = self.comm.read_sensor_data()
            if not sensor_data:
                continue
                
            angle = sensor_data['angle']
            left = sensor_data['left']
            right = sensor_data['right']
            front = sensor_data['front']
            
            if DEBUG:
                print(f"Sensor: {sensor_data}, Block: {block}")
            
            # Wrong direction failsafe
            if (angle_diff(target_angle, angle) > 45 and time.time() - last_turn < 5) or (angle_diff(target_angle, angle) > 70):
                print("GOING WRONG DIRECTION FAILSAFE")
                self.comm.send_command("MOVE", SPEED, 0, -40)  # Opposite direction from anticlockwise
                time.sleep(0.5)
                self.comm.send_command("STOP")
                continue
            
            # Speed adjustment
            current_speed = SPEED if (not block[0] and time.time() - last_turn > 5 and time.time() - last_block_pass > 2) else SPEED_NO_AURA_FARM
            
            # Multi-frame reverse confirmation
            reverse_cond = (block[1] and block[1] < 20 and 40 < block[2] < 60)
            self.reverse_trigger.push(reverse_cond)
            
            if self.reverse_trigger.should_reverse():
                print("REVERSE TRIGGERED")
                self.comm.send_command("MOVE", 80, 1, 0)
                time.sleep(1)
                self.comm.send_command("STOP")
                self.comm.flush()
                self.reverse_trigger.clear()
                continue
            
            # Inner/Outer stuck failsafes (swapped for clockwise)
            if right < 5 and left > 60:  # Inner wall for clockwise
                if not InnerStuckFailsafeFlag:
                    InnerStuckFailsafeFlag = True
                    last_inner_stuck = time.time()
                elif time.time() - last_inner_stuck > 5:
                    print("INNER STUCK FAILSAFE")
                    self.comm.send_command("MOVE", 80, 1, 10)   # Opposite steer
                    time.sleep(1)
                    self.comm.send_command("MOVE", 80, 0, -10)
                    time.sleep(1)
                    self.comm.send_command("STOP")
                    InnerStuckFailsafeFlag = False
                    continue
            
            if left < 5 and right > 60:  # Outer wall for clockwise
                if not OuterStuckFailsafeFlag:
                    OuterStuckFailsafeFlag = True
                    last_outer_stuck = time.time()
                elif time.time() - last_outer_stuck > 5:
                    print("OUTER STUCK FAILSAFE")
                    self.comm.send_command("MOVE", 80, 1, -10)  # Opposite steer
                    time.sleep(1)
                    self.comm.send_command("MOVE", 80, 0, 10)
                    time.sleep(1)
                    self.comm.send_command("STOP")
                    OuterStuckFailsafeFlag = False
                    continue
            
            # Block avoidance logic
            if block[0] and block[3] > 4000:
                color, y, x, area, cell, special_case = block
                
                if special_case:
                    print("DOUBLE TROUBLE FAILSAFE ACTIVATED")
                    self.comm.send_command("MOVE", 80, 0, 0)
                    time.sleep(2)
                    self.comm.send_command("STOP")
                    continue
                
                if color in ["red", "green"]:
                    last_block_pass = time.time()
                
                if color == "red":
                    if red_turning_values[cell-1] != -1:
                        steer_val = red_turning_values[cell-1]
                        if area < 3000:
                            steer_val = red_turning_values_less_than_3000[cell-1]
                        self.comm.send_command("MOVE", current_speed, 0, int(steer_val))
                        continue
                    else:
                        self.comm.send_command("MOVE", 80, 1, 0)
                        time.sleep(1)
                        self.comm.send_command("STOP")
                        continue
                        
                elif color == "green":
                    if green_turning_values[cell-1] != -1:
                        steer_val = green_turning_values[cell-1]
                        # Use clockwise specific values if available
                        if hasattr(self, 'green_turning_values_4th_turn_clock'):
                            steer_val = green_turning_values_4th_turn_clock[cell-1]
                        self.comm.send_command("MOVE", current_speed, 0, int(steer_val))
                        continue
                    else:
                        self.comm.send_command("MOVE", 80, 1, 0)
                        time.sleep(1)
                        self.comm.send_command("STOP")
                        continue
            
            # Parking check
            if (COUNTER == COUNTER_MAX - 1 and front < 15 and not block[0] and 
                time.time() - last_turn > 7.5 and right > 100 and abs(angle_diff(0, angle)) < 15):  # Check right wall instead of left
                print("PARKING SEQUENCE")
                self.parking()
                return
            
            # Turn logic (clockwise - opposite direction)
            if (front < 15 and block[0] is None and 
                (right > 100 or ((COUNTER + 1) % 4 == 0 and right > 50)) and  # Check right wall
                COUNTER != COUNTER_MAX - 1):
                
                # False turn failsafe
                if abs(angle_diff(normalize_angle(target_angle - 5), angle)) > 20:  # Opposite direction
                    self.comm.send_command("MOVE", 80, 1, 0)
                    time.sleep(1)
                    self.comm.send_command("STOP")
                    continue
                
                # Execute turn (clockwise = +90)
                target_angle = normalize_angle(target_angle + 90)
                COUNTER += 1
                print(f"TURN {COUNTER}, TARGET={target_angle}")
                
                if left >= 25:  # Check left wall for clearance
                    while True:
                        self.comm.send_command("MOVE", TURNING_SPEED, 1, -40)  # Turn right (negative steer)
                        data = self.wait_for_sensor_data()
                        if not data:
                            break
                        
                        err = angle_diff(normalize_angle(target_angle - 10), data['angle'])  # Opposite offset
                        if err <= 10:
                            self.comm.send_command("MOVE", 80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            self.comm.send_command("STOP")
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                else:
                    self.comm.send_command("MOVE", 80, 1, 0)
                    time.sleep(BACK_BEFORE_TURN_TIME)
                    self.comm.send_command("STOP")
                    
                    while True:
                        self.comm.send_command("MOVE", TURNING_SPEED, 0, 40)  # Turn right (positive steer)
                        data = self.wait_for_sensor_data()
                        if not data:
                            break
                        
                        err = angle_diff(normalize_angle(target_angle - 10), data['angle'])
                        if err <= 10:
                            self.comm.send_command("MOVE", 80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            self.comm.send_command("STOP")
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                
                self.comm.flush()
                continue
            
            # PID control
            error = angle_diff(target_angle, angle)
            error = dampen_after_block(error)
            self.comm.send_command("MOVE", current_speed, 0, error)
    
    def parking(self):
        """Execute parking sequence"""
        global DIRECTION
        
        print("Starting parking sequence")
        self.comm.send_command("STOP")
        
        if DIRECTION == "anticlockwise":
            data = self.wait_for_sensor_data()
            if not data:
                return
                
            front = data['front']
            left = data['left']
            
            # Position adjustment
            if front < 25:
                self.comm.send_command("MOVE", 75, 1, 0)
                time.sleep(1.5)
                self.comm.send_command("STOP")
            elif front > 70:
                self.comm.send_command("MOVE", 75, 0, 0)
                time.sleep(1)
                self.comm.send_command("STOP")
            
            print("PARKING POSITIONED; STARTING TURN")
            time.sleep(1)
            
            # Wait for left wall clearance
            self.comm.send_command("MOVE", 50, 1, 0)
            while True:
                data = self.wait_for_sensor_data()
                if not data:
                    break
                if data['left'] <= 80 or (data['front'] and data['front'] < 80):
                    break
                self.comm.flush()
            
            self.comm.send_command("STOP")
            
            # Execute parking turn
            self.steer_until_angle(0, -80, 75, 1, 55)
            
            # Drive into parking space
            self.comm.send_command("MOVE", 60, 1, 0)
            time.sleep(3)
            self.comm.send_command("STOP")
            
        print("PARKING COMPLETE")
    
    def run(self):
        """Main execution function"""
        try:
            print("RPI Controller Initialized")
            self.comm.wait_for_ok()
            print("Starting main sequence...")
            
            self.comm.flush()
            
            if not self.exit_parking_lot():
                print("Failed to exit parking lot")
                return
            
            self.first_block_sequence()
            self.main_logic()
            self.parking()
            
        except KeyboardInterrupt:
            print("\nController stopped by user")
        except Exception as e:
            print(f"Controller error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        self.running = False
        
        if hasattr(self, 'comm'):
            self.comm.send_command("STOP")
            self.comm.close()
        
        if hasattr(self, 'cap'):
            self.cap.release()
        
        cv2.destroyAllWindows()

# ================== MAIN ==================
if __name__ == "__main__":
    controller = RPiController()
    controller.run()
