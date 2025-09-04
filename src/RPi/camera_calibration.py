#!/usr/bin/env python3
"""
Camera Calibration Tool for WRO Future Engineers Robot
Interactive color threshold adjustment with live preview
Saves calibrated values to JSON for rpi_controller.py
"""

import cv2
import numpy as np
import json
import os
import time
from threading import Thread, Lock

class ColorCalibrator:
    def __init__(self, camera_device=0):
        self.camera_device = camera_device
        self.cap = None
        self.current_frame = None
        self.frame_lock = Lock()
        self.running = True
        
        # Default color ranges (LAB color space)
        self.color_ranges = {
            "red": {
                "lower": [0, 184, 108],
                "upper": [255, 255, 255],
                "min_area": 500
            },
            "green": {
                "lower": [0, 0, 0],
                "upper": [255, 101, 255],
                "min_area": 500
            },
            "blue": {
                "lower": [0, 153, 0],
                "upper": [255, 186, 90],
                "min_area": 300
            },
            "orange": {
                "lower": [0, 146, 144],
                "upper": [255, 255, 255],
                "min_area": 300
            }
        }
        
        self.current_color = "red"
        self.current_channel = "lower"  # "lower" or "upper"
        self.current_component = 0  # 0=L, 1=A, 2=B
        
        # Image processing settings
        self.brightness = 0
        self.contrast = 3.0
        self.gamma = 0.7
        self.use_clahe = True
        
        # Create gamma correction table
        self.update_gamma_table()
        
        # UI state
        self.show_original = False
        self.show_mask = True
        self.show_contours = True
        
    def update_gamma_table(self):
        """Update gamma correction lookup table"""
        inv_gamma = 1.0 / self.gamma
        self.gamma_table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")
    
    def setup_camera(self):
        """Initialize camera with optimal settings"""
        # Try different camera backends
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        
        for backend in backends:
            try:
                self.cap = cv2.VideoCapture(self.camera_device, backend)
                if self.cap.isOpened():
                    break
            except:
                continue
        
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self.camera_device}")
        
        # Set camera properties
        width = int(os.getenv("CAM_W", "1280"))
        height = int(os.getenv("CAM_H", "720"))
        fps = int(os.getenv("CAM_FPS", "30"))
        
        # Try to set MJPG format for better performance
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except:
            pass
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except:
            pass
        
        print(f"Camera initialized: {width}x{height}@{fps}fps")
    
    def camera_thread(self):
        """Capture frames in separate thread"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.current_frame = frame.copy()
            time.sleep(0.01)  # ~100 FPS max
    
    def adjust_frame(self, frame):
        """Apply image adjustments (brightness, contrast, gamma)"""
        # Brightness and contrast
        adjusted = cv2.convertScaleAbs(frame, alpha=self.contrast, beta=self.brightness)
        
        # Gamma correction
        adjusted = cv2.LUT(adjusted, self.gamma_table)
        
        return adjusted
    
    def process_frame_for_color(self, frame, color_name):
        """Process frame for specific color detection"""
        # Apply adjustments
        adjusted = self.adjust_frame(frame)
        
        # Convert to LAB
        lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
        
        # Apply CLAHE if enabled
        if self.use_clahe:
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l_clahe = clahe.apply(l)
            lab = cv2.merge((l_clahe, a, b))
        
        # Create mask
        color_info = self.color_ranges[color_name]
        lower_bound = np.array(color_info["lower"], dtype=np.uint8)
        upper_bound = np.array(color_info["upper"], dtype=np.uint8)
        
        mask = cv2.inRange(lab, lower_bound, upper_bound)
        
        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        return adjusted, lab, mask
    
    def find_contours_and_draw(self, frame, mask, color_name):
        """Find contours and draw them on frame"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Color for drawing
        colors = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "blue": (255, 0, 0),
            "orange": (0, 165, 255)
        }
        color = colors.get(color_name, (255, 255, 255))
        
        valid_contours = 0
        total_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.color_ranges[color_name]["min_area"]:
                valid_contours += 1
                total_area += area
                
                if self.show_contours:
                    # Draw bounding rectangle
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    
                    # Draw area text
                    cv2.putText(frame, f"{int(area)}", (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return frame, valid_contours, total_area
    
    def create_ui(self):
        """Create trackbars for color adjustment"""
        window_name = "Color Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        # Color selection
        colors = list(self.color_ranges.keys())
        cv2.createTrackbar("Color", window_name, 0, len(colors)-1, self.on_color_change)
        
        # Channel selection (lower/upper)
        cv2.createTrackbar("Channel", window_name, 0, 1, self.on_channel_change)
        
        # LAB components
        cv2.createTrackbar("L", window_name, 0, 255, self.on_l_change)
        cv2.createTrackbar("A", window_name, 0, 255, self.on_a_change)
        cv2.createTrackbar("B", window_name, 0, 255, self.on_b_change)
        
        # Min area
        cv2.createTrackbar("Min Area", window_name, 100, 2000, self.on_min_area_change)
        
        # Image processing
        cv2.createTrackbar("Brightness", window_name, 50, 100, self.on_brightness_change)
        cv2.createTrackbar("Contrast", window_name, 10, 50, self.on_contrast_change)
        cv2.createTrackbar("Gamma", window_name, 1, 30, self.on_gamma_change)
        cv2.createTrackbar("CLAHE", window_name, 1, 1, self.on_clahe_change)
        
        # Display options
        cv2.createTrackbar("Show Original", window_name, 0, 1, self.on_show_original_change)
        cv2.createTrackbar("Show Mask", window_name, 1, 1, self.on_show_mask_change)
        cv2.createTrackbar("Show Contours", window_name, 1, 1, self.on_show_contours_change)
        
        # Initialize trackbar values
        self.update_trackbars()
        
        return window_name
    
    def update_trackbars(self):
        """Update trackbar positions based on current values"""
        window_name = "Color Calibration"
        colors = list(self.color_ranges.keys())
        
        cv2.setTrackbarPos("Color", window_name, colors.index(self.current_color))
        cv2.setTrackbarPos("Channel", window_name, 0 if self.current_channel == "lower" else 1)
        
        # Get current color values
        color_info = self.color_ranges[self.current_color]
        values = color_info[self.current_channel]
        
        cv2.setTrackbarPos("L", window_name, values[0])
        cv2.setTrackbarPos("A", window_name, values[1])
        cv2.setTrackbarPos("B", window_name, values[2])
        cv2.setTrackbarPos("Min Area", window_name, color_info["min_area"])
        
        # Image processing values
        cv2.setTrackbarPos("Brightness", window_name, self.brightness + 50)
        cv2.setTrackbarPos("Contrast", window_name, int(self.contrast * 10))
        cv2.setTrackbarPos("Gamma", window_name, int(self.gamma * 10))
        cv2.setTrackbarPos("CLAHE", window_name, 1 if self.use_clahe else 0)
        
        # Display options
        cv2.setTrackbarPos("Show Original", window_name, 1 if self.show_original else 0)
        cv2.setTrackbarPos("Show Mask", window_name, 1 if self.show_mask else 0)
        cv2.setTrackbarPos("Show Contours", window_name, 1 if self.show_contours else 0)
    
    # Trackbar callbacks
    def on_color_change(self, val):
        colors = list(self.color_ranges.keys())
        if val < len(colors):
            self.current_color = colors[val]
            self.update_trackbars()
    
    def on_channel_change(self, val):
        self.current_channel = "lower" if val == 0 else "upper"
        self.update_trackbars()
    
    def on_l_change(self, val):
        self.color_ranges[self.current_color][self.current_channel][0] = val
    
    def on_a_change(self, val):
        self.color_ranges[self.current_color][self.current_channel][1] = val
    
    def on_b_change(self, val):
        self.color_ranges[self.current_color][self.current_channel][2] = val
    
    def on_min_area_change(self, val):
        self.color_ranges[self.current_color]["min_area"] = val
    
    def on_brightness_change(self, val):
        self.brightness = val - 50
    
    def on_contrast_change(self, val):
        self.contrast = val / 10.0
    
    def on_gamma_change(self, val):
        self.gamma = val / 10.0
        self.update_gamma_table()
    
    def on_clahe_change(self, val):
        self.use_clahe = val == 1
    
    def on_show_original_change(self, val):
        self.show_original = val == 1
    
    def on_show_mask_change(self, val):
        self.show_mask = val == 1
    
    def on_show_contours_change(self, val):
        self.show_contours = val == 1
    
    def save_calibration(self, filename="camera_calibration.json"):
        """Save current calibration to JSON file"""
        calibration_data = {
            "color_ranges": self.color_ranges,
            "image_processing": {
                "brightness": self.brightness,
                "contrast": self.contrast,
                "gamma": self.gamma,
                "use_clahe": self.use_clahe
            },
            "timestamp": time.time(),
            "notes": "Camera calibration for WRO Future Engineers robot"
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            print(f"Calibration saved to {filename}")
            return True
        except Exception as e:
            print(f"Error saving calibration: {e}")
            return False
    
    def load_calibration(self, filename="camera_calibration.json"):
        """Load calibration from JSON file"""
        try:
            with open(filename, 'r') as f:
                calibration_data = json.load(f)
            
            if "color_ranges" in calibration_data:
                self.color_ranges = calibration_data["color_ranges"]
            
            if "image_processing" in calibration_data:
                img_proc = calibration_data["image_processing"]
                self.brightness = img_proc.get("brightness", 0)
                self.contrast = img_proc.get("contrast", 3.0)
                self.gamma = img_proc.get("gamma", 0.7)
                self.use_clahe = img_proc.get("use_clahe", True)
                self.update_gamma_table()
            
            print(f"Calibration loaded from {filename}")
            self.update_trackbars()
            return True
        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False
    
    def print_instructions(self):
        """Print usage instructions"""
        print("\n=== Camera Calibration Tool ===")
        print("Controls:")
        print("  - Use trackbars to adjust color thresholds")
        print("  - 's' - Save calibration to JSON")
        print("  - 'l' - Load calibration from JSON")
        print("  - 'r' - Reset to default values")
        print("  - 'Space' - Toggle between lower/upper bounds")
        print("  - '1-4' - Select color (1=red, 2=green, 3=blue, 4=orange)")
        print("  - 'q' - Quit")
        print("\nAdjust the trackbars until objects are detected correctly")
        print("Save when satisfied with the calibration")
        print("The saved JSON file can be used by rpi_controller.py\n")
    
    def run(self):
        """Main calibration loop"""
        try:
            self.setup_camera()
            
            # Start camera thread
            camera_thread = Thread(target=self.camera_thread, daemon=True)
            camera_thread.start()
            
            # Create UI
            window_name = self.create_ui()
            
            # Print instructions
            self.print_instructions()
            
            # Try to load existing calibration
            if os.path.exists("camera_calibration.json"):
                self.load_calibration()
            
            while self.running:
                with self.frame_lock:
                    if self.current_frame is not None:
                        frame = self.current_frame.copy()
                    else:
                        continue
                
                # Process frame for current color
                adjusted, lab, mask = self.process_frame_for_color(frame, self.current_color)
                
                # Find contours and draw
                result_frame, contour_count, total_area = self.find_contours_and_draw(
                    adjusted if not self.show_original else frame, mask, self.current_color)
                
                # Create display
                if self.show_mask:
                    # Show mask alongside original
                    mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
                    display = np.hstack([result_frame, mask_colored])
                else:
                    display = result_frame
                
                # Add info text
                info_text = [
                    f"Color: {self.current_color} ({self.current_channel})",
                    f"Contours: {contour_count}, Area: {int(total_area)}",
                    f"L: {self.color_ranges[self.current_color][self.current_channel][0]}",
                    f"A: {self.color_ranges[self.current_color][self.current_channel][1]}",
                    f"B: {self.color_ranges[self.current_color][self.current_channel][2]}",
                    f"Min Area: {self.color_ranges[self.current_color]['min_area']}"
                ]
                
                for i, text in enumerate(info_text):
                    cv2.putText(display, text, (10, 30 + i * 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow(window_name, display)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.save_calibration()
                elif key == ord('l'):
                    self.load_calibration()
                elif key == ord('r'):
                    self.__init__(self.camera_device)  # Reset to defaults
                    self.setup_camera()
                    self.update_trackbars()
                elif key == ord(' '):
                    self.current_channel = "upper" if self.current_channel == "lower" else "lower"
                    self.update_trackbars()
                elif key >= ord('1') and key <= ord('4'):
                    colors = list(self.color_ranges.keys())
                    idx = key - ord('1')
                    if idx < len(colors):
                        self.current_color = colors[idx]
                        self.update_trackbars()
        
        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    import sys
    
    camera_device = 0
    if len(sys.argv) > 1:
        try:
            camera_device = int(sys.argv[1])
        except ValueError:
            camera_device = sys.argv[1]
    
    print(f"Starting camera calibration with device: {camera_device}")
    
    calibrator = ColorCalibrator(camera_device)
    calibrator.run()

if __name__ == "__main__":
    main()
