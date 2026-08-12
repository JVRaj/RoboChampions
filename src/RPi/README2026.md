WRO Future Engineers – Block Detector (ONNX)

This project is a real‑time green/red block detector built for the
World Robot Olympiad (WRO) Future Engineers challenge. It captures frames
from a USB camera, runs an ONNX exported object detection model (e.g.
YOLOv8‑nano), and sends navigation commands (CLEAR, RED, GREEN, REVERSE)
over a serial port to a motor control board (Arduino / ESP32).

A Kalman filter smooths bounding‑box coordinates and provides velocity
estimates, while a voting window suppresses false positives. The system is
designed to run on a Raspberry Pi or similar edge device.


Features

- ONNX inference – runs on CPU (or optional CUDA) for low latency.
- Threaded camera capture using PyAV – no busy‑wait, compatible with
  MJPEG/YUYV USB cameras.
- Voting‑based confirmation – requires N consecutive detections to
  lock onto a block, preventing jitter from single‑frame misses.
- Kalman filter – tracks center_x and center_y even when the block
  is temporarily occluded; also outputs vx/vy velocity estimates.
- Competition‑ready logic – distinguishes between left/right safe zones,
  close‑range reverse, and far‑range ignore, matching WRO FE rules.
- Serial command output – CLEAR\n, RED,<x>,<y>,<w>,<h>\n, etc.,
  sent only when the command changes (with CLEAR debouncing).
- Live display – 3× upscaled window with bounding boxes, danger‑zone
  lines, and debug overlays.


Hardware Requirements

- Raspberry Pi 4 (or 5) – tested; any Linux SBC with USB should work.
- USB Webcam – preferably with MJPEG support (640×480 recommended).
- Serial connection to a motor controller (/dev/ttyUSB1 by default).
- A pre‑trained ONNX model that outputs (1, 300, 6) arrays:
  [x1, y1, x2, y2, confidence, class_id] (Ultralytics export format).


Dependencies

Install on your device (Raspberry Pi OS / Ubuntu):

sudo apt update
sudo apt install -y python3-opencv python3-numpy v4l-utils

pip3 install --upgrade pip
pip3 install av onnxruntime

If you want CUDA acceleration (optional, NVidia Jetson or x86 with GPU):

pip3 install onnxruntime-gpu

For serial communication:

pip3 install pyserial

All packages can be installed together with:

pip3 install opencv-python numpy av onnxruntime pyserial

Important: v4l-utils provides the v4l2-ctl tool used to lock camera
exposure and white balance.


Installation

1. Clone this repository (or copy the Python script) onto your robot’s SBC.
2. Place your ONNX model file in the same directory and rename it to
   best_ncnn.onnx, or adjust the ONNX_MODEL_PATH variable.
3. Connect your webcam and serial cable.
4. Identify the camera device: usually /dev/video0.
   Check available cameras with v4l2-ctl --list-devices.
5. Identify the serial port: /dev/ttyUSB1 by default;
   adjust the serial.Serial('/dev/ttyUSB1', ...) line as needed.


Configuration

All adjustable parameters are at the top of the script:

ONNX_MODEL_PATH = "best_ncnn.onnx"   # Path to ONNX model
MODEL_INPUT_SIZE = 224               # Model expects 224×224 input
CLASS_NAMES = {0: "green", 1: "red"} # Must match your training classes
CONF_THRESHOLD = 0.6                 # Minimum confidence to consider a detection
USE_CUDA_IF_AVAILABLE = True         # Attempt GPU acceleration

Zone & navigation thresholds:

MIN_SWERVE_HEIGHT = 25   # Ignore blocks shorter than this (too far)
REVERSE_HEIGHT = 80      # Block height that triggers REVERSE command
LEFT_SIDE_MAX = 90       # Pixel column: left side safe for RED blocks
RIGHT_SIDE_MIN = 150     # Pixel column: right side safe for GREEN blocks

Voting / anti‑jitter:

MIN_VOTES_HIGH_CONF = 5   # Consecutive frames needed for high‑confidence detections
MIN_VOTES_LOW_CONF = 6    # Frames needed for low‑confidence detections
CONFIDENCE_FLOOR = 0.55   # Boundary between “high” and “low” confidence

Serial / display:
The serial port is hard‑coded in main(); change the serial.Serial(...) line.
The display window can be closed by pressing 'q'.


How It Works

1. Camera Capture Thread
   A separate thread continuously reads frames from the webcam via PyAV,
   resizes them to 240×240 (for speed), and puts the latest frame into a
   queue with maxsize=1. The main loop always gets the freshest frame.

2. ONNX Inference
   Each frame is preprocessed:
   - Letterbox‑resize to 224×224 with grey padding (114,114,114).
   - Normalised to [0,1] and transposed to NCHW.
   - Feed to ONNX Runtime.

   The output tensor (1,300,6) is decoded:
   - Only the single best detection per class is kept (highest confidence).
   - Coordinates are remapped back to the original 240×240 space.

3. Voting Filter
   Detections are pushed into a deque (length 7). A block is considered
   confirmed only if the number of consecutive non‑None entries reaches
   the required vote threshold (depending on confidence).

4. Kalman Filter
   A separate Kalman filter (4‑state: [x, y, vx, vy]) tracks the primary
   detected colour. It resets only when the tracked colour changes, not
   when the zone decision flips, so a block wobbling near a boundary doesn’t
   cause filter thrash. The filter predicts forward when a frame has no
   detection (brief occlusion).

5. Navigation Decision
   Based on the confirmed block’s height and center_x:

   Condition                                   Command
   -----------------------------------------   -----------
   Block height < MIN_SWERVE_HEIGHT            CLEAR
   Block height > REVERSE_HEIGHT               REVERSE
   RED block & center_x > LEFT_SIDE_MAX        RED
   GREEN block & center_x < RIGHT_SIDE_MIN     GREEN
   Otherwise (block in danger zone)            CLEAR

   When two blocks are confirmed, the taller one takes precedence.

6. Serial Output
   Commands are sent only when they differ from the previous one.
   CLEAR is additionally debounced: it must persist for 10 consecutive
   frames before being transmitted, preventing brief drop‑outs.


Usage

Run the script from the terminal:

python3 block_detector.py

- A window titled "WRO Block Detector (ONNX)" will appear showing the
  camera feed, bounding boxes, danger‑zone lines, and smoothed coordinates.
- Press 'q' to quit.
- Terminal output includes frame count, raw detections, and Kalman state.

Serial output format:

CLEAR
RED,145,120,32,45
GREEN,89,110,28,40
REVERSE,112,180,50,70

The motor controller should parse these comma‑separated strings to steer
the robot accordingly.


Troubleshooting

- No frames from camera
  Ensure camera_id in main() matches your device (e.g. 0 for /dev/video0).
  Use v4l2-ctl --list-devices to list cameras.
  Try different pixel formats: the script first tries mjpeg, then yuyv422.

- ONNX model fails to load
  Verify the model path. The script expects the Ultralytics export format
  (output shape 1×300×6). If you used a different framework, adjust
  decode_onnx_output() accordingly.

- Serial port not found
  Change the hard‑coded /dev/ttyUSB1 to your actual port.
  You can list available ports with:
  python3 -m serial.tools.list_ports

- Poor detection accuracy
  - Tune CONF_THRESHOLD (lower = more detections, higher = fewer false positives).
  - Adjust CONFIDENCE_FLOOR and voting thresholds.
  - Make sure the camera’s exposure and white balance are locked (script does this automatically).

- Display window too small/large
  Modify the upscale_for_display() scale factor (default 3).


License

This project is provided for educational and competitive use.
You are free to modify and distribute it.


Credits

Developed for the WRO Future Engineers challenge.
Uses ONNX Runtime, OpenCV, PyAV, and the robust Kalman filter from OpenCV.