# import cv2
# import numpy as np
# import onnxruntime as ort


# MODEL_PATH  = 'best_ncnn.onnx'
# LABELS      = ['green', 'red']
# COLORS      = [(0, 255, 0), (0, 0, 255)]  # green, red
# CONF_THRESH = 0.6
# IMG_SIZE    = 224


# session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
# input_name = session.get_inputs()[0].name


# def preprocess(frame):
#     h, w = frame.shape[:2]
#     scale = IMG_SIZE / max(h, w)
#     nh, nw = int(h * scale), int(w * scale)
#     resized = cv2.resize(frame, (nw, nh))

#     canvas = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)  # grey pad, matches ultralytics default
#     top = (IMG_SIZE - nh) // 2
#     left = (IMG_SIZE - nw) // 2
#     canvas[top:top+nh, left:left+nw] = resized

#     img = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
#     img = np.transpose(img, (2, 0, 1))
#     img = np.expand_dims(img, axis=0)
#     return img, scale, left, top

# def postprocess(outputs, scale, left, top):
#     preds = outputs[0][0]
#     boxes = []
#     for pred in preds:
#         x1, y1, x2, y2, conf, cls_id = pred
#         if conf < CONF_THRESH:
#             continue
#         x1 = int((x1 - left) / scale)
#         y1 = int((y1 - top) / scale)
#         x2 = int((x2 - left) / scale)
#         y2 = int((y2 - top) / scale)
#         boxes.append((x1, y1, x2, y2, float(conf), int(cls_id)))
#     return boxes



# def draw(frame, boxes):
#     for x1, y1, x2, y2, conf, cls_id in boxes:
#         color = COLORS[cls_id] if cls_id < len(COLORS) else (255,255,255)
#         label = f"{LABELS[cls_id]} {conf:.2f}"
#         cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
#         cv2.putText(frame, label, (x1, y1 - 8),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
#     return frame

# # ── Main loop ──
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("Cannot open camera")
#     exit()

# print("Running... press 'q' to quit")

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break



#     inp, scale, left, top = preprocess(frame)
#     outputs = session.run(None, {input_name: inp})
#     boxes = postprocess(outputs, scale, left, top)
#     frame = draw(frame, boxes)

#     # Show count
#     green_count = sum(1 for *_, cls_id in boxes if cls_id == 0)
#     red_count   = sum(1 for *_, cls_id in boxes if cls_id == 1)
#     cv2.putText(frame, f"Green: {green_count}  Red: {red_count}",
#                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

#     cv2.imshow("WRO Block Detector", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()




#!/usr/bin/env python3
"""
WRO Future Engineers block detector — ONNX inference version.
Serial output: commands + CSV coordinates (center_x,center_y,width,height).
"""

import time
import threading
import queue
import subprocess
import numpy as np
import cv2
import av
import onnxruntime as ort
from collections import deque

# ---------------------------------------------------------------------------
# CONFIG — edit to match your ONNX export
# ---------------------------------------------------------------------------
ONNX_MODEL_PATH = "best_ncnn.onnx"
MODEL_INPUT_SIZE = 224
CLASS_NAMES = {0: "green", 1: "red"}   # class index -> color name
CONF_THRESHOLD = 0.6
USE_CUDA_IF_AVAILABLE = True

# Danger zone / navigation thresholds
MIN_SWERVE_HEIGHT = 25
REVERSE_HEIGHT = 80
LEFT_SIDE_MAX = 90
RIGHT_SIDE_MIN = 150

MIN_VOTES_HIGH_CONF = 5
MIN_VOTES_LOW_CONF = 6
CONFIDENCE_FLOOR = 0.55   # ONNX score below this counts as "low confidence" for voting

# ---------------------------------------------------------------------------
# ONNX session setup
# ---------------------------------------------------------------------------
def load_onnx_session(model_path: str) -> tuple:
    providers = ["CPUExecutionProvider"]
    if USE_CUDA_IF_AVAILABLE and "CUDAExecutionProvider" in ort.get_available_providers():
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

    session = ort.InferenceSession(model_path, providers=providers)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]
    print(f"Loaded ONNX model '{model_path}' | providers={session.get_providers()} "
          f"| input={input_name} | outputs={output_names}")
    return session, input_name, output_names

# ---------------------------------------------------------------------------
# Preprocessing — letterbox to a square, track scale/offset to map boxes back
# ---------------------------------------------------------------------------
def preprocess(frame: np.ndarray, size: int) -> tuple:
    h, w = frame.shape[:2]
    scale = size / max(h, w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(frame, (nw, nh))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    top = (size - nh) // 2
    left = (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized

    tensor = canvas.astype(np.float32) / 255.0
    tensor = tensor.transpose(2, 0, 1)          # HWC -> CHW
    tensor = np.expand_dims(tensor, axis=0)     # -> NCHW
    return np.ascontiguousarray(tensor), scale, left, top

# ---------------------------------------------------------------------------
# Postprocessing — decode Ultralytics-style output + NMS, map back to frame
# ---------------------------------------------------------------------------
def decode_onnx_output(raw_output: np.ndarray, scale: float, left: int, top: int,
                        conf_thresh: float) -> dict:
    """raw_output: (1, 300, 6) -> [x1,y1,x2,y2,conf,cls_id]."""
    preds = raw_output[0]   # (300, 6)

    best_per_class = {}
    for pred in preds:
        x1, y1, x2, y2, conf, cls_id = pred
        if conf < conf_thresh:
            continue
        cls_id = int(cls_id)
        if cls_id not in best_per_class or conf > best_per_class[cls_id][0]:
            best_per_class[cls_id] = (float(conf), x1, y1, x2, y2)

    results = {}
    for cls_id, (conf, x1, y1, x2, y2) in best_per_class.items():
        color = CLASS_NAMES.get(cls_id)
        if color is None:
            continue
        ox1 = (x1 - left) / scale
        oy1 = (y1 - top) / scale
        ox2 = (x2 - left) / scale
        oy2 = (y2 - top) / scale
        ow = ox2 - ox1
        oh = oy2 - oy1

        results[color] = {
            "x": int(round(ox1)), "y": int(round(oy1)),
            "width": int(round(ow)), "height": int(round(oh)),
            "center_x": int(round(ox1 + ow / 2)), "center_y": int(round(oy1 + oh / 2)),
            "confidence": conf,
            "low_confidence": conf < CONFIDENCE_FLOOR,
        }
    return results

def detect_blocks_onnx(frame: np.ndarray, session, input_name: str) -> tuple:
    tensor, scale, left, top = preprocess(frame, MODEL_INPUT_SIZE)
    outputs = session.run(None, {input_name: tensor})
    decoded = decode_onnx_output(outputs[0], scale, left, top, CONF_THRESHOLD)
    return decoded.get("red"), decoded.get("green")

# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------
def upscale_for_display(frame_bgr: np.ndarray, scale: int = 3) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict) -> np.ndarray:
    out = frame_bgr.copy()
    for box, bgr_color, label in ((red_box, (0, 0, 255), "RED"), (green_box, (0, 255, 0), "GREEN")):
        if not box:
            continue
        x, y, w, h = box['x'], box['y'], box['width'], box['height']
        cx, cy = box['center_x'], box['center_y']
        cv2.rectangle(out, (x, y), (x + w, y + h), bgr_color, 2)
        cv2.circle(out, (cx, cy), 3, bgr_color, -1)
        conf = box.get('confidence')
        conf_str = f" conf={conf:.2f}" if conf is not None else ""
        cv2.putText(out, f"{label} {w}x{h}px{conf_str}", (x, max(0, y - 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, bgr_color, 1)
        cv2.putText(out, f"pos=({x},{y}) center=({cx},{cy})", (x, max(0, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, bgr_color, 1)
    return out

# ---------------------------------------------------------------------------
# Camera capture
# ---------------------------------------------------------------------------
def open_camera(camera_id: int):
    try:
        container = av.open(f'/dev/video{camera_id}', format='v4l2',
                             options={'video_size': '640x480', 'framerate': '30', 'input_format': 'mjpeg'})
        stream = container.streams.video[0]
        stream.thread_type = 'AUTO'
        return container, stream
    except Exception:
        try:
            container = av.open(f'/dev/video{camera_id}', format='v4l2',
                                 options={'video_size': '640x480', 'framerate': '30', 'input_format': 'yuyv422'})
            stream = container.streams.video[0]
            stream.thread_type = 'AUTO'
            return container, stream
        except Exception as e:
            print(f"Camera error: {e}")
            return None, None

def resize_frame(frame: np.ndarray, target_w: int = 240, target_h: int = 240) -> np.ndarray:
    if frame is None or frame.size == 0:
        return None
    return cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)

def start_capture_thread(container, stream, frame_size=240):
    frame_q = queue.Queue(maxsize=1)
    stop_flag = threading.Event()

    def capture_loop():
        try:
            for packet in container.demux(stream):
                if stop_flag.is_set():
                    break
                for frame in packet.decode():
                    if stop_flag.is_set():
                        break
                    try:
                        if frame.format.name != 'rgb24':
                            frame = frame.reformat(format='rgb24')
                        img = frame.to_ndarray(format='rgb24')
                        if img is not None and img.size > 0:
                            img = resize_frame(img, frame_size, frame_size)
                            if img is not None:
                                if frame_q.full():
                                    try:
                                        frame_q.get_nowait()
                                    except queue.Empty:
                                        pass
                                frame_q.put(img)
                    except Exception as e:
                        print(f"Frame processing error: {e}")
                        continue
        except Exception as e:
            print(f"\nCapture thread stopped: {e}")

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    return t, frame_q, stop_flag

def set_manual_camera_controls(camera_id: int, exposure_value: int = 156, wb_temperature: int = 4500):
    dev = f'/dev/video{camera_id}'
    cmds = [
        ['v4l2-ctl', '-d', dev, '-c', 'auto_exposure=1'],
        ['v4l2-ctl', '-d', dev, '-c', f'exposure_time_absolute={exposure_value}'],
        ['v4l2-ctl', '-d', dev, '-c', 'white_balance_automatic=0'],
        ['v4l2-ctl', '-d', dev, '-c', f'white_balance_temperature={wb_temperature}'],
    ]
    for cmd in cmds:
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Warning: could not run {' '.join(cmd)} ({e})")
    print(f"Camera controls locked: exposure={exposure_value}, wb_temp={wb_temperature}")

# ---------------------------------------------------------------------------
# Kalman filter
# ---------------------------------------------------------------------------
def create_kalman_filter():
    kf = cv2.KalmanFilter(4, 2, 0, type=cv2.CV_64F)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float64)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float64)
    kf.processNoiseCov = np.eye(4, dtype=np.float64) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float64) * 1.0
    return kf

def kalman_update(kf, box, initialized: bool):
    if box is not None:
        measurement = np.array([[np.float64(box['center_x'])], [np.float64(box['center_y'])]])
        if not initialized:
            kf.statePre = np.array([[box['center_x']], [box['center_y']], [0], [0]], np.float64)
            kf.statePost = np.array([[box['center_x']], [box['center_y']], [0], [0]], np.float64)
            initialized = True
        else:
            kf.predict()
        kf.correct(measurement)
        smoothed = {'center_x': int(kf.statePost[0, 0]), 'center_y': int(kf.statePost[1, 0]),
                    'vx': float(kf.statePost[2, 0]), 'vy': float(kf.statePost[3, 0])}
        return smoothed, initialized
    else:
        if not initialized:
            return None, initialized
        predicted = kf.predict()
        smoothed = {'center_x': int(predicted[0, 0]), 'center_y': int(predicted[1, 0]),
                    'vx': float(predicted[2, 0]), 'vy': float(predicted[3, 0])}
        return smoothed, initialized

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(camera_id: int = 0, frame_size: int = 240):
    set_manual_camera_controls(camera_id, exposure_value=200, wb_temperature=4500)

    try:
        import serial
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        print("Serial port opened")
    except Exception as e:
        print(f"Could not open serial port: {e}")
        ser = None

    session, input_name, _ = load_onnx_session(ONNX_MODEL_PATH)

    container, stream = open_camera(camera_id)
    if container is None:
        print("Cannot open webcam.")
        return

    t, frame_q, stop_flag = start_capture_thread(container, stream, frame_size)

    def get_frame(timeout=1.0):
        try:
            return frame_q.get(timeout=timeout)
        except queue.Empty:
            return None

    print("Testing camera...")
    test_frames = 0
    for _ in range(10):
        frame = get_frame(timeout=2.0)
        if frame is not None:
            test_frames += 1
            print(f"Got test frame {test_frames}, shape: {frame.shape}")
        time.sleep(0.1)

    if test_frames == 0:
        print("No frames received from camera!")
        stop_flag.set()
        t.join(timeout=2.0)
        container.close()
        return

    window_name = "WRO Block Detector (ONNX)"
    cv2.namedWindow(window_name)

    history_len = 7
    red_hist = deque(maxlen=history_len)
    green_hist = deque(maxlen=history_len)

    kf = create_kalman_filter()
    kf_initialized = False
    kf_color = None   # tracks 'red' / 'green' / None — NOT the zone decision

    last_sent = None
    frame_count = 0
    clear_counter = 0
    CLEAR_HISTORY = 10

    try:
        while True:
            frame = get_frame(timeout=0.5)
            if frame is None:
                continue

            red_box, green_box = detect_blocks_onnx(frame, session, input_name)
            red_hist.append(red_box)
            green_hist.append(green_box)

            def confirmed(hist):
                boxes = [b for b in hist if b is not None]
                if not boxes:
                    return False
                latest = boxes[-1]
                required = MIN_VOTES_LOW_CONF if latest.get('low_confidence') else MIN_VOTES_HIGH_CONF
                return len(boxes) >= required

            red_confirmed = confirmed(red_hist)
            green_confirmed = confirmed(green_hist)

            primary_box = None
            primary_color = None
            if red_confirmed and green_confirmed:
                if red_box is not None and green_box is not None:
                    primary_box, primary_color = (red_box, 'red') if red_box['height'] >= green_box['height'] else (green_box, 'green')
                elif red_box is not None:
                    primary_box, primary_color = red_box, 'red'
                elif green_box is not None:
                    primary_box, primary_color = green_box, 'green'
            elif red_confirmed:
                primary_box, primary_color = red_box, 'red'
            elif green_confirmed:
                primary_box, primary_color = green_box, 'green'

            # ---- Zone decision logic ----
            decision = 'CLEAR'
            active_box = None

            if primary_box is not None:
                block_height = primary_box['height']
                if block_height < MIN_SWERVE_HEIGHT:
                    pass   # too far, ignore
                elif block_height > REVERSE_HEIGHT:
                    decision = 'REVERSE'
                    active_box = primary_box
                else:
                    if primary_color == 'red':
                        if primary_box['center_x'] > LEFT_SIDE_MAX:
                            decision = 'RED'
                        else:
                            decision = 'CLEAR'
                    else:  # green
                        if primary_box['center_x'] < RIGHT_SIDE_MIN:
                            decision = 'GREEN'
                        else:
                            decision = 'CLEAR'
                    if decision != 'CLEAR':
                        active_box = primary_box

            # ---- Kalman filter handling ----
            # Reset only when the tracked COLOR changes (or track is lost),
            # not when the zone decision (CLEAR vs RED/GREEN/REVERSE) flips —
            # a block hovering near a zone boundary shouldn't thrash the filter.
            track_color = primary_color  # 'red', 'green', or None if nothing confirmed

            if track_color != kf_color:
                kf = create_kalman_filter()
                kf_initialized = False
                kf_color = track_color

            smoothed = None
            if track_color is not None and primary_box is not None:
                # Always feed the filter the confirmed detection, even when
                # decision == 'CLEAR' (block confirmed but outside trigger zone/height) —
                # this keeps velocity estimates warm instead of resetting on every
                # boundary crossing.
                smoothed, kf_initialized = kalman_update(kf, primary_box, kf_initialized)
            elif kf_color is not None and kf_initialized:
                # Track color still active but this frame had no box (brief occlusion) —
                # let the filter predict-forward instead of dropping smoothing entirely.
                smoothed, kf_initialized = kalman_update(kf, None, kf_initialized)

            clear_counter = clear_counter + 1 if decision == 'CLEAR' else 0

            # ---- Build serial command string ----
            if decision in ('RED', 'GREEN', 'REVERSE') and active_box is not None:
                cmd_str = f"{decision},{active_box['center_x']},{active_box['center_y']},{active_box['width']},{active_box['height']}\n"
            else:
                cmd_str = "CLEAR\n"

            # ---- Send over serial if command changed ----
            if cmd_str != last_sent and ser is not None:
                if not (decision == 'CLEAR' and clear_counter < CLEAR_HISTORY):  # debounce CLEAR
                    try:
                        ser.write(cmd_str.encode())
                        print(f">>> Sent {cmd_str.strip()}")
                        last_sent = cmd_str
                    except Exception as e:
                        print(f"Serial write failed: {e}")

            # Display drawing (unchanged)
            display_red = red_box if red_confirmed else None
            display_green = green_box if green_confirmed else None
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            display = draw_boxes(bgr, display_red, display_green)

            cv2.line(display, (LEFT_SIDE_MAX, 0), (LEFT_SIDE_MAX, frame_size - 1), (0, 165, 255), 1)
            cv2.putText(display, "RED SAFE <", (2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 165, 255), 1)
            cv2.line(display, (RIGHT_SIDE_MIN, 0), (RIGHT_SIDE_MIN, frame_size - 1), (0, 255, 0), 1)
            cv2.putText(display, "GREEN SAFE >", (RIGHT_SIDE_MIN + 2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
            cv2.putText(display, "DANGER", (LEFT_SIDE_MAX + 5, frame_size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

            display = upscale_for_display(display, scale=3)
            cv2.imshow(window_name, display)
            frame_count += 1

            smoothed_str = (f"SMOOTHED[{kf_color},x={smoothed['center_x']},y={smoothed['center_y']},"
                             f"vx={smoothed['vx']:.1f},vy={smoothed['vy']:.1f}]" if smoothed is not None else "SMOOTHED:None")
            print(f"Frame {frame_count} | RED:{red_box} | GREEN:{green_box} | {smoothed_str}", flush=True)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()
        t.join(timeout=2.0)
        container.close()
        if ser is not None:
            ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(camera_id=0, frame_size=240)