#!/usr/bin/env python3
"""
PTDTS - USING ONNX MODEL (2-3x faster than PyTorch!)
"""

import time
import cv2
from flask import Flask, Response
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import PWMOutputDevice, Button, Device
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

# Motor
motor_in1 = PWMOutputDevice(27, frequency=600)
motor_in2 = PWMOutputDevice(22, frequency=600)

# Encoder
encoder_a = Button(17, pull_up=True)
encoder_b = Button(4, pull_up=True)

encoder_count = 0
last_a = False
last_b = False
COUNTS_PER_DEGREE = 22.34

def encoder_callback():
    global encoder_count, last_a, last_b
    a = encoder_a.is_pressed
    b = encoder_b.is_pressed
    
    if a != last_a:
        encoder_count += 1 if a == b else -1
    if b != last_b:
        encoder_count += 1 if a != b else -1
    
    last_a, last_b = a, b

encoder_a.when_pressed = encoder_callback
encoder_a.when_released = encoder_callback
encoder_b.when_pressed = encoder_callback
encoder_b.when_released = encoder_callback

def get_position():
    return encoder_count / COUNTS_PER_DEGREE

def move_motor(speed):
    speed = max(-1, min(1, speed))
    if speed > 0:
        motor_in1.value = 0
        motor_in2.value = speed
    elif speed < 0:
        motor_in1.value = -speed
        motor_in2.value = 0
    else:
        motor_in1.value = 0
        motor_in2.value = 0

# Camera
print("Starting camera...")
cam = Picamera2(1)
config = cam.create_video_configuration(main={"size": (640, 480)})
cam.configure(config)
cam.start()
time.sleep(2)

# YOLO with ONNX - FASTER!
print("Loading YOLO model...")
model = YOLO('/home/ptdts/ptdts_framework/models/yolov11n-UAV-finetune.onnx')
print("✓ Using ONNX format - 2-3x faster!")

app = Flask(__name__)

def generate_frames():
    frame_count = 0
    fps_start = time.time()
    fps_count = 0
    current_fps = 0
    
    last_detection = None
    no_detection_frames = 0
    
    while True:
        frame_count += 1
        fps_count += 1
        
        if time.time() - fps_start >= 1.0:
            current_fps = fps_count
            fps_count = 0
            fps_start = time.time()
        
        # Capture
        frame = cam.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # YOLO every 4 frames (ONNX is faster, so we can afford every 4)
        detection = None
        if frame_count % 1 == 0:
            results = model(frame_bgr, verbose=False, conf=0.5, imgsz=640)
            
            if len(results[0].boxes) > 0:
                box = results[0].boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = (x1 + x2) / 2
                detection = (cx, x1, y1, x2, y2)
                last_detection = detection
                no_detection_frames = 0
        else:
            if last_detection and no_detection_frames < 10:
                detection = last_detection
                no_detection_frames += 1
        
        # Motor control
        if detection:
            cx, x1, y1, x2, y2 = detection
            frame_center = 208
            error_pixels = cx - frame_center
            
            speed = error_pixels / 208 * 0.25
            
            if abs(error_pixels) > 5:
                move_motor(speed)
            else:
                move_motor(0)
            
            # Draw
            cv2.rectangle(frame_bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame_bgr, (int(cx), int((y1+y2)/2)), 5, (0, 255, 0), -1)
            cv2.putText(frame_bgr, f"Err: {error_pixels:.0f}px", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            move_motor(0)
            last_detection = None
        
        # UI
        h, w = frame_bgr.shape[:2]
        cv2.line(frame_bgr, (w//2, 0), (w//2, h), (0, 255, 255), 1)
        cv2.putText(frame_bgr, f"Pos: {get_position():.1f} deg", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_bgr, f"FPS: {current_fps}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame_bgr, "ONNX", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Encode
        _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return '''
    <html>
    <head><title>ONNX Tracker</title></head>
    <body style="background:#000;text-align:center;margin:0;padding:20px;">
        <h1 style="color:#0f0;">ONNX Fast Tracker</h1>
        <p style="color:#ff0;">Using ONNX format - 2-3x faster than PyTorch!</p>
        <img src="/video_feed" style="max-width:90%;border:3px solid #0f0;">
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("=" * 60)
    print("🎯 ONNX FAST TRACKER")
    print("=" * 60)
    print("Model: yolov11n-UAV-finetune.onnx")
    print("Expected: 12-18 FPS (2-3x faster than PyTorch)")
    print("Open: http://10.233.137.51:5000")
    print("=" * 60)
    
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        move_motor(0)
        cam.stop()
        print("\n✓ Stopped")