#!/usr/bin/env python3
"""
Simplified dual camera switching - based on working minimal version
"""

from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import time

app = Flask(__name__)

print("Loading model...")
model = YOLO('/home/ptdts/ptdts_framework/models/yolov11n-UAV-finetune.onnx')
print("✓ Model loaded")

print("Initializing cameras...")
hq_cam = Picamera2(1)
hq_config = hq_cam.create_preview_configuration(main={"size": (416, 312)})
hq_cam.configure(hq_config)
hq_cam.start()

gs_cam = Picamera2(0)
gs_config = gs_cam.create_preview_configuration(main={"size": (416, 312)})
gs_cam.configure(gs_config)
gs_cam.start()
time.sleep(2)
print("✓ Cameras ready")

# Simple state tracking
current_camera = "HQ"  # or "GS"
last_detection_time = 0
frame_count = 0

def generate_frames():
    global current_camera, last_detection_time, frame_count
    
    print("Frame generation started")
    
    while True:
        frame_count += 1
        current_time = time.time()
        
        # Decide which camera to use
        if current_camera == "HQ":
            # DETECTION MODE
            frame = hq_cam.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Run detection every 6 frames
            if frame_count % 6 == 0:
                results = model(frame_bgr, conf=0.5, verbose=False, imgsz=416)
                
                if len(results[0].boxes) > 0:
                    print("🎯 DRONE DETECTED! Switching to GS camera")
                    current_camera = "GS"
                    last_detection_time = current_time
                    annotated = results[0].plot()
                else:
                    annotated = frame_bgr
            else:
                annotated = frame_bgr
            
            # Overlay
            cv2.putText(annotated, "DETECTION MODE - HQ Camera", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                       
        else:  # GS camera
            # TRACKING MODE
            frame = gs_cam.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Run detection every frame
            results = model(frame_bgr, conf=0.4, verbose=False, imgsz=416)
            
            if len(results[0].boxes) > 0:
                last_detection_time = current_time
                annotated = results[0].plot()
                
                if frame_count % 10 == 0:
                    print(f"Tracking drone...")
            else:
                annotated = frame_bgr
                
                # Switch back after 3 seconds
                if current_time - last_detection_time > 3.0:
                    print("🔍 Lost target. Switching back to HQ camera")
                    current_camera = "HQ"
            
            # Overlay
            cv2.putText(annotated, "TRACKING MODE - GS Camera", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Add frame counter
        cv2.putText(annotated, f"Frame: {frame_count}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Encode
        ret, buffer = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        if not ret:
            print("Failed to encode frame")
            continue
        
        frame_bytes = buffer.tobytes()
        
        # Debug output every 100 frames
        if frame_count % 100 == 0:
            print(f"Frame {frame_count}: {len(frame_bytes)} bytes, camera={current_camera}")
        
        # Yield
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + 
               frame_bytes + b'\r\n')

@app.route('/')
def index():
    html = """
    <html>
    <head>
        <title>Dual Camera Test</title>
        <style>
            body {background:#1a1a1a;text-align:center;color:#fff;}
            h1 {color:#0f0;}
            img {max-width:90%;border:3px solid #0f0;margin:20px;}
        </style>
    </head>
    <body>
        <h1>🎯 Dual Camera Switching</h1>
        <p>HQ (wide) → detects → switches to GS (narrow) → tracks</p>
        <img src="/video_feed" id="feed">
        <script>
            // Monitor loading
            document.getElementById('feed').onerror = function() {
                console.error('Video feed error');
            };
            document.getElementById('feed').onload = function() {
                console.log('Video loaded');
            };
        </script>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    print("Video feed requested from browser")
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    print("\nStarting server on port 5000...")
    print("Open: http://<your-pi-ip>:5000\n")
    app.run(host='0.0.0.0', port=5000, threaded=True)