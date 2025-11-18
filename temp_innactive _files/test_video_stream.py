#!/usr/bin/env python3
"""
Quick test: Camera video streaming
Tests both cameras and web interface
"""

from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import time

app = Flask(__name__)

# Initialize cameras
print("Initializing cameras...")
hq_cam = Picamera2(1)  # Your HQ camera
gs_cam = Picamera2(0)  # Your GS camera

# Configure HQ camera for streaming
hq_config = hq_cam.create_preview_configuration(
    main={"size": (1280, 720)}
)
hq_cam.configure(hq_config)
hq_cam.start()

print("Cameras initialized!")

def generate_frames():
    """Generate video frames"""
    while True:
        # Capture frame
        frame = hq_cam.capture_array()
        
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Add timestamp overlay
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame_bgr, timestamp, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Add camera info
        cv2.putText(frame_bgr, "HQ Camera - Port 1", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', frame_bgr)
        frame_bytes = buffer.tobytes()
        
        # Yield frame in multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + 
               frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Web page with video feed"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>PTDTS Video Test</title>
        <style>
            body { 
                background: #1a1a1a; 
                color: #fff; 
                font-family: Arial; 
                text-align: center;
                margin: 0;
                padding: 20px;
            }
            h1 { color: #00ff00; }
            #videoFeed { 
                max-width: 90%; 
                border: 3px solid #00ff00;
                margin: 20px auto;
                display: block;
            }
            .status {
                background: #333;
                padding: 20px;
                border-radius: 10px;
                max-width: 600px;
                margin: 20px auto;
            }
        </style>
    </head>
    <body>
        <h1>🎥 PTDTS Video Stream Test</h1>
        <div class="status">
            <h2>✓ Camera System Active</h2>
            <p>If you see video below, streaming is working!</p>
        </div>
        <img id="videoFeed" src="/video_feed">
        <div class="status">
            <p>Camera: HQ (IMX477) - Port 1</p>
            <p>Resolution: 1280x720</p>
            <p>Access from any device on network</p>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    print("\n" + "="*50)
    print("🎥 PTDTS Video Stream Test Server")
    print("="*50)
    print(f"\nOpen browser to: http://<your-pi-ip>:5000")
    print("\nPress Ctrl+C to stop\n")
    
    app.run(host='0.0.0.0', port=5000, threaded=True)