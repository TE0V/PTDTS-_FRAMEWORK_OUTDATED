#!/usr/bin/env python3
"""
Dual camera with pan control - DEBUG VERSION
Reduced gain, lots of debug output
"""

from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import PWMOutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import cv2
import time

# Setup GPIO
Device.pin_factory = LGPIOFactory()

app = Flask(__name__)

# Load model
print("Loading UAV model...")
model = YOLO('/home/ptdts/ptdts_framework/models/yolov11n-UAV-finetune_ncnn_model')
print("✓ Model loaded")

# Initialize cameras
print("Initializing cameras...")
hq_cam = Picamera2(1)
hq_config = hq_cam.create_preview_configuration(main={"size": (640, 480)})
hq_cam.configure(hq_config)
hq_cam.start()

gs_cam = Picamera2(0)
gs_config = gs_cam.create_preview_configuration(main={"size": (640, 480)})
gs_cam.configure(gs_config)
gs_cam.start()
time.sleep(2)
print("✓ Cameras ready")

# Initialize motor control
print("Initializing pan motor...")
motor_in1 = PWMOutputDevice(27, frequency=1000)
motor_in2 = PWMOutputDevice(22, frequency=1000)

# Initialize encoder
encoder_a = Button(17, pull_up=True)
encoder_b = Button(4, pull_up=True)

print("✓ Motor initialized")

# Motor control class
class PanMotor:
    def __init__(self):
        self.encoder_count = 0
        self.last_a = False
        self.last_b = False
        self.target_count = 0  # Target is set once, not recalculated
        self.is_moving = False
        
        # Setup encoder callbacks
        encoder_a.when_pressed = self._encoder_callback
        encoder_a.when_released = self._encoder_callback
        encoder_b.when_pressed = self._encoder_callback
        encoder_b.when_released = self._encoder_callback
        
    def _encoder_callback(self):
        """Quadrature encoder decoding"""
        a = encoder_a.is_pressed
        b = encoder_b.is_pressed
        
        if a != self.last_a:
            if a == b:
                self.encoder_count += 1
            else:
                self.encoder_count -= 1
                
        if b != self.last_b:
            if a != b:
                self.encoder_count += 1
            else:
                self.encoder_count -= 1
                
        self.last_a = a
        self.last_b = b
    
    def set_target_relative(self, count_delta):
        """
        Set target position relative to current position
        Call this ONCE when starting a movement
        """
        self.target_count = self.encoder_count + count_delta
        self.is_moving = True
        print(f"  Target set: {self.encoder_count} + {count_delta} = {self.target_count}")
    
    def update(self, tolerance=50):
        """
        Update motor to reach target
        Call this every frame to move toward target
        Returns True when target reached
        """
        if not self.is_moving:
            return True
        
        error = self.target_count - self.encoder_count
        
        # Check if we're close enough
        if abs(error) < tolerance:
            motor_in1.value = 0
            motor_in2.value = 0
            self.is_moving = False
            return True
        
        # Proportional control
        if abs(error) < 200:
            speed = 0.4 + (abs(error) / 200.0) * 0.2  # 40-60%
        else:
            speed = 0.6 + min(abs(error) / 1000.0, 0.2)  # 60-80%
        
        # Motor direction
        if error > 0:  # Need more counts
            motor_in1.value = 0
            motor_in2.value = speed
        else:  # Need fewer counts
            motor_in1.value = speed
            motor_in2.value = 0
        
        return False
    
    def stop(self):
        """Stop motor immediately"""
        motor_in1.value = 0
        motor_in2.value = 0
        self.is_moving = False

pan_motor = PanMotor()

# System state
class SystemState:
    def __init__(self):
        self.mode = "DETECTING"
        self.target_count_delta = 0
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        self.panning_started = False
        
state = SystemState()

# Camera FOV - START CONSERVATIVE
HQ_FOV_HORIZONTAL = 60.0  # degrees
GS_FOV_HORIZONTAL = 45.0  # degrees

# CRITICAL: Gain reduction factor
ANGLE_GAIN = 0.3  # Only move 30% of calculated angle

def pixel_to_encoder_counts(pixel_x, image_width, camera_fov, gain=ANGLE_GAIN):
    """
    Convert pixel position to encoder counts
    With aggressive gain reduction
    """
    # Center of image
    center_x = image_width / 2
    
    # Pixels from center
    offset_pixels = pixel_x - center_x
    
    # What fraction of image width?
    fraction = offset_pixels / (image_width / 2)  # -1.0 to +1.0
    
    # Convert to angle offset (half FOV on each side of center)
    angle_offset = fraction * (camera_fov / 2)
    
    # Apply gain reduction
    angle_offset *= gain
    
    # Rough estimate: 24 counts per degree
    # But we'll be conservative
    counts_per_degree = 20  # Lower estimate = smaller movements
    count_delta = int(angle_offset * counts_per_degree)
    
    return count_delta, angle_offset

def generate_frames():
    global state
    
    print("Frame generation started")
    print("DEBUG MODE: Reduced gain, detailed logging")
    
    while True:
        state.frame_count += 1
        current_time = time.time()
        
        # === DETECTING MODE (HQ Camera - Wide FOV) ===
        if state.mode == "DETECTING":
            frame = hq_cam.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Run detection every 6 frames (~5 FPS)
            if state.frame_count % 6 == 0:
                results = model(frame_bgr, conf=0.5, verbose=False, imgsz=640)
                
                if len(results[0].boxes) > 0:
                    # Get first detection
                    box = results[0].boxes[0]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
    
                    # Calculate center of bounding box
                    bbox_center_x = (x1 + x2) / 2
                    image_center_x = 1280 / 2
    
                    # Convert to encoder counts with REDUCED GAIN
                    count_delta, angle_estimate = pixel_to_encoder_counts(
                        bbox_center_x, 1280, HQ_FOV_HORIZONTAL
                    )
    
                    state.target_count_delta = count_delta
    
                    # SET TARGET ONCE HERE
                    pan_motor.set_target_relative(count_delta)  # <-- CHANGED
    
                    # Switch to panning mode
                    state.mode = "PANNING"
                    state.detection_count += 1
                    
                    print(f"\n{'='*70}")
                    print(f"🎯 DRONE DETECTED #{state.detection_count}")
                    print(f"{'='*70}")
                    print(f"Bounding box center: {bbox_center_x:.0f} px")
                    print(f"Image center: {image_center_x:.0f} px")
                    print(f"Offset from center: {bbox_center_x - image_center_x:.0f} px")
                    print(f"Estimated angle offset: {angle_estimate:.1f}°")
                    print(f"Encoder count delta: {count_delta} counts")
                    print(f"Current encoder count: {pan_motor.encoder_count}")
                    # Target is now printed in set_target_relative()
                    print(f"{'='*70}\n")
    
                    annotated = results[0].plot()
                else:
                    annotated = frame_bgr
            else:
                annotated = frame_bgr
            
            # Draw crosshair at center
            h, w = annotated.shape[:2]
            cv2.line(annotated, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 255), 2)
            cv2.line(annotated, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 255), 2)
            
            # Overlay
            cv2.putText(annotated, "DETECTING - Scanning", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(annotated, f"Encoder: {pan_motor.encoder_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # === PANNING MODE (Centering target) ===
        elif state.mode == "PANNING":
            # Still show HQ camera while panning
            frame = hq_cam.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # DON'T re-detect during panning
            
            # Update motor (no longer sets target, just moves toward it)
            reached = pan_motor.update(tolerance=50)  # <-- CHANGED
            
            if reached:
                print(f"✓ Pan complete!")
                print(f"  Final encoder count: {pan_motor.encoder_count}")
                print(f"  Switching to GS camera...\n")
                state.mode = "TRACKING"
                state.last_detection_time = current_time
                pan_motor.stop()
            else:
                # Print progress every 10 frames
                if state.frame_count % 10 == 0:
                    error = pan_motor.target_count - pan_motor.encoder_count
                    print(f"Panning... current={pan_motor.encoder_count}, "
                        f"target={pan_motor.target_count}, error={error}")
            
            # Draw crosshair
            h, w = frame_bgr.shape[:2]
            cv2.line(frame_bgr, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 2)
            cv2.line(frame_bgr, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 2)
            
            annotated = frame_bgr
            
            # Overlay
            error = pan_motor.target_count - pan_motor.encoder_count
            cv2.putText(annotated, "PANNING - Centering", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.putText(annotated, f"Current: {pan_motor.encoder_count} | Target: {pan_motor.target_count}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.putText(annotated, f"Error: {error} counts", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        # === TRACKING MODE (GS Camera - Narrow FOV) ===
        elif state.mode == "TRACKING":
            frame = gs_cam.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Run detection every frame
            results = model(frame_bgr, conf=0.4, verbose=False, imgsz=416)
            
            if len(results[0].boxes) > 0:
                state.last_detection_time = current_time
                
                # Get target position
                box = results[0].boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox_center_x = (x1 + x2) / 2
                
                # Small corrections - even MORE conservative
                count_delta, angle_offset = pixel_to_encoder_counts(
                    bbox_center_x, 640, GS_FOV_HORIZONTAL, gain=0.2
                )
                
                # Only adjust if significantly off center
                if abs(count_delta) > 20:
                    pan_motor.set_target_relative(count_delta)  # <-- CHANGED
                    
                    if state.frame_count % 10 == 0:
                        print(f"Tracking adjust: {count_delta} counts (angle: {angle_offset:.1f}°)")
                
                # Always update motor
                pan_motor.update(tolerance=20)  # <-- CHANGED
                
                annotated = results[0].plot()
            else:
                annotated = frame_bgr
                pan_motor.stop()
                
                # Lost target - switch back
                if current_time - state.last_detection_time > 3.0:
                    print(f"\n🔍 Lost target. Returning to detection mode\n")
                    state.mode = "DETECTING"
            
            # Draw crosshair
            h, w = annotated.shape[:2]
            cv2.line(annotated, (w//2 - 15, h//2), (w//2 + 15, h//2), (0, 255, 0), 2)
            cv2.line(annotated, (w//2, h//2 - 15), (w//2, h//2 + 15), (0, 255, 0), 2)
            
            # Overlay
            cv2.putText(annotated, "TRACKING - Target locked", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(annotated, f"Encoder: {pan_motor.encoder_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Common overlays
        cv2.putText(annotated, f"Frame: {state.frame_count} | Det: {state.detection_count}", 
                   (10, annotated.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Encode and yield
        ret, buffer = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        if not ret:
            continue
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + 
               buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    html = """
    <html>
    <head>
        <title>PTDTS Pan Debug</title>
        <style>
            body {background:#1a1a1a;text-align:center;color:#fff;font-family:monospace;}
            h1 {color:#0f0;}
            img {max-width:95%;border:3px solid #0f0;margin:20px;}
            .status {background:#222;padding:20px;max-width:800px;margin:20px auto;border-radius:10px;border:1px solid #0f0;}
        </style>
    </head>
    <body>
        <h1>🎯 PTDTS Pan Debug Mode</h1>
        <div class="status">
            <p><strong>Debug Features:</strong></p>
            <p>✓ 30% gain reduction (moves less)</p>
            <p>✓ No re-detection during panning (no oscillation)</p>
            <p>✓ Encoder count display</p>
            <p>✓ Detailed terminal logging</p>
            <p><strong>Check terminal for debug output!</strong></p>
        </div>
        <img src="/video_feed">
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("\n" + "="*70)
    print("🎯 PTDTS Pan Debug Mode")
    print("="*70)
    print("\n✓ Reduced gain: 30% (prevents overshooting)")
    print("✓ No re-detection during pan (prevents oscillation)")
    print("✓ Detailed logging enabled")
    print("\nWatch terminal for detailed debug output")
    print("Open: http://<your-pi-ip>:5000")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        print("\nStopping motor...")
        pan_motor.stop()