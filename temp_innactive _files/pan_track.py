#!/usr/bin/env python3
"""
PTDTS Dual-Camera Optimized Tracking System
HQ: Wide detection @ 5 FPS | GS: Fast tracking @ 30 FPS
"""

import time
import threading
import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify
from flask_cors import CORS
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import PWMOutputDevice, Button, Device
from gpiozero.pins.lgpio import LGPIOFactory
from collections import deque
import os

Device.pin_factory = LGPIOFactory()

class ImprovedMotorController:
    """Motor controller with proper PID and anti-oscillation"""
    
    def __init__(self):
        # Motor pins
        self.motor_in1 = PWMOutputDevice(27, frequency=1000)
        self.motor_in2 = PWMOutputDevice(22, frequency=1000)
        
        # Encoder pins
        self.encoder_a = Button(17, pull_up=True)
        self.encoder_b = Button(4, pull_up=True)
        
        # Encoder state
        self.encoder_count = 0
        self.last_a = False
        self.last_b = False
        
        # Calibration
        self.COUNTS_PER_REV = 8043
        self.counts_per_degree = self.COUNTS_PER_REV / 360.0
        
        # Position state
        self.current_position = 0.0
        self.target_position = 0.0
        self.velocity = 0.0
        
        # PID parameters - tuned for smooth response
        self.kp = 0.50
        self.ki = 0.015
        self.kd = 0.008
        
        # PID state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        
        # Motion parameters
        self.max_speed = 0.4
        self.min_speed = 0.15
        self.position_tolerance = 5.0
        self.velocity_threshold = 2.0
        self.integral_max = 50.0
        
        # Smoothing
        self.position_filter = deque(maxlen=3)
        
        # Setup encoder callbacks
        self.encoder_a.when_pressed = self._encoder_callback
        self.encoder_a.when_released = self._encoder_callback
        self.encoder_b.when_pressed = self._encoder_callback
        self.encoder_b.when_released = self._encoder_callback
        
        # Start control thread
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        print(f"✓ Motor initialized - {self.counts_per_degree:.2f} counts/degree")
        
    def _encoder_callback(self):
        """Quadrature encoder decoding"""
        a = self.encoder_a.is_pressed
        b = self.encoder_b.is_pressed
        
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
        
    def _control_loop(self):
        """PID control loop"""
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_time
            
            if dt >= 0.01:  # 100Hz control
                raw_position = self.encoder_count / self.counts_per_degree
                self.position_filter.append(raw_position)
                self.current_position = np.mean(self.position_filter)
                
                if dt > 0:
                    self.velocity = (self.current_position - self.last_error) / dt
                
                error = self.target_position - self.current_position
                
                # Wrap error
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                
                # Dead zone
                if abs(error) < self.position_tolerance and abs(self.velocity) < self.velocity_threshold:
                    self.stop()
                    self.integral = 0
                else:
                    # PID
                    p_term = self.kp * error
                    
                    self.integral += error * dt
                    self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
                    i_term = self.ki * self.integral
                    
                    d_term = -self.kd * self.velocity
                    
                    control_output = p_term + i_term + d_term
                    
                    # Speed shaping
                    distance_factor = min(1.0, abs(error) / 30.0)
                    control_output *= distance_factor
                    
                    if abs(control_output) < self.min_speed and abs(error) > self.position_tolerance:
                        control_output = self.min_speed * np.sign(control_output)
                    
                    control_output = np.clip(control_output, -self.max_speed, self.max_speed)
                    
                    self.set_motor_speed(control_output)
                
                self.last_error = self.current_position
                self.last_time = current_time
                
            time.sleep(0.001)
    
    def set_motor_speed(self, speed):
        """Set motor speed with direction"""
        if speed > 0:
            self.motor_in1.value = 0
            self.motor_in2.value = abs(speed)
        elif speed < 0:
            self.motor_in1.value = abs(speed)
            self.motor_in2.value = 0
        else:
            self.stop()
    
    def stop(self):
        self.motor_in1.value = 0
        self.motor_in2.value = 0
    
    def move_to(self, target_degrees):
        """Move to target position"""
        self.target_position = target_degrees
        self.integral = 0
    
    def home(self):
        """Return to home"""
        print(f"\n⌂ Homing from {self.current_position:.1f}°")
        old_kp = self.kp
        self.kp = 0.012
        
        self.target_position = 0
        start_time = time.time()
        while self.running and abs(self.current_position) > self.position_tolerance:
            if time.time() - start_time > 10:
                break
            time.sleep(0.01)
        
        self.kp = old_kp
        print(f"✓ Home at {self.current_position:.1f}°")
    
    def get_position(self):
        return self.current_position
    
    def cleanup(self):
        self.running = False
        self.stop()
        time.sleep(0.1)
        self.motor_in1.close()
        self.motor_in2.close()


class DualCameraTracker:
    """Optimized dual-camera tracking system"""
    
    def __init__(self):
        # Initialize motor
        self.motor = ImprovedMotorController()
        
        # Initialize HQ Camera (Detection)
        print("Initializing HQ Camera (Detection)...")
        self.hq_cam = Picamera2(1)
        hq_config = self.hq_cam.create_video_configuration(
            main={"size": (1280, 720), "format": "RGB888"},
            controls={"FrameRate": 5}  # 5 FPS for detection
        )
        self.hq_cam.configure(hq_config)
        self.hq_cam.start()
        print("✓ HQ Camera: 1280x720 @ 5 FPS")
        
        # Initialize GS Camera (Tracking)
        print("Initializing GS Camera (Tracking)...")
        self.gs_cam = Picamera2(0)
        gs_config = self.gs_cam.create_video_configuration(
            main={"size": (600, 480), "format": "RGB888"},  # Low res, high speed
            controls={"FrameRate": 30}
        )
        self.gs_cam.configure(gs_config)
        self.gs_cam.start()
        print("✓ GS Camera: 416x320 @ 30 FPS")
        
        time.sleep(2)
        
        # Load YOLO model
        print("Loading YOLO model...")
        model_path = '/home/ptdts/ptdts_framework/models/yolov11n-UAV-finetune_ncnn_model'
        if not os.path.exists(model_path):
            model_path = '/home/ptdts/ptdts_framework/models/yolo11n.pt'
        self.model = YOLO(model_path)
        print("✓ YOLO model loaded")
        
        # Camera parameters
        self.HQ_FOV = 60.0  # degrees
        self.GS_FOV = 45.0  # degrees
        self.CENTERING_GAIN = 0.15
        
        # System state
        self.mode = "DETECTING"  # DETECTING, PANNING, TRACKING
        self.last_detection_time = 0
        self.switch_to_tracking_time = 0
        
        # Detection smoothing
        self.detection_buffer = deque(maxlen=2)  # 2 samples for HQ
        self.tracking_buffer = deque(maxlen=3)   # 3 samples for GS
        
        # Auto-home
        self.last_activity_time = time.time()
        self.auto_home_delay = 5.0
        
        # Performance tracking
        self.frame_count = 0
        self.detection_count = 0
        self.hq_fps = 0
        self.gs_fps = 0
        self.last_fps_time = time.time()
        self.fps_frames = 0
        
        # Frame for display
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Start threads
        self.running = True
        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.detection_thread.start()
        self.tracking_thread.start()
        
        print("✓ Tracker initialized")
    
    def _process_hq_detection(self, frame):
        """Process HQ camera detection"""
        results = self.model(frame, verbose=False, conf=0.5, imgsz=640)
        
        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            if box.conf[0] > 0.5:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                
                # Convert to angle
                frame_width = 1280
                pixel_offset = cx - frame_width / 2
                angle_offset = (pixel_offset / frame_width) * self.HQ_FOV * self.CENTERING_GAIN
                
                return angle_offset, (x1, y1, x2, y2)
        return None, None
    
    def _process_gs_detection(self, frame):
        """Process GS camera tracking"""
        results = self.model(frame, verbose=False, conf=0.3, imgsz=320)  # Lower conf, smaller size
        
        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            if box.conf[0] > 0.3:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                
                # Convert to angle
                frame_width = 416
                pixel_offset = cx - frame_width / 2
                angle_offset = (pixel_offset / frame_width) * self.GS_FOV * 0.2  # Gentler for tracking
                
                return angle_offset, (x1, y1, x2, y2)
        return None, None
    
    def _detection_loop(self):
        """HQ camera detection loop @ 5 FPS"""
        while self.running:
            try:
                if self.mode == "DETECTING":
                    # Capture from HQ camera
                    frame = self.hq_cam.capture_array()
                    
                    # Run detection (every frame = 5 FPS)
                    angle_offset, bbox = self._process_hq_detection(frame)
                    
                    if angle_offset is not None:
                        self.detection_buffer.append(angle_offset)
                        
                        if len(self.detection_buffer) >= 2:
                            avg_offset = np.mean(self.detection_buffer)
                            
                            print(f"\n🎯 DETECTED #{self.detection_count + 1}")
                            print(f"  Angle: {avg_offset:.1f}°")
                            print(f"  Samples: {[f'{x:.1f}' for x in self.detection_buffer]}")
                            
                            # Command pan
                            target = self.motor.get_position() + avg_offset
                            self.motor.move_to(target)
                            
                            # Switch to panning mode
                            self.mode = "PANNING"
                            self.detection_count += 1
                            self.detection_buffer.clear()
                    
                    # Update display frame
                    with self.frame_lock:
                        display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        
                        # Draw detection box
                        if bbox:
                            x1, y1, x2, y2 = bbox
                            cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        
                        # Crosshair
                        h, w = display.shape[:2]
                        cv2.line(display, (w//2-25, h//2), (w//2+25, h//2), (0, 255, 255), 2)
                        cv2.line(display, (w//2, h//2-25), (w//2, h//2+25), (0, 255, 255), 2)
                        
                        # Status
                        cv2.putText(display, "DETECTING - HQ Camera", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                        cv2.putText(display, f"Samples: {len(self.detection_buffer)}/2", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        cv2.putText(display, f"Pos: {self.motor.get_position():.1f} deg", (10, 90),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                        
                        self.current_frame = display
                    
                    time.sleep(0.2)  # 5 FPS
                
                elif self.mode == "PANNING":
                    # Wait for pan to complete
                    error = abs(self.motor.target_position - self.motor.get_position())
                    
                    if error < self.motor.position_tolerance:
                        print("✓ Centered - Switching to GS camera")
                        self.switch_to_tracking_time = time.time() + 0.5
                        self.mode = "SETTLING"
                    
                    time.sleep(0.05)
                
                elif self.mode == "SETTLING":
                    # Settling delay before switching cameras
                    if time.time() >= self.switch_to_tracking_time:
                        print("→ TRACKING mode (GS camera)")
                        self.mode = "TRACKING"
                        self.last_detection_time = time.time()
                        self.last_activity_time = time.time()
                    
                    time.sleep(0.05)
                
                else:
                    # In TRACKING mode, just sleep
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Detection loop error: {e}")
                time.sleep(0.1)
    
    def _tracking_loop(self):
        """GS camera tracking loop @ 30 FPS"""
        gs_frame_count = 0
        
        while self.running:
            try:
                # Auto-home check
                if time.time() - self.last_activity_time > self.auto_home_delay:
                    if abs(self.motor.get_position()) > 5 and self.mode == "DETECTING":
                        self.motor.home()
                    self.last_activity_time = time.time()
                
                if self.mode == "TRACKING":
                    gs_frame_count += 1
                    
                    # Capture from GS camera
                    frame = self.gs_cam.capture_array()
                    
                    # Run YOLO every 3 frames (still 10 FPS YOLO on 30 FPS camera)
                    angle_offset = None
                    bbox = None
                    
                    if gs_frame_count % 3 == 0:
                        angle_offset, bbox = self._process_gs_detection(frame)
                    
                    if angle_offset is not None:
                        self.tracking_buffer.append(angle_offset)
                        self.last_detection_time = time.time()
                        self.last_activity_time = time.time()
                        
                        if len(self.tracking_buffer) >= 2:
                            avg_offset = np.mean(self.tracking_buffer)
                            
                            # Only move if significant offset
                            if abs(avg_offset) > 3:
                                target = self.motor.get_position() + avg_offset
                                self.motor.move_to(target)
                                
                                if gs_frame_count % 30 == 0:
                                    print(f"  Tracking: {avg_offset:.1f}°")
                    else:
                        # Check if lost
                        if time.time() - self.last_detection_time > 2.0:
                            print("🔍 Lost target - Returning to DETECTING")
                            self.mode = "DETECTING"
                            self.tracking_buffer.clear()
                    
                    # Update display frame
                    with self.frame_lock:
                        display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        
                        # Draw detection box
                        if bbox:
                            x1, y1, x2, y2 = bbox
                            cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            
                            # Center line
                            cx = int((x1 + x2) / 2)
                            cy = int((y1 + y2) / 2)
                            cv2.line(display, (display.shape[1]//2, display.shape[0]//2), 
                                   (cx, cy), (255, 255, 0), 2)
                        
                        # Crosshair
                        h, w = display.shape[:2]
                        cv2.line(display, (w//2-15, h//2), (w//2+15, h//2), (255, 255, 255), 2)
                        cv2.line(display, (w//2, h//2-15), (w//2, h//2+15), (255, 255, 255), 2)
                        
                        # Status
                        cv2.putText(display, "TRACKING - GS Camera", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(display, f"Pos: {self.motor.get_position():.1f} deg", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        
                        if angle_offset is not None:
                            cv2.putText(display, f"Offset: {angle_offset:.1f} deg", (10, 90),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            cv2.putText(display, "NO DETECTION", (10, 90),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
                        self.current_frame = display
                    
                    time.sleep(0.033)  # ~30 FPS
                else:
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Tracking loop error: {e}")
                time.sleep(0.1)
    
    def get_frame(self):
        """Get current display frame"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
        return None
    
    def stop(self):
        """Stop system"""
        print("Stopping tracker...")
        self.running = False
        time.sleep(0.5)
        self.motor.cleanup()
        self.hq_cam.stop()
        self.gs_cam.stop()


# Flask app
app = Flask(__name__)
CORS(app)
tracker = None

@app.route('/')
def index():
    return render_template('improved_tracker.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if tracker:
                frame = tracker.get_frame()
                if frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.033)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    if tracker:
        return jsonify({
            'position': tracker.motor.get_position(),
            'mode': tracker.mode,
            'velocity': tracker.motor.velocity,
            'detections': tracker.detection_count
        })
    return jsonify({'error': 'Not initialized'})

@app.route('/api/home')
def go_home():
    if tracker:
        tracker.motor.home()
        return jsonify({'success': True})
    return jsonify({'error': 'Not initialized'})

def main():
    global tracker
    
    print("=" * 70)
    print("🎯 PTDTS DUAL-CAMERA OPTIMIZED SYSTEM")
    print("=" * 70)
    print("\n📹 Camera Configuration:")
    print("  HQ Camera (Port 1): 1280x720 @ 5 FPS  → Detection")
    print("  GS Camera (Port 0): 416x320 @ 30 FPS  → Tracking")
    print("\n⚡ Performance:")
    print("  Detection: 5 FPS YOLO on HQ")
    print("  Tracking: 10 FPS YOLO on GS (every 3rd frame)")
    print("\nStarting system...")
    
    try:
        tracker = DualCameraTracker()
        
        print(f"\n✓ System ready!")
        print(f"Open: http://10.233.137.51:5000")
        print("Press Ctrl+C to stop\n")
        
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
        
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        if tracker:
            tracker.stop()
        print("✓ Stopped")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        if tracker:
            tracker.stop()

if __name__ == "__main__":
    main()