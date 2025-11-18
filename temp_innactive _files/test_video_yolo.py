#!/usr/bin/env python3
"""
PTDTS Improved Tracking System - Fixed for DRV8874 Motor Driver
Corrections:
- Increased PID gains for better response
- Higher minimum speed to overcome static friction
- Less aggressive speed shaping
- Better velocity calculation
- Added motor startup boost
"""

import time
import threading
import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify, request
from flask_cors import CORS
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import PWMOutputDevice, Button, Device
from gpiozero.pins.lgpio import LGPIOFactory
from collections import deque
import os

# Configure GPIO
Device.pin_factory = LGPIOFactory()

class ImprovedMotorController:
    """Motor controller with proper PID for DRV8874 driver"""
    
    def __init__(self):
        # Motor pins - DRV8874 in PWM/PWM (IN/IN) mode
        # IMPORTANT: PMODE and SLEEP must be tied to VDD!
        self.motor_in1 = PWMOutputDevice(27, frequency=1000)
        self.motor_in2 = PWMOutputDevice(22, frequency=1000)
        
        # Encoder pins
        self.encoder_a = Button(17, pull_up=True)
        self.encoder_b = Button(4, pull_up=True)
        
        # Encoder state with thread safety
        self.encoder_count = 0
        self.encoder_lock = threading.Lock()
        self.last_a = False
        self.last_b = False
        
        # Calibration
        self.COUNTS_PER_REV = 8043
        self.counts_per_degree = self.COUNTS_PER_REV / 360.0
        
        # Position state
        self.current_position = 0.0  # degrees
        self.target_position = 0.0
        self.last_position = 0.0  # For velocity calculation
        self.velocity = 0.0  # degrees/sec
        
        # PID controller parameters - BALANCED for smooth tracking
        self.kp = 0.030  # Proportional gain (reduced from 0.050 to prevent overshoot)
        self.ki = 0.001  # Integral gain (reduced)
        self.kd = 0.020  # Derivative gain (increased from 0.015 for better damping)
        
        # PID state
        self.integral = 0.0
        self.last_time = time.time()
        
        # Motion parameters - BALANCED for smooth performance
        self.max_speed = 0.70  # Maximum motor speed (reduced from 0.85)
        self.min_speed = 0.15  # Minimum speed to overcome friction
        self.startup_boost = 0.22  # Extra boost when starting from stop (reduced)
        self.position_tolerance = 4.0  # Acceptable position error (increased from 3.0)
        self.velocity_threshold = 1.5  # degrees/sec to consider "stopped"
        
        # Anti-windup for integral
        self.integral_max = 30.0
        
        # Smoothing
        self.position_filter = deque(maxlen=3)
        
        # Motion state
        self.is_moving = False
        self.stopped_time = time.time()
        
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
        print(f"  DRV8874 PWM/PWM mode on pins 27 & 22")
        print(f"  ⚠ Ensure PMODE and SLEEP are tied to VDD!")
        
    def _encoder_callback(self):
        """Quadrature encoder decoding with thread safety"""
        a = self.encoder_a.is_pressed
        b = self.encoder_b.is_pressed
        
        delta = 0
        
        if a != self.last_a:
            if a == b:
                delta = 1
            else:
                delta = -1
        
        if b != self.last_b:
            if a != b:
                delta += 1
            else:
                delta -= 1
        
        if delta != 0:
            with self.encoder_lock:
                self.encoder_count += delta
                
        self.last_a = a
        self.last_b = b
        
    def _control_loop(self):
        """PID control loop optimized for DRV8874"""
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_time
            
            if dt >= 0.05:  # 200Hz control loop (increased from 100Hz)
                # Update position with thread safety
                with self.encoder_lock:
                    raw_position = self.encoder_count / self.counts_per_degree
                
                # Apply filtering
                self.position_filter.append(raw_position)
                self.current_position = np.mean(self.position_filter)
                
                # Calculate velocity correctly
                if dt > 0:
                    self.velocity = (self.current_position - self.last_position) / dt
                
                # Calculate error
                error = self.target_position - self.current_position
                
                # Wrap error to [-180, 180]
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                
                # Dead zone to prevent hunting
                if abs(error) < self.position_tolerance and abs(self.velocity) < self.velocity_threshold:
                    if self.is_moving:
                        self.stop()
                        self.integral = 0  # Reset integral when in position
                        self.is_moving = False
                        self.stopped_time = current_time
                else:
                    # PID calculation
                    # Proportional term
                    p_term = self.kp * error
                    
                    # Integral term with anti-windup
                    self.integral += error * dt
                    self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
                    i_term = self.ki * self.integral
                    
                    # Derivative term (using velocity for smoother response)
                    d_term = -self.kd * self.velocity
                    
                    # Total control output
                    control_output = p_term + i_term + d_term
                    
                    # IMPROVED speed shaping - less aggressive
                    # Only reduce speed within 10 degrees (reduced from 15)
                    if abs(error) < 10:
                        distance_factor = abs(error) / 10.0
                        # Don't reduce below 0.7 of calculated output (increased from 0.6)
                        distance_factor = max(0.7, distance_factor)
                        control_output *= distance_factor
                    
                    # Apply startup boost if just starting from stop
                    time_since_stop = current_time - self.stopped_time
                    if not self.is_moving and time_since_stop > 0.02:  # 20ms after stop (reduced from 50ms)
                        if abs(error) > self.position_tolerance:
                            # Add startup boost to overcome static friction
                            boost_sign = np.sign(control_output)
                            control_output = boost_sign * max(abs(control_output), self.startup_boost)
                    
                    # Apply minimum speed threshold
                    if abs(control_output) < self.min_speed and abs(error) > self.position_tolerance:
                        control_output = self.min_speed * np.sign(control_output)
                    
                    # Limit to max speed
                    control_output = np.clip(control_output, -self.max_speed, self.max_speed)
                    
                    # Set motor speed
                    self.set_motor_speed(control_output)
                    self.is_moving = True
                
                self.last_position = self.current_position
                self.last_time = current_time
                
            time.sleep(0.001)
    
    def set_motor_speed(self, speed):
        """
        Set motor speed for DRV8874 in PWM/PWM (IN/IN) mode
        Forward:  IN1=LOW,  IN2=PWM
        Reverse:  IN1=PWM,  IN2=LOW
        Brake:    IN1=LOW,  IN2=LOW
        """
        if speed > 0:
            # Forward
            self.motor_in1.value = 0
            self.motor_in2.value = 0
        elif speed < 0:
            # Reverse
            self.motor_in1.value = 0
            self.motor_in2.value = 0
        else:
            self.stop()
    
    def stop(self):
        """Stop motor - both pins low for brake"""
        self.motor_in1.value = 0
        self.motor_in2.value = 0
    
    def move_to(self, target_degrees, wait=False, timeout=5.0):
        """Move to target position with optional waiting"""
        self.target_position = target_degrees
        self.integral = 0  # Reset integral for new move
        
        if wait:
            start_time = time.time()
            while self.running:
                error = abs(self.target_position - self.current_position)
                if error < self.position_tolerance and abs(self.velocity) < self.velocity_threshold:
                    return True
                if time.time() - start_time > timeout:
                    print(f"⚠ Move timeout at {self.current_position:.1f}°")
                    return False
                time.sleep(0.01)
        return True
    
    def home(self):
        """Return to home position smoothly"""
        print(f"\n⌂ Homing from {self.current_position:.1f}°")
        
        # Use gentler PID settings for homing
        old_kp = self.kp
        old_max = self.max_speed
        self.kp = 0.025  # Gentler proportional gain for homing
        self.max_speed = 0.5  # Lower max speed for homing
        
        self.move_to(0, wait=True, timeout=10)
        
        # Restore normal PID
        self.kp = old_kp
        self.max_speed = old_max
        
        print(f"✓ Home at {self.current_position:.1f}°")
    
    def get_position(self):
        """Get current position in degrees"""
        return self.current_position
    
    def get_status(self):
        """Get detailed status"""
        return {
            'position': self.current_position,
            'target': self.target_position,
            'velocity': self.velocity,
            'error': self.target_position - self.current_position,
            'is_moving': self.is_moving
        }
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.stop()
        self.motor_in1.close()
        self.motor_in2.close()


class ImprovedTracker:
    """Object tracking with improved motor control"""
    
    def __init__(self):
        print("Initializing tracking system...")
        
        # DIRECTION CORRECTION - Set to -1 if motor moves away from target
        self.direction_multiplier = 1  # Change to -1 if direction is inverted
        print(f"Direction multiplier: {self.direction_multiplier} ({'NORMAL' if self.direction_multiplier == 1 else 'INVERTED'})")
        
        # Initialize motor
        self.motor = ImprovedMotorController()
        
        # Target tracking
        self.estimated_target = 0.0  # Track where we think target is
        
        # Initialize camera
        print("Starting camera...")
        self.camera = Picamera2(1)
        config = self.camera.create_preview_configuration(
            main={"size": (1280, 960), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()
        time.sleep(2)
        print("✓ Camera ready")
        
        # Load YOLO model
        print("Loading UAV-tuned YOLO model...")
        model_path = '/home/ptdts/ptdts_framework/models/best.pt'
        self.model = YOLO(model_path)
        print("✓ Model loaded")
        
        # Tracking state
        self.mode = "DETECTING"
        self.last_detection_time = 0
        self.no_detection_count = 0
        self.detection_timeout = 2.0  # seconds
        
        # Detection smoothing
        self.detection_buffer = deque(maxlen=3)  # Reduced from 5 for faster response
        self.confidence_threshold = 0.40  # Lowered from 0.45 for better detection
        
        # Auto-home timer
        self.last_activity_time = time.time()
        self.auto_home_delay = 10.0  # seconds (increased from 5.0 to give more time)
        
        # Frame processing
        self.frame_count = 0
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Start tracking thread
        self.running = True
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()
        
        print("✓ Tracker initialized")
    
    def _process_detection(self, results):
        """Process YOLO detection with smoothing"""
        if len(results) > 0 and results[0].boxes is not None and len(results[0].boxes) > 0:
            # Find highest confidence detection
            best_conf = 0
            best_box = None
            
            for box in results[0].boxes:
                if box.conf[0] > best_conf and box.conf[0] > self.confidence_threshold:
                    best_conf = box.conf[0]
                    best_box = box
            
            if best_box is not None:
                x1, y1, x2, y2 = best_box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                
                # Convert to angle offset
                frame_width = 640
                fov = 62  # degrees (typical for Pi Camera V2)
                pixel_offset = cx - frame_width / 2
                
                # INCREASED gain for more responsive tracking
                gain = 0.8  # Increased from 0.12 - much more responsive!
                angle_offset = (pixel_offset / frame_width) * fov * gain
                
                # Return offset, confidence, AND bounding box coordinates
                return angle_offset, best_conf, (int(x1), int(y1), int(x2), int(y2))
        return None, 0, None
    
    def _tracking_loop(self):
        """Main tracking loop with improved logic"""
        print("\n🔍 TRACKING LOOP STARTED - Watching for detections...")
        detection_frame_count = 0
        
        while self.running:
            try:
                # Check for auto-home
                if time.time() - self.last_activity_time > self.auto_home_delay:
                    if abs(self.motor.get_position()) > 5:
                        self.motor.home()
                    self.last_activity_time = time.time()
                
                # Get frame
                frame = self.camera.capture_array()
                
                # Process detection
                results = self.model(frame, verbose=False, conf=self.confidence_threshold)
                angle_offset, confidence, bbox = self._process_detection(results)
                
                # DEBUG: Print detection status every 60 frames (1 second at 60fps)
                if self.frame_count % 60 == 0:
                    if angle_offset is not None:
                        print(f"[DEBUG] Detection found! Offset: {angle_offset:.2f}°, Conf: {confidence:.2f}")
                    else:
                        print(f"[DEBUG] No detection (frame {self.frame_count})")
                
                if angle_offset is not None:
                    # Detection found
                    detection_frame_count += 1
                    
                    # Apply direction correction to offset
                    corrected_offset = angle_offset * self.direction_multiplier
                    
                    self.detection_buffer.append(corrected_offset)
                    self.last_detection_time = time.time()
                    self.last_activity_time = time.time()
                    
                    # Use median of recent detections for stability
                    if len(self.detection_buffer) >= 3:
                        avg_offset = np.median(self.detection_buffer)
                        
                        if self.mode == "DETECTING":
                            print(f"\n{'='*60}")
                            print(f"🎯 TARGET ACQUIRED!")
                            print(f"  Offset: {avg_offset:.1f}° (conf: {confidence:.2f})")
                            print(f"  Detection count: {detection_frame_count}")
                            print(f"{'='*60}\n")
                            self.mode = "TRACKING"
                            # Initialize estimated target
                            self.estimated_target = self.motor.get_position()
                        
                        # Update estimated target position with MODERATE smoothing
                        # Balance between responsiveness and stability
                        self.estimated_target += avg_offset * 0.4  # Reduced from 0.6 to prevent delay
                        
                        # Get motor status
                        status = self.motor.get_status()
                        current_pos = status['position']
                        current_target = status['target']
                        
                        # Calculate if we need to update the motor target
                        target_error = abs(self.estimated_target - current_target)
                        
                        # More responsive updates to reduce delay
                        if abs(avg_offset) > 1.0:
                            should_update = False
                            
                            if not status['is_moving']:
                                # Motor stopped, update if error > 1.0° (reduced for faster response)
                                should_update = target_error > 1.0
                            else:
                                # Motor moving, update if error > 2.5° (reduced from 3.0)
                                should_update = target_error > 2.5
                            
                            if should_update:
                                if self.frame_count % 20 == 0:  # ~3x per second at 60fps
                                    print(f"[TRACK] Estimated: {self.estimated_target:.1f}° | Motor: {current_pos:.1f}° → Target: {self.estimated_target:.1f}°")
                                self.motor.move_to(self.estimated_target)
                            elif self.frame_count % 60 == 0:  # Once per second
                                print(f"[HOLD] Target: {current_target:.1f}° | Estimated: {self.estimated_target:.1f}° (diff {target_error:.1f}°)")
                            
                            # Status output
                            if self.frame_count % 60 == 0:  # Once per second
                                print(f"  Motor: Pos={current_pos:.1f}° → {current_target:.1f}° | Vel={status['velocity']:.1f}°/s")
                    
                    self.no_detection_count = 0
                else:
                    # No detection
                    self.no_detection_count += 1
                    
                    if self.mode == "TRACKING" and self.no_detection_count > 90:  # ~1.5 seconds at 60fps (increased from 60)
                        print("🔍 Target lost - DETECTING")
                        self.mode = "DETECTING"
                        self.detection_buffer.clear()
                        self.estimated_target = self.motor.get_position()  # Reset to current position
                        detection_frame_count = 0
                
                # Store frame for streaming
                with self.frame_lock:
                    # Convert RGB to BGR for cv2
                    display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                    # Draw bounding box if detection found
                    if bbox is not None:
                        x1, y1, x2, y2 = bbox
                        # Draw green box around detected object
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                        # Draw confidence label
                        label = f"Drone {confidence:.2f}"
                        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(display_frame, (x1, y1 - label_size[1] - 10), 
                                    (x1 + label_size[0], y1), (0, 255, 0), -1)
                        cv2.putText(display_frame, label, (x1, y1 - 5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    
                    # Add text overlays
                    if angle_offset is not None:
                        cv2.putText(display_frame, f"Offset: {angle_offset:.1f}deg", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    status = self.motor.get_status()
                    cv2.putText(display_frame, f"Pos: {status['position']:.1f}deg → {status['target']:.1f}deg", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(display_frame, f"Mode: {self.mode} | Vel: {status['velocity']:.1f}deg/s", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Draw center crosshair
                    h, w = display_frame.shape[:2]
                    cv2.line(display_frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 255), 2)
                    cv2.line(display_frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 255), 2)
                    
                    self.current_frame = display_frame
                
                self.frame_count += 1
                time.sleep(0.016)  # ~60 fps (increased from 30 fps)
                
            except Exception as e:
                print(f"Error in tracking loop: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def get_frame(self):
        """Get current frame for streaming"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
        return None
    
    def stop(self):
        """Stop tracking system"""
        print("Stopping tracker...")
        self.running = False
        time.sleep(0.5)
        self.motor.cleanup()
        self.camera.stop()

# Flask app for web interface
app = Flask(__name__)
CORS(app)
tracker = None

@app.route('/')
def index():
    return render_template('improved_tracker.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global tracker
        while True:
            if tracker:
                frame = tracker.get_frame()
                if frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    if tracker:
        status = tracker.motor.get_status()
        return jsonify({
            'position': status['position'],
            'target': status['target'],
            'mode': tracker.mode,
            'velocity': status['velocity'],
            'error': status['error'],
            'is_moving': status['is_moving']
        })
    return jsonify({'error': 'Not initialized'})

@app.route('/api/home')
def go_home():
    if tracker:
        tracker.motor.home()
        return jsonify({'success': True})
    return jsonify({'error': 'Not initialized'})

@app.route('/api/test_move')
def test_move():
    """Test motor movement with a simple command"""
    if tracker:
        angle = request.args.get('angle', 30, type=float)
        print(f"\n[TEST] Manual move command to {angle}°")
        tracker.motor.move_to(angle)
        return jsonify({
            'success': True, 
            'target': angle,
            'current': tracker.motor.get_position()
        })
    return jsonify({'error': 'Not initialized'})

def main():
    global tracker
    
    print("=" * 60)
    print("🎯 PTDTS TRACKING SYSTEM - DRV8874 OPTIMIZED")
    print("=" * 60)
    print("\n✓ Increased PID gains for 12V geared motor")
    print("✓ Higher minimum speed (18%) to overcome friction")
    print("✓ Startup boost for static friction")
    print("✓ Optimized for DRV8874 PWM/PWM mode")
    print("\n⚠ IMPORTANT: Verify PMODE and SLEEP are tied to VDD!")
    print("\nStarting system...")
    
    try:
        tracker = ImprovedTracker()
        
        print(f"\nSystem ready!")
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
        import traceback
        traceback.print_exc()
        if tracker:
            tracker.stop()

if __name__ == "__main__":
    main()