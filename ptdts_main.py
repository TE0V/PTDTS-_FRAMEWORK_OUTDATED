#!/usr/bin/env python3
"""
PTDTS Comprehensive Framework
Pan Tilt Drone Detection System - Full Implementation
Author: PTDTS Framework
Date: October 2025
"""

import os
import sys
import time
import json
import threading
import queue
import socket
import numpy as np
from dataclasses import dataclass, asdict
from typing import Tuple, Optional, List, Dict, Any
from datetime import datetime
import logging
from enum import Enum, auto

# GPIO and hardware control
logger = logging.getLogger(__name__)

from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button, Device
try:
    from gpiozero.pins.lgpio import LGPIOFactory
    # Set lgpio as the pin factory for Raspberry Pi 5 compatibility
    Device.pin_factory = LGPIOFactory()
    logger.info("Using lgpio pin factory for GPIO access")
except ImportError:
    logger.warning("lgpio not available, using default pin factory")
    # Will fall back to RPiGPIOFactory on older Pi models

# Camera and vision
import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

# Scientific computing
from scipy.signal import butter, lfilter, find_peaks
from scipy.spatial.transform import Rotation

# Kalman filter for lead prediction
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

# Web interface and communication
from flask import Flask, render_template, jsonify, request, Response
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import zmq

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/ptdts/ptdts.log'),
        logging.StreamHandler()
    ]
)

# System constants
ENCODER_CPR = 1920  # Counts per motor revolution
GEAR_RATIO = 4.5    # 24:108 gear reduction
COUNTS_PER_360 = int(ENCODER_CPR * GEAR_RATIO)  # Counts per system revolution

# ============================================================================
# Data Classes and Enums
# ============================================================================

class SystemState(Enum):
    """System operational states"""
    IDLE = auto()
    SCANNING = auto()
    DETECTING = auto()
    TRACKING = auto()
    ENGAGING = auto()
    ERROR = auto()

class DetectionSource(Enum):
    """Detection source types"""
    ACOUSTIC = auto()
    VISUAL = auto()
    MANUAL = auto()
    EXTERNAL = auto()

@dataclass
class Target:
    """Target information"""
    id: str
    timestamp: float
    source: DetectionSource
    position: Tuple[float, float, float]  # x, y, z in meters
    velocity: Tuple[float, float, float]  # vx, vy, vz in m/s
    confidence: float
    classification: str = "unknown"
    
@dataclass
class SystemConfig:
    """System configuration parameters"""
    # Acoustic settings
    acoustic_enabled: bool = True
    acoustic_threshold: float = 0.6
    acoustic_frequency_min: float = 100.0
    acoustic_frequency_max: float = 500.0
    
    # Visual settings
    visual_enabled: bool = True
    detection_confidence: float = 0.5
    tracking_confidence: float = 0.3
    yolo_model: str = "yolo11n"
    
    # Motion settings
    pan_speed_max: float = 1.0
    tilt_speed_max: float = 1.0
    lead_prediction_enabled: bool = True
    lead_time: float = 1.0  # seconds
    
    # Network settings
    remote_control_enabled: bool = True
    web_port: int = 5000
    odas_port: int = 9000
    zmq_port: int = 5555

# ============================================================================
# Acoustic Detection Module (ODAS Integration)
# ============================================================================

class AcousticDetector:
    """Acoustic detection using ODAS framework"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.running = False
        self.detection_queue = queue.Queue()
        self.odas_socket = None
        self.detection_thread = None
        
        # Bandpass filter coefficients for drone signature
        nyquist = 8000  # Half of 16kHz sampling rate
        low = config.acoustic_frequency_min / nyquist
        high = config.acoustic_frequency_max / nyquist
        self.b, self.a = butter(4, [low, high], btype='band')
        
    def start(self):
        """Start acoustic detection"""
        if not self.config.acoustic_enabled:
            logger.info("Acoustic detection disabled in config")
            return
            
        try:
            # Connect to ODAS output
            self.odas_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.odas_socket.connect(('localhost', self.config.odas_port))
            self.odas_socket.settimeout(1.0)
            
            self.running = True
            self.detection_thread = threading.Thread(target=self._detection_loop)
            self.detection_thread.start()
            
            logger.info("Acoustic detection started")
            
        except Exception as e:
            logger.error(f"Failed to start acoustic detection: {e}")
            
    def _detection_loop(self):
        """Main acoustic detection loop"""
        buffer = ""
        
        while self.running:
            try:
                # Receive ODAS JSON data
                data = self.odas_socket.recv(4096).decode('utf-8')
                buffer += data
                
                # Process complete JSON messages
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line:
                        self._process_odas_message(line)
                        
            except socket.timeout:
                continue
            except Exception as e:
                logger.error(f"Acoustic detection error: {e}")
                
    def _process_odas_message(self, message: str):
        """Process ODAS JSON message"""
        try:
            data = json.loads(message)
            
            # Extract sound source tracking data
            if 'src' in data:
                for source in data['src']:
                    if source['E'] > self.config.acoustic_threshold:
                        # Detected sound source above threshold
                        azimuth = source['azimuth']
                        elevation = source['elevation']
                        energy = source['E']
                        
                        # Check if frequency matches drone signature
                        if self._is_drone_signature(source):
                            detection = {
                                'type': 'acoustic',
                                'azimuth': azimuth,
                                'elevation': elevation,
                                'confidence': energy,
                                'timestamp': time.time()
                            }
                            self.detection_queue.put(detection)
                            logger.info(f"Acoustic detection: Az={azimuth:.1f}°, El={elevation:.1f}°, Conf={energy:.2f}")
                            
        except json.JSONDecodeError:
            logger.warning(f"Invalid ODAS JSON: {message}")
            
    def _is_drone_signature(self, source: dict) -> bool:
        """Check if acoustic signature matches drone characteristics"""
        # TODO: Implement frequency analysis
        # For now, return True if energy is above threshold
        return True
        
    def get_detection(self) -> Optional[dict]:
        """Get latest acoustic detection"""
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return None
            
    def stop(self):
        """Stop acoustic detection"""
        self.running = False
        if self.detection_thread:
            self.detection_thread.join()
        if self.odas_socket:
            self.odas_socket.close()
        logger.info("Acoustic detection stopped")

# ============================================================================
# Motor Control Module with Encoder Feedback
# ============================================================================

class MotorController:
    """Advanced motor control with PID and encoder feedback"""
    
    def __init__(self):
        # Motor driver pins
        self.motor_in1 = PWMOutputDevice(27, frequency=20000)  # Pin 13
        self.motor_in2 = PWMOutputDevice(22, frequency=20000)  # Pin 15
        
        # Encoder pins with pull-up resistors
        self.encoder_a = Button(17, pull_up=True)  # Pin 11
        self.encoder_b = Button(4, pull_up=True)   # Pin 7
        
        # Position tracking
        self.encoder_count = 0
        self.last_encoder_count = 0
        self.position_degrees = 0.0
        self.velocity_dps = 0.0  # degrees per second
        
        # PID controller parameters
        self.kp = 0.8
        self.ki = 0.1
        self.kd = 0.2
        self.integral = 0.0
        self.last_error = 0.0
        
        # Motion profile
        self.target_position = 0.0
        self.max_velocity = 180.0  # degrees per second
        self.max_acceleration = 90.0  # degrees per second^2
        
        # Setup encoder interrupts
        self._setup_encoder()
        
        # Start position update thread
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.start()
        
    def _setup_encoder(self):
        """Setup quadrature encoder decoding"""
        self.last_a = False
        self.last_b = False
        
        self.encoder_a.when_pressed = self._encoder_callback
        self.encoder_a.when_released = self._encoder_callback
        self.encoder_b.when_pressed = self._encoder_callback
        self.encoder_b.when_released = self._encoder_callback
        
    def _encoder_callback(self):
        """Quadrature encoder state machine"""
        a = self.encoder_a.is_pressed
        b = self.encoder_b.is_pressed
        
        # Gray code decoding
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
        
    def _update_loop(self):
        """Update position and velocity estimates"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= 0.01:  # 100Hz update rate
                # Calculate position
                self.position_degrees = (self.encoder_count / COUNTS_PER_360) * 360.0
                self.position_degrees = self.position_degrees % 360.0
                
                # Calculate velocity
                encoder_delta = self.encoder_count - self.last_encoder_count
                self.velocity_dps = (encoder_delta / COUNTS_PER_360) * 360.0 / dt
                
                self.last_encoder_count = self.encoder_count
                last_time = current_time
                
            time.sleep(0.001)
            
    def set_velocity(self, velocity_dps: float):
        """Set motor velocity in degrees per second"""
        # Clamp velocity
        velocity_dps = max(-self.max_velocity, min(self.max_velocity, velocity_dps))
        
        # Convert to motor command (-1 to 1)
        motor_command = velocity_dps / self.max_velocity
        
        if motor_command > 0:
            self.motor_in1.value = abs(motor_command)
            self.motor_in2.value = 0
        elif motor_command < 0:
            self.motor_in1.value = 0
            self.motor_in2.value = abs(motor_command)
        else:
            self.motor_in1.value = 0
            self.motor_in2.value = 0
            
    def move_to_position(self, target_degrees: float, callback=None):
        """Move to target position with PID control"""
        self.target_position = target_degrees % 360
        
        def control_loop():
            while True:
                # Calculate error
                error = self.target_position - self.position_degrees
                
                # Handle wrap-around
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                    
                # Check if reached target
                if abs(error) < 1.0:  # Within 1 degree
                    self.set_velocity(0)
                    if callback:
                        callback()
                    break
                    
                # PID calculation
                self.integral += error * 0.01  # dt = 0.01
                derivative = (error - self.last_error) / 0.01
                
                output = self.kp * error + self.ki * self.integral + self.kd * derivative
                
                self.set_velocity(output)
                self.last_error = error
                
                time.sleep(0.01)
                
        threading.Thread(target=control_loop).start()
        
    def stop(self):
        """Stop motor and cleanup"""
        self.set_velocity(0)
        self.running = False
        if self.update_thread:
            self.update_thread.join()

# ============================================================================
# Servo Control Module
# ============================================================================

class ServoController:
    """Advanced servo control with analog position feedback"""
    
    def __init__(self):
        # Servo control pin
        self.servo = PWMOutputDevice(25, frequency=50)
        
        # Initialize ADS1115 for position feedback
        try:
            from ads1115_reader import ADS1115Reader
            self.adc_reader = ADS1115Reader(channel=0)  # Using channel 0 of ADS1115
            self.has_position_feedback = True
            logger.info("Servo position feedback enabled via ADS1115")
        except Exception as e:
            logger.warning(f"No ADS1115 position feedback available: {e}")
            self.adc_reader = None
            self.has_position_feedback = False
        
        # Position tracking
        self.current_angle = 90.0  # Start centered
        self.target_angle = 90.0
        self.measured_angle = 90.0  # From analog feedback
        self.velocity = 0.0
        
        # Motion limits
        self.min_angle = 0.0
        self.max_angle = 180.0
        self.max_velocity = 180.0  # degrees per second
        
        # Control parameters
        self.use_feedback = self.has_position_feedback
        self.position_tolerance = 2.0  # degrees
        
        # Smooth motion thread
        self.running = True
        self.motion_thread = threading.Thread(target=self._motion_loop)
        self.motion_thread.start()
        
    def _angle_to_duty_cycle(self, angle: float) -> float:
        """Convert angle to PWM duty cycle"""
        pulse_ms = 1.0 + (angle / 180.0)
        duty_cycle = pulse_ms / 20.0
        return duty_cycle
        
    def _motion_loop(self):
        """Smooth motion control loop with position feedback"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= 0.01:  # 100Hz update
                # Read actual position if available
                if self.has_position_feedback and self.adc_reader:
                    try:
                        position_data = self.adc_reader.read_position()
                        self.measured_angle = position_data.angle_degrees
                        
                        # Use measured position for closed-loop control
                        if self.use_feedback:
                            error = self.target_angle - self.measured_angle
                        else:
                            error = self.target_angle - self.current_angle
                    except:
                        # Fallback to open-loop if reading fails
                        error = self.target_angle - self.current_angle
                else:
                    # Open-loop control
                    error = self.target_angle - self.current_angle
                
                if abs(error) > self.position_tolerance:
                    # P controller with velocity limiting
                    self.velocity = error * 2.0  # P gain = 2.0
                    self.velocity = max(-self.max_velocity, min(self.max_velocity, self.velocity))
                    
                    # Update position estimate
                    self.current_angle += self.velocity * dt
                    self.current_angle = max(self.min_angle, min(self.max_angle, self.current_angle))
                    
                    # Set servo position
                    duty_cycle = self._angle_to_duty_cycle(self.current_angle)
                    self.servo.value = duty_cycle
                    
                last_time = current_time
                
            time.sleep(0.001)
            
    def get_position(self) -> tuple[float, float]:
        """Get current position (commanded, measured)"""
        return self.current_angle, self.measured_angle

# ============================================================================
# Kalman Filter for Lead Prediction and Position Calculation
# ============================================================================

class LeadPredictor:
    """Kalman filter-based lead prediction for target tracking"""
    
    def __init__(self):
        # Initialize 6-state Kalman filter (x, y, z, vx, vy, vz)
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        
        # State transition matrix (constant velocity model)
        dt = 0.1  # 10Hz update
        self.kf.F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])
        
        # Measurement matrix (observe position only)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Measurement noise
        self.kf.R = np.eye(3) * 0.1
        
        # Process noise
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1, block_size=3)
        
        # Initial state
        self.kf.x = np.zeros(6)
        self.kf.P = np.eye(6) * 100
        
    def update(self, position: Tuple[float, float, float]):
        """Update filter with new measurement"""
        measurement = np.array(position)
        self.kf.predict()
        self.kf.update(measurement)
        
    def predict_position(self, time_ahead: float) -> Tuple[float, float, float]:
        """Predict target position at future time"""
        # Current state
        x, y, z, vx, vy, vz = self.kf.x
        
        # Predict future position
        future_x = x + vx * time_ahead
        future_y = y + vy * time_ahead
        future_z = z + vz * time_ahead
        
        return (future_x, future_y, future_z)
        
    def get_velocity(self) -> Tuple[float, float, float]:
        """Get current velocity estimate"""
        return tuple(self.kf.x[3:6])

class PositionCalculator:
    """Enhanced position and velocity calculation for tracked targets"""
    
    def __init__(self, camera_config: dict):
        # Camera parameters
        self.hq_fov_h = camera_config.get('hq_camera_fov', {}).get('h', 60.0)
        self.hq_fov_v = camera_config.get('hq_camera_fov', {}).get('v', 40.0)
        self.gs_fov_h = camera_config.get('gs_camera_fov', {}).get('h', 45.0)
        self.gs_fov_v = camera_config.get('gs_camera_fov', {}).get('v', 34.0)
        
        # Kalman filter for smooth velocity estimation
        self.position_filter = KalmanFilter(dim_x=6, dim_z=3)
        self._setup_kalman_filter()
        
    def _setup_kalman_filter(self):
        """Setup Kalman filter for position/velocity estimation"""
        dt = 0.1
        # State transition (constant velocity model)
        self.position_filter.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix
        self.position_filter.H = np.eye(3, 6)
        
        # Process noise
        self.position_filter.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1, block_size=3)
        
        # Measurement noise
        self.position_filter.R = np.eye(3) * 0.5
        
        # Initial state
        self.position_filter.x = np.zeros(6)
        self.position_filter.P = np.eye(6) * 100
        
    def calculate_3d_position(
        self,
        pan_angle: float,
        tilt_angle: float,
        pixel_x: float,
        pixel_y: float,
        frame_width: int,
        frame_height: int,
        estimated_range: float = 50.0,
        camera_mode: str = 'hq'
    ) -> Tuple[float, float, float]:
        """
        Calculate 3D position from pan/tilt and pixel coordinates
        
        Returns:
            (x, y, z) position in meters (ENU coordinate system)
        """
        # Select FOV based on camera
        if camera_mode == 'hq':
            fov_h = self.hq_fov_h
            fov_v = self.hq_fov_v
        else:
            fov_h = self.gs_fov_h
            fov_v = self.gs_fov_v
            
        # Convert pixel to angular offset from camera center
        pixel_offset_h = (pixel_x - frame_width/2) / frame_width * fov_h
        pixel_offset_v = (pixel_y - frame_height/2) / frame_height * fov_v
        
        # Total angles (pan/tilt + pixel offset)
        azimuth = pan_angle + pixel_offset_h
        elevation = 90 - (tilt_angle + pixel_offset_v)  # Convert tilt to elevation
        
        # Convert to radians
        az_rad = np.radians(azimuth)
        el_rad = np.radians(elevation)
        
        # Calculate 3D position (assuming East-North-Up coordinate system)
        x = estimated_range * np.cos(el_rad) * np.sin(az_rad)  # East
        y = estimated_range * np.cos(el_rad) * np.cos(az_rad)  # North
        z = estimated_range * np.sin(el_rad)  # Up
        
        return x, y, z
        
    def update_target_state(self, position: Tuple[float, float, float]) -> dict:
        """Update target state with Kalman filter"""
        # Predict
        self.position_filter.predict()
        
        # Update with measurement
        self.position_filter.update(np.array(position))
        
        # Extract state
        state = self.position_filter.x
        
        return {
            'position': tuple(state[:3]),
            'velocity': tuple(state[3:6]),
            'speed_mps': np.linalg.norm(state[3:6]),
            'heading': np.degrees(np.arctan2(state[4], state[3]))  # North = 0°
        }

# ============================================================================
# Camera and Vision Module
# ============================================================================

class VisionSystem:
    """Dual camera vision system with YOLO detection and tracking"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        
        # Initialize cameras (Note: swapped ports per your config)
        self.hq_cam = Picamera2(1)  # HQ on port 1
        self.gs_cam = Picamera2(0)  # GS on port 0
        
        # Configure cameras
        self._configure_cameras()
        
        # Load YOLO models
        self._load_models()
        
        # Camera state
        self.active_camera = 'hq'  # Default to HQ camera
        self.manual_override = False
        self.current_target = None
        self.frame_queue = queue.Queue(maxsize=5)
        
        # Processing thread
        self.running = False
        self.process_thread = None
        
    def set_camera_mode(self, mode: str, manual: bool = False):
        """Switch between HQ and GS cameras"""
        if mode in ['hq', 'gs']:
            self.active_camera = mode
            self.manual_override = manual
            logger.info(f"Camera switched to {mode.upper()} {'(manual)' if manual else '(auto)'}")
            
    def get_active_frame(self) -> np.ndarray:
        """Get frame from currently active camera"""
        if self.active_camera == 'hq':
            return self.hq_cam.capture_array()
        else:
            return self.gs_cam.capture_array()
            
    def _process_loop(self):
        """Main vision processing loop with automatic camera switching"""
        while self.running:
            try:
                # Auto-switch cameras based on target state (unless manual override)
                if not self.manual_override:
                    if self.current_target is None:
                        # No target - use HQ for detection
                        self.active_camera = 'hq'
                    else:
                        # Target acquired - use GS for tracking
                        self.active_camera = 'gs'
                
                # Get frame from active camera
                frame = self.get_active_frame()
                
                # Process based on mode
                if self.active_camera == 'hq':
                    detections = self._detect_drones(frame)
                    if detections and not self.current_target:
                        self.current_target = detections[0]
                        if not self.manual_override:
                            self.active_camera = 'gs'  # Auto-switch to tracking
                else:
                    # Tracking mode with GS camera
                    if self.current_target:
                        tracked = self._track_target(frame)
                        if not tracked:
                            self.current_target = None
                            if not self.manual_override:
                                self.active_camera = 'hq'  # Return to detection
                
                # Add frame info overlay
                frame_info = {
                    'camera': self.active_camera.upper(),
                    'mode': 'TRACKING' if self.current_target else 'DETECTION',
                    'timestamp': time.time()
                }
                
                # Add to queue for streaming
                if not self.frame_queue.full():
                    self.frame_queue.put((frame, frame_info))
                    
            except Exception as e:
                logger.error(f"Vision processing error: {e}")
                
            time.sleep(0.01)
            
    def _detect_drones(self, frame: np.ndarray) -> List[Dict]:
        """Detect drones in frame"""
        results = self.detection_model(
            frame,
            conf=self.config.detection_confidence,
            verbose=False
        )
        
        detections = []
        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    # Extract detection info
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    confidence = box.conf[0].item()
                    
                    # Calculate position in frame
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    width = x2 - x1
                    height = y2 - y1
                    
                    # Convert to angles (approximate)
                    frame_h, frame_w = frame.shape[:2]
                    azimuth_offset = (cx - frame_w/2) / frame_w * 60
                    elevation_offset = (cy - frame_h/2) / frame_h * 40
                    
                    detection = {
                        'bbox': [x1, y1, x2, y2],
                        'center': [cx, cy],
                        'size': [width, height],
                        'confidence': confidence,
                        'azimuth_offset': azimuth_offset,
                        'elevation_offset': elevation_offset,
                        'timestamp': time.time()
                    }
                    
                    detections.append(detection)
                    
        return detections
        
    def _track_target(self, frame: np.ndarray) -> bool:
        """Track current target in frame"""
        if not self.current_target:
            return False
            
        # Run detection with lower threshold for tracking
        results = self.detection_model(
            frame,
            conf=self.config.tracking_confidence,
            verbose=False
        )
        
        # Find closest detection to last known position
        # TODO: Implement more sophisticated tracking (e.g., DeepSORT)
        
        return len(results[0].boxes) > 0 if results[0].boxes is not None else False
        
    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest processed frame"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
            
    def stop(self):
        """Stop vision system"""
        self.running = False
        if self.process_thread:
            self.process_thread.join()
        self.hq_cam.stop()
        self.gs_cam.stop()
        logger.info("Vision system stopped")

# ============================================================================
# Main PTDTS Controller
# ============================================================================

class PTDTSController:
    """Main controller for the Pan Tilt Drone Detection System"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.state = SystemState.IDLE
        
        # Initialize subsystems
        self.motor = MotorController()
        self.servo = ServoController()
        self.acoustic = AcousticDetector(config)
        self.vision = VisionSystem(config)
        self.predictor = LeadPredictor()
        
        # Target tracking
        self.targets = {}
        self.current_target_id = None
        
        # Control threads
        self.running = False
        self.control_thread = None
        self.scan_thread = None
        
        # Statistics
        self.stats = {
            'detections_total': 0,
            'detections_acoustic': 0,
            'detections_visual': 0,
            'tracking_time': 0,
            'system_uptime': time.time()
        }
        
    def start(self):
        """Start PTDTS system"""
        logger.info("Starting PTDTS Controller")
        
        # Start subsystems
        self.acoustic.start()
        self.vision.start()
        
        # Start control loop
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()
        
        # Set initial state
        self.set_state(SystemState.SCANNING)
        
        logger.info("PTDTS Controller started successfully")
        
    def _control_loop(self):
        """Main control loop"""
        while self.running:
            try:
                # Process based on current state
                if self.state == SystemState.SCANNING:
                    self._scanning_behavior()
                elif self.state == SystemState.DETECTING:
                    self._detection_behavior()
                elif self.state == SystemState.TRACKING:
                    self._tracking_behavior()
                elif self.state == SystemState.ENGAGING:
                    self._engagement_behavior()
                    
                # Check for acoustic detections
                acoustic_detection = self.acoustic.get_detection()
                if acoustic_detection:
                    self._handle_acoustic_detection(acoustic_detection)
                    
                # Update statistics
                self._update_statistics()
                
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                self.set_state(SystemState.ERROR)
                
            time.sleep(0.01)
            
    def _scanning_behavior(self):
        """Scanning pattern behavior"""
        if not self.scan_thread or not self.scan_thread.is_alive():
            # Start new scan pattern
            self.scan_thread = threading.Thread(target=self._execute_scan_pattern)
            self.scan_thread.start()
            
    def _execute_scan_pattern(self):
        """Execute 360° scanning pattern"""
        scan_speed = 30  # degrees per second
        tilt_positions = [45, 90, 135]  # Multiple elevation angles
        
        for tilt in tilt_positions:
            self.servo.set_angle(tilt)
            time.sleep(0.5)
            
            # Continuous rotation scan
            for _ in range(int(360 / scan_speed)):
                if self.state != SystemState.SCANNING:
                    break
                self.motor.set_velocity(scan_speed)
                time.sleep(1.0)
                
            self.motor.set_velocity(0)
            
            if self.state != SystemState.SCANNING:
                break
                
    def _detection_behavior(self):
        """Detection mode behavior"""
        # Check for visual detections
        frame = self.vision.get_frame()
        if frame is not None:
            # Vision system handles detection internally
            pass
            
    def _tracking_behavior(self):
        """Active tracking behavior"""
        if self.current_target_id and self.current_target_id in self.targets:
            target = self.targets[self.current_target_id]
            
            # Update Kalman filter
            self.predictor.update(target.position)
            
            # Predict future position
            if self.config.lead_prediction_enabled:
                future_pos = self.predictor.predict_position(self.config.lead_time)
                # Convert to pan/tilt angles
                pan_angle, tilt_angle = self._position_to_angles(future_pos)
            else:
                # Direct tracking
                pan_angle, tilt_angle = self._position_to_angles(target.position)
                
            # Command motors
            self.motor.move_to_position(pan_angle)
            self.servo.set_angle(tilt_angle)
            
    def _engagement_behavior(self):
        """Target engagement behavior"""
        # TODO: Implement engagement logic
        # This could include:
        # - Sending target data to external system
        # - Activating countermeasures
        # - Recording high-resolution imagery
        pass
        
    def _handle_acoustic_detection(self, detection: dict):
        """Handle acoustic detection event"""
        logger.info(f"Acoustic detection: {detection}")
        
        # Create target from acoustic data
        target_id = f"acoustic_{int(time.time()*1000)}"
        
        # Convert spherical to cartesian (assume 100m range for acoustic)
        azimuth_rad = np.radians(detection['azimuth'])
        elevation_rad = np.radians(detection['elevation'])
        range_m = 100.0
        
        x = range_m * np.cos(elevation_rad) * np.cos(azimuth_rad)
        y = range_m * np.cos(elevation_rad) * np.sin(azimuth_rad)
        z = range_m * np.sin(elevation_rad)
        
        target = Target(
            id=target_id,
            timestamp=detection['timestamp'],
            source=DetectionSource.ACOUSTIC,
            position=(x, y, z),
            velocity=(0, 0, 0),
            confidence=detection['confidence'],
            classification="drone_suspected"
        )
        
        self.targets[target_id] = target
        self.stats['detections_acoustic'] += 1
        
        # Slew to detection
        self.motor.move_to_position(detection['azimuth'])
        self.servo.set_angle(90 + detection['elevation'])
        
        # Switch to detection mode
        self.set_state(SystemState.DETECTING)
        
    def _position_to_angles(self, position: Tuple[float, float, float]) -> Tuple[float, float]:
        """Convert 3D position to pan/tilt angles"""
        x, y, z = position
        
        # Calculate azimuth (pan)
        azimuth = np.degrees(np.arctan2(y, x))
        azimuth = azimuth % 360
        
        # Calculate elevation (tilt)
        range_horizontal = np.sqrt(x**2 + y**2)
        elevation = np.degrees(np.arctan2(z, range_horizontal))
        tilt = 90 - elevation  # Convert to servo angle
        
        return azimuth, tilt
        
    def _update_statistics(self):
        """Update system statistics"""
        self.stats['detections_total'] = (
            self.stats['detections_acoustic'] + 
            self.stats['detections_visual']
        )
        
    def set_state(self, new_state: SystemState):
        """Change system state"""
        if new_state != self.state:
            logger.info(f"State change: {self.state.name} -> {new_state.name}")
            self.state = new_state
            
    def get_status(self) -> Dict[str, Any]:
        """Get system status"""
        return {
            'state': self.state.name,
            'pan_angle': self.motor.position_degrees,
            'pan_velocity': self.motor.velocity_dps,
            'tilt_angle': self.servo.get_angle(),
            'targets': len(self.targets),
            'current_target': self.current_target_id,
            'stats': self.stats,
            'uptime': time.time() - self.stats['system_uptime']
        }
        
    def manual_control(self, pan: Optional[float] = None, tilt: Optional[float] = None):
        """Manual position control"""
        if pan is not None:
            self.motor.move_to_position(pan)
        if tilt is not None:
            self.servo.set_angle(tilt)
        self.set_state(SystemState.IDLE)
        
    def stop(self):
        """Stop PTDTS system"""
        logger.info("Stopping PTDTS Controller")
        
        self.running = False
        
        # Stop control thread
        if self.control_thread:
            self.control_thread.join()
            
        # Stop subsystems
        self.motor.stop()
        self.servo.stop()
        self.acoustic.stop()
        self.vision.stop()
        
        logger.info("PTDTS Controller stopped")

# ============================================================================
# Web Interface and API
# ============================================================================

app = Flask(__name__)
app.config['SECRET_KEY'] = 'ptdts-secret-key'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

ptdts_controller = None

@app.route('/')
def index():
    """Main web interface"""
    return render_template('dashboard.html')

@app.route('/api/status')
def api_status():
    """Get system status"""
    if ptdts_controller:
        return jsonify(ptdts_controller.get_status())
    return jsonify({'error': 'System not initialized'}), 503

@app.route('/api/control/pan/<float:angle>')
def api_pan(angle):
    """Pan to angle"""
    if ptdts_controller:
        ptdts_controller.manual_control(pan=angle)
        return jsonify({'success': True, 'angle': angle})
    return jsonify({'error': 'System not initialized'}), 503

@app.route('/api/control/tilt/<float:angle>')
def api_tilt(angle):
    """Tilt to angle"""
    if ptdts_controller:
        ptdts_controller.manual_control(tilt=angle)
        return jsonify({'success': True, 'angle': angle})
    return jsonify({'error': 'System not initialized'}), 503

@app.route('/api/control/state/<state>')
def api_set_state(state):
    """Set system state"""
    if ptdts_controller:
        try:
            new_state = SystemState[state.upper()]
            ptdts_controller.set_state(new_state)
            return jsonify({'success': True, 'state': new_state.name})
        except KeyError:
            return jsonify({'error': 'Invalid state'}), 400
    return jsonify({'error': 'System not initialized'}), 503

@app.route('/api/targets')
def api_targets():
    """Get target list"""
    if ptdts_controller:
        targets = [asdict(t) for t in ptdts_controller.targets.values()]
        return jsonify(targets)
    return jsonify([])

@app.route('/video_feed')
def video_feed():
    """Video streaming endpoint"""
    def generate():
        while True:
            if ptdts_controller and ptdts_controller.vision:
                frame = ptdts_controller.vision.get_frame()
                if frame is not None:
                    # Encode frame as JPEG
                    _, buffer = cv2.imencode('.jpg', frame)
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.1)
            
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/camera/mode/<mode>')
def api_camera_mode(mode):
    """Switch camera mode"""
    if ptdts_controller and ptdts_controller.vision:
        manual = request.args.get('manual', 'false').lower() == 'true'
        ptdts_controller.vision.set_camera_mode(mode, manual)
        return jsonify({'success': True, 'mode': mode, 'manual': manual})
    return jsonify({'error': 'System not initialized'}), 503

@app.route('/api/radar/data')
def api_radar_data():
    """Get radar data for top-down view"""
    if ptdts_controller:
        targets = []
        for target_id, target in ptdts_controller.targets.items():
            targets.append({
                'id': target_id,
                'x': target.position[0],
                'y': target.position[1],
                'z': target.position[2],
                'heading': np.degrees(np.arctan2(target.velocity[1], target.velocity[0])),
                'speed': np.linalg.norm(target.velocity[:2]),
                'classification': target.classification
            })
        
        return jsonify({
            'targets': targets,
            'system_heading': ptdts_controller.motor.position_degrees,
            'detection_range': 100  # meters
        })
    return jsonify({'targets': []})

@app.route('/video_feed')
def video_feed():
    """Enhanced video streaming with camera info overlay"""
    def generate():
        while True:
            if ptdts_controller and ptdts_controller.vision:
                result = ptdts_controller.vision.frame_queue.get()
                if result:
                    frame, info = result
                    
                    # Add text overlay
                    cv2.putText(frame, f"Camera: {info['camera']}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Mode: {info['mode']}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Encode frame
                    _, buffer = cv2.imencode('.jpg', frame)
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.1)
            
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    logger.info('Client connected')
    emit('connected', {'data': 'Connected to PTDTS'})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    logger.info('Client disconnected')

def broadcast_status():
    """Broadcast system status via WebSocket"""
    while True:
        if ptdts_controller:
            socketio.emit('status_update', ptdts_controller.get_status())
        time.sleep(0.5)

# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    """Main entry point for PTDTS framework"""
    global ptdts_controller
    
    # Print banner
    print("=" * 60)
    print("  PTDTS - Pan Tilt Drone Detection System")
    print("  Comprehensive Framework v1.0")
    print("=" * 60)
    
    # Load configuration
    config = SystemConfig()
    
    # Check for config file
    if os.path.exists('config.json'):
        with open('config.json', 'r') as f:
            config_data = json.load(f)
            for key, value in config_data.items():
                if hasattr(config, key):
                    setattr(config, key, value)
        logger.info("Loaded configuration from config.json")
    
    # Initialize controller
    ptdts_controller = PTDTSController(config)
    
    try:
        # Start system
        ptdts_controller.start()
        
        # Start status broadcast thread
        broadcast_thread = threading.Thread(target=broadcast_status)
        broadcast_thread.daemon = True
        broadcast_thread.start()
        
        # Start web interface
        logger.info(f"Starting web interface on port {config.web_port}")
        socketio.run(app, host='0.0.0.0', port=config.web_port, debug=False)
        
    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
    finally:
        # Clean shutdown
        if ptdts_controller:
            ptdts_controller.stop()
        logger.info("System shutdown complete")

if __name__ == "__main__":
    # Create log directory
    os.makedirs('/var/log/ptdts', exist_ok=True)
    
    # Run main program
    main()
