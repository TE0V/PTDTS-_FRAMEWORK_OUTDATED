#!/usr/bin/env python3
"""
PTDTS Calibration Utility
Calibrates encoder counts, camera alignment, and acoustic array orientation
"""

import time
import json
import numpy as np
from gpiozero import PWMOutputDevice, Button, Device
from gpiozero.pins.lgpio import LGPIOFactory
import logging
from picamera2 import Picamera2
import cv2

# Setup lgpio for Pi 5
try:
    Device.pin_factory = LGPIOFactory()
except:
    pass  # Use default factory

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CalibrationUtility:
    """System calibration tool"""
    
    def __init__(self):
        # Motor setup
        self.motor_in1 = PWMOutputDevice(27, frequency=1000)  # Pin 13
        self.motor_in2 = PWMOutputDevice(22, frequency=1000)  # Pin 15
        
        # Encoder setup
        self.encoder_a = Button(17, pull_up=True)  # Pin 11
        self.encoder_b = Button(4, pull_up=True)   # Pin 7
        
        # Servo setup
        self.servo = PWMOutputDevice(25, frequency=50)  # Pin 22
        
        # Calibration data
        self.calibration_data = {
            'encoder_counts_per_revolution': 8640,
            'pan_offset': 0,
            'tilt_offset': 90,
            'camera_fov_h': 60,
            'camera_fov_v': 40
        }
        
        # Encoder tracking
        self.encoder_count = 0
        self.last_a = False
        self.last_b = False
        
        # Setup encoder interrupts
        self.encoder_a.when_pressed = self._encoder_callback
        self.encoder_a.when_released = self._encoder_callback
        self.encoder_b.when_pressed = self._encoder_callback
        self.encoder_b.when_released = self._encoder_callback
        
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
    
    #!!! This callibration module needs to be changed such that manual rotation of the system generates encoder output !!!
    def calibrate_encoder(self):
        """Calibrate encoder counts per revolution"""
        logger.info("=== Encoder Calibration ===")
        logger.info("The system will rotate 360 degrees slowly")
        logger.info("Mark the starting position visually")
        
        input("Press Enter when ready to start...")
        
        # Reset encoder
        self.encoder_count = 0
        start_count = 0
        
        # Rotate slowly
        logger.info("Rotating... Watch for one complete revolution")
        self.motor_in1.value = 0.3  # Slow speed
        self.motor_in2.value = 0
        
        input("Press Enter when back at starting position...")
        
        # Stop motor
        self.motor_in1.value = 0
        self.motor_in2.value = 0
        
        # Calculate counts
        total_counts = abs(self.encoder_count - start_count)
        logger.info(f"Measured {total_counts} counts for 360 degrees")
        
        # Update calibration
        self.calibration_data['encoder_counts_per_revolution'] = total_counts
        
        # Test calibration
        logger.info("Testing calibration...")
        self._test_angles()
        
    def _test_angles(self):
        """Test movement to specific angles"""
        test_angles = [0, 90, 180, 270, 0]
        
        for angle in test_angles:
            logger.info(f"Moving to {angle} degrees...")
            self._move_to_angle(angle)
            time.sleep(2)
            
    def _move_to_angle(self, target_degrees):
        """Simple movement to target angle"""
        counts_per_degree = self.calibration_data['encoder_counts_per_revolution'] / 360
        target_count = int(target_degrees * counts_per_degree)
        
        while abs(self.encoder_count - target_count) > 10:
            error = target_count - self.encoder_count
            
            if error > 0:
                self.motor_in1.value = min(0.5, abs(error) / 1000)
                self.motor_in2.value = 0
            else:
                self.motor_in1.value = 0
                self.motor_in2.value = min(0.5, abs(error) / 1000)
                
            time.sleep(0.01)
            
        # Stop
        self.motor_in1.value = 0
        self.motor_in2.value = 0
        
    def calibrate_servo_limits(self):
        """Calibrate servo min/max angles"""
        logger.info("=== Servo Calibration ===")
        logger.info("Testing servo range of motion")
        
        # Test angles
        for angle in [0, 45, 90, 135, 180]:
            logger.info(f"Setting servo to {angle} degrees...")
            pulse_ms = 1.0 + (angle / 180.0)
            duty_cycle = pulse_ms / 20.0
            self.servo.value = duty_cycle
            time.sleep(1)
            
        # Find safe limits
        logger.info("Finding safe operating limits...")
        
        min_safe = int(input("Enter minimum safe angle (default 0): ") or "0")
        max_safe = int(input("Enter maximum safe angle (default 180): ") or "180")
        center = int(input("Enter center/home angle (default 90): ") or "90")
        
        self.calibration_data['servo_min'] = min_safe
        self.calibration_data['servo_max'] = max_safe
        self.calibration_data['tilt_offset'] = center
        
        # Return to center
        pulse_ms = 1.0 + (center / 180.0)
        self.servo.value = pulse_ms / 20.0
        
    def calibrate_camera_fov(self):
        """Calibrate camera field of view"""
        logger.info("=== Camera FOV Calibration ===")
        
        try:
            # Initialize cameras
            hq_cam = Picamera2(0)
            gs_cam = Picamera2(1)
            
            # Configure HQ camera
            hq_config = hq_cam.create_preview_configuration(
                main={"size": (1920, 1080)}
            )
            hq_cam.configure(hq_config)
            hq_cam.start()
            
            logger.info("HQ Camera (Detection) - Port 0:")
            logger.info("Place an object at known distance and measure apparent size")
            logger.info("Default FOV: 60° horizontal, 40° vertical")
            
            # Capture test frame
            frame = hq_cam.capture_array()
            cv2.imwrite("hq_calibration.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            logger.info("Saved calibration image: hq_calibration.jpg")
            
            hq_fov_h = float(input("Enter measured horizontal FOV (default 60): ") or "60")
            hq_fov_v = float(input("Enter measured vertical FOV (default 40): ") or "40")
            
            hq_cam.stop()
            
            # Configure GS camera
            gs_config = gs_cam.create_preview_configuration(
                main={"size": (1280, 720)}
            )
            gs_cam.configure(gs_config)
            gs_cam.start()
            
            logger.info("\nGS Camera (Tracking) - Port 1:")
            logger.info("Default FOV: 45° horizontal, 34° vertical")
            
            frame = gs_cam.capture_array()
            cv2.imwrite("gs_calibration.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            logger.info("Saved calibration image: gs_calibration.jpg")
            
            gs_fov_h = float(input("Enter measured horizontal FOV (default 45): ") or "45")
            gs_fov_v = float(input("Enter measured vertical FOV (default 34): ") or "34")
            
            gs_cam.stop()
            
            # Save calibration
            self.calibration_data['hq_camera_fov'] = {'h': hq_fov_h, 'v': hq_fov_v}
            self.calibration_data['gs_camera_fov'] = {'h': gs_fov_h, 'v': gs_fov_v}
            
        except Exception as e:
            logger.error(f"Camera calibration error: {e}")
            
    def calibrate_acoustic_orientation(self):
        """Calibrate acoustic array orientation relative to cameras"""
        logger.info("=== Acoustic Array Calibration ===")
        logger.info("This calibrates the reSpeaker orientation relative to pan axis")
        
        logger.info("1. Place a sound source directly in front (0°)")
        logger.info("2. Make noise and observe detected angle")
        
        # This would connect to ODAS and measure offset
        acoustic_offset = float(
            input("Enter angular offset of acoustic array (default 0): ") or "0"
        )
        
        self.calibration_data['acoustic_offset'] = acoustic_offset
        
    def motor_test(self):
        """Test motor control"""
        logger.info("=== Motor Test ===")
        logger.info("Commands: f=forward, b=backward, s=stop, q=quit")
        
        while True:
            cmd = input("Command: ").lower()
            
            if cmd == 'f':
                self.motor_in1.value = 0.5
                self.motor_in2.value = 0
                logger.info(f"Forward - Encoder: {self.encoder_count}")
            elif cmd == 'b':
                self.motor_in1.value = 0
                self.motor_in2.value = 0.5
                logger.info(f"Backward - Encoder: {self.encoder_count}")
            elif cmd == 's':
                self.motor_in1.value = 0
                self.motor_in2.value = 0
                logger.info(f"Stopped - Encoder: {self.encoder_count}")
            elif cmd == 'q':
                break
            else:
                logger.info("Unknown command")
                
        self.motor_in1.value = 0
        self.motor_in2.value = 0
        
    def save_calibration(self, filename="calibration.json"):
        """Save calibration data"""
        with open(filename, 'w') as f:
            json.dump(self.calibration_data, f, indent=2)
        logger.info(f"Calibration saved to {filename}")
        
    def load_calibration(self, filename="calibration.json"):
        """Load calibration data"""
        try:
            with open(filename, 'r') as f:
                self.calibration_data = json.load(f)
            logger.info(f"Calibration loaded from {filename}")
        except FileNotFoundError:
            logger.warning("No calibration file found, using defaults")
            
    def run_full_calibration(self):
        """Run complete calibration sequence"""
        logger.info("=== PTDTS Full Calibration ===")
        
        # Load existing calibration if available
        self.load_calibration()
        
        steps = [
            ("Motor Test", self.motor_test),
            ("Encoder Calibration", self.calibrate_encoder),
            ("Servo Limits", self.calibrate_servo_limits),
            ("Camera FOV", self.calibrate_camera_fov),
            ("Acoustic Orientation", self.calibrate_acoustic_orientation)
        ]
        
        for name, func in steps:
            logger.info(f"\n--- {name} ---")
            if input(f"Run {name}? (y/n): ").lower() == 'y':
                func()
                
        # Save calibration
        self.save_calibration()
        
        logger.info("\n=== Calibration Complete ===")
        logger.info("Calibration data:")
        for key, value in self.calibration_data.items():
            logger.info(f"  {key}: {value}")
            
    def cleanup(self):
        """Cleanup GPIO"""
        self.motor_in1.close()
        self.motor_in2.close()
        self.servo.close()

def main():
    """Main calibration routine"""
    cal = CalibrationUtility()
    
    try:
        cal.run_full_calibration()
    except KeyboardInterrupt:
        logger.info("\nCalibration interrupted")
    finally:
        cal.cleanup()

if __name__ == "__main__":
    main()
