#!/usr/bin/env python3
"""
Minimal Motor Control - No Threading
Simplest possible implementation to verify basic functionality
"""

import time
from gpiozero import PWMOutputDevice, Button, Device
from gpiozero.pins.lgpio import LGPIOFactory

# Configure GPIO
Device.pin_factory = LGPIOFactory()

class MinimalMotorControl:
    def __init__(self):
        print("Initializing minimal motor control...")
        
        # Motor pins
        self.motor_in1 = PWMOutputDevice(27, frequency=1000)
        self.motor_in2 = PWMOutputDevice(22, frequency=1000)
        print("✓ Motor pins initialized")
        
        # Encoder pins
        self.encoder_a = Button(17, pull_up=True)
        self.encoder_b = Button(4, pull_up=True)
        print("✓ Encoder pins initialized")
        
        # Encoder state
        self.encoder_count = 0
        self.last_a = self.encoder_a.is_pressed
        self.last_b = self.encoder_b.is_pressed
        
        # Calibration
        self.COUNTS_PER_REV = 8043
        self.counts_per_degree = self.COUNTS_PER_REV / 360.0
        
        # Simple PID parameters
        self.kp = 0.01  # Start conservative
        self.ki = 0.0
        self.kd = 0.0
        
        # Position tracking
        self.current_position = 0.0
        self.target_position = 0.0
        
        # Setup encoder callbacks
        self.encoder_a.when_pressed = self.encoder_tick
        self.encoder_a.when_released = self.encoder_tick
        self.encoder_b.when_pressed = self.encoder_tick
        self.encoder_b.when_released = self.encoder_tick
        
        print(f"✓ Setup complete")
        print(f"  Counts per degree: {self.counts_per_degree:.2f}")
    
    def encoder_tick(self):
        """Simple encoder callback"""
        a = self.encoder_a.is_pressed
        b = self.encoder_b.is_pressed
        
        # Quadrature decoding
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
    
    def stop(self):
        """Stop motor"""
        self.motor_in1.value = 0
        self.motor_in2.value = 0
    
    def set_speed(self, speed):
        """Set motor speed directly"""
        if abs(speed) < 0.01:
            self.stop()
        elif speed > 0:
            self.motor_in1.value = min(abs(speed), 0.5)  # Limit to 50%
            self.motor_in2.value = 0
        else:
            self.motor_in1.value = 0
            self.motor_in2.value = min(abs(speed), 0.5)  # Limit to 50%
    
    def move_to_position(self, target_degrees, timeout=10):
        """Simple blocking move to position"""
        print(f"\nMoving to {target_degrees}°...")
        self.target_position = target_degrees
        
        start_time = time.time()
        last_print = start_time
        
        # Control loop
        while time.time() - start_time < timeout:
            # Get current position
            self.current_position = self.encoder_count / self.counts_per_degree
            
            # Calculate error
            error = self.target_position - self.current_position
            
            # Check if we're close enough
            if abs(error) < 2.0:
                self.stop()
                print(f"✓ Reached target! Position: {self.current_position:.1f}°")
                return True
            
            # Simple P control
            control_output = self.kp * error
            
            # Limit speed
            control_output = max(-0.3, min(0.3, control_output))
            
            # Apply to motor
            self.set_speed(control_output)
            
            # Print status periodically
            if time.time() - last_print > 0.5:
                print(f"  Pos: {self.current_position:.1f}°, "
                      f"Target: {self.target_position:.1f}°, "
                      f"Error: {error:.1f}°, "
                      f"Output: {control_output:.3f}")
                last_print = time.time()
            
            # Small delay
            time.sleep(0.01)
        
        self.stop()
        print(f"⚠ Timeout! Final position: {self.current_position:.1f}°")
        return False
    
    def test_encoder(self):
        """Test if encoder is working"""
        print("\n[ENCODER TEST]")
        print("Testing encoder by running motor briefly...")
        
        initial_count = self.encoder_count
        
        # Run motor at low speed
        print("Running motor forward at 15% speed for 1 second...")
        self.set_speed(0.15)
        time.sleep(1)
        self.stop()
        
        counts_moved = self.encoder_count - initial_count
        degrees_moved = counts_moved / self.counts_per_degree
        
        print(f"Encoder counts changed: {counts_moved}")
        print(f"Degrees moved: {degrees_moved:.1f}°")
        
        if abs(counts_moved) < 10:
            print("❌ ENCODER NOT WORKING! No counts detected.")
            return False
        else:
            print("✓ Encoder is working!")
            return True
    
    def run_test_sequence(self):
        """Run a simple test sequence"""
        print("\n" + "="*60)
        print("RUNNING TEST SEQUENCE")
        print("="*60)
        
        # First test encoder
        if not self.test_encoder():
            print("\n⚠ Cannot proceed without working encoder!")
            return
        
        # Reset position
        print("\n[RESET]")
        print("Resetting encoder count to zero...")
        self.encoder_count = 0
        time.sleep(0.5)
        
        # Test movements
        test_positions = [45, 0, -30, 0, 90, 0]
        
        for i, target in enumerate(test_positions, 1):
            print(f"\n[TEST {i}] Target: {target}°")
            success = self.move_to_position(target, timeout=5)
            if not success:
                print("Test failed - stopping sequence")
                break
            time.sleep(1)  # Pause between moves
        
        print("\n" + "="*60)
        print("TEST COMPLETE")
        print(f"Final position: {self.current_position:.1f}°")
        print(f"Final encoder count: {self.encoder_count}")
    
    def manual_control(self):
        """Manual control mode for testing"""
        print("\n" + "="*60)
        print("MANUAL CONTROL MODE")
        print("="*60)
        print("Commands:")
        print("  f/F - Forward (slow/fast)")
        print("  b/B - Backward (slow/fast)")
        print("  s   - Stop")
        print("  r   - Reset encoder")
        print("  p   - Print position")
        print("  g   - Go to position")
        print("  q   - Quit")
        print()
        
        try:
            while True:
                cmd = input("Command: ").strip().lower()
                
                if cmd == 'f':
                    print("Forward slow (15%)")
                    self.set_speed(0.15)
                elif cmd == 'F':
                    print("Forward fast (30%)")
                    self.set_speed(0.30)
                elif cmd == 'b':
                    print("Backward slow (-15%)")
                    self.set_speed(-0.15)
                elif cmd == 'B':
                    print("Backward fast (-30%)")
                    self.set_speed(-0.30)
                elif cmd == 's':
                    print("Stop")
                    self.stop()
                elif cmd == 'r':
                    print("Resetting encoder")
                    self.encoder_count = 0
                elif cmd == 'p':
                    pos = self.encoder_count / self.counts_per_degree
                    print(f"Position: {pos:.1f}° (counts: {self.encoder_count})")
                elif cmd == 'g':
                    target = float(input("Target position (degrees): "))
                    self.move_to_position(target)
                elif cmd == 'q':
                    break
                else:
                    print("Unknown command")
        
        except KeyboardInterrupt:
            print("\nInterrupted")
        
        finally:
            self.stop()
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.motor_in1.close()
        self.motor_in2.close()
        self.encoder_a.close()
        self.encoder_b.close()


def main():
    print("="*60)
    print("MINIMAL MOTOR CONTROL TEST")
    print("="*60)
    print("\nThis is a simplified version to diagnose issues.")
    print("No threading, just basic control loop.\n")
    
    motor = MinimalMotorControl()
    
    print("\nSelect mode:")
    print("1. Run automatic test sequence")
    print("2. Manual control")
    print("3. Just test encoder")
    
    choice = input("\nChoice (1/2/3): ").strip()
    
    try:
        if choice == '1':
            motor.run_test_sequence()
        elif choice == '2':
            motor.manual_control()
        elif choice == '3':
            motor.test_encoder()
        else:
            print("Invalid choice")
    
    except Exception as e:
        print(f"\nError: {e}")
    
    finally:
        motor.cleanup()
        print("\nCleanup complete")


if __name__ == "__main__":
    main()