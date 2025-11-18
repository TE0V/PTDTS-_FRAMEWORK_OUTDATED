#!/usr/bin/env python3
"""
Manual Encoder Calibration
Turn the pan mechanism by hand to measure encoder resolution
"""

from gpiozero import Button
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import time
import sys

Device.pin_factory = LGPIOFactory()

# Encoder
encoder_a = Button(17, pull_up=True)
encoder_b = Button(4, pull_up=True)

class Encoder:
    def __init__(self):
        self.count = 0
        self.last_a = False
        self.last_b = False
        self.last_count = 0
        
        encoder_a.when_pressed = self._callback
        encoder_a.when_released = self._callback
        encoder_b.when_pressed = self._callback
        encoder_b.when_released = self._callback
        
    def _callback(self):
        a = encoder_a.is_pressed
        b = encoder_b.is_pressed
        
        if a != self.last_a:
            if a == b:
                self.count += 1
            else:
                self.count -= 1
                
        if b != self.last_b:
            if a != b:
                self.count += 1
            else:
                self.count -= 1
                
        self.last_a = a
        self.last_b = b
    
    def reset(self):
        self.count = 0
        self.last_count = 0

encoder = Encoder()

print("\n" + "="*70)
print("🔧 MANUAL ENCODER CALIBRATION")
print("="*70)
print("\nThis measures how many encoder counts = 1 full rotation")
print("\nInstructions:")
print("  1. Mark the current position with tape/marker")
print("  2. Press ENTER to start counting")
print("  3. MANUALLY rotate the pan mechanism ONE full rotation (360°)")
print("  4. Stop when you're back at the starting position")
print("  5. Press ENTER to finish")
print("\nTip: Rotate slowly and steadily for best accuracy")
print("\n" + "="*70)

input("\nPress ENTER when ready to start...")

# Reset encoder
encoder.reset()
start_count = encoder.count

print("\n✓ Counting started!")
print("  Current count: 0")
print("\n🔄 Rotate the mechanism ONE full rotation (360°)")
print("  Watch the count update in real-time...")
print("\n  Press ENTER when you've completed one full rotation\n")

# Real-time count display
try:
    while True:
        current = encoder.count - start_count
        
        # Update display every 10 counts or so
        if abs(current - encoder.last_count) >= 10 or current == 0:
            # Move cursor up and clear line
            sys.stdout.write(f"\r  Count: {current:6d}  ")
            sys.stdout.flush()
            encoder.last_count = current
        
        # Check if user pressed Enter (non-blocking)
        import select
        if select.select([sys.stdin], [], [], 0.01)[0]:
            input()  # Consume the Enter
            break
        
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

# Final count
final_count = encoder.count - start_count

print("\n\n" + "="*70)
print("📊 CALIBRATION RESULTS")
print("="*70)
print(f"\n  Total encoder counts for 360°: {final_count}")
print(f"  Counts per degree: {final_count / 360:.2f}")
print(f"  Degrees per count: {360 / abs(final_count):.4f}°")

if abs(final_count) > 0:
    print("\n" + "="*70)
    print("✓ USE THESE VALUES IN YOUR CODE:")
    print("="*70)
    print(f"\n  COUNTS_PER_ROTATION = {abs(final_count)}")
    print(f"  COUNTS_PER_DEGREE = {abs(final_count) / 360:.2f}")
    print("\n" + "="*70)
    
    # Give context
    print("\n💡 For reference:")
    if abs(final_count) < 1000:
        print("  • Low resolution encoder (< 1000 counts/rotation)")
        print("  • Typical for basic DC motor encoders")
    elif abs(final_count) < 5000:
        print("  • Medium resolution encoder (1000-5000 counts/rotation)")
        print("  • Good for most positioning tasks")
    else:
        print("  • High resolution encoder (> 5000 counts/rotation)")
        print("  • Excellent precision!")
    
    # Estimate position accuracy
    accuracy = 360 / abs(final_count)
    print(f"\n  • Position accuracy: ±{accuracy:.2f}° per count")
    
    if accuracy > 1.0:
        print(f"  ⚠️  Low accuracy - may need tighter tolerances")
    elif accuracy < 0.1:
        print(f"  ✓ Very high accuracy - excellent for tracking")
    else:
        print(f"  ✓ Good accuracy for drone tracking")

else:
    print("\n⚠️  No counts detected!")
    print("\nPossible issues:")
    print("  • Encoder not connected properly")
    print("  • Wrong GPIO pins")
    print("  • Encoder not moving with pan mechanism")

print("\n" + "="*70 + "\n")
