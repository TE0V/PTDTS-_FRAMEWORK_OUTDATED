#!/usr/bin/env python3
"""
PTDTS System Diagnostics
Comprehensive health check and troubleshooting utility
"""

import os
import sys
import time
import subprocess
import json
import psutil
import logging
from pathlib import Path
from gpiozero import Device, PWMOutputDevice, Button
try:
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()
    GPIO_BACKEND = "lgpio"
except ImportError:
    GPIO_BACKEND = "default"

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

class SystemDiagnostics:
    """Comprehensive system health checker"""
    
    def __init__(self):
        self.results = {
            'system': {},
            'gpio': {},
            'cameras': {},
            'audio': {},
            'network': {},
            'services': {},
            'performance': {}
        }
        
    def run_all_checks(self):
        """Run complete diagnostic suite"""
        print("=" * 60)
        print("PTDTS SYSTEM DIAGNOSTICS")
        print("=" * 60)
        
        tests = [
            ("System Information", self.check_system),
            ("GPIO Subsystem", self.check_gpio),
            ("Camera System", self.check_cameras),
            ("Audio System", self.check_audio),
            ("Network Configuration", self.check_network),
            ("Services Status", self.check_services),
            ("Performance Metrics", self.check_performance)
        ]
        
        for name, test_func in tests:
            print(f"\n[{name}]")
            print("-" * 40)
            test_func()
            
        # Generate report
        self.generate_report()
        
    def check_system(self):
        """Check basic system information"""
        # Raspberry Pi model
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip()
                self.results['system']['model'] = model
                print(f"✓ Model: {model}")
                
                if "Raspberry Pi 5" in model:
                    print("✓ Pi 5 detected - using lgpio for GPIO")
                elif "Raspberry Pi 4" in model:
                    print("⚠ Pi 4 detected - may need pigpio instead of lgpio")
        except:
            print("✗ Could not detect Raspberry Pi model")
            
        # OS version
        try:
            os_info = subprocess.check_output(
                ["cat", "/etc/os-release"],
                text=True
            )
            for line in os_info.split('\n'):
                if line.startswith('PRETTY_NAME'):
                    os_name = line.split('=')[1].strip('"')
                    self.results['system']['os'] = os_name
                    print(f"✓ OS: {os_name}")
                    break
        except:
            print("✗ Could not detect OS version")
            
        # Python version
        python_version = sys.version.split()[0]
        self.results['system']['python'] = python_version
        print(f"✓ Python: {python_version}")
        
        # Memory
        mem = psutil.virtual_memory()
        self.results['system']['memory'] = f"{mem.total // (1024**3)}GB"
        print(f"✓ Memory: {mem.total // (1024**3)}GB ({mem.percent}% used)")
        
        # Disk space
        disk = psutil.disk_usage('/')
        self.results['system']['disk'] = f"{disk.total // (1024**3)}GB"
        print(f"✓ Disk: {disk.free // (1024**3)}GB free of {disk.total // (1024**3)}GB")
        
        # CPU temperature
        try:
            temp = subprocess.check_output(
                ["vcgencmd", "measure_temp"],
                text=True
            ).strip()
            self.results['system']['temperature'] = temp
            print(f"✓ CPU {temp}")
        except:
            print("⚠ Could not read CPU temperature")
            
    def check_gpio(self):
        """Check GPIO subsystem"""
        print(f"GPIO Backend: {GPIO_BACKEND}")
        
        # Check lgpio installation
        if GPIO_BACKEND == "lgpio":
            print("✓ lgpio library detected (Pi 5 compatible)")
        else:
            print("⚠ lgpio not available, using default backend")
            
        # Test GPIO pins
        test_pins = {
            27: "Motor IN1",
            22: "Motor IN2", 
            17: "Encoder A",
            4: "Encoder B",
            25: "Servo PWM"
        }
        
        for pin, name in test_pins.items():
            try:
                # Try to create a pin object
                if pin == 18:  # PWM pin
                    test_pin = PWMOutputDevice(pin, frequency=50)
                else:
                    test_pin = Button(pin, pull_up=True)
                    
                test_pin.close()
                print(f"✓ GPIO {pin} ({name}) - OK")
                self.results['gpio'][f'pin_{pin}'] = "OK"
            except Exception as e:
                print(f"✗ GPIO {pin} ({name}) - Error: {e}")
                self.results['gpio'][f'pin_{pin}'] = f"Error: {e}"
                
    def check_cameras(self):
        """Check camera system"""
        # Check libcamera
        try:
            result = subprocess.run(
                ["rpicam-hello", "--list-cameras"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            cameras = []
            for line in result.stdout.split('\n'):
                if ':' in line and any(char.isdigit() for char in line.split(':')[0]):
                    cameras.append(line.strip())
                    
            if len(cameras) >= 2:
                print(f"✓ Dual cameras detected: {len(cameras)} cameras")
                self.results['cameras']['count'] = len(cameras)
            elif len(cameras) == 1:
                print(f"⚠ Only 1 camera detected (need 2)")
                self.results['cameras']['count'] = 1
            else:
                print("✗ No cameras detected")
                self.results['cameras']['count'] = 0
                
            # Check each camera
            for i in range(len(cameras)):
                try:
                    test = subprocess.run(
                        ["rpicam-hello", "--camera", str(i), "-t", "1"],
                        capture_output=True,
                        timeout=3
                    )
                    if test.returncode == 0:
                        print(f"✓ Camera {i} functional")
                        self.results['cameras'][f'cam_{i}'] = "OK"
                    else:
                        print(f"✗ Camera {i} error")
                        self.results['cameras'][f'cam_{i}'] = "Error"
                except:
                    print(f"✗ Camera {i} test failed")
                    self.results['cameras'][f'cam_{i}'] = "Failed"
                    
        except Exception as e:
            print(f"✗ Camera system error: {e}")
            self.results['cameras']['error'] = str(e)
            
    def check_audio(self):
        """Check audio system and reSpeaker"""
        # List audio devices
        try:
            result = subprocess.run(
                ["arecord", "-l"],
                capture_output=True,
                text=True
            )
            
            if "XVF3800" in result.stdout or "reSpeaker" in result.stdout:
                print("✓ reSpeaker XVF3800 detected")
                self.results['audio']['respeaker'] = "Detected"
            else:
                print("✗ reSpeaker XVF3800 not found")
                self.results['audio']['respeaker'] = "Not found"
                
            # Count audio devices
            cards = result.stdout.count("card")
            print(f"  Audio devices: {cards}")
            self.results['audio']['devices'] = cards
            
        except Exception as e:
            print(f"✗ Audio system error: {e}")
            self.results['audio']['error'] = str(e)
            
        # Check ODAS installation
        if Path("/usr/local/bin/odaslive").exists():
            print("✓ ODAS installed")
            self.results['audio']['odas'] = "Installed"
        else:
            print("✗ ODAS not installed")
            self.results['audio']['odas'] = "Not installed"
            
    def check_network(self):
        """Check network configuration"""
        # Get IP address
        try:
            result = subprocess.run(
                ["hostname", "-I"],
                capture_output=True,
                text=True
            )
            ips = result.stdout.strip().split()
            if ips:
                print(f"✓ IP Address: {ips[0]}")
                self.results['network']['ip'] = ips[0]
                
                # Check web interface
                import socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                result = sock.connect_ex(('localhost', 5000))
                sock.close()
                
                if result == 0:
                    print("✓ Web interface accessible on port 5000")
                    self.results['network']['web_interface'] = "Running"
                else:
                    print("⚠ Web interface not running on port 5000")
                    self.results['network']['web_interface'] = "Not running"
                    
            else:
                print("✗ No IP address found")
                self.results['network']['ip'] = "None"
                
        except Exception as e:
            print(f"✗ Network error: {e}")
            self.results['network']['error'] = str(e)
            
    def check_services(self):
        """Check system services"""
        services = [
            ("ptdts", "Main PTDTS service"),
            ("odas", "ODAS acoustic service"),
            ("pigpiod", "GPIO daemon (Pi 4)"),
        ]
        
        for service_name, description in services:
            try:
                result = subprocess.run(
                    ["systemctl", "is-active", service_name],
                    capture_output=True,
                    text=True
                )
                status = result.stdout.strip()
                
                if status == "active":
                    print(f"✓ {description}: Running")
                    self.results['services'][service_name] = "Running"
                elif status == "inactive":
                    print(f"⚠ {description}: Stopped")
                    self.results['services'][service_name] = "Stopped"
                else:
                    print(f"⚠ {description}: {status}")
                    self.results['services'][service_name] = status
                    
            except:
                print(f"  {description}: Not installed")
                self.results['services'][service_name] = "Not installed"
                
    def check_performance(self):
        """Check system performance"""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        print(f"CPU Usage: {cpu_percent}%")
        self.results['performance']['cpu'] = f"{cpu_percent}%"
        
        if cpu_percent > 80:
            print("⚠ High CPU usage detected")
        else:
            print("✓ CPU usage normal")
            
        # Memory usage
        mem = psutil.virtual_memory()
        print(f"Memory Usage: {mem.percent}%")
        self.results['performance']['memory'] = f"{mem.percent}%"
        
        if mem.percent > 80:
            print("⚠ High memory usage detected")
        else:
            print("✓ Memory usage normal")
            
        # Check for thermal throttling
        try:
            result = subprocess.check_output(
                ["vcgencmd", "get_throttled"],
                text=True
            ).strip()
            
            if "throttled=0x0" in result:
                print("✓ No thermal throttling")
                self.results['performance']['throttling'] = "None"
            else:
                print(f"⚠ Throttling detected: {result}")
                self.results['performance']['throttling'] = result
                
        except:
            print("⚠ Could not check throttling status")
            
        # Check YOLO model
        model_path = Path("models/yolo11n.pt")
        if model_path.exists():
            size_mb = model_path.stat().st_size / (1024**2)
            print(f"✓ YOLO model present ({size_mb:.1f} MB)")
            self.results['performance']['yolo_model'] = f"{size_mb:.1f} MB"
        else:
            print("✗ YOLO model not found")
            self.results['performance']['yolo_model'] = "Not found"
            
    def generate_report(self):
        """Generate diagnostic report"""
        print("\n" + "=" * 60)
        print("DIAGNOSTIC REPORT SUMMARY")
        print("=" * 60)
        
        # Save report to file
        report_file = f"diagnostic_report_{time.strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_file, 'w') as f:
            json.dump(self.results, f, indent=2)
            
        print(f"\nFull report saved to: {report_file}")
        
        # Quick health assessment
        issues = []
        warnings = []
        
        # Check for critical issues
        if self.results.get('gpio', {}).get('pin_27') != "OK":
            issues.append("Motor control pins not accessible")
            
        if self.results.get('cameras', {}).get('count', 0) < 2:
            issues.append("Dual cameras not detected")
            
        if self.results.get('audio', {}).get('respeaker') != "Detected":
            warnings.append("Acoustic array not detected")
            
        if GPIO_BACKEND != "lgpio" and "Pi 5" in self.results.get('system', {}).get('model', ''):
            issues.append("lgpio not configured for Pi 5")
            
        # Print assessment
        print("\nHEALTH ASSESSMENT:")
        if not issues and not warnings:
            print("✓ System appears healthy")
        else:
            if issues:
                print("\nCRITICAL ISSUES:")
                for issue in issues:
                    print(f"  ✗ {issue}")
            if warnings:
                print("\nWARNINGS:")
                for warning in warnings:
                    print(f"  ⚠ {warning}")
                    
        print("\nRECOMMENDATIONS:")
        if issues or warnings:
            print("  1. Review the diagnostic report")
            print("  2. Check hardware connections")
            print("  3. Run calibration.py after fixing issues")
        else:
            print("  1. Run calibration.py if not already done")
            print("  2. Start system with: sudo systemctl start ptdts")
            
def main():
    """Run diagnostics"""
    print("Starting PTDTS System Diagnostics...")
    print("This will check all system components\n")
    
    diag = SystemDiagnostics()
    
    try:
        diag.run_all_checks()
    except KeyboardInterrupt:
        print("\n\nDiagnostics interrupted")
    except Exception as e:
        print(f"\nDiagnostic error: {e}")

if __name__ == "__main__":
    main()
