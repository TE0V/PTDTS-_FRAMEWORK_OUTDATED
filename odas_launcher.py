#!/usr/bin/env python3
"""
ODAS Launcher for PTDTS
Manages ODAS acoustic detection process and configuration
"""

import os
import sys
import subprocess
import json
import socket
import time
import threading
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ODASLauncher:
    """Manages ODAS process for acoustic detection"""
    
    def __init__(self, config_path="/etc/odas/xvf3800.cfg"):
        self.config_path = config_path
        self.odas_process = None
        self.output_socket = None
        self.running = False
        
        # Verify reSpeaker is connected
        self._verify_respeaker()
        
    def _verify_respeaker(self):
        """Check if reSpeaker XVF3800 is connected"""
        try:
            # Check for USB audio device
            result = subprocess.run(
                ["arecord", "-l"],
                capture_output=True,
                text=True
            )
            
            if "XVF3800" not in result.stdout and "reSpeaker" not in result.stdout:
                logger.warning("reSpeaker XVF3800 not detected in audio devices")
                logger.info("Available audio devices:\n" + result.stdout)
            else:
                logger.info("reSpeaker XVF3800 detected")
                
        except Exception as e:
            logger.error(f"Error checking for reSpeaker: {e}")
            
    def generate_config(self):
        """Generate ODAS configuration for XVF3800 4-mic array"""
        config = {
            "version": "2.1",
            
            # Microphone array geometry (circular, 28.5mm radius)
            "mics": [
                {
                    "mu": [0.0285, 0.0000, 0.0000],
                    "sigma2": [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6],
                    "direction": [0, 0, 1],
                    "angle": [80, 100]
                },
                {
                    "mu": [0.0000, 0.0285, 0.0000],
                    "sigma2": [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6],
                    "direction": [0, 0, 1],
                    "angle": [80, 100]
                },
                {
                    "mu": [-0.0285, 0.0000, 0.0000],
                    "sigma2": [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6],
                    "direction": [0, 0, 1],
                    "angle": [80, 100]
                },
                {
                    "mu": [0.0000, -0.0285, 0.0000],
                    "sigma2": [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6],
                    "direction": [0, 0, 1],
                    "angle": [80, 100]
                }
            ],
            
            # Sound source localization
            "ssl": {
                "nTheta": 181,
                "usePhat": 1,
                "searchRange": [-180, 180],  # Full 360° coverage
                "nPairs": 6,  # All mic pairs
                "pairs": [
                    [0, 1], [0, 2], [0, 3],
                    [1, 2], [1, 3], [2, 3]
                ]
            },
            
            # Sound source tracking
            "sst": {
                "nTracked": 2,  # Track up to 2 drones
                "theta_new": 0.3,
                "N_prob": 5,
                "theta_prob": 0.8,
                "N_inactive": [250, 250, 250, 250],
                "inactive_prob": 0.05,
                "trackingRange": [-180, 180, -90, 90]  # Full sphere
            },
            
            # Filtering for drone frequencies (100-500 Hz)
            "filtering": {
                "freq_min": 100,
                "freq_max": 500,
                "enable_bandpass": True
            },
            
            # Output configuration
            "interface": {
                "type": "socket",
                "ip": "127.0.0.1",
                "port": 9000,
                "format": "json"
            }
        }
        
        return config
        
    def start_odas(self):
        """Start ODAS process"""
        try:
            # Check if ODAS is installed
            if not Path("/usr/local/bin/odaslive").exists():
                logger.error("ODAS not installed. Run install.sh first.")
                return False
                
            # Start ODAS with configuration
            cmd = ["/usr/local/bin/odaslive", "-c", self.config_path]
            
            self.odas_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.running = True
            logger.info("ODAS process started")
            
            # Start monitoring thread
            threading.Thread(target=self._monitor_process, daemon=True).start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to start ODAS: {e}")
            return False
            
    def _monitor_process(self):
        """Monitor ODAS process output"""
        while self.running and self.odas_process:
            line = self.odas_process.stderr.readline()
            if line:
                logger.debug(f"ODAS: {line.decode().strip()}")
            else:
                # Process ended
                if self.running:
                    logger.error("ODAS process terminated unexpectedly")
                    self.restart_odas()
                break
                
    def restart_odas(self):
        """Restart ODAS process"""
        logger.info("Restarting ODAS...")
        self.stop_odas()
        time.sleep(2)
        self.start_odas()
        
    def stop_odas(self):
        """Stop ODAS process"""
        self.running = False
        
        if self.odas_process:
            self.odas_process.terminate()
            try:
                self.odas_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.odas_process.kill()
            
            self.odas_process = None
            logger.info("ODAS process stopped")
            
    def test_detection(self):
        """Test acoustic detection by listening to ODAS output"""
        try:
            # Connect to ODAS output socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('localhost', 9000))
            sock.settimeout(1.0)
            
            logger.info("Listening for acoustic detections...")
            logger.info("Make some noise to test detection...")
            
            buffer = ""
            start_time = time.time()
            
            while time.time() - start_time < 10:  # Test for 10 seconds
                try:
                    data = sock.recv(1024).decode('utf-8')
                    buffer += data
                    
                    # Process complete JSON messages
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line:
                            try:
                                msg = json.loads(line)
                                if 'src' in msg:
                                    for source in msg['src']:
                                        if source['E'] > 0.3:  # Energy threshold
                                            logger.info(
                                                f"Detection: Azimuth={source['azimuth']:.1f}°, "
                                                f"Elevation={source['elevation']:.1f}°, "
                                                f"Energy={source['E']:.2f}"
                                            )
                            except json.JSONDecodeError:
                                pass
                                
                except socket.timeout:
                    continue
                    
            sock.close()
            logger.info("Detection test complete")
            
        except Exception as e:
            logger.error(f"Test failed: {e}")

def main():
    """Main entry point for ODAS launcher"""
    launcher = ODASLauncher()
    
    # Generate config if needed
    if not Path(launcher.config_path).exists():
        logger.info("Generating ODAS configuration...")
        config = launcher.generate_config()
        # Save config (simplified for this example)
        logger.info(f"Config should be saved to {launcher.config_path}")
        
    # Start ODAS
    if launcher.start_odas():
        logger.info("ODAS started successfully")
        
        # Run test
        time.sleep(2)  # Wait for ODAS to initialize
        launcher.test_detection()
        
        # Keep running
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            
        launcher.stop_odas()
    else:
        logger.error("Failed to start ODAS")
        sys.exit(1)

if __name__ == "__main__":
    main()
