# PTDTS - Pan Tilt Drone Detection System

## Comprehensive Framework Documentation

### Overview

The Pan Tilt Drone Detection System (PTDTS) is an advanced autonomous drone detection and tracking platform that combines acoustic wake detection with visual AI tracking. The system uses a 4-microphone array for initial detection and dual cameras for visual confirmation and tracking.

### Key Features

- **Acoustic Detection**: ODAS-based sound source localization using reSpeaker XVF3800
- **Dual Camera System**: High-resolution detection + high-FPS tracking
- **Kalman Filter Lead Prediction**: Predictive targeting for moving drones
- **360° Continuous Pan**: Unlimited rotation with slip ring power
- **Web Interface**: Real-time control and monitoring
- **Raspberry Pi 5 Compatible**: Uses lgpio for GPIO control

### Hardware Components

#### Compute
- Raspberry Pi 5 (16GB RAM, 512GB M.2 SSD)

#### Motion Control
- **Pan Motor**: 30:1 Metal Gearmotor 37Dx68L with 64 CPR Encoder
- **Motor Driver**: Pololu DRV8874 Single Brushed DC Motor Driver
- **Tilt Servo**: Axon MAX MK2 (6V, configurable modes)
- **Gear Ratio**: 24:108 (1:4.5 reduction)

#### Sensors
- **Acoustic**: reSpeaker XMOS XVF3800 4-mic array
- **Camera 1**: Raspberry Pi HQ Camera (Sony IMX477) with 2.8-12mm varifocal lens
- **Camera 2**: Raspberry Pi Global Shutter Camera (Sony IMX296) with 25mm F1.4 lens

#### Power
- **Main Supply**: LRS-150-24 (24V, 6.25A)
- **Voltage Regulators**: 
  - D36V50F5 (5V for Pi)
  - D36V50F6 (6V for servo)
  - D36V50F12 (12V for motor)
- **Power Distribution**: 2-wire slip ring for continuous rotation

### Software Architecture

```
PTDTS Framework
├── Core Controller (ptdts_main.py)
│   ├── Motor Control (PID + Encoder)
│   ├── Servo Control (Smooth motion)
│   ├── State Machine (IDLE/SCANNING/DETECTING/TRACKING)
│   └── Target Management
│
├── Detection Systems
│   ├── Acoustic Detector (ODAS integration)
│   │   ├── DoA Estimation
│   │   ├── Frequency Filtering (100-500Hz)
│   │   └── Drone Signature Analysis
│   │
│   └── Vision System (Dual Camera)
│       ├── YOLO Detection (HQ Camera)
│       ├── Object Tracking (GS Camera)
│       └── NCNN Optimization
│
├── Prediction & Tracking
│   ├── Kalman Filter (6-state model)
│   ├── Lead Prediction
│   └── Coordinate Transformation
│
└── Interfaces
    ├── Web Dashboard (Flask + SocketIO)
    ├── REST API
    └── WebRTC Video Streaming
```

### Installation

#### Prerequisites
- Raspberry Pi 5 with latest Raspberry Pi OS
- Internet connection for package downloads
- All hardware components connected as per wiring diagram

#### Quick Install (Raspberry Pi 5)
```bash
# Clone repository (or copy files)
cd ~
# Assuming files are in ptdts_framework/

# Run installation script
cd ptdts_framework
chmod +x install_pi5.sh
./install_pi5.sh
```

#### Manual Installation
```bash
# 1. Install lgpio (Pi 5 GPIO support)
cd /tmp
wget https://github.com/joan2937/lg/archive/master.zip
unzip master.zip
cd lg-master
make
sudo make install

# 2. Install system dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-gpiozero \
    libcamera-dev python3-picamera2 \
    libfftw3-dev libconfig-dev libasound2-dev

# 3. Install Python packages
pip3 install --break-system-packages -r requirements_pi5.txt

# 4. Install ODAS (acoustic detection)
git clone https://github.com/introlab/odas.git
cd odas && mkdir build && cd build
cmake .. && make && sudo make install

# 5. Download YOLO model
cd ~/ptdts_framework/models
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolo11n.pt
```

### GPIO Pin Assignments

| Function | GPIO Pin | Physical Pin | Notes |
|----------|----------|--------------|-------|
| Motor IN1 | GPIO 23 | Pin 16 | PWM control |
| Motor IN2 | GPIO 24 | Pin 18 | PWM control |
| Encoder A | GPIO 17 | Pin 11 | Pull-up enabled |
| Encoder B | GPIO 27 | Pin 13 | Pull-up enabled |
| Servo PWM | GPIO 18 | Pin 12 | Hardware PWM |
| 5V Power | - | Pin 2, 4 | From D36V50F5 |
| Ground | - | Pin 6, 9, 14, 20 | Common ground |

### Configuration

Edit `config.json` to customize system parameters:

```json
{
    "acoustic_enabled": true,
    "acoustic_threshold": 0.6,
    "acoustic_frequency_min": 100.0,
    "acoustic_frequency_max": 500.0,
    
    "visual_enabled": true,
    "detection_confidence": 0.5,
    "yolo_model": "yolo11n",
    
    "lead_prediction_enabled": true,
    "lead_time": 1.0,
    
    "web_port": 5000
}
```

### Calibration

Run the calibration utility before first use:

```bash
cd ~/ptdts_framework
python3 calibration.py
```

This will calibrate:
1. Encoder counts per revolution
2. Servo travel limits
3. Camera field of view
4. Acoustic array orientation

### Usage

#### Starting the System

**As a service (recommended):**
```bash
sudo systemctl start ptdts
sudo systemctl enable ptdts  # Auto-start on boot
```

**Manual start:**
```bash
cd ~/ptdts_framework
python3 ptdts_main.py
```

**Quick test system (48-hour demo):**
```bash
cd ~/ptdts_test_system
python3 ptdts_test.py
```

#### Web Interface

Access the dashboard at: `http://<pi-ip>:5000`

Features:
- Real-time video feed
- Pan/tilt manual control
- System state monitoring
- Target list and tracking
- Statistics and logging
- Emergency stop

#### API Endpoints

- `GET /api/status` - System status
- `GET /api/control/pan/<angle>` - Pan to angle (0-360°)
- `GET /api/control/tilt/<angle>` - Tilt to angle (0-180°)
- `GET /api/control/state/<state>` - Set system state
- `GET /api/targets` - Get target list
- `GET /video_feed` - Live video stream

### System States

- **IDLE**: System on standby
- **SCANNING**: 360° search pattern
- **DETECTING**: Processing potential target
- **TRACKING**: Actively following target
- **ENGAGING**: Target locked (for external systems)
- **ERROR**: Fault condition

### Troubleshooting

#### GPIO Access Issues (Pi 5)
```bash
# Ensure lgpio is installed
ls /usr/local/lib/liblgpio*
# Should see liblgpio.so

# Test GPIO access
python3 -c "from gpiozero import LED; LED(23).on()"
```

#### Camera Not Detected
```bash
# List cameras
libcamera-hello --list-cameras

# Test single camera
rpicam-hello --camera 0  # HQ camera
rpicam-hello --camera 1  # GS camera
```

#### Acoustic Detection Not Working
```bash
# Check USB audio devices
arecord -l  # Should show XVF3800

# Test ODAS
cd ~/ptdts_framework
python3 odas_launcher.py
```

#### Motor Not Moving
```bash
# Check wiring and power
# Run motor test
python3 calibration.py
# Select "Motor Test" option
```

### Performance Optimization

#### For Raspberry Pi 5:
- Uses lgpio for efficient GPIO access
- NCNN format for YOLO (62% faster than PyTorch)
- Hardware H.264 encoding for streaming
- Dual camera pipeline optimization

#### Expected Performance:
- Acoustic detection: < 100ms latency
- Visual detection: 15-25 FPS (YOLO11n)
- Tracking: 40-60 FPS (reduced resolution)
- Web streaming: 200ms latency (WebRTC)
- Lead prediction: 10Hz update rate

### Safety Considerations

1. **Power**: Always connect 24V supply before powering Pi
2. **Motors**: Keep clear of moving parts during operation
3. **Emergency Stop**: Web interface has E-stop button
4. **Limits**: Servo has configurable min/max angles
5. **Encoder**: Regular calibration ensures accurate positioning

### Advanced Features

#### Lead Prediction
The Kalman filter predicts target position 1 second ahead:
- 6-state model (position + velocity)
- Constant velocity assumption
- Adjustable lead time in config

#### Acoustic Filtering
Targets DJI Phantom signature:
- Fundamental: ~200Hz
- Bandpass: 100-500Hz
- 4-rotor modulation pattern

#### Scan Patterns
Configurable in `config.json`:
- **Default**: Spiral pattern
- **Fast**: Continuous rotation
- **Thorough**: Grid pattern with multiple elevations

### Development

#### Project Structure
```
ptdts_framework/
├── ptdts_main.py          # Main controller
├── odas_launcher.py       # Acoustic system
├── calibration.py         # Calibration utility
├── config.json            # Configuration
├── requirements_pi5.txt   # Dependencies
├── install_pi5.sh         # Installation script
├── templates/
│   └── dashboard.html     # Web interface
├── models/                # AI models
│   └── yolo11n.pt
└── logs/                  # System logs
```

#### Adding Custom Detection Models
1. Train YOLO model on drone dataset
2. Export to NCNN format for ARM optimization
3. Place in `models/` directory
4. Update `config.json` with model name

#### Extending the System
- ZMQ interface for external systems (port 5555)
- Modular detection pipeline
- Plugin architecture for new sensors

### Logging

Logs are stored in `/var/log/ptdts/ptdts.log`

View logs:
```bash
tail -f /var/log/ptdts/ptdts.log
```

### Performance Metrics

Typical system performance:
- Detection range: 50-100m (acoustic), 30-50m (visual)
- Angular accuracy: ±5° close range, ±15° at max range  
- Response time: < 1 second from detection to tracking
- Tracking speed: Up to 180°/second pan, 180°/second tilt
- Power consumption: ~30W typical, 50W max

### Support and Contributing

For issues, feature requests, or contributions:
1. Document issue with system logs
2. Include hardware configuration
3. Provide steps to reproduce

### License

This project is provided as-is for educational and research purposes.

### Acknowledgments

- ODAS by IntRoLab for acoustic processing
- Ultralytics for YOLO implementation
- gpiozero team for Pi GPIO abstraction
- lgpio by Joan for Pi 5 GPIO support

---

## Quick Reference Card

### Emergency Procedures
```bash
# Stop all motors immediately
sudo systemctl stop ptdts

# Disable autostart
sudo systemctl disable ptdts

# Manual motor stop (if software fails)
# Disconnect motor driver power (12V line)
```

### Common Commands
```bash
# System control
sudo systemctl start ptdts
sudo systemctl stop ptdts
sudo systemctl status ptdts

# View logs
journalctl -u ptdts -f
tail -f /var/log/ptdts/ptdts.log

# Calibration
python3 calibration.py

# Test motors
python3 -c "from gpiozero import PWMOutputDevice; m=PWMOutputDevice(23); m.pulse()"

# Test cameras
libcamera-hello --list-cameras
rpicam-hello --camera 0
```

### LED Status Indicators (Web Dashboard)
- 🟢 Green: System operational
- 🟡 Yellow: Scanning/Detecting  
- 🔵 Blue: Target tracking
- 🔴 Red: Error/Emergency stop
- ⚫ Gray: System idle

---

*Version 1.0 - October 2025*
*Designed for Raspberry Pi 5 with lgpio support*
