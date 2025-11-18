#!/bin/bash

# PTDTS Installation Script - Raspberry Pi 5 Compatible
# Uses lgpio for GPIO access (pigpio not compatible with Pi 5)

set -e

echo "========================================"
echo "PTDTS Installation (Pi 5 Compatible)"
echo "========================================"

# Check Raspberry Pi model
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model)
    echo "Detected: $MODEL"
fi

# Update system
echo "1. Updating system..."
sudo apt-get update
sudo apt-get upgrade -y

# Install lgpio for Pi 5 GPIO support
echo "2. Installing lgpio for GPIO access..."
cd /tmp
if [ ! -d "lg" ]; then
    wget https://github.com/joan2937/lg/archive/master.zip
    unzip -q master.zip
    cd lg-master
    make
    sudo make install
    sudo ldconfig
fi

# Install system dependencies
echo "3. Installing system dependencies..."
sudo apt-get install -y \
    python3-pip python3-venv python3-dev \
    build-essential cmake git \
    libfftw3-dev libconfig-dev libasound2-dev libjson-c-dev \
    libatlas-base-dev libopenblas-dev \
    libcamera-dev libcamera-apps libcamera-tools \
    python3-picamera2 python3-opencv \
    python3-gpiozero \
    portaudio19-dev \
    ffmpeg \
    libusb-1.0-0-dev \
    i2c-tools

# Install Python lgpio binding
echo "4. Installing Python GPIO libraries..."
pip3 install --break-system-packages lgpio gpiozero

# NO pigpiod service needed for Pi 5 - lgpio handles GPIO directly

# Enable I2C for reSpeaker
echo "5. Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

# Install ODAS for acoustic detection
echo "6. Installing ODAS..."
if [ ! -d "/opt/odas" ]; then
    cd /tmp
    git clone https://github.com/introlab/odas.git
    cd odas
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
fi

# Create directories
echo "7. Creating project directories..."
sudo mkdir -p /var/log/ptdts
sudo mkdir -p /etc/ptdts
sudo chown pi:pi /var/log/ptdts

# Install Python dependencies
echo "8. Installing Python packages..."
cd /home/pi/ptdts_framework
pip3 install --break-system-packages -r requirements_pi5.txt

# Download YOLO models
echo "9. Downloading AI models..."
mkdir -p models
cd models
if [ ! -f "yolo11n.pt" ]; then
    wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolo11n.pt
fi
cd ..

# Setup audio for reSpeaker
echo "10. Configuring audio..."
sudo tee /etc/asound.conf > /dev/null <<'EOF'
pcm.!default {
    type asym
    playback.pcm "plughw:0,0"
    capture.pcm "plughw:1,0"  # reSpeaker on USB
}

ctl.!default {
    type hw
    card 1
}
EOF

# Create systemd service
echo "11. Creating systemd service..."
sudo tee /etc/systemd/system/ptdts.service > /dev/null <<EOF
[Unit]
Description=PTDTS - Pan Tilt Drone Detection System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/ptdts_framework
Environment="PYTHONPATH=/home/pi/ptdts_framework"
ExecStart=/usr/bin/python3 /home/pi/ptdts_framework/ptdts_main.py
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable ptdts.service

echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "System is configured for Raspberry Pi 5 with:"
echo "  - lgpio for GPIO access (pigpio not needed)"
echo "  - ODAS for acoustic detection"
echo "  - Dual camera support"
echo "  - Web interface on port 5000"
echo ""
echo "To start the system:"
echo "  sudo systemctl start ptdts"
echo ""
echo "To test the quick demo:"
echo "  cd ~/ptdts_test_system"
echo "  python3 ptdts_test.py"
echo ""
echo "Access web interface at:"
echo "  http://$(hostname -I | cut -d' ' -f1):5000"
echo ""
echo "Please reboot to ensure all changes take effect:"
echo "  sudo reboot"
