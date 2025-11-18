#!/usr/bin/env python3
"""
ADS1115 ADC Reader for Axon MAX MK2 Position Feedback
High-precision servo position reading through 16-bit ADC
"""

import time
import logging
from dataclasses import dataclass
from typing import Optional
import smbus2
import numpy as np

logger = logging.getLogger(__name__)

@dataclass
class ServoPositionData:
    """Servo position data from analog feedback"""
    raw_value: int
    voltage: float
    angle_degrees: float
    timestamp: float

class ADS1115Reader:
    """Interface for ADS1115 16-bit ADC"""
    
    # ADS1115 I2C address
    ADS1115_ADDRESS = 0x48  # Default, can be 0x49, 0x4A, or 0x4B
    
    # Registers
    REG_CONVERT = 0x00
    REG_CONFIG = 0x01
    
    # Config register bits
    CONFIG_OS_SINGLE = 0x8000
    CONFIG_MUX_AIN0_GND = 0x4000  # AIN0 vs GND
    CONFIG_PGA_4_096V = 0x0200    # +/- 4.096V range
    CONFIG_MODE_SINGLE = 0x0100
    CONFIG_DR_128SPS = 0x0080     # 128 samples per second
    CONFIG_COMP_QUE_DISABLE = 0x0003
    
    def __init__(self, i2c_bus: int = 1, address: int = 0x48, channel: int = 0):
        """
        Initialize ADS1115 reader
        
        Args:
            i2c_bus: I2C bus number (usually 1 on Pi)
            address: I2C address of ADS1115
            channel: ADC channel (0-3) for servo feedback
        """
        self.bus = smbus2.SMBus(i2c_bus)
        self.address = address
        self.channel = channel
        
        # Servo calibration values
        self.voltage_min = 0.5  # Voltage at 0 degrees
        self.voltage_max = 2.5  # Voltage at 180 degrees
        self.angle_min = 0.0
        self.angle_max = 180.0
        
        # Moving average filter
        self.filter_size = 5
        self.filter_buffer = []
        
        logger.info(f"ADS1115 initialized on bus {i2c_bus}, address 0x{address:02x}")
        
    def read_raw(self) -> int:
        """Read raw ADC value"""
        # Configure for single-ended reading on selected channel
        config = (
            self.CONFIG_OS_SINGLE |
            (0x4000 + (self.channel * 0x1000)) |  # Channel selection
            self.CONFIG_PGA_4_096V |
            self.CONFIG_MODE_SINGLE |
            self.CONFIG_DR_128SPS |
            self.CONFIG_COMP_QUE_DISABLE
        )
        
        # Write config
        self.bus.write_i2c_block_data(
            self.address,
            self.REG_CONFIG,
            [(config >> 8) & 0xFF, config & 0xFF]
        )
        
        # Wait for conversion (1/128 SPS = ~8ms)
        time.sleep(0.01)
        
        # Read result
        result = self.bus.read_i2c_block_data(self.address, self.REG_CONVERT, 2)
        value = (result[0] << 8) | result[1]
        
        # Handle negative values (two's complement)
        if value > 0x7FFF:
            value -= 0x10000
            
        return value
        
    def read_voltage(self) -> float:
        """Read voltage from ADC"""
        raw_value = self.read_raw()
        # Convert to voltage (4.096V reference, 16-bit ADC)
        voltage = raw_value * 4.096 / 32768.0
        return voltage
        
    def read_position(self) -> ServoPositionData:
        """Read servo position with filtering"""
        raw_value = self.read_raw()
        voltage = raw_value * 4.096 / 32768.0
        
        # Apply moving average filter
        self.filter_buffer.append(voltage)
        if len(self.filter_buffer) > self.filter_size:
            self.filter_buffer.pop(0)
        filtered_voltage = np.mean(self.filter_buffer)
        
        # Convert voltage to angle
        voltage_range = self.voltage_max - self.voltage_min
        angle_range = self.angle_max - self.angle_min
        
        if voltage_range > 0:
            angle = ((filtered_voltage - self.voltage_min) / voltage_range) * angle_range + self.angle_min
            angle = np.clip(angle, self.angle_min, self.angle_max)
        else:
            angle = 90.0  # Default center position
            
        return ServoPositionData(
            raw_value=raw_value,
            voltage=filtered_voltage,
            angle_degrees=angle,
            timestamp=time.time()
        )
        
    def calibrate(self, voltage_at_min: float, voltage_at_max: float):
        """Calibrate voltage to angle mapping"""
        self.voltage_min = voltage_at_min
        self.voltage_max = voltage_at_max
        logger.info(f"Calibrated: {voltage_at_min}V at {self.angle_min}°, {voltage_at_max}V at {self.angle_max}°")
