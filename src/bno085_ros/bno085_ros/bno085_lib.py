#!/usr/bin/env python3
"""
BNO085 9-Axis IMU Driver Library
=================================

A Python library for interfacing with the Hillcrest BNO085 9-axis IMU sensor
via I2C using the SHTP/SH2 protocol.

Features:
- SHTP Protocol implementation
- AR/VR Stabilized Rotation Vector (Quaternion)
- Calibrated Accelerometer, Gyroscope, Magnetometer
- Dynamic Calibration Data (DCD) management

Note: Works best with RST pin connected to GPIO for hardware reset.
      Without RST pin, uses extended soft reset timing.

Author: User
License: MIT
"""

import time
from smbus2 import SMBus, i2c_msg

# ========================================================================
# Constants
# ========================================================================

# I2C Addresses
BNO085_I2C_ADDR_DEFAULT = 0x4A
BNO085_I2C_ADDR_ALT     = 0x4B

# SHTP Channels
CHANNEL_COMMAND = 0
CHANNEL_EXECUTABLE = 1
CHANNEL_CONTROL = 2
CHANNEL_REPORTS = 3
CHANNEL_WAKE_REPORTS = 4
CHANNEL_GYRO_ROTATION = 5

# Report IDs
REPORT_ACCELEROMETER             = 0x01
REPORT_GYROSCOPE                 = 0x02
REPORT_MAGNETOMETER              = 0x03
REPORT_LINEAR_ACCELERATION       = 0x04
REPORT_ROTATION_VECTOR           = 0x05
REPORT_GRAVITY                   = 0x06
REPORT_GAME_ROTATION_VECTOR      = 0x08
REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09
REPORT_ARVR_STABILIZED_RV        = 0x28
REPORT_ARVR_STABILIZED_GAME_RV   = 0x29

# Command IDs (Channel 2)
CMD_SET_FEATURE_COMMAND          = 0xFD
CMD_GET_FEATURE_REQUEST          = 0xFE
CMD_COMMAND_REQUEST              = 0xF2
CMD_DCD                          = 0x06
CMD_ME_CALIBRATE                 = 0x07

# Sub-commands
DCD_SAVE_DCD                     = 0x01
ME_CAL_CONFIG                    = 0x00

# Q-Point Scalars
Q_POINT_14 = 2.0 ** -14  # Rotation Vector
Q_POINT_12 = 2.0 ** -12  # Accuracy
Q_POINT_9  = 2.0 ** -9   # Gyroscope
Q_POINT_8  = 2.0 ** -8   # Accelerometer
Q_POINT_4  = 2.0 ** -4   # Magnetometer


class BNO085:
    """BNO085 IMU Driver using SHTP protocol over I2C."""
    
    def __init__(self, bus: int, i2c_addr: int = BNO085_I2C_ADDR_DEFAULT):
        self.i2c_addr = i2c_addr
        self.bus_num = bus
        self.i2cbus = SMBus(bus)
        self.sequence_number = [0] * 6
        
        self.data = {
            'accel': [0.0, 0.0, 0.0],
            'gyro': [0.0, 0.0, 0.0],
            'mag': [0.0, 0.0, 0.0],
            'quat': [0.0, 0.0, 0.0, 1.0],
            'accuracy_quat': 0.0,
            'accuracy_status': 0
        }
        self._initialized = False

    def begin(self, enable_arvr_stabilized_rv=True) -> bool:
        """Initialize the sensor with extended soft reset timing."""
        try:
            # Extended soft reset sequence (for boards without RST pin)
            for attempt in range(3):
                # Send soft reset on executable channel
                self._send_packet(CHANNEL_EXECUTABLE, [0x01])
                time.sleep(0.8)  # Extended delay for soft reset
                
                # Drain any boot/advertisement packets
                for _ in range(10):
                    self._read_packet()
                    time.sleep(0.02)
                
                # Enable Raw Reports (100Hz = 10000us)
                self._enable_report(REPORT_ACCELEROMETER, 10000)
                time.sleep(0.05)
                self._enable_report(REPORT_GYROSCOPE, 10000)
                time.sleep(0.05)
                self._enable_report(REPORT_MAGNETOMETER, 10000)
                time.sleep(0.05)
                
                # Disable Rotation Vector types
                # self._enable_report(REPORT_ARVR_STABILIZED_RV, 0)
                pass
                
                # Check if we get sensor data
                for _ in range(10):
                    ch, data = self._read_packet()
                    if ch == CHANNEL_REPORTS and data:
                        self._initialized = True
                        return True
                    time.sleep(0.02)
            
            return False
            
        except Exception as e:
            print(f"BNO085 init failed: {e}")
            return False

    def _enable_report(self, report_id: int, interval_us: int):
        """Enable a sensor report at specified interval (microseconds)."""
        data = [
            CMD_SET_FEATURE_COMMAND,
            report_id,
            0x00, 0x00, 0x00,  # Flags, Change sensitivity
            (interval_us) & 0xFF,
            (interval_us >> 8) & 0xFF,
            (interval_us >> 16) & 0xFF,
            (interval_us >> 24) & 0xFF,
            0x00, 0x00, 0x00, 0x00,  # Batch interval
            0x00, 0x00, 0x00, 0x00   # Sensor specific
        ]
        self._send_packet(CHANNEL_CONTROL, data)

    def save_calibration(self) -> bool:
        """Save Dynamic Calibration Data (DCD) to flash."""
        data = [CMD_COMMAND_REQUEST, 0x00, CMD_DCD, DCD_SAVE_DCD,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self._send_packet(CHANNEL_CONTROL, data)
            time.sleep(0.1)
            return True
        except:
            return False

    def configure_calibration(self, accel=True, gyro=True, mag=True):
        """Configure which sensors participate in calibration."""
        data = [CMD_COMMAND_REQUEST, 0x00, CMD_ME_CALIBRATE, ME_CAL_CONFIG,
                0x01 if accel else 0x00,
                0x01 if gyro else 0x00,
                0x01 if mag else 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self._send_packet(CHANNEL_CONTROL, data)
            return True
        except:
            return False

    def get_data(self):
        """Poll sensor and return latest data dictionary."""
        # Poll up to 2 times to drain FIFO without blocking too long
        for _ in range(2):
            ch, data = self._read_packet()
            if ch == CHANNEL_REPORTS and data:
                self._parse_input_reports(data)
        return self.data

    # ========================================================================
    # Low Level SHTP / I2C
    # ========================================================================

    def _send_packet(self, channel: int, data: list):
        """Send SHTP packet."""
        length = len(data) + 4
        packet = [length & 0xFF, (length >> 8) & 0x7F, channel,
                  self.sequence_number[channel]] + data
        self.sequence_number[channel] = (self.sequence_number[channel] + 1) & 0xFF
        w = i2c_msg.write(self.i2c_addr, packet)
        self.i2cbus.i2c_rdwr(w)

    def _read_packet(self):
        """Read SHTP packet, return (channel, payload) or (None, None)."""
        try:
            # Read 64 bytes to cover batched reports (Accel+Gyro+Mag = ~30 bytes + overhead)
            # 64 * 9bits / 100kHz ~= 5.8ms. Safe for 100Hz.
            r = i2c_msg.read(self.i2c_addr, 64)
            self.i2cbus.i2c_rdwr(r)
            buf = list(r)
            
            length = ((buf[1] & 0x7F) << 8) | buf[0]
            channel = buf[2]
            
            if length <= 4 or length > 128:
                return None, None
            
            return channel, buf[4:length]
        except:
            return None, None

    def _parse_input_reports(self, data: list):
        """Parse sensor reports from channel 3 data."""
        i = 0
        while i < len(data):
            if i >= len(data):
                break
            report_id = data[i]
            
            # Timestamp reports (skip)
            if report_id == 0xFB or report_id == 0xFA:
                i += 5
                continue
            
            # AR/VR Stabilized Rotation Vector (0x28) - 14 bytes
            if report_id == REPORT_ARVR_STABILIZED_RV:
                if i + 14 <= len(data):
                    self.data['accuracy_status'] = data[i+2] & 0x03
                    self.data['quat'] = [
                        self._int16(data, i+4) * Q_POINT_14,
                        self._int16(data, i+6) * Q_POINT_14,
                        self._int16(data, i+8) * Q_POINT_14,
                        self._int16(data, i+10) * Q_POINT_14
                    ]
                    self.data['accuracy_quat'] = self._int16(data, i+12) * Q_POINT_12
                    i += 14
                else:
                    break
                    
            # Accelerometer (0x01) - 10 bytes
            elif report_id == REPORT_ACCELEROMETER:
                if i + 10 <= len(data):
                    self.data['accel'] = [
                        self._int16(data, i+4) * Q_POINT_8,
                        self._int16(data, i+6) * Q_POINT_8,
                        self._int16(data, i+8) * Q_POINT_8
                    ]
                    i += 10
                else:
                    break

            # Gyroscope (0x02) - 10 bytes
            elif report_id == REPORT_GYROSCOPE:
                if i + 10 <= len(data):
                    self.data['gyro'] = [
                        self._int16(data, i+4) * Q_POINT_9,
                        self._int16(data, i+6) * Q_POINT_9,
                        self._int16(data, i+8) * Q_POINT_9
                    ]
                    i += 10
                else:
                    break

            # Magnetometer (0x03) - 10 bytes
            elif report_id == REPORT_MAGNETOMETER:
                if i + 10 <= len(data):
                    self.data['accuracy_status'] = data[i+2] & 0x03
                    self.data['mag'] = [
                        self._int16(data, i+4) * Q_POINT_4,
                        self._int16(data, i+6) * Q_POINT_4,
                        self._int16(data, i+8) * Q_POINT_4
                    ]
                    i += 10
                else:
                    break
            
            # Rotation Vector (0x05) - 14 bytes
            elif report_id == REPORT_ROTATION_VECTOR:
                i += 14 if i + 14 <= len(data) else len(data)
            
            # Game Rotation Vector (0x08) - 12 bytes
            elif report_id == REPORT_GAME_ROTATION_VECTOR:
                i += 12 if i + 12 <= len(data) else len(data)
            
            else:
                i += 1  # Unknown, skip byte

    def _int16(self, data, idx) -> int:
        """Parse signed 16-bit little-endian integer."""
        val = (data[idx+1] << 8) | data[idx]
        return val - 0x10000 if val & 0x8000 else val

    def close(self):
        try:
            self.i2cbus.close()
        except:
            pass

    def __del__(self):
        self.close()


if __name__ == '__main__':
    print("BNO085 Test")
    imu = BNO085(1)
    if imu.begin():
        print("✅ BNO085 Initialized")
        try:
            while True:
                d = imu.get_data()
                q = d['quat']
                a = d['accel']
                print(f"Q: [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}] | A: [{a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}]")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped")
    else:
        print("❌ Failed to init BNO085")
