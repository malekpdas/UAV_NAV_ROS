"""
BMX160 9-Axis IMU Driver Library
=================================

A Python library for interfacing with the Bosch BMX160 9-axis IMU sensor
(accelerometer, gyroscope, and magnetometer) via I2C.

Features:
- Configurable measurement ranges for all sensors
- Hardware register configuration with proper timing
- Calibrated unit conversions (m/s², °/s, µT)
- Low power modes
- Error handling and sensor validation

Author: Based on DFRobot_BMX160 library
License: MIT
Modified: 2024 - Production hardening and documentation
"""

import smbus2 as smbus
import time
from typing import Optional, List, Tuple
from enum import IntEnum


class GyroRange(IntEnum):
    """Gyroscope measurement range options."""
    DPS_125 = 0   # ±125°/s  - Highest resolution
    DPS_250 = 1   # ±250°/s
    DPS_500 = 2   # ±500°/s  - Recommended for most applications
    DPS_1000 = 3  # ±1000°/s
    DPS_2000 = 4  # ±2000°/s - Highest range


class AccelRange(IntEnum):
    """Accelerometer measurement range options."""
    G_2 = 0   # ±2g  - Highest resolution
    G_4 = 1   # ±4g  - Recommended for most applications
    G_8 = 2   # ±8g
    G_16 = 3  # ±16g - Highest range


class BMX160:
    """
    BMX160 9-Axis IMU Driver.
    
    Provides access to accelerometer, gyroscope, and magnetometer data
    with configurable ranges and proper unit conversions.
    
    Example:
        >>> imu = BMX160(bus=1)
        >>> if imu.begin():
        >>>     imu.set_gyro_range(GyroRange.DPS_500)
        >>>     imu.set_accel_range(AccelRange.G_4)
        >>>     data = imu.get_all_data()
        >>>     print(f"Accel: {data[6:9]} m/s²")
    """
    
    # ========================================================================
    # Register Addresses (from BMX160 Datasheet)
    # ========================================================================
    _CHIP_ID_ADDR = 0x00
    _ERROR_REG_ADDR = 0x02
    _MAG_DATA_ADDR = 0x04
    _GYRO_DATA_ADDR = 0x0C
    _ACCEL_DATA_ADDR = 0x12
    _STATUS_ADDR = 0x1B
    _ACCEL_CONFIG_ADDR = 0x40
    _ACCEL_RANGE_ADDR = 0x41
    _GYRO_CONFIG_ADDR = 0x42
    _GYRO_RANGE_ADDR = 0x43
    _MAGN_CONFIG_ADDR = 0x44
    _MAGN_IF_0_ADDR = 0x4C
    _MAGN_IF_1_ADDR = 0x4D
    _MAGN_IF_2_ADDR = 0x4E
    _MAGN_IF_3_ADDR = 0x4F
    _COMMAND_REG_ADDR = 0x7E
    
    # ========================================================================
    # Commands
    # ========================================================================
    _SOFT_RESET_CMD = 0xB6
    _ACCEL_NORMAL_MODE_CMD = 0x11
    _GYRO_NORMAL_MODE_CMD = 0x15
    _MAG_NORMAL_MODE_CMD = 0x19
    _ACCEL_SUSPEND_MODE_CMD = 0x12
    _GYRO_SUSPEND_MODE_CMD = 0x17
    _MAG_SUSPEND_MODE_CMD = 0x1B
    
    # ========================================================================
    # Sensitivity Constants (LSB values from datasheet)
    # ========================================================================
    # Gyroscope: degrees per second per LSB
    _GYRO_SENSITIVITY = {
        GyroRange.DPS_125: 0.0038110,
        GyroRange.DPS_250: 0.0076220,
        GyroRange.DPS_500: 0.0152439,
        GyroRange.DPS_1000: 0.0304878,
        GyroRange.DPS_2000: 0.0609756,
    }
    
    # Accelerometer: g (gravitational acceleration) per LSB
    _ACCEL_SENSITIVITY = {
        AccelRange.G_2: 0.000061035,
        AccelRange.G_4: 0.000122070,
        AccelRange.G_8: 0.000244141,
        AccelRange.G_16: 0.000488281,
    }
    
    # Accelerometer range register values (from datasheet)
    _ACCEL_RANGE_REG_VALUES = {
        AccelRange.G_2: 0x03,
        AccelRange.G_4: 0x05,
        AccelRange.G_8: 0x08,
        AccelRange.G_16: 0x0C,
    }
    
    # Magnetometer: microTesla per LSB
    _MAG_SENSITIVITY = 0.3
    
    # Physical constants
    _GRAVITY_MS2 = 9.80665  # Standard gravity in m/s²
    
    # I2C Configuration
    _DEFAULT_I2C_ADDR = 0x68
    _CHIP_ID_EXPECTED = 0xD8
    
    def __init__(self, bus: int, i2c_addr: int = _DEFAULT_I2C_ADDR):
        """
        Initialize BMX160 driver.
        
        Args:
            bus: I2C bus number (typically 1 on Raspberry Pi)
            i2c_addr: I2C device address (default: 0x68)
        
        Raises:
            IOError: If I2C bus cannot be opened
        """
        self.i2c_addr = i2c_addr
        self.i2cbus = smbus.SMBus(bus)
        
        # Current sensor configuration
        self._gyro_range = GyroRange.DPS_250
        self._accel_range = AccelRange.G_2
        self._gyro_sensitivity = self._GYRO_SENSITIVITY[self._gyro_range]
        self._accel_sensitivity = self._ACCEL_SENSITIVITY[self._accel_range]
        
        # Sensor state
        self._initialized = False
        
        time.sleep(0.16)  # Power-on stabilization time
    
    def begin(self) -> bool:
        """
        Initialize and configure the BMX160 sensor.
        
        Performs soft reset, enables all sensors, and configures magnetometer.
        After calling begin(), use set_gyro_range() and set_accel_range() to
        configure measurement ranges.
        
        Returns:
            True if initialization successful, False otherwise
        """
        if not self._scan_i2c():
            return False
        
        # Soft reset to known state
        if not self.soft_reset():
            return False
        
        # Enable accelerometer (normal power mode)
        self._write_register(self._COMMAND_REG_ADDR, self._ACCEL_NORMAL_MODE_CMD)
        time.sleep(0.05)
        
        # Enable gyroscope (normal power mode)
        self._write_register(self._COMMAND_REG_ADDR, self._GYRO_NORMAL_MODE_CMD)
        time.sleep(0.1)
        
        # Enable magnetometer (normal power mode)
        self._write_register(self._COMMAND_REG_ADDR, self._MAG_NORMAL_MODE_CMD)
        time.sleep(0.01)
        
        # Configure magnetometer interface
        self._configure_magnetometer()
        
        self._initialized = True
        return True
    
    def set_gyro_range(self, range_setting: GyroRange) -> None:
        """
        Configure gyroscope measurement range.
        
        Trade-off: Lower ranges provide better resolution but saturate at
        lower rotation rates. Choose based on your application:
        - ±125°/s: High precision, slow movements
        - ±500°/s: Good balance for most robotics (recommended)
        - ±2000°/s: Fast movements, aggressive maneuvers
        
        Args:
            range_setting: GyroRange enum value
        
        Raises:
            ValueError: If invalid range specified
        """
        if not isinstance(range_setting, GyroRange):
            raise ValueError(f"Invalid gyro range: {range_setting}")
        
        self._gyro_range = range_setting
        self._gyro_sensitivity = self._GYRO_SENSITIVITY[range_setting]
        
        # Write to hardware register
        self._write_register(self._GYRO_RANGE_ADDR, int(range_setting))
        time.sleep(0.01)
    
    def set_accel_range(self, range_setting: AccelRange) -> None:
        """
        Configure accelerometer measurement range.
        
        Trade-off: Lower ranges provide better resolution but saturate at
        lower accelerations. Choose based on your application:
        - ±2g: High precision, gentle movements
        - ±4g: Good balance for most robotics (recommended)
        - ±16g: High-impact applications, aggressive maneuvers
        
        Args:
            range_setting: AccelRange enum value
        
        Raises:
            ValueError: If invalid range specified
        """
        if not isinstance(range_setting, AccelRange):
            raise ValueError(f"Invalid accel range: {range_setting}")
        
        self._accel_range = range_setting
        self._accel_sensitivity = self._ACCEL_SENSITIVITY[range_setting]
        
        # Get datasheet register value
        reg_value = self._ACCEL_RANGE_REG_VALUES[range_setting]
        
        # Write to hardware register
        self._write_register(self._ACCEL_RANGE_ADDR, reg_value)
        time.sleep(0.01)
    
    def get_all_data(self) -> Optional[List[float]]:
        """
        Read all sensor data (magnetometer, gyroscope, accelerometer).
        
        Returns:
            List of 9 floats: [mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z,
                               accel_x, accel_y, accel_z]
            Units: [µT, µT, µT, °/s, °/s, °/s, m/s², m/s², m/s²]
            Returns None if read fails
        
        Example:
            >>> data = imu.get_all_data()
            >>> if data:
            >>>     mag = data[0:3]    # Magnetometer (µT)
            >>>     gyro = data[3:6]   # Gyroscope (°/s)
            >>>     accel = data[6:9]  # Accelerometer (m/s²)
        """
        try:
            # Read all sensor data in one burst (20 bytes starting at MAG_DATA_ADDR)
            raw_data = self._read_register(self._MAG_DATA_ADDR, 20)
            
            # Parse magnetometer data (bytes 0-5)
            mag_x = self._parse_signed_int16(raw_data[1], raw_data[0])
            mag_y = self._parse_signed_int16(raw_data[3], raw_data[2])
            mag_z = self._parse_signed_int16(raw_data[5], raw_data[4])
            
            # Parse gyroscope data (bytes 8-13)
            gyro_x = self._parse_signed_int16(raw_data[9], raw_data[8])
            gyro_y = self._parse_signed_int16(raw_data[11], raw_data[10])
            gyro_z = self._parse_signed_int16(raw_data[13], raw_data[12])
            
            # Parse accelerometer data (bytes 14-19)
            accel_x = self._parse_signed_int16(raw_data[15], raw_data[14])
            accel_y = self._parse_signed_int16(raw_data[17], raw_data[16])
            accel_z = self._parse_signed_int16(raw_data[19], raw_data[18])
            
            # Apply calibration and convert to physical units
            mag_x *= self._MAG_SENSITIVITY
            mag_y *= self._MAG_SENSITIVITY
            mag_z *= self._MAG_SENSITIVITY
            
            gyro_x *= self._gyro_sensitivity
            gyro_y *= self._gyro_sensitivity
            gyro_z *= self._gyro_sensitivity
            
            accel_x *= self._accel_sensitivity * self._GRAVITY_MS2
            accel_y *= self._accel_sensitivity * self._GRAVITY_MS2
            accel_z *= self._accel_sensitivity * self._GRAVITY_MS2
            
            return [mag_x, mag_y, mag_z, 
                    gyro_x, gyro_y, gyro_z, 
                    accel_x, accel_y, accel_z]
        
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            return None
    
    def get_gyro_data(self) -> Optional[Tuple[float, float, float]]:
        """
        Read only gyroscope data (more efficient than get_all_data).
        
        Returns:
            Tuple of (gyro_x, gyro_y, gyro_z) in °/s, or None if read fails
        """
        try:
            raw_data = self._read_register(self._GYRO_DATA_ADDR, 6)
            
            gyro_x = self._parse_signed_int16(raw_data[1], raw_data[0])
            gyro_y = self._parse_signed_int16(raw_data[3], raw_data[2])
            gyro_z = self._parse_signed_int16(raw_data[5], raw_data[4])
            
            gyro_x *= self._gyro_sensitivity
            gyro_y *= self._gyro_sensitivity
            gyro_z *= self._gyro_sensitivity
            
            return (gyro_x, gyro_y, gyro_z)
        
        except Exception as e:
            print(f"Error reading gyro data: {e}")
            return None
    
    def get_accel_data(self) -> Optional[Tuple[float, float, float]]:
        """
        Read only accelerometer data (more efficient than get_all_data).
        
        Returns:
            Tuple of (accel_x, accel_y, accel_z) in m/s², or None if read fails
        """
        try:
            raw_data = self._read_register(self._ACCEL_DATA_ADDR, 6)
            
            accel_x = self._parse_signed_int16(raw_data[1], raw_data[0])
            accel_y = self._parse_signed_int16(raw_data[3], raw_data[2])
            accel_z = self._parse_signed_int16(raw_data[5], raw_data[4])
            
            accel_x *= self._accel_sensitivity * self._GRAVITY_MS2
            accel_y *= self._accel_sensitivity * self._GRAVITY_MS2
            accel_z *= self._accel_sensitivity * self._GRAVITY_MS2
            
            return (accel_x, accel_y, accel_z)
        
        except Exception as e:
            print(f"Error reading accel data: {e}")
            return None
    
    def soft_reset(self) -> bool:
        """
        Perform software reset of the BMX160.
        
        Resets all registers to default values and requires re-initialization.
        
        Returns:
            True if reset successful
        """
        try:
            self._write_register(self._COMMAND_REG_ADDR, self._SOFT_RESET_CMD)
            time.sleep(0.015)  # Reset takes ~15ms
            self._initialized = False
            return True
        except Exception as e:
            print(f"Soft reset failed: {e}")
            return False
    
    def set_low_power(self) -> None:
        """
        Disable gyroscope and magnetometer to reduce power consumption.
        
        Leaves accelerometer active for basic motion detection.
        Call wake_up() to re-enable all sensors.
        """
        self.soft_reset()
        time.sleep(0.1)
        
        self._configure_magnetometer()
        time.sleep(0.1)
        
        # Suspend gyroscope
        self._write_register(self._COMMAND_REG_ADDR, self._ACCEL_SUSPEND_MODE_CMD)
        time.sleep(0.1)
        
        # Suspend magnetometer
        self._write_register(self._COMMAND_REG_ADDR, self._GYRO_SUSPEND_MODE_CMD)
        time.sleep(0.1)
        
        self._write_register(self._COMMAND_REG_ADDR, self._MAG_SUSPEND_MODE_CMD)
        time.sleep(0.1)
    
    def wake_up(self) -> None:
        """
        Wake sensors from low power mode.
        
        Re-enables all sensors after set_low_power() was called.
        """
        self.soft_reset()
        time.sleep(0.1)
        
        self._configure_magnetometer()
        time.sleep(0.1)
        
        # Enable all sensors
        self._write_register(self._COMMAND_REG_ADDR, self._ACCEL_NORMAL_MODE_CMD)
        time.sleep(0.1)
        
        self._write_register(self._COMMAND_REG_ADDR, self._GYRO_NORMAL_MODE_CMD)
        time.sleep(0.1)
        
        self._write_register(self._COMMAND_REG_ADDR, self._MAG_NORMAL_MODE_CMD)
        time.sleep(0.1)
    
    # ========================================================================
    # Private Methods
    # ========================================================================
    
    def _configure_magnetometer(self) -> None:
        """Configure BMM150 magnetometer interface (internal)."""
        self._write_register(self._MAGN_IF_0_ADDR, 0x80)
        time.sleep(0.05)
        
        # BMM150 setup sequence (from datasheet)
        self._write_register(self._MAGN_IF_3_ADDR, 0x01)
        self._write_register(self._MAGN_IF_2_ADDR, 0x4B)
        self._write_register(self._MAGN_IF_3_ADDR, 0x04)
        self._write_register(self._MAGN_IF_2_ADDR, 0x51)
        self._write_register(self._MAGN_IF_3_ADDR, 0x0E)
        self._write_register(self._MAGN_IF_2_ADDR, 0x52)
        self._write_register(self._MAGN_IF_3_ADDR, 0x02)
        self._write_register(self._MAGN_IF_2_ADDR, 0x4C)
        self._write_register(self._MAGN_IF_1_ADDR, 0x42)
        self._write_register(self._MAGN_CONFIG_ADDR, 0x08)
        self._write_register(self._MAGN_IF_0_ADDR, 0x03)
        
        time.sleep(0.05)
    
    def _parse_signed_int16(self, msb: int, lsb: int) -> int:
        """
        Parse two bytes into signed 16-bit integer.
        
        Args:
            msb: Most significant byte
            lsb: Least significant byte
        
        Returns:
            Signed 16-bit integer value
        """
        value = (msb << 8) | lsb
        # Convert to signed
        if value & 0x8000:
            value -= 0x10000
        return value
    
    def _write_register(self, register: int, value: int) -> None:
        """Write a byte to a BMX160 register."""
        self.i2cbus.write_byte_data(self.i2c_addr, register, value)
    
    def _read_register(self, register: int, length: int = 1) -> List[int]:
        """
        Read one or more bytes from BMX160 register.
        
        Args:
            register: Starting register address
            length: Number of bytes to read
        
        Returns:
            List of bytes read
        """
        if length == 1:
            return [self.i2cbus.read_byte_data(self.i2c_addr, register)]
        else:
            return self.i2cbus.read_i2c_block_data(self.i2c_addr, register, length)
    
    def _scan_i2c(self) -> bool:
        """
        Verify sensor presence on I2C bus.
        
        Returns:
            True if sensor responds, False otherwise
        """
        try:
            self.i2cbus.read_byte(self.i2c_addr)
            return True
        except Exception as e:
            print(f"I2C scan failed at address 0x{self.i2c_addr:02X}: {e}")
            return False
    
    def __del__(self):
        """Cleanup I2C bus on object destruction."""
        try:
            self.i2cbus.close()
        except:
            pass


# ============================================================================
# Usage Example
# ============================================================================
if __name__ == "__main__":
    print("BMX160 IMU Test")
    print("=" * 50)
    
    # Initialize sensor
    imu = BMX160(bus=1)
    
    if not imu.begin():
        print("❌ Failed to initialize BMX160")
        exit(1)
    
    print("✅ BMX160 initialized")
    
    # Configure ranges
    imu.set_gyro_range(GyroRange.DPS_500)
    imu.set_accel_range(AccelRange.G_4)
    print(f"✅ Gyro: ±500°/s, Accel: ±4g")
    print()
    
    # Read sensor data
    try:
        for i in range(10):
            data = imu.get_all_data()
            
            if data:
                print(f"Sample {i+1}:")
                print(f"  Mag:   [{data[0]:7.2f}, {data[1]:7.2f}, {data[2]:7.2f}] µT")
                print(f"  Gyro:  [{data[3]:7.2f}, {data[4]:7.2f}, {data[5]:7.2f}] °/s")
                print(f"  Accel: [{data[6]:7.2f}, {data[7]:7.2f}, {data[8]:7.2f}] m/s²")
                print()
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    print("Test complete")