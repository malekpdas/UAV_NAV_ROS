import time
import math
import struct
from smbus2 import SMBus, i2c_msg

# Registers
BMX160_CHIP_ID = 0x00
BMX160_PMU_STATUS = 0x03
BMX160_MAG_DATA_0 = 0x04
BMX160_GYRO_DATA_0 = 0x0C
BMX160_ACCEL_DATA_0 = 0x12
BMX160_STATUS = 0x1B
BMX160_CMD = 0x7E

# Configuration Registers
BMX160_MAG_CONF = 0x44
BMX160_MAG_IF_0 = 0x4C
BMX160_ACCEL_CONF = 0x40
BMX160_ACCEL_RANGE = 0x41
BMX160_GYRO_CONF = 0x42
BMX160_GYRO_RANGE = 0x43

# Commands
CMD_SOFT_RESET = 0xB6
CMD_PMU_ACCEL_NORMAL = 0x11
CMD_PMU_GYRO_NORMAL = 0x15
CMD_PMU_MAG_NORMAL = 0x19

# Constants
GRAVITY = 9.80665

class BMX160:
    def __init__(self, bus_num=1, address=0x68):
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self.accel_range = 2.0 # g
        self.gyro_range = 2000.0 # deg/s
        self.mag_range = 1.0 # uT conversion factor placeholder

    def open(self):
        try:
            self.bus = SMBus(self.bus_num)
            return True
        except Exception as e:
            print(f"Failed to open I2C bus {self.bus_num}: {e}")
            return False

    def check_id(self):
        try:
            chip_id = self.bus.read_byte_data(self.address, BMX160_CHIP_ID)
            return chip_id == 0xD8
        except Exception:
            return False

    def soft_reset(self):
        try:
            self.bus.write_byte_data(self.address, BMX160_CMD, CMD_SOFT_RESET)
            time.sleep(0.1)
        except Exception:
            pass

    def init_device(self, accel_range=4, gyro_range=2000):
        # 1. Bring up sensors (PMU Normal)
        # Sequence matters. Mag often requires special handling.
        # For BMX160, Mag is strictly internal but accessed via aux interface or direct if configured.
        # Default behavior:
        self.bus.write_byte_data(self.address, BMX160_CMD, CMD_PMU_ACCEL_NORMAL)
        time.sleep(0.02)
        self.bus.write_byte_data(self.address, BMX160_CMD, CMD_PMU_GYRO_NORMAL)
        time.sleep(0.02)
        self.bus.write_byte_data(self.address, BMX160_CMD, CMD_PMU_MAG_NORMAL)
        time.sleep(0.02)

        # 2. Configure Accel
        # Range
        # 0x03=2g, 0x05=4g, 0x08=8g, 0x0C=16g
        range_map = {2: 0x03, 4: 0x05, 8: 0x08, 16: 0x0C}
        val = range_map.get(accel_range, 0x05)
        self.bus.write_byte_data(self.address, BMX160_ACCEL_RANGE, val)
        self.accel_range = accel_range

        # ODR 100Hz = 0x28 (example, check datasheet. 0x08=100Hz approx? No, 0x0B=800, 0x08=100Hz)
        # Docs: 0x2B=800Hz? No.
        # Let's set default usable ODR (e.g., 100Hz -> 0x08)
        self.bus.write_byte_data(self.address, BMX160_ACCEL_CONF, 0x28) # ODR 100Hz, BW Normal

        # 3. Configure Gyro
        # Range: 0x00=2000, 0x01=1000, 0x02=500, 0x03=250, 0x04=125
        # We generally use 2000 for UAVs
        self.bus.write_byte_data(self.address, BMX160_GYRO_RANGE, 0x00) 
        self.bus.write_byte_data(self.address, BMX160_GYRO_CONF, 0x28) # ODR 100Hz

        # 4. Configure Mag
        # Prepare MAG interface to be enabled.
        # Reading mag data from BMX160 can be tricky; it usually puts mag data into data registers 0x04-0x0B
        # once the MAG PMU is on. 
        # Set ODR for Mag (25Hz or 50Hz)
        self.bus.write_byte_data(self.address, BMX160_MAG_CONF, 0x08) # 25Hz

    def read_all_data(self):
        # Read 20 bytes starting from 0x04 (Mag: 8, Gyro: 6, Accel: 6)
        # 0x04 -> Mag X LSB
        try:
            data = self.bus.read_i2c_block_data(self.address, BMX160_MAG_DATA_0, 20)
            
            # Unpack Mag
            # Mag X, Y, Z (int16)
            # Scaling: 16-bit signed. 1 bit ~ 0.3 uT usually on default
            mx = struct.unpack('<h', bytes(data[0:2]))[0]
            my = struct.unpack('<h', bytes(data[2:4]))[0]
            mz = struct.unpack('<h', bytes(data[4:6]))[0]
            # data[6:8] is Hall resistance usually, skip
            
            # Unpack Gyro
            gx = struct.unpack('<h', bytes(data[8:10]))[0]
            gy = struct.unpack('<h', bytes(data[10:12]))[0]
            gz = struct.unpack('<h', bytes(data[12:14]))[0]

            # Unpack Accel
            ax = struct.unpack('<h', bytes(data[14:16]))[0]
            ay = struct.unpack('<h', bytes(data[16:18]))[0]
            az = struct.unpack('<h', bytes(data[18:20]))[0]

            # Convert to physical units
            
            # Accel: Range +/- 4g -> 32768 = 4g
            # 1 g = 9.80665 m/s^2
            accel_scale = (self.accel_range * GRAVITY) / 32768.0
            ax_mps = ax * accel_scale
            ay_mps = ay * accel_scale
            az_mps = az * accel_scale

            # Gyro: Range +/- 2000 deg/s -> 32768 = 2000
            # rad/s = deg/s * pi / 180
            gyro_scale = (2000.0 / 32768.0) * (math.pi / 180.0)
            gx_rad = gx * gyro_scale
            gy_rad = gy * gyro_scale
            gz_rad = gz * gyro_scale

            # Mag: BMX160 typical sensitivity is ~0.3uT/LSB
            mag_scale = 0.3 # uT
            mx_ut = mx * mag_scale
            my_ut = my * mag_scale
            mz_ut = mz * mag_scale

            return {
                'accel': (ax_mps, ay_mps, az_mps),
                'gyro': (gx_rad, gy_rad, gz_rad),
                'mag': (mx_ut, my_ut, mz_ut)
            }

        except Exception as e:
            # print(f"Read error: {e}")
            return None
