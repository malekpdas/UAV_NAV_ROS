"""
Driver library for Garmin Lidar Lite v3HP via I2C.
"""
import time
import smbus2

class LidarLite:
    """
    Interface for Garmin Lidar Lite v3HP.
    Default Address: 0x62
    """
    
    # Predefined hardware configuration presets (values in decimal)
    PRESETS = {
        'balanced': {
            'sig_count_val': 128,      # 0x80
            'acq_config_reg': 8,       # 0x08
            'ref_count_val': 3,        # 0x03
            'threshold_bypass': 0      # 0x00
        },
        'high_speed': {
            'sig_count_val': 29,       # 0x1d
            'acq_config_reg': 0,       # 0x00
            'ref_count_val': 3,        # 0x03
            'threshold_bypass': 0      # 0x00
        },
        'high_accuracy': {
            'sig_count_val': 255,      # 0xff
            'acq_config_reg': 8,       # 0x08
            'ref_count_val': 5,        # 0x05
            'threshold_bypass': 0      # 0x00
        },
        'long_range': {
            'sig_count_val': 192,      # 0xc0
            'acq_config_reg': 8,       # 0x08
            'ref_count_val': 3,        # 0x03
            'threshold_bypass': 0      # 0x00
        }
    }
    
    ACQ_COMMAND = 0x00
    STATUS_REG = 0x01
    SIG_COUNT_VAL = 0x02
    ACQ_CONFIG_REG = 0x04
    THRESHOLD_BYPASS = 0x1c
    REF_COUNT_VAL = 0x12
    FULL_DELAY_HIGH = 0x8f # Auto-increment read for 0x0f and 0x10
    
    def __init__(self, bus_id=1, address=0x62):
        self.bus = smbus2.SMBus(bus_id)
        self.address = address
        self.connected = False
        
        try:
            # Check connection
            self.bus.write_quick(self.address)
            self.connected = True
        except Exception as e:
            print(f"Lidar Lite not found on bus {bus_id}: {e}")
            
    def write_register(self, reg, val):
        if not self.connected:
            return False
        try:
            self.bus.write_byte_data(self.address, reg, val)
            return True
        except Exception as e:
            print(f"Write error to 0x{reg:02x}: {e}")
            return False

    def configure(self, settings=None):
        """
        Configure optional settings from a dictionary.
        """
        if not self.connected:
            return False
        
        success = True
        if settings:
            if 'sig_count_val' in settings:
                success &= self.write_register(self.SIG_COUNT_VAL, settings['sig_count_val'])
            if 'acq_config_reg' in settings:
                success &= self.write_register(self.ACQ_CONFIG_REG, settings['acq_config_reg'])
            if 'threshold_bypass' in settings:
                success &= self.write_register(self.THRESHOLD_BYPASS, settings['threshold_bypass'])
            if 'ref_count_val' in settings:
                success &= self.write_register(self.REF_COUNT_VAL, settings['ref_count_val'])
                
        return success

    def read_distance(self):
        """
        Trigger measurement and read distance in cm.
        Returns: distance (float) in meters, or None if failed.
        """
        if not self.connected:
            return None
            
        try:
            # 1. Write 0x04 to register 0x00 to take distance measurement 
            # with receiver bias correction
            self.bus.write_byte_data(self.address, self.ACQ_COMMAND, 0x04)
            
            # 2. Wait for busy flag (bit 0) in status register to go low
            # Timeout after ~100ms
            for _ in range(100):
                status = self.bus.read_byte_data(self.address, self.STATUS_REG)
                if not (status & 0x01): # Busy bit is 0
                    break
                time.sleep(0.001)
            else:
                # Timed out
                return None
                
            # 3. Read 2 bytes from 0x8f (High byte, Low byte of distance)
            # Note: v3HP reads from 0x8f for last measurement
            # Using read_i2c_block_data to read 2 bytes
            val = self.bus.read_i2c_block_data(self.address, self.FULL_DELAY_HIGH, 2)
            dist_cm = (val[0] << 8) | val[1]
            
            return dist_cm / 100.0 # Convert to meters
            
        except Exception as e:
            print(f"Read error: {e}")
            return None

    def close(self):
        if self.bus:
            self.bus.close()
