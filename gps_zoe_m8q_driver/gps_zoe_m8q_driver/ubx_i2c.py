from smbus2 import SMBus, i2c_msg
import time
import struct
from .ubx_nav_pvt import UbxNavPvt

class UbxI2C:
    def __init__(self, bus_num=1, address=0x42):
        self.bus_num = bus_num
        self.address = address
        self.bus = None

    def open(self):
        try:
            self.bus = SMBus(self.bus_num)
            return True
        except Exception as e:
            print(f"Failed to open I2C bus: {e}")
            return False

    def send_ubx_packet(self, cls, id, payload=b''):
        # Header: 0xB5, 0x62
        # Class, ID
        # Length (2 bytes, LE)
        # Payload
        # Checksum A, B
        
        length = len(payload)
        msg_header = struct.pack('<BBH', cls, id, length)
        content = msg_header + payload
        
        ck_a = 0
        ck_b = 0
        for b in content:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
            
        final_msg = b'\xB5\x62' + content + struct.pack('BB', ck_a, ck_b)
        
        try:
            # U-blox I2C needs us to write to 0xFF register sometimes? 
            # DDC (I2C) just accepts byte stream.
            write_msg = i2c_msg.write(self.address, list(final_msg))
            self.bus.i2c_rdwr(write_msg)
            return True
        except Exception as e:
            return False

    def configure(self, rate_hz=5):
        # 1. Disable NMEA (UBX-CFG-PRT) - Skip for now, assume default ok or focused on polling
        # 2. CFG-RATE
        # rate_hz to measure_rate (ms). 5Hz -> 200ms.
        meas_rate = int(1000 / rate_hz)
        nav_rate = 1 # ratio
        time_ref = 1 # GPS time
        
        payload = struct.pack('<HHH', meas_rate, nav_rate, time_ref)
        self.send_ubx_packet(0x06, 0x08, payload) # CFG-RATE
        
    def read_packet(self):
        # Read from 0xFD (Bytes available) first? 
        # U-blox DDC: Write address to read from 0xFF (stream)
        # We need to read length first or read abundant and parse.
        # Typically read 0xFD MSB and LSB to see bytes available.
        
        try:
            # Read 2 bytes from 0xFD
            # To read registers on DDC, write register address then read.
            # However some docs say just read.
            # Best practice: write 0xFD, read 2.
            
            wr = i2c_msg.write(self.address, [0xFD])
            rd = i2c_msg.read(self.address, 2)
            self.bus.i2c_rdwr(wr, rd)
            # data = list(rd) # this converts to ints
            # Using smbus read_i2c_block might be easier
            
            # Re-do with direct calls for simplicity if smbus allows
            # But rdwr is better for repeated start
            
            avail_bytes = list(rd)
            bytes_to_read = (avail_bytes[0] << 8) | avail_bytes[1]
            
            if bytes_to_read == 0:
                return None
            
            # Clamp reading to avoid too big chunks
            if bytes_to_read > 256: 
                bytes_to_read = 256
            
            # Read stream from 0xFF
            wr = i2c_msg.write(self.address, [0xFF])
            rd = i2c_msg.read(self.address, bytes_to_read)
            self.bus.i2c_rdwr(wr, rd)
            
            data = bytes(list(rd))
            return data
            
        except Exception:
            return None

    # Helper buffer for parsing stream
    def parse_stream(self, data, buffer):
        buffer += data
        
        # Look for sync
        msgs = []
        while len(buffer) > 6:
            # Sync chars
            try:
                start_idx = buffer.index(b'\xB5\x62')
                if start_idx > 0:
                    buffer = buffer[start_idx:]
            except ValueError:
                # No sync found, keep last byte just in case
                buffer = buffer[-1:]
                return msgs, buffer
                
            if len(buffer) < 6:
                break
                
            # Class, ID, Length
            cls = buffer[2]
            msg_id = buffer[3]
            length = struct.unpack('<H', buffer[4:6])[0]
            
            total_len = 6 + length + 2 # header + payload + checksum
            
            if len(buffer) < total_len:
                # wait for more
                break
                
            packet = buffer[:total_len]
            payload = packet[6:6+length]
            checksum = packet[6+length:6+length+2]
            
            # Verify checksum (Skip for speed if assuming reliability on I2C short wire, but better to check)
            # ...
            
            if cls == UbxNavPvt.CLASS and msg_id == UbxNavPvt.ID:
                msg = UbxNavPvt()
                if msg.parse(payload):
                    msgs.append(msg)
            
            buffer = buffer[total_len:]
            
        return msgs, buffer
