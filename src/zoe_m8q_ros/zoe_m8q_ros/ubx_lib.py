"""
Library for parsing UBX protocol messages from u-blox GPS receivers.
"""
import time
import struct
from dataclasses import dataclass
from typing import Optional
from smbus2 import SMBus, i2c_msg

# Constants
I2C_BUS = 1
DEFAULT_GPS_ADDR = 0x42
REG_STREAM = 0xFF
READ_LEN = 128

@dataclass
class NAVPVTData:
    """Parsed NAV-PVT data"""
    # Time
    iTOW: int           # GPS time of week (ms)
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int
    
    # Validity flags
    valid_date: bool
    valid_time: bool
    valid_mag: bool
    
    # Time accuracy
    tAcc: int           # Time accuracy estimate (ns)
    nano: int           # Fraction of second (ns)
    
    # Position (WGS84)
    lon: float          # Longitude (deg)
    lat: float          # Latitude (deg)
    height: float       # Height above ellipsoid (m)
    hMSL: float         # Height above MSL (m)
    
    # Accuracy
    hAcc: float         # Horizontal accuracy (m)
    vAcc: float         # Vertical accuracy (m)
    
    # Velocity (NED frame)
    velN: float         # North velocity (m/s)
    velE: float         # East velocity (m/s)
    velD: float         # Down velocity (m/s)
    
    # Ground speed and heading
    gSpeed: float       # Ground speed (m/s)
    headMot: float      # Heading of motion (deg)
    
    # Accuracy
    sAcc: float         # Speed accuracy (m/s)
    headAcc: float      # Heading accuracy (deg)
    
    # Fix quality
    fixType: int        # Fix type (0=no fix, 2=2D, 3=3D fix)
    flags: int          # Fix status flags
    numSV: int          # Number of satellites
    
    # DOP
    pDOP: float         # Position DOP
    
    @property
    def has_3d_fix(self) -> bool:
        """Check if 3D fix available"""
        return self.fixType >= 3
    
    @property
    def has_valid_time(self) -> bool:
        """Check if time is valid"""
        return self.valid_date and self.valid_time

class UbxNavPvt:
    """
    UBX NAV-PVT message parser compatible with original class interface
    ZOE-M8Q u-blox M8 protocol specification
    """
    CLASS = 0x01
    ID = 0x07
    PAYLOAD_LEN = 92
    
    def __init__(self):
        """Initialize with default values"""
        self.iTOW = 0
        self.year = 0
        self.month = 0
        self.day = 0
        self.hour = 0
        self.min = 0
        self.sec = 0
        self.valid = 0
        self.tAcc = 0
        self.nano = 0
        self.fixType = 0
        self.flags = 0
        self.flags2 = 0
        self.numSV = 0
        self.lon = 0      # degrees
        self.lat = 0      # degrees
        self.height = 0   # meters
        self.hMSL = 0     # meters
        self.hAcc = 0     # meters
        self.vAcc = 0     # meters
        self.velN = 0     # m/s
        self.velE = 0     # m/s
        self.velD = 0     # m/s
        self.gSpeed = 0   # m/s
        self.headMot = 0  # degrees
        self.sAcc = 0     # m/s
        self.headAcc = 0  # degrees
        self.pDOP = 0     # unitless
    
    def parse(self, payload):
        """
        Parse NAV-PVT payload (92 bytes)
        NOTE: This modifies the instance attributes directly.
        """
        if len(payload) < self.PAYLOAD_LEN:
            return False
        
        try:
            # Parse all fields with correct byte offsets
            self.iTOW = struct.unpack('<I', payload[0:4])[0]
            self.year = struct.unpack('<H', payload[4:6])[0]
            self.month = payload[6]
            self.day = payload[7]
            self.hour = payload[8]
            self.min = payload[9]
            self.sec = payload[10]
            self.valid = payload[11]
            self.tAcc = struct.unpack('<I', payload[12:16])[0]
            self.nano = struct.unpack('<i', payload[16:20])[0]
            self.fixType = payload[20]
            self.numSV = payload[23]
            
            # Position (convert from 1e-7 degrees and mm)
            self.lon = struct.unpack('<i', payload[24:28])[0] * 1e-7
            self.lat = struct.unpack('<i', payload[28:32])[0] * 1e-7
            self.height = struct.unpack('<i', payload[32:36])[0] * 1e-3
            self.hMSL = struct.unpack('<i', payload[36:40])[0] * 1e-3
            
            # Accuracy (convert from mm)
            self.hAcc = struct.unpack('<I', payload[40:44])[0] * 1e-3
            self.vAcc = struct.unpack('<I', payload[44:48])[0] * 1e-3
            
            # Velocity (convert from mm/s to m/s)
            self.velN = struct.unpack('<i', payload[48:52])[0] * 1e-3
            self.velE = struct.unpack('<i', payload[52:56])[0] * 1e-3
            self.velD = struct.unpack('<i', payload[56:60])[0] * 1e-3
            self.gSpeed = struct.unpack('<i', payload[60:64])[0] * 1e-3
            
            # Heading (convert from 1e-5 degrees)
            self.headMot = struct.unpack('<i', payload[64:68])[0] * 1e-5
            
            # Speed/heading accuracy (convert from mm/s and 1e-5 degrees)
            self.sAcc = struct.unpack('<I', payload[68:72])[0] * 1e-3
            self.headAcc = struct.unpack('<I', payload[72:76])[0] * 1e-5
            
            # DOP (convert from 0.01)
            self.pDOP = struct.unpack('<H', payload[76:78])[0] * 0.01
            
            return True
            
        except Exception as e:
            print(f"NAV-PVT parse error: {e}")
            return False

class NAVPVTParser:
    """Parser for UBX NAV-PVT messages (dataclass interface)"""
    
    # Expected payload length
    PAYLOAD_LEN = 92
    
    @staticmethod
    def parse(payload: bytes) -> Optional[NAVPVTData]:
        """
        Parse NAV-PVT payload into dataclass
        
        Args:
            payload: UBX message payload
            
        Returns:
            NAVPVTData or None if parse fails
        """
        if len(payload) != NAVPVTParser.PAYLOAD_LEN:
            return None
        
        try:
            # Use the class parser
            parser = UbxNavPvt()
            if not parser.parse(payload):
                return None
            
            # Parse validity flags
            valid_date = (parser.valid & 0x01) != 0
            valid_time = (parser.valid & 0x02) != 0
            valid_mag = (parser.valid & 0x08) != 0
            
            return NAVPVTData(
                iTOW=parser.iTOW,
                year=parser.year,
                month=parser.month,
                day=parser.day,
                hour=parser.hour,
                minute=parser.min,
                second=parser.sec,
                valid_date=valid_date,
                valid_time=valid_time,
                valid_mag=valid_mag,
                tAcc=parser.tAcc,
                nano=parser.nano,
                lon=parser.lon,
                lat=parser.lat,
                height=parser.height,
                hMSL=parser.hMSL,
                hAcc=parser.hAcc,
                vAcc=parser.vAcc,
                velN=parser.velN,
                velE=parser.velE,
                velD=parser.velD,
                gSpeed=parser.gSpeed,
                headMot=parser.headMot,
                sAcc=parser.sAcc,
                headAcc=parser.headAcc,
                fixType=parser.fixType,
                flags=parser.flags,
                numSV=parser.numSV,
                pDOP=parser.pDOP
            )
            
        except Exception as e:
            print(f"NAV-PVT parse error: {e}")
            return None

# Low-level Utils
def ubx_ck(b):
    a = 0
    c = 0
    for x in b:
        a = (a + x) & 0xFF
        c = (c + a) & 0xFF
    return a, c

def ubx_pack(cls_, id_, payload):
    ln = len(payload)
    head = [0xB5, 0x62, cls_, id_, ln & 0xFF, (ln >> 8) & 0xFF]
    ck = ubx_ck(head[2:] + payload)
    return bytes(head + payload + [ck[0], ck[1]])

def i2c_write(bus, data: bytes):
    for i in range(0, len(data), 32):
        bus.write_i2c_block_data(DEFAULT_GPS_ADDR, REG_STREAM, list(data[i:i+32]))
        time.sleep(0.002)

def set_rate_hz(bus, hz=1.0):
    meas_ms = max(10, int(round(1000.0 / hz)))
    payload = [
        meas_ms & 0xFF, (meas_ms >> 8) & 0xFF,
        0x01, 0x00,  # navRate = 1
        0x01, 0x00   # timeRef = GPS
    ]
    i2c_write(bus, ubx_pack(0x06, 0x08, payload))

def cfg_msg_rate(bus, cls_, id_, rate):
    i2c_write(bus, ubx_pack(0x06, 0x01, [cls_, id_, rate]))

def enable_nav_pvt_only(bus):
    # Disable NMEA
    for mid in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x0D]:
        cfg_msg_rate(bus, 0xF0, mid, 0x00)
    # Enable NAV-PVT
    cfg_msg_rate(bus, 0x01, 0x07, 0x01)

def read_chunk(bus, n=READ_LEN):
    msg = i2c_msg.read(DEFAULT_GPS_ADDR, n)
    bus.i2c_rdwr(msg)
    return bytes(msg)

def parse_ubx_stream(buf):
    """
    Parse UBX stream.
    Returns list of (cls, id, payload) and remainder buffer.
    """
    out = []
    i = 0
    L = len(buf)
    while i + 8 <= L:
        if not (buf[i] == 0xB5 and buf[i+1] == 0x62):
            i += 1
            continue
        if i + 6 > L:
            break
        cls_ = buf[i+2]
        id_ = buf[i+3]
        ln = buf[i+4] | (buf[i+5] << 8)
        end = i + 6 + ln + 2
        if end > L:
            break
        payload = buf[i+6:i+6+ln]
        ckA, ckB = ubx_ck([cls_, id_, buf[i+4], buf[i+5]] + list(payload))
        if ckA == buf[i+6+ln] and ckB == buf[i+6+ln+1]:
            out.append((cls_, id_, bytes(payload)))
            i = end
        else:
            i += 1
    return out, buf[i:] if i < L else bytearray()
