"""
Library for parsing UBX protocol messages from u-blox GPS receivers.
Supports both I2C and UART interfaces.
"""
import time
import struct
from dataclasses import dataclass
from typing import Optional, Union
from abc import ABC, abstractmethod

# I2C imports (optional - may not be available on all systems)
try:
    from smbus2 import SMBus, i2c_msg
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False

# UART imports
try:
    import serial
    UART_AVAILABLE = True
except ImportError:
    UART_AVAILABLE = False

# Constants
I2C_BUS = 1
DEFAULT_GPS_ADDR = 0x42
REG_STREAM = 0xFF
READ_LEN = 128

# Default UART settings for ZOE-M8Q
DEFAULT_UART_PORT = '/dev/ttyAMA0'  # RPi UART0 (GPIO 14/15)
DEFAULT_UART_BAUD = 9600            # ZOE-M8Q default baud


# =============================================================================
# Interface Abstraction
# =============================================================================

class GPSInterface(ABC):
    """Abstract base class for GPS communication interface."""
    
    @abstractmethod
    def write(self, data: bytes) -> bool:
        """Write data to GPS module."""
        pass
    
    @abstractmethod
    def read(self, n: int = READ_LEN) -> bytes:
        """Read up to n bytes from GPS module."""
        pass
    
    @abstractmethod
    def close(self):
        """Close the interface."""
        pass


class I2CInterface(GPSInterface):
    """I2C interface for GPS communication."""
    
    def __init__(self, bus: int = I2C_BUS, address: int = DEFAULT_GPS_ADDR):
        if not I2C_AVAILABLE:
            raise ImportError("smbus2 not available for I2C communication")
        self.address = address
        self.bus = SMBus(bus)
    
    def write(self, data: bytes) -> bool:
        try:
            for i in range(0, len(data), 32):
                self.bus.write_i2c_block_data(self.address, REG_STREAM, list(data[i:i+32]))
                time.sleep(0.002)
            return True
        except Exception:
            return False
    
    def read(self, n: int = READ_LEN) -> bytes:
        try:
            msg = i2c_msg.read(self.address, n)
            self.bus.i2c_rdwr(msg)
            return bytes(msg)
        except Exception:
            return b''
    
    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


class UARTInterface(GPSInterface):
    """UART interface for GPS communication."""
    
    def __init__(self, port: str = DEFAULT_UART_PORT, baudrate: int = DEFAULT_UART_BAUD):
        if not UART_AVAILABLE:
            raise ImportError("pyserial not available for UART communication")
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.1,  # Non-blocking with short timeout
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        # Flush any stale data
        self.serial.reset_input_buffer()
    
    def write(self, data: bytes) -> bool:
        try:
            self.serial.write(data)
            self.serial.flush()
            return True
        except Exception:
            return False
    
    def read(self, n: int = READ_LEN) -> bytes:
        try:
            # Read available data up to n bytes
            available = self.serial.in_waiting
            if available > 0:
                return self.serial.read(min(available, n))
            return b''
        except Exception:
            return b''
    
    def close(self):
        try:
            self.serial.close()
        except Exception:
            pass
    
    def set_baudrate(self, baudrate: int):
        """Change UART baudrate (after GPS config)."""
        self.serial.baudrate = baudrate

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


# =============================================================================
# Interface-agnostic configuration functions (NEW API)
# =============================================================================

def configure_rate(interface: GPSInterface, hz: float = 1.0):
    """Set GPS measurement rate (works with any interface)."""
    meas_ms = max(10, int(round(1000.0 / hz)))
    payload = [
        meas_ms & 0xFF, (meas_ms >> 8) & 0xFF,
        0x01, 0x00,  # navRate = 1
        0x01, 0x00   # timeRef = GPS
    ]
    interface.write(ubx_pack(0x06, 0x08, payload))

def configure_msg_rate(interface: GPSInterface, cls_: int, id_: int, rate: int):
    """Configure message rate (works with any interface)."""
    interface.write(ubx_pack(0x06, 0x01, [cls_, id_, rate]))

def configure_nav_pvt_only(interface: GPSInterface):
    """Enable only NAV-PVT messages, disable NMEA (works with any interface)."""
    # Disable NMEA messages
    for mid in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x0D]:
        configure_msg_rate(interface, 0xF0, mid, 0x00)
        time.sleep(0.01)  # Small delay between commands
    # Enable NAV-PVT
    configure_msg_rate(interface, 0x01, 0x07, 0x01)


def configure_uart_baudrate(interface: UARTInterface, new_baud: int = 115200):
    """
    Configure GPS module to use a higher UART baud rate.
    
    This sends CFG-PRT command to change GPS UART1 baud rate,
    then switches local serial port to match.
    
    Common baud rates: 9600, 19200, 38400, 57600, 115200, 230400
    For 15Hz+ you need at least 38400, 115200 recommended.
    """
    # UBX-CFG-PRT (0x06 0x00) - Configure UART1 port
    # Payload: portID, reserved, txReady, mode, baudRate, inProtoMask, outProtoMask, flags, reserved2
    port_id = 0x01  # UART1
    tx_ready = 0x0000  # Disabled
    mode = 0x000008D0  # 8N1 (8 data bits, no parity, 1 stop bit)
    in_proto = 0x0007  # UBX + NMEA + RTCM
    out_proto = 0x0001  # UBX only
    flags = 0x0000
    
    payload = [
        port_id,
        0x00,  # reserved
        tx_ready & 0xFF, (tx_ready >> 8) & 0xFF,
        mode & 0xFF, (mode >> 8) & 0xFF, (mode >> 16) & 0xFF, (mode >> 24) & 0xFF,
        new_baud & 0xFF, (new_baud >> 8) & 0xFF, (new_baud >> 16) & 0xFF, (new_baud >> 24) & 0xFF,
        in_proto & 0xFF, (in_proto >> 8) & 0xFF,
        out_proto & 0xFF, (out_proto >> 8) & 0xFF,
        flags & 0xFF, (flags >> 8) & 0xFF,
        0x00, 0x00  # reserved
    ]
    
    # Send at current baud rate
    interface.write(ubx_pack(0x06, 0x00, payload))
    time.sleep(0.1)  # Wait for GPS to process
    
    # Switch local UART to new baud rate
    interface.set_baudrate(new_baud)
    time.sleep(0.1)  # Wait for connection to stabilize


def read_from_interface(interface: GPSInterface, n: int = READ_LEN) -> bytes:
    """Read data from interface (works with any interface)."""
    return interface.read(n)


# =============================================================================
# Legacy I2C-specific functions (for backward compatibility)
# =============================================================================

def i2c_write(bus, data: bytes):
    """Legacy: Direct I2C write (use GPSInterface.write instead)."""
    for i in range(0, len(data), 32):
        bus.write_i2c_block_data(DEFAULT_GPS_ADDR, REG_STREAM, list(data[i:i+32]))
        time.sleep(0.002)

def set_rate_hz(bus, hz=1.0):
    """Legacy: Set rate via I2C bus (use configure_rate instead)."""
    meas_ms = max(10, int(round(1000.0 / hz)))
    payload = [
        meas_ms & 0xFF, (meas_ms >> 8) & 0xFF,
        0x01, 0x00,  # navRate = 1
        0x01, 0x00   # timeRef = GPS
    ]
    i2c_write(bus, ubx_pack(0x06, 0x08, payload))

def cfg_msg_rate(bus, cls_, id_, rate):
    """Legacy: Configure message rate via I2C (use configure_msg_rate instead)."""
    i2c_write(bus, ubx_pack(0x06, 0x01, [cls_, id_, rate]))

def enable_nav_pvt_only(bus):
    """Legacy: Enable NAV-PVT only via I2C (use configure_nav_pvt_only instead)."""
    # Disable NMEA
    for mid in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x0D]:
        cfg_msg_rate(bus, 0xF0, mid, 0x00)
    # Enable NAV-PVT
    cfg_msg_rate(bus, 0x01, 0x07, 0x01)

def read_chunk(bus, n=READ_LEN):
    """Legacy: Read chunk via I2C (use read_from_interface instead)."""
    if I2C_AVAILABLE:
        msg = i2c_msg.read(DEFAULT_GPS_ADDR, n)
        bus.i2c_rdwr(msg)
        return bytes(msg)
    return b''


# =============================================================================
# UBX Stream Parser
# =============================================================================

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

