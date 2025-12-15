"""
UBX NAV-PVT message parser
Position Velocity Time solution
Corrected for ZOE-M8Q u-blox M8 protocol
"""
import struct
from dataclasses import dataclass
from typing import Optional


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
        
        NAV-PVT Structure (u-blox M8):
        Byte offset | Type   | Name      | Unit       | Scale
        0           | U4     | iTOW      | ms         | 1
        4           | U2     | year      | y          | 1
        6           | U1     | month     | month      | 1
        7           | U1     | day       | d          | 1
        8           | U1     | hour      | h          | 1
        9           | U1     | min       | min        | 1
        10          | U1     | sec       | s          | 1
        11          | X1     | valid     | -          | -
        12          | U4     | tAcc      | ns         | 1
        16          | I4     | nano      | ns         | 1
        20          | U1     | fixType   | -          | -
        21          | X1     | flags     | -          | -
        22          | X1     | flags2    | -          | -
        23          | U1     | numSV     | -          | 1
        24          | I4     | lon       | deg        | 1e-7
        28          | I4     | lat       | deg        | 1e-7
        32          | I4     | height    | mm         | 1
        36          | I4     | hMSL      | mm         | 1
        40          | U4     | hAcc      | mm         | 1
        44          | U4     | vAcc      | mm         | 1
        48          | I4     | velN      | mm/s       | 1
        52          | I4     | velE      | mm/s       | 1
        56          | I4     | velD      | mm/s       | 1
        60          | I4     | gSpeed    | mm/s       | 1
        64          | I4     | headMot   | deg        | 1e-5
        68          | U4     | sAcc      | mm/s       | 1
        72          | U4     | headAcc   | deg        | 1e-5
        76          | U2     | pDOP      | -          | 0.01
        78          | X1[6]  | reserved  | -          | -
        84          | I4     | headVeh   | deg        | 1e-5 (not used)
        88          | I2     | magDec    | deg        | 1e-2 (not used)
        90          | U2     | magAcc    | deg        | 1e-2 (not used)
        
        Args:
            payload: 92-byte payload
            
        Returns:
            True if successful, False otherwise
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
            self.flags = payload[21]
            self.flags2 = payload[22]
            self.numSV = payload[23]
            
            # Position (convert from 1e-7 degrees and mm)
            self.lon = struct.unpack('<i', payload[24:28])[0]
            self.lat = struct.unpack('<i', payload[28:32])[0]
            self.height = struct.unpack('<i', payload[32:36])[0]
            self.hMSL = struct.unpack('<i', payload[36:40])[0]
            
            # Accuracy (convert from mm)
            self.hAcc = struct.unpack('<I', payload[40:44])[0]
            self.vAcc = struct.unpack('<I', payload[44:48])[0]
            
            # Velocity (convert from mm/s to m/s)
            self.velN = struct.unpack('<i', payload[48:52])[0]
            self.velE = struct.unpack('<i', payload[52:56])[0]
            self.velD = struct.unpack('<i', payload[56:60])[0]
            self.gSpeed = struct.unpack('<i', payload[60:64])[0]
            
            # Heading (convert from 1e-5 degrees)
            self.headMot = struct.unpack('<i', payload[64:68])[0]
            
            # Speed/heading accuracy (convert from mm/s and 1e-5 degrees)
            self.sAcc = struct.unpack('<I', payload[68:72])[0]
            self.headAcc = struct.unpack('<I', payload[72:76])[0]
            
            # DOP (convert from 0.01)
            self.pDOP = struct.unpack('<H', payload[76:78])[0]
            
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