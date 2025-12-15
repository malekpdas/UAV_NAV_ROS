import struct

class UbxNavPvt:
    CLASS = 0x01
    ID = 0x07
    
    def __init__(self):
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
        self.lon = 0
        self.lat = 0
        self.height = 0
        self.hMSL = 0
        self.hAcc = 0
        self.vAcc = 0
        self.velN = 0
        self.velE = 0
        self.velD = 0
        self.gSpeed = 0
        self.headMot = 0
        self.sAcc = 0
        self.headAcc = 0
        self.pDOP = 0
        
    def parse(self, payload):
        if len(payload) < 92:
            return False
        
        # Structure definition from U-blox M8 Interface Description
        # <I (iTOW), <H (year), <B (month), <B (day), <B (hour), <B (min), <B (sec), <B (valid), <I (tAcc), <i (nano), <B (fixType), <B (flags), <B (flags2), <B (numSV), <i (lon), <i (lat), <i (height), <i (hMSL), <I (hAcc), <I (vAcc), <i (velN), <i (velE), <i (velD), <i (gSpeed), <i (headMot), <I (sAcc), <I (headAcc), <H (pDOP)
        # We'll use struct unpack for parts
        
        try:
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
            self.fixType = payload[20] # 0=NoFix, 2=2D, 3=3D
            self.flags = payload[21]
            self.flags2 = payload[22]
            self.numSV = payload[23]
            self.lon = struct.unpack('<i', payload[24:28])[0] # deg * 1e-7
            self.lat = struct.unpack('<i', payload[28:32])[0] # deg * 1e-7
            self.height = struct.unpack('<i', payload[32:36])[0] # mm
            self.hMSL = struct.unpack('<i', payload[36:40])[0] # mm
            self.hAcc = struct.unpack('<I', payload[40:44])[0] # mm
            self.vAcc = struct.unpack('<I', payload[44:48])[0] # mm
            self.velN = struct.unpack('<i', payload[48:52])[0] # mm/s
            self.velE = struct.unpack('<i', payload[52:56])[0] # mm/s
            self.velD = struct.unpack('<i', payload[56:60])[0] # mm/s
            self.gSpeed = struct.unpack('<i', payload[60:64])[0] # mm/s
            self.headMot = struct.unpack('<i', payload[64:68])[0] # deg * 1e-5
            self.sAcc = struct.unpack('<I', payload[68:72])[0] # mm/s
            self.headAcc = struct.unpack('<I', payload[72:76])[0] # deg * 1e-5
            self.pDOP = struct.unpack('<H', payload[76:78])[0] * 0.01

            return True
        except Exception:
            return False
