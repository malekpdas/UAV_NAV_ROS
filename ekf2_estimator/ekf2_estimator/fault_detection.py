import numpy as np

class InnovationGating:
    def __init__(self):
        self.gate_pos = 5.0 # Sigma
        self.gate_vel = 5.0
        self.gate_mag = 3.0
        
    def check_gate(self, innovation, variance, gate_sigma):
        # NIS = y^T S^-1 y
        # For scalar: (innov^2) / var
        # Gate is usually defined as sqrt(NIS) < sigma  OR  NIS < sigma^2
        # PX4 uses sigma directly for check? 
        # "GPS position gate 5-7" usually means sigma.
        
        nis = (innovation ** 2) / variance
        limit = gate_sigma ** 2
        
        return nis < limit, nis

class FaultDetector:
    def __init__(self):
        # Counters
        self.fault_counts = {
            'gps_pos': 0,
            'gps_vel': 0,
            'mag_head': 0,
            'mag_field': 0
        }
        
        # Thresholds (consecutive frames or time)
        self.fault_limit = 100 # Approx 2 seconds at 50Hz
        
        # Flags
        self.gps_fault = False
        self.mag_fault = False
        
    def report_fault(self, sensor_key, is_fault):
        if is_fault:
            self.fault_counts[sensor_key] += 1
        else:
            self.fault_counts[sensor_key] = max(0, self.fault_counts[sensor_key] - 1)
            
        # Logic to trigger permanent/sticky flags
        if self.fault_counts[sensor_key] > self.fault_limit:
            if 'gps' in sensor_key:
                self.gps_fault = True
            if 'mag' in sensor_key:
                self.mag_fault = True
        else:
            # Auto-recovery logic could go here, or keep it sticky until reset
            # PX4 often attempts recovery.
            if self.fault_counts[sensor_key] == 0:
                if 'gps' in sensor_key:
                    self.gps_fault = False
                if 'mag' in sensor_key:
                    self.mag_fault = False

    def is_gps_glitch(self):
        return self.gps_fault
        
    def is_mag_glitch(self):
        return self.mag_fault
