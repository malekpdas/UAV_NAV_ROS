"""
Utility functions for Hybrid EKF+AHRS Estimator
"""
import numpy as np


def lla_to_ned(lat, lon, alt, ref_lla, r):
    """
    Simplified LLA to NED conversion (good for <100km range)
    """
    lat0, lon0, alt0 = ref_lla
    
    # Convert to radians
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)
    
    # NED coordinates
    north = (lat - lat0) * (np.pi/180) * r
    east = (lon - lon0) * (np.pi/180) * r * np.cos(lat0_rad)
    down = -(alt - alt0)
    
    return np.array([north, east, down])


def ned_to_lla(north, east, down, lat0, lon0, alt0):
    """
    Convert NED coordinates to Lat/Lon/Alt
    
    Args:
        north, east, down: NED position in meters
        lat0, lon0, alt0: Origin position
    
    Returns:
        tuple: (lat, lon, alt)
    """
    r = 6378137.0
    f = 1.0 / 298.257223563
    
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)
    
    sin_lat0 = np.sin(lat0_rad)
    N = r / np.sqrt(1 - (2*f - f**2) * sin_lat0**2)
    
    # Convert NED to lat/lon/alt
    dlat = north / (N * (1 - f)**2 + alt0)
    dlon = east / ((N + alt0) * np.cos(lat0_rad))
    dalt = -down
    
    lat = np.degrees(lat0_rad + dlat)
    lon = np.degrees(lon0_rad + dlon)
    alt = alt0 + dalt
    
    return lat, lon, alt


def quat_mult(q1, q2):
    """
    Quaternion multiplication: q1 * q2
    Args:
        q1, q2: Quaternions as [w, x, y, z]
    Returns:
        Product quaternion [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return np.array([w, x, y, z])


def quat_conjugate(q):
    """
    Quaternion conjugate
    Args:
        q: Quaternion [w, x, y, z]
    Returns:
        Conjugate [w, -x, -y, -z]
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    """
    Normalize quaternion to unit length
    """
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def rotate_vector(v, q):
    """
    Rotate vector v by quaternion q
    Computes: v' = q * v * q_conj
    
    Args:
        v: 3D vector to rotate
        q: Quaternion [w, x, y, z]
    Returns:
        Rotated vector
    """
    # Convert v to pure quaternion [0, vx, vy, vz]
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    
    # q * v * q_conj
    temp = quat_mult(q, v_quat)
    result = quat_mult(temp, quat_conjugate(q))
    
    return result[1:4]


def quat_to_rot_mat(q):
    """
    Convert quaternion to rotation matrix
    Args:
        q: Quaternion [w, x, y, z]
    Returns:
        3x3 rotation matrix
    """
    w, x, y, z = q
    
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    
    return R


def quat_to_euler(q):
    """
    Convert quaternion to Euler angles (ZYX convention)
    Args:
        q: Quaternion [w, x, y, z]
    Returns:
        [roll, pitch, yaw] in radians
    """
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])


def euler_to_quat(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion (ZYX convention)
    Args:
        roll, pitch, yaw: Angles in radians
    Returns:
        Quaternion [w, x, y, z]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])


def wrap_pi(angle):
    """
    Wrap angle to [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def wrap_2pi(angle):
    """
    Wrap angle to [0, 2*pi]
    """
    return angle % (2 * np.pi)


def skew_symmetric(v):
    """
    Create skew-symmetric matrix from vector
    Used for cross product: [v]_x * u = v × u
    
    Args:
        v: 3D vector
    Returns:
        3x3 skew-symmetric matrix
    """
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0]
    ])


def dcm_from_euler(roll, pitch, yaw):
    """
    Direction Cosine Matrix (DCM) from Euler angles
    ZYX rotation sequence
    """
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    
    dcm = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,           cp*cr  ]
    ])
    
    return dcm


def euler_from_dcm(dcm):
    """
    Extract Euler angles from Direction Cosine Matrix
    """
    pitch = np.arcsin(-dcm[2, 0])
    
    if np.abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(dcm[2, 1], dcm[2, 2])
        yaw = np.arctan2(dcm[1, 0], dcm[0, 0])
    else:
        # Gimbal lock
        roll = 0.0
        yaw = np.arctan2(-dcm[0, 1], dcm[1, 1])
    
    return roll, pitch, yaw