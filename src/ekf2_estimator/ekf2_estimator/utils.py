import numpy as np

def quat_mult(q_p, q_q):
    """
    Multiply two quaternions [w, x, y, z].
    q_out = q_p * q_q
    """
    w1, x1, y1, z1 = q_p
    w2, x2, y2, z2 = q_q
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n

def rotate_vector(v, q):
    """
    Rotate vector v by quaternion q.
    v_rot = q * v * q_inv
    q is [w, x, y, z] is transformation q_nb (NED to Body)?
    Wait, check usage: 
    If q represents "Rotation FROM A TO B", then v_b = R(q) * v_a.
    Standard Hamilton: v_rotated = q * v * q_inv.
    """
    q_v = np.array([0, v[0], v[1], v[2]])
    q_inv = quat_conjugate(q)
    return quat_mult(quat_mult(q, q_v), q_inv)[1:]

def quat_to_rot_mat(q):
    """
    Convert Quaternion [w, x, y, z] to 3x3 Rotation Matrix.
    """
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

def euler_to_quat(roll, pitch, yaw):
    """
    Convert RPY (radians) to Quaternion [w, x, y, z]
    Rotation sequence: Z-Y-X (Yaw-Pitch-Roll)
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])

def quat_to_euler(q):
    """
    Convert Quaternion [w, x, y, z] to RPY
    """
    w, x, y, z = q
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def lla_to_ned(lat, lon, alt, lat0, lon0, alt0):
    """
    Convert LLA to NED using Flat Earth approximation.
    """
    R = 6378137.0 # Earth Radius
    
    dlat = np.deg2rad(lat - lat0)
    dlon = np.deg2rad(lon - lon0)
    
    lat0_rad = np.deg2rad(lat0)
    
    x = dlat * R
    y = dlon * R * np.cos(lat0_rad)
    z = -(alt - alt0)
    
    return np.array([x, y, z])

def wrap_pi(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi
