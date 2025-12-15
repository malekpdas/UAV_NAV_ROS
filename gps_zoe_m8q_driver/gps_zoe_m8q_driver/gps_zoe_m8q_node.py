import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from .ubx_i2c import UbxI2C
from .ubx_nav_pvt import UbxNavPvt

class GpsZoeM8qNode(Node):
    def __init__(self):
        super().__init__('gps_zoe_m8q_driver')
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x42)
        self.declare_parameter('update_rate', 10.0) # Hz
        self.declare_parameter('frame_id', 'gps_link')
        
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.rate = self.get_parameter('update_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.update_rate = self.get_parameter('update_rate').value
        
        self.ubx = UbxI2C(self.i2c_bus, self.i2c_address, self.get_logger())
        if not self.ubx.open():
            self.get_logger().error("Could not open I2C bus for GPS")
            return
            
        self.ubx.configure(rate_hz=int(self.update_rate))
        self.get_logger().info("GPS Driver Initialized - Waiting for fix...")
        
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, '/gps/vel', 10)
        
        self.timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback) # Run fast to drain buffer? 
        # Actually UBX update is driven by device, we should poll frequently.
        
        self.buffer = b''
        
    def timer_callback(self):
        data = self.ubx.read_packet()
        if data:
            msgs, self.buffer= self.ubx.parse_stream(data, self.buffer)
            for m in msgs:
                self.publish_pvt(m)
        else:
            self.get_logger().warn("No data from GPS - I2C Empty", throttle_duration_sec=2.0)
            pass
                
    def publish_pvt(self, pvt):
        now = self.get_clock().now().to_msg()
        
        # FIX Message
        fix_msg = NavSatFix()
        fix_msg.header.stamp = now
        fix_msg.header.frame_id = self.frame_id
        
        # Status
        # fixType: 0=No, 2=2D, 3=3D, 4=GNSS+DR?
        if pvt.fixType in [2, 3]:
            fix_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        
        fix_msg.latitude = pvt.lat * 1e-7
        fix_msg.longitude = pvt.lon * 1e-7
        fix_msg.altitude = pvt.hMSL * 1e-3 # mm to m
        
        # Covariance (hAcc is mm)
        h_var = (pvt.hAcc * 1e-3) ** 2
        v_var = (pvt.vAcc * 1e-3) ** 2
        
        fix_msg.position_covariance = [
            h_var, 0.0, 0.0,
            0.0, h_var, 0.0,
            0.0, 0.0, v_var
        ]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.fix_pub.publish(fix_msg)
        
        # VELOCITY Message
        vel_msg = TwistWithCovarianceStamped()
        vel_msg.header.stamp = now
        vel_msg.header.frame_id = self.frame_id
        
        # NED velocity from PVT (mm/s to m/s)
        vn = pvt.velN * 1e-3
        ve = pvt.velE * 1e-3
        vd = pvt.velD * 1e-3
        
        # Twist uses Body frame usually? No, for GPS it's ENU or NED relative to a datum?
        # TwistWithCovarianceStamped in ROS is usually strictly body frame if it's 'cmd_vel' but for GPS it is Ground Speed in ENU/NED.
        # But Twist message is (linear x, y, z).
        # We need to route this correctly carefully.
        # PX4 EKF expects VELOCITY in NED (or ENU). 
        # The prompt says: "Parse Velocity NED".
        # We'll publish as ENU for standard ROS compliance:
        # ENU X = East (ve)
        # ENU Y = North (vn)
        # ENU Z = Up (-vd)
        
        vel_msg.twist.twist.linear.x = float(ve)
        vel_msg.twist.twist.linear.y = float(vn)
        vel_msg.twist.twist.linear.z = float(-vd)
        
        s_acc = pvt.sAcc * 1e-3
        s_var = s_acc ** 2
        
        vel_msg.twist.covariance = [
            s_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, s_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, s_var, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsZoeM8qNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
