import board
import busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    _BNO_CHANNEL_CONTROL,
)

BNO085_I2C_ADDR_DEFAULT = 0x4A


class BNO085:
    def __init__(self, rate_hz=100.0, address=BNO085_I2C_ADDR_DEFAULT):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(self.i2c, address=address)

        self._enable_feature_with_rate(BNO_REPORT_ACCELEROMETER, rate_hz)
        self._enable_feature_with_rate(BNO_REPORT_GYROSCOPE, rate_hz)
        self._enable_feature_with_rate(BNO_REPORT_MAGNETOMETER, rate_hz)

    def read_latest_snapshot(self):
        # Drain all pending packets so readings reflect the newest available data.
        self.bno._process_available_packets()
        readings = self.bno._readings
        return (
            readings.get(BNO_REPORT_ACCELEROMETER),
            readings.get(BNO_REPORT_GYROSCOPE),
            readings.get(BNO_REPORT_MAGNETOMETER),
        )

    def _enable_feature_with_rate(self, feature_id, rate_hz):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        # First enable with library defaults, then update the report interval.
        self.bno.enable_feature(feature_id)
        interval_us = int(1_000_000 / rate_hz)
        set_feature_report = self.bno._get_feature_enable_report(feature_id, report_interval=interval_us)
        self.bno._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)
        # Pull a few packets to apply the new interval quickly.
        self.bno._process_available_packets(max_packets=10)

    def close_sensor(self):
        # Best-effort cleanup: release I2C resources.
        try:
            if self.i2c is not None:
                self.i2c.deinit()
        except Exception:
            pass
