#!/usr/bin/env python3
import time
import argparse
import math
import board
import busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    _BNO_CHANNEL_CONTROL,
)

def _read_latest_snapshot(bno):
    # Drain all pending packets so readings reflect the newest available data.
    bno._process_available_packets()
    readings = bno._readings
    return (
        readings.get(BNO_REPORT_ACCELEROMETER),
        readings.get(BNO_REPORT_GYROSCOPE),
        readings.get(BNO_REPORT_MAGNETOMETER),
    )

def _enable_feature_with_rate(bno, feature_id, rate_hz):
    if rate_hz <= 0:
        raise ValueError("rate_hz must be > 0")
    # First enable with library defaults, then update the report interval.
    bno.enable_feature(feature_id)
    interval_us = int(1_000_000 / rate_hz)
    set_feature_report = bno._get_feature_enable_report(feature_id, report_interval=interval_us)
    bno._send_packet(_BNO_CHANNEL_CONTROL, set_feature_report)
    # Pull a few packets to apply the new interval quickly.
    bno._process_available_packets(max_packets=10)

def _init_stats():
    return {"count": 0, "mean": [0.0, 0.0, 0.0], "m2": [0.0, 0.0, 0.0]}

def _update_stats(stats, vec):
    stats["count"] += 1
    n = stats["count"]
    for i in range(3):
        x = float(vec[i])
        delta = x - stats["mean"][i]
        stats["mean"][i] += delta / n
        delta2 = x - stats["mean"][i]
        stats["m2"][i] += delta * delta2

def _finalize_stats(stats):
    n = stats["count"]
    if n < 2:
        return stats["mean"], [0.0, 0.0, 0.0]
    var = [m2 / (n - 1) for m2 in stats["m2"]]
    std = [math.sqrt(v) for v in var]
    return stats["mean"], std

def _probe_i2c_device(i2c, address):
    try:
        while not i2c.try_lock():
            time.sleep(0.001)
        devices = i2c.scan()
    finally:
        try:
            i2c.unlock()
        except Exception:
            pass
    return address in devices

def _close_sensor(bno, i2c):
    # Best-effort cleanup: release I2C resources.
    try:
        if i2c is not None:
            i2c.deinit()
    except Exception:
        pass

def main():
    parser = argparse.ArgumentParser(description="Read live accel/gyro/mag from BNO085 over I2C")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x4A, help="I2C address (default: 0x4A)")
    parser.add_argument("--rate", type=float, default=100.0, help="Report rate in Hz (default: 50)")
    parser.add_argument("--print-rate", type=float, default=100.0, help="Print loop rate in Hz (default: 50)")
    parser.add_argument("--warmup", type=float, default=5.0, help="Warmup time in seconds (default: 2)")
    parser.add_argument("--duration", type=float, default=20.0, help="Run duration in seconds (default: 10)")
    args = parser.parse_args()

    # Initialize I2C
    i2c = busio.I2C(board.SCL, board.SDA)
    if not _probe_i2c_device(i2c, args.addr):
        print(f"ERROR: No I2C device found at address 0x{args.addr:02X}. Check wiring/power.")
        _close_sensor(None, i2c)
        return
    try:
        bno = BNO08X_I2C(i2c, address=args.addr)
    except Exception as exc:
        print(f"ERROR: Failed to initialize BNO085: {exc}")
        _close_sensor(None, i2c)
        return

    # Set report rates (Hz)
    _enable_feature_with_rate(bno, BNO_REPORT_ACCELEROMETER, args.rate)
    _enable_feature_with_rate(bno, BNO_REPORT_GYROSCOPE, args.rate)
    _enable_feature_with_rate(bno, BNO_REPORT_MAGNETOMETER, args.rate)

    period = 1.0 / args.print_rate
    print(f"BNO085 ready. Warming up for {args.warmup:.1f}s...")
    if args.warmup > 0:
        warmup_end = time.monotonic() + args.warmup
        next_w = time.monotonic()
        while time.monotonic() < warmup_end:
            _read_latest_snapshot(bno)
            next_w += period
            sleep_time = next_w - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_w = time.monotonic()

    try:
        print("Starting capture...")
        next_t = time.monotonic()
        start_t = time.monotonic()
        last_t = None
        dts = []
        sample_count = 0
        accel_stats = _init_stats()
        gyro_stats = _init_stats()
        mag_stats = _init_stats()

        while True:
            accel, gyro, mag = _read_latest_snapshot(bno)

            t = time.time()  # wall clock for "current time"
            # print(f"{t:.3f} | accel={accel} m/s^2 | gyro={gyro} rad/s | mag={mag} uT")

            now = time.monotonic()
            if last_t is not None:
                dts.append(now - last_t)
            last_t = now
            sample_count += 1

            if accel is not None:
                _update_stats(accel_stats, accel)
            if gyro is not None:
                _update_stats(gyro_stats, gyro)
            if mag is not None:
                _update_stats(mag_stats, mag)

            if now - start_t >= args.duration:
                elapsed = now - start_t
                if dts:
                    min_dt = min(dts)
                    max_dt = max(dts)
                    avg_dt = sum(dts) / len(dts)
                    avg_hz = len(dts) / sum(dts)
                else:
                    min_dt = max_dt = avg_dt = avg_hz = 0.0

                print("\n=== Stats ===")
                print(f"Elapsed: {elapsed:.3f}s | Samples: {sample_count}")
                print(f"Rate: avg={avg_hz:.2f} Hz | dt min={min_dt*1000:.2f} ms | dt max={max_dt*1000:.2f} ms | dt avg={avg_dt*1000:.2f} ms")
                if accel_stats["count"] > 0:
                    mean, std = _finalize_stats(accel_stats)
                    print(f"Accel mean={tuple(mean)} std={tuple(std)}")
                if gyro_stats["count"] > 0:
                    mean, std = _finalize_stats(gyro_stats)
                    print(f"Gyro  mean={tuple(mean)} std={tuple(std)}")
                if mag_stats["count"] > 0:
                    mean, std = _finalize_stats(mag_stats)
                    print(f"Mag   mean={tuple(mean)} std={tuple(std)}")
                break

            # Timing loop
            next_t += period
            sleep_time = next_t - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're lagging, reset the schedule to avoid drift
                next_t = time.monotonic()
    finally:
        _close_sensor(bno, i2c)

if __name__ == "__main__":
    main()
