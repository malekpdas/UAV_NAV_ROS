import lgpio
import time

GPIOCHIP = 4
GPIO = 17

h = lgpio.gpiochip_open(GPIOCHIP)
lgpio.gpio_claim_alert(h, GPIO, lgpio.BOTH_EDGES)

last_rise = None
pulse_us = 0
new_pulse = False

def cb(chip, gpio, level, tick):
    global last_rise, pulse_us, new_pulse

    if level == 1:
        last_rise = tick
    elif level == 0 and last_rise is not None:
        # Convert nanoseconds to microseconds
        pulse_us = (tick - last_rise) // 1000
        new_pulse = True

lgpio.callback(h, GPIO, lgpio.BOTH_EDGES, cb)

print("Measuring CH1 pulse width (Ctrl+C to exit)")
print("Expected range: 1000-2000 µs\n")

try:
    while True:
        if new_pulse:
            # Validate typical RC range
            if 800 < pulse_us < 2200:
                print(f"CH1: {pulse_us:4d} µs")
                
            new_pulse = False
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\n\nStopping...")
finally:
    lgpio.gpiochip_close(h)