# SPDX-License-Identifier: MIT
"""
SEN0140 live readout: streams every sensor so you can move the board around
and check that the numbers make sense. Readings are calibrated with the values
in settings.toml, if there are any.

Each line shows:

  acc    size of the acceleration. Still, in any position: about 9.8 m/s^2.
  turn   how far the board has turned on each axis since you last pressed z.
         Turn it a quarter turn: one axis should read about 90 degrees.
  field  size of the magnetic field with the stored offset removed. Should
         stay near the Earth's field (25 to 65 uT) however the board is held.
  raw    the same straight from the chip, for comparison.
  hPa    air pressure.
  elev   elevation change since you last pressed h, from air pressure.
         Lift the board 1 m: it should read about +1.0 m.
  temps  temperature in degrees C from two different chips: the barometer
         and the gyro. They should agree to within a degree or two.

Keys (type in the serial console, no Enter needed):
  z  zero the turn angles      h  zero the elevation
  c  calibrate, step by step: gyro, then magnetometer, then the lines to
     paste into settings.toml
  p  show the settings.toml lines for the calibration now in use
  g  run the guided pass/fail test
"""

import math
import sys
import time

import adafruit_bmp280
import supervisor

import imu
import imu_calibration

# Which pins the sensor is on, as (SCL, SDA). None means the board's default
# I2C pins. A Raspberry Pi Pico has no default, so use ("GP5", "GP4") there.
I2C_PINS = None

PRINT_EVERY_S = 0.1
HEADER_EVERY = 50  # lines


def length(v):
    return math.sqrt(sum(c * c for c in v))


def key_pressed():
    if supervisor.runtime.serial_bytes_available:
        return sys.stdin.read(1).lower()
    return ""


def print_header():
    print()
    print(
        "z: zero turn angles, h: zero elevation, c: calibrate, "
        "p: show settings, g: run guided test"
    )
    print(
        "acc m/s^2 | turn x / y / z (deg) | field uT  raw | hPa     elev m "
        "|  baro  gyro temps"
    )


def print_calibration(sensors):
    print()
    print("Calibration in use:")
    bias = ", ".join(f"{v:.2f}" for v in sensors.gyro_bias)
    offset = ", ".join(f"{v:.1f}" for v in sensors.magnetometer_offset)
    print(f"  gyro bias           {bias} deg/s  ({sensors.gyro_source})")
    print(f"  magnetometer offset {offset} uT  ({sensors.magnetometer_source})")
    if "none" in (sensors.gyro_source, sensors.magnetometer_source):
        print("  Press c in the readout to calibrate.")


def run_guided_test():
    import test_sensors  # runs the test on import

    # The test sets the chips up its own way, so restart the readout after it.
    test_sensors.ask("Press Enter to go back to the live readout.")
    supervisor.reload()


def main():
    i2c = imu.open_i2c(*I2C_PINS) if I2C_PINS else imu.open_i2c()
    raw = imu.IMU(i2c, strict=False)
    raw.report()
    if raw.barometer is not None:
        # Measure continuously so reading the pressure never waits, and let
        # the chip's own filter smooth out air-current wobble.
        raw.barometer.mode = adafruit_bmp280.MODE_NORMAL
        raw.barometer.iir_filter = adafruit_bmp280.IIR_FILTER_X16
        time.sleep(1.0)  # let the filter settle before the elevation is zeroed

    sensors = imu_calibration.CalibratedIMU(raw)
    print_calibration(sensors)
    time.sleep(2.0)  # long enough to read before the readout starts

    angles = [0.0, 0.0, 0.0]
    last_rate = None
    last_ns = time.monotonic_ns()
    next_print = last_ns
    lines = 0

    while True:
        now = time.monotonic_ns()
        dt = (now - last_ns) / 1e9
        last_ns = now

        # The gyro is read on every pass, not just when printing, so the
        # turn angles add up accurately.
        if raw.gyro:
            rate = sensors.rotation
            if last_rate is not None:
                for i in range(3):
                    angles[i] += (rate[i] + last_rate[i]) / 2 * dt
            last_rate = rate

        if now >= next_print:
            next_print = now + int(PRINT_EVERY_S * 1e9)
            if lines % HEADER_EVERY == 0:
                print_header()
            lines += 1

            if raw.accelerometer:
                acc = f"{length(sensors.acceleration):9.2f}"
            else:
                acc = "       --"
            if raw.gyro:
                turn = " ".join(f"{a:+6.1f}" for a in angles)
                gyro_temp = f"{raw.gyro.temperature:5.1f}"
            else:
                turn, gyro_temp = "                  --", "   --"
            if not raw.magnetometer:
                field = "    --     --"
            else:
                reading = raw.magnetic
                if raw.magnetometer.saturated:
                    field = "  SATURATED  "
                else:
                    corrected = [
                        m - o for m, o in zip(reading, sensors.magnetometer_offset)
                    ]
                    field = f"{length(corrected):6.1f} {length(reading):6.1f}"
            if raw.barometer:
                baro = f"{sensors.pressure:7.3f} {sensors.elevation_change:+6.2f}"
                baro_temp = f"{sensors.temperature:5.1f}"
            else:
                baro, baro_temp = "     --     --", "   --"
            print(f"{acc} | {turn} | {field} | {baro} | {baro_temp} {gyro_temp}")

        key = key_pressed()
        if key == "z":
            angles = [0.0, 0.0, 0.0]
            print("-- turn angles zeroed")
        elif key == "h" and raw.barometer:
            sensors.zero_elevation()
            print("-- elevation zeroed")
        elif key == "c":
            sensors.calibrate()
            angles = [0.0, 0.0, 0.0]
        elif key == "p":
            print_calibration(sensors)
            print()
            print(sensors.settings_lines())
            imu_calibration.wait_for_key("Press Enter to go back to the readout.")
        elif key == "g":
            run_guided_test()

        if key in ("c", "p"):
            # The readout was paused: don't count that time as turning, and
            # show the header again.
            last_rate = None
            last_ns = time.monotonic_ns()
            lines = 0

        time.sleep(0.005)


try:
    main()
except RuntimeError as error:
    print(f"\nSTOPPED: {error}")
