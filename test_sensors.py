# SPDX-License-Identifier: MIT
"""
SEN0140 verification script. Run it once with the board plugged in and a
serial console open; it walks you through six checks and prints PASS/FAIL
for each, with the value it expected.

What it's really looking for is *wrong scaling*: readings that look
reasonable but are 2x or 10x off. Each check has a known right answer
(gravity, a quarter turn, the Earth's field, the air pressure change over
one meter) so a bad scale factor shows up as a FAIL.

To run: copy this file to the CIRCUITPY drive, open the serial console,
and at the >>> prompt type:   import test_sensors
"""

import math
import time

import imu
import imu_calibration

# ---- Things you might need to change --------------------------------------

# Which pins the sensor is on, as (SCL, SDA). None means the board's default
# I2C pins. A Raspberry Pi Pico has no default, so use ("GP5", "GP4") there.
I2C_PINS = None

# Earth's magnetic field where you are, in microtesla. Santa Cruz, CA is
# about 48 uT. Look yours up with NOAA's "Magnetic Field Calculator"
# (World Magnetic Model), and use the "Total Field" number divided by 1000.
EXPECTED_FIELD_UT = 48.0

# Normal air pressure range where you are, in hPa. Near sea level this is
# about 980-1040. Subtract about 12 hPa for every 100 m you are above sea level.
PRESSURE_RANGE_HPA = (950.0, 1050.0)

# ---- Pass/fail limits ------------------------------------------------------

GRAVITY = 9.80665  # m/s^2
# The accelerometer's sensitivity can be 10% off. A 2x error is far outside.
ACCEL_LIMITS = (8.8, 10.8)

# The largest bias the gyro's datasheet allows, in deg/s.
GYRO_BIAS_LIMIT = 40.0
# A still gyro wobbles far less than this many deg/s. More means the board
# moved.
GYRO_NOISE_LIMIT = 1.0
# A careful hand turn lands within about 10 degrees of a quarter turn. A 2x
# scale error gives 45 or 180.
TURN_LIMITS = (80.0, 100.0)
# How fast the board must turn to count as started and as stopped, in deg/s,
# and how long it must stay stopped.
TURN_START_SPEED = 10.0
TURN_STOP_SPEED = 3.0
TURN_STOP_S = 0.75
TURN_WAIT_S = 15.0
TURN_GIVE_UP_S = 30.0

# Earth's field is 25-65 uT everywhere. Nearby iron can push an uncalibrated
# reading around, so this first check only catches big errors.
EARTH_FIELD_LIMITS = (25.0, 65.0)
# After rotating, the estimate should be within 20% of the expected field,
# which is tight enough to catch a magnetometer set to the wrong range.
CALIBRATED_TOLERANCE = 0.20
# Each axis should swing about the same amount if the board was turned
# through every direction.
AXIS_BALANCE_TOLERANCE = 0.25

# Air near sea level weighs about 1.225 kg/m^3, so pressure drops by
# 1.225 * 9.81 = 12 Pa = 0.12 hPa for every meter you go up.
EXPECTED_HPA_PER_METER = 0.12
LIFT_LIMITS = (0.08, 0.17)

# ---- Results ---------------------------------------------------------------

results = []


def record(name, verdict, measured, expected, hint=""):
    results.append((name, verdict, measured, expected))
    print(f"  [{verdict}] {name}: measured {measured}, expected {expected}")
    if hint:
        print("         " + hint)


def check(name, value, limits, unit, hint_if_fail=""):
    low, high = limits
    passed = low <= value <= high
    record(
        name,
        "PASS" if passed else "FAIL",
        f"{value:.2f} {unit}",
        f"{low:g} to {high:g} {unit}",
        "" if passed else hint_if_fail,
    )
    return passed


def section(title):
    print()
    print("=" * 60)
    print(title)
    print("=" * 60)


def ask(prompt, typing=False):
    """Show a prompt and wait for a reply. ``typing`` allows more time between
    keys, for answers like "1.5"."""
    reply = imu_calibration.wait_for_key(
        prompt + "\n    (press Enter, or send any key)",
        quiet_seconds=2.0 if typing else 0.3,
    )
    if reply:
        print("    got: " + reply)
    return reply


def length(v):
    return math.sqrt(sum(c * c for c in v))


def fmt(v, digits=2):
    return "(" + ", ".join(f"{c:.{digits}f}" for c in v) + ")"


def average(read, seconds, interval=0.02):
    """Average a reading (a number or a tuple) over some seconds."""
    readings = []
    end = time.monotonic_ns() + int(seconds * 1e9)
    while time.monotonic_ns() < end:
        readings.append(read())
        time.sleep(interval)
    if isinstance(readings[0], tuple):
        n = len(readings)
        return tuple(sum(r[i] for r in readings) / n for i in range(len(readings[0])))
    return sum(readings) / len(readings)


def scale_hint(ratio):
    """If a reading is a suspiciously round multiple off, say so."""
    for factor in (10.0, 2.0):
        for r, word in ((factor, "too big"), (1 / factor, "too small")):
            if abs(ratio / r - 1) < 0.15:
                return (
                    f"This is about {factor:g}x {word} - a scale factor is "
                    "probably wrong."
                )
    return ""


# ---- The checks ------------------------------------------------------------


def check_bus():
    section("1. I2C scan - which chips are on the bus?")
    i2c = imu.open_i2c(*I2C_PINS) if I2C_PINS else imu.open_i2c()
    addresses = imu.scan(i2c)
    for address in addresses:
        if imu.guess(address).startswith("unknown"):
            record(
                f"unexpected chip at 0x{address:02X}",
                "WARN",
                "present",
                "absent",
            )

    print("\nAsking each chip what it is...")
    sensors = imu.IMU(i2c, strict=False)
    sensors.report()
    for job, sensor in (
        ("accelerometer", sensors.accelerometer),
        ("gyroscope", sensors.gyro),
        ("magnetometer", sensors.magnetometer),
        ("barometer", sensors.barometer),
    ):
        if sensor is None:
            record(job + " detected", "FAIL", "not found", "found")
        else:
            chip = [c for c in sensors.chips if c[0] == job][0]
            record(
                job + " detected",
                "PASS",
                f"{chip[1]} at 0x{chip[2]:02X}, ID {imu.format_chip_id(chip[3])}",
                "found",
            )
    return sensors


def check_accelerometer(sensors):
    section("2. Accelerometer at rest - should feel gravity, 9.81 m/s^2")
    if sensors.accelerometer is None:
        record("accelerometer at rest", "SKIP", "-", "-")
        return
    ask("Put the board flat on the table and let go of it.")
    reading = average(lambda: sensors.acceleration, 1.5)
    size = length(reading)
    print(f"  Average reading (x, y, z): {fmt(reading)} m/s^2")
    check(
        "gravity",
        size,
        ACCEL_LIMITS,
        "m/s^2",
        scale_hint(size / GRAVITY) or "Was the board still?",
    )


def check_gyro_at_rest(sensors):
    section("3. Gyro at rest - should read near zero once calibrated")
    if sensors.gyro is None:
        record("gyro at rest", "SKIP", "-", "-")
        return None
    ask("Leave the board flat and still. Don't touch the table for 3 seconds.")
    bias = imu_calibration.measure_gyro_bias(sensors.gyro, seconds=3.0)
    print(f"  Averaged {bias.samples} readings.")
    print(f"  Bias (reading while still): {fmt(bias.bias)} deg/s")
    print(f"  Noise (spread, bias removed): {fmt(bias.noise, 3)} deg/s")

    worst_bias = max(abs(b) for b in bias.bias)
    check(
        "gyro bias",
        worst_bias,
        (0.0, GYRO_BIAS_LIMIT),
        "deg/s",
        "Was the board moving?",
    )
    worst_noise = max(bias.noise)
    if worst_noise == 0:
        record(
            "gyro noise",
            "FAIL",
            "exactly 0",
            "a small wobble",
            "The readings never change - the gyro may not be measuring.",
        )
    else:
        check(
            "gyro noise",
            worst_noise,
            (0.0, GYRO_NOISE_LIMIT),
            "deg/s",
            "Too jumpy. Was the board or table bumped? Try again.",
        )
    check(
        "gyro chip temperature",
        sensors.gyro.temperature,
        (10.0, 50.0),
        "C",
        "Is the board unusually hot or cold?",
    )
    return bias


def check_gyro_turn(sensors, bias):
    section("4. Gyro quarter turn - should add up to 90 degrees")
    if sensors.gyro is None or bias is None:
        record("gyro quarter turn", "SKIP", "-", "-")
        return
    while True:
        print("\n  Line one edge of the board up with the edge of the table.")
        ask(
            "Press Enter, then slowly turn the board a quarter turn (90 degrees) "
            "flat on the table, like turning a steering wheel lying flat. "
            "Line it up with the table edge again, then hold still."
        )
        angles = integrate_turn(sensors.gyro, bias)
        if angles is None:
            record(
                "gyro quarter turn",
                "FAIL",
                "no turn seen",
                "90 deg",
                f"Nothing moved in {TURN_WAIT_S:g} s. Is the gyro working? "
                "(check 3 above)",
            )
        else:
            print(f"  Turned (x, y, z): {fmt(angles, 1)} degrees")
            axis = max(range(3), key=lambda i: abs(angles[i]))
            turned = abs(angles[axis])
            print(
                f"  Biggest turn was around the {'xyz'[axis]} axis "
                f"({angles[axis]:+.1f} deg)."
            )
            check(
                "gyro quarter turn",
                turned,
                TURN_LIMITS,
                "deg",
                scale_hint(turned / 90.0) or "Try again, more slowly and squarely.",
            )
        if (
            ask("Type r and press Enter to try again, or just press Enter to go on.")
            != "r"
        ):
            return
        results.pop()  # the new attempt replaces this one


def integrate_turn(gyro, bias):
    """Add up rotation from now until the board has turned and stopped.
    Returns the angle turned on each axis, or None if it never moved."""
    angles = [0.0, 0.0, 0.0]
    started = False
    still_since = None
    start = last = time.monotonic_ns()
    previous = bias.correct(gyro.rotation)
    while True:
        now = time.monotonic_ns()
        rate = bias.correct(gyro.rotation)
        dt = (now - last) / 1e9
        last = now
        for i in range(3):
            angles[i] += (rate[i] + previous[i]) / 2 * dt
        previous = rate
        speed = length(rate)
        elapsed = (now - start) / 1e9

        if not started:
            if speed > TURN_START_SPEED:
                started = True
                print("  ...turning...")
            elif elapsed > TURN_WAIT_S:
                return None
        elif speed >= TURN_STOP_SPEED:
            still_since = None
        elif still_since is None:
            still_since = now
        elif (now - still_since) / 1e9 > TURN_STOP_S:
            return tuple(angles)

        if elapsed > TURN_GIVE_UP_S:
            print(f"  (stopped after {TURN_GIVE_UP_S:g} s)")
            return tuple(angles)
        time.sleep(0.005)


def check_magnetometer(sensors):
    section(
        "5. Magnetometer - should read the Earth's field, "
        f"about {EXPECTED_FIELD_UT:g} uT"
    )
    mag = sensors.magnetometer
    if mag is None:
        record("magnetometer", "SKIP", "-", "-")
        return
    print(f"  Chip: {mag.chip}")
    ask("Put the board flat on the table, away from phones, magnets and laptops.")
    reading = average(lambda: mag.magnetic, 1.0)
    if mag.saturated or any(math.isnan(v) for v in reading):
        record(
            "magnetometer range",
            "FAIL",
            "off the scale",
            "in range",
            "Something magnetic is too close, or the range setting is wrong.",
        )
        return
    size = length(reading)
    print(f"  Average reading (x, y, z): {fmt(reading)} uT")
    raw_ok = check(
        "field strength (uncalibrated)",
        size,
        EARTH_FIELD_LIMITS,
        "uT",
        "Either metal near the sensor adds an offset, or the scale is wrong. "
        "The next check tells them apart.",
    )
    raw_index = len(results) - 1

    print("\n  Next: a sharper check. Turning the board through every direction")
    print("  measures the Earth's field without help from nearby metal, and")
    print("  would catch a scale factor that is only 25% off.")
    if (
        ask(
            "Press Enter, then for 20 seconds slowly roll, tip and flip the board "
            "so it points every which way (type s to skip)."
        )
        == "s"
    ):
        record("field strength (calibrated)", "SKIP", "-", "-")
        return
    offsets = imu_calibration.measure_magnetometer_offsets(mag, seconds=20.0)
    print(f"  Offset from nearby metal: {fmt(offsets.offset)} uT")
    print(f"  Swing on each axis (half): {fmt(offsets.half_range)} uT")
    estimate = offsets.field_estimate
    low = EXPECTED_FIELD_UT * (1 - CALIBRATED_TOLERANCE)
    high = EXPECTED_FIELD_UT * (1 + CALIBRATED_TOLERANCE)
    if estimate > high:
        # Missing some directions can only make the swing smaller.
        hint = (
            "Turning can't make this too big, so the scale factor is probably "
            f"wrong ({estimate / EXPECTED_FIELD_UT:.2f}x)."
        )
    else:
        hint = "Try again, turning through more directions."
    calibrated_ok = check(
        "field strength (calibrated)",
        estimate,
        (low, high),
        "uT",
        scale_hint(estimate / EXPECTED_FIELD_UT) or hint,
    )
    if calibrated_ok and not raw_ok:
        # The scale is right, so the raw reading is off because magnetized
        # metal near the sensor adds a fixed offset. That is not a fault.
        name, _, measured, expected = results[raw_index]
        results[raw_index] = (name, "WARN", measured, expected)
        print(f"  [WARN] {name}: changed from FAIL to WARN.")
        print(
            "         The scale is right; metal near the sensor adds an offset of "
            f"about {length(offsets.offset):.0f} uT."
        )
        print("         Calibrate (see imu_calibration) before using field_strength.")
    worst = max(abs(h / estimate - 1) for h in offsets.half_range)
    passed = worst <= AXIS_BALANCE_TOLERANCE
    record(
        "axes agree",
        "PASS" if passed else "WARN",
        f"within {worst * 100:.0f}%",
        f"within {AXIS_BALANCE_TOLERANCE * 100:.0f}%",
        (
            ""
            if passed
            else "The board may not have pointed every way. If this keeps "
            "happening, one axis is scaled differently from the others."
        ),
    )


def check_barometer(sensors):
    section("6. Barometer - pressure should drop about 0.12 hPa per meter up")
    baro = sensors.barometer
    if baro is None:
        record("barometer", "SKIP", "-", "-")
        return
    check(
        "air pressure",
        baro.pressure,
        PRESSURE_RANGE_HPA,
        "hPa",
        "Check PRESSURE_RANGE_HPA at the top of this file for your altitude.",
    )
    check("air temperature", baro.temperature, (5.0, 45.0), "C")

    print("\n  Live pressure for 5 seconds (breathe on it, or just watch):")
    end = time.monotonic_ns() + int(5e9)
    while time.monotonic_ns() < end:
        print(f"    {baro.pressure:.3f} hPa")
        time.sleep(0.5)

    answer = ask(
        "How far will you lift it, in meters? Type a number, or press Enter for 1.",
        typing=True,
    )
    try:
        height = float(answer) if answer else 1.0
    except ValueError:
        height = 1.0
    if height <= 0:
        height = 1.0

    def measure(label):
        print(f"  Measuring {label} - hold still...")
        value = average(lambda: baro.pressure, 4.0, interval=0.0)
        print(f"  {value:.3f} hPa")
        return value

    ask("Hold the board at table height.")
    low_1 = measure("low")
    ask(f"Now lift it {height:g} m straight up and hold it there.")
    high = measure("high")
    ask("Bring it back down to table height.")
    low_2 = measure("low again")

    drop = (low_1 + low_2) / 2 - high
    per_meter = drop / height
    drift = abs(low_2 - low_1)
    print(f"  Pressure dropped {drop:.3f} hPa going up {height:g} m.")
    check(
        "pressure change per meter",
        per_meter,
        LIFT_LIMITS,
        "hPa/m",
        scale_hint(per_meter / EXPECTED_HPA_PER_METER)
        or "Check the height, and that the two low readings match.",
    )
    if drift > 0.3 * EXPECTED_HPA_PER_METER * height:
        record(
            "pressure steady",
            "WARN",
            f"{drift:.3f} hPa drift",
            "less",
            "The low readings didn't match - a door or air conditioner may "
            "have changed the room's pressure. Try again.",
        )


def summary():
    section("Summary")
    counts = {}
    for name, verdict, measured, expected in results:
        counts[verdict] = counts.get(verdict, 0) + 1
        print(f"  {verdict:<5} {name:<32} {measured}")
    print()
    print(
        f"  {counts.get('PASS', 0)} passed, {counts.get('FAIL', 0)} failed, "
        f"{counts.get('WARN', 0)} warnings, {counts.get('SKIP', 0)} skipped"
    )
    if counts.get("FAIL", 0) == 0 and counts.get("SKIP", 0) == 0:
        print("  All checks passed - the scale factors look right.")
    else:
        print("  Not everything passed - see the notes above each FAIL.")


def main():
    print(
        f"SEN0140 sensor check (imu {imu.__version__}, "
        f"itg3205 {imu.itg3205.__version__}, qmc5883 {imu.qmc5883.__version__})"
    )
    try:
        sensors = check_bus()
        check_accelerometer(sensors)
        bias = check_gyro_at_rest(sensors)
        check_gyro_turn(sensors, bias)
        check_magnetometer(sensors)
        check_barometer(sensors)
    except RuntimeError as error:
        print(f"\nSTOPPED: {error}")
    except KeyboardInterrupt:
        print("\nStopped early.")
    summary()


main()
