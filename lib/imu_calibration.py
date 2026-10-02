# SPDX-License-Identifier: MIT
"""
Calibrated readings for the SEN0140.

The drivers report what the chips say. ``CalibratedIMU`` wraps an ``imu.IMU``
and gives the same readings with the corrections already applied:

    import imu, imu_calibration

    sensors = imu_calibration.CalibratedIMU(imu.IMU())
    sensors.rotation           # gyro bias removed
    sensors.magnetic           # magnetometer offset removed
    sensors.field_strength     # size of the corrected field
    sensors.elevation_change   # meters since zero_elevation()
    sensors.raw.rotation       # the uncorrected reading

The corrections come from, in order: values you pass in, then these two lines
in settings.toml, then nothing (readings pass through unchanged):

    SEN0140_GYRO_BIAS = "-0.42, 0.53, -0.79"
    SEN0140_MAG_OFFSET = "-112.4, 8.1, 21.7"

``sensors.calibrate()`` walks through measuring both, one step at a time,
and shows the lines to paste.

Gyro bias: a gyro held still should read zero, but an ITG-3205 may read up
to +/-40 deg/s on each axis, and the value drifts with temperature.

Magnetometer offset: iron and magnets near the sensor add a fixed field.
Turning the board through every direction shows how far each axis swings; the
middle of each swing is the offset. It changes when the sensor is mounted
somewhere new.
"""

import math
import os
import sys
import time

import supervisor

__version__ = "0.2.0"

GYRO_BIAS_SETTING = "SEN0140_GYRO_BIAS"
MAGNETOMETER_OFFSET_SETTING = "SEN0140_MAG_OFFSET"

# ---- When to refuse a calibration -----------------------------------------
# A gyro held still wobbles by about 0.05 deg/s. Much more than that means the
# board was moving.
STILL_NOISE_LIMIT = 0.5
# A new bias this many deg/s from the one in use is too big a change to be
# temperature drift. A slow, steady turn looks like this: low noise, wrong bias.
BIAS_CHANGE_LIMIT = 3.0
# The largest bias the gyro's datasheet allows, in deg/s.
BIAS_SPEC_LIMIT = 40.0
# Earth's field is 25 to 65 uT everywhere, with a little allowed either side.
EARTH_FIELD_LIMITS = (20.0, 70.0)
# Every axis must swing at least this fraction of the biggest swing, or the
# board was not turned through enough directions.
AXIS_COVERAGE = 0.6

# ---- Pressure to elevation ------------------------------------------------
# Hypsometric equation: height = (R / (g * M)) * T * ln(p0 / p), where
# R / (g * M) is 29.27 m/K for dry air.
_METERS_PER_KELVIN = 29.27
_KELVIN = 273.15
#: Sea-level temperature of the standard atmosphere, in degrees C.
STANDARD_TEMPERATURE = 15.0
# Each temperature reading costs a measurement, so one is reused for this long.
_TEMPERATURE_REFRESH_NS = 5 * 1000000000


def elevation_difference(
    pressure, reference_pressure, temperature=STANDARD_TEMPERATURE
):
    """Meters above the place where the pressure was ``reference_pressure``.

    Both pressures in hPa. ``temperature`` is the air temperature in degrees
    C: warm air is thinner, so the same pressure change means more height
    (about 3.5% more for every 10 degrees).
    """
    return (
        _METERS_PER_KELVIN
        * (temperature + _KELVIN)
        * math.log(reference_pressure / pressure)
    )


class CalibratedIMU:
    """An ``imu.IMU`` with calibration applied to its readings.

    :param raw: the ``imu.IMU`` to wrap. Leave as ``None`` to make one.
    :param gyro_bias: ``(x, y, z)`` in degrees per second. Leave as ``None``
        to read it from settings.toml.
    :param magnetometer_offset: ``(x, y, z)`` in microtesla. Leave as
        ``None`` to read it from settings.toml.

    Anything not listed here (``gyro``, ``report()``, ...) is passed straight
    to the wrapped IMU.
    """

    def __init__(self, raw=None, gyro_bias=None, magnetometer_offset=None):
        if raw is None:
            import imu

            raw = imu.IMU()
        #: The uncalibrated IMU.
        self.raw = raw
        #: ``(x, y, z)`` subtracted from every gyro reading, in deg/s, and
        #: where it came from: "given", "settings.toml", "measured" or "none".
        self.gyro_bias, self.gyro_source = _starting_value(gyro_bias, GYRO_BIAS_SETTING)
        #: The same for the magnetometer, in uT.
        self.magnetometer_offset, self.magnetometer_source = _starting_value(
            magnetometer_offset, MAGNETOMETER_OFFSET_SETTING
        )
        #: What the last calibrate_gyro() or calibrate_magnetometer() found.
        self.message = ""
        #: Pressure in hPa that counts as zero elevation.
        self.reference_pressure = None
        self._temperature = None
        self._temperature_ns = 0
        if getattr(raw, "barometer", None) is not None:
            self.zero_elevation()

    def __getattr__(self, name):
        return getattr(self.raw, name)

    # ---- Readings ---------------------------------------------------------

    @property
    def acceleration(self):
        """``(x, y, z)`` in m/s^2. Not corrected."""
        return self.raw.acceleration

    @property
    def rotation(self):
        """``(x, y, z)`` turning speed in degrees per second, bias removed."""
        return tuple(r - b for r, b in zip(self.raw.rotation, self.gyro_bias))

    @property
    def magnetic(self):
        """``(x, y, z)`` magnetic field in microtesla, offset removed."""
        return tuple(m - o for m, o in zip(self.raw.magnetic, self.magnetometer_offset))

    @property
    def field_strength(self):
        """Size of the corrected magnetic field in microtesla. Earth's field
        is 25 to 65 uT; well outside that means magnetic interference."""
        x, y, z = self.magnetic
        return math.sqrt(x * x + y * y + z * z)

    @property
    def pressure(self):
        """Pressure in hPa. Not corrected."""
        return self.raw.pressure

    @property
    def temperature(self):
        """Temperature in degrees C, from the barometer. Not corrected."""
        return self.raw.temperature

    @property
    def elevation_change(self):
        """Meters above where ``zero_elevation()`` was last called (it is
        called once at start-up), corrected for air temperature. One reading
        wobbles by about 0.3 m, so average several for a steadier value."""
        if self.reference_pressure is None:
            self.zero_elevation()
        return elevation_difference(
            self.raw.pressure, self.reference_pressure, self._air_temperature()
        )

    def zero_elevation(self, samples=10):
        """Call the current height zero. Averages a few pressure readings."""
        total = 0.0
        for _ in range(samples):
            total += self.raw.pressure
        self.reference_pressure = total / samples

    def _air_temperature(self):
        now = time.monotonic_ns()
        if (
            self._temperature is None
            or now - self._temperature_ns > _TEMPERATURE_REFRESH_NS
        ):
            self._temperature = self.raw.temperature
            self._temperature_ns = now
        return self._temperature

    # ---- Calibrating ------------------------------------------------------

    def calibrate_gyro(self, seconds=2.0, force=False):
        """Measure the gyro bias with the board held still.

        Returns True and uses the new bias if the measurement is believable.
        Returns False and keeps the bias already in use if the board was
        moving, or if the result is far from that bias. ``message`` says
        which. ``force=True`` skips the comparison with the bias in use, for
        when that one is wrong.
        """
        measured = measure_gyro_bias(self.raw, seconds)
        noise = max(measured.noise)
        biggest = max(abs(b) for b in measured.bias)
        change = max(abs(n - o) for n, o in zip(measured.bias, self.gyro_bias))
        if noise > STILL_NOISE_LIMIT:
            problem = f"the board was moving (noise {noise:.2f} deg/s)"
        elif biggest > BIAS_SPEC_LIMIT:
            problem = f"the bias of {biggest:.1f} deg/s is more than the chip allows"
        elif self.gyro_source != "none" and not force and change > BIAS_CHANGE_LIMIT:
            problem = f"it is {change:.1f} deg/s away from the bias in use"
        else:
            self.gyro_bias = measured.bias
            self.gyro_source = "measured"
            self.message = f"Gyro bias measured: {_join(measured.bias, 2)} deg/s."
            return True
        in_use = self._in_use("gyro")
        self.message = f"Gyro bias not changed: {problem}. {in_use}"
        return False

    def calibrate_magnetometer(self, seconds=20.0, show_progress=True):
        """Measure the magnetometer offset while the board is turned slowly
        through every direction (roll it, tip it, flip it over).

        Returns True and uses the new offset if the board saw a believable
        Earth field on every axis. Returns False and keeps the offset already
        in use otherwise. ``message`` says why.
        """
        measured = measure_magnetometer_offsets(self.raw, seconds, show_progress)
        field = measured.field_estimate
        low, high = EARTH_FIELD_LIMITS
        if min(measured.half_range) < AXIS_COVERAGE * field:
            problem = "the board was not turned through enough directions"
        elif not low <= field <= high:
            problem = f"the field measured {field:.0f} uT, which is not the Earth's"
        else:
            self.magnetometer_offset = measured.offset
            self.magnetometer_source = "measured"
            self.message = (
                f"Magnetometer offset measured: {_join(measured.offset, 1)} uT "
                f"(Earth's field {field:.0f} uT)."
            )
            return True
        in_use = self._in_use("magnetometer")
        self.message = f"Magnetometer offset not changed: {problem}. {in_use}"
        return False

    def settings_lines(self):
        """The lines to paste into settings.toml to keep the calibration now
        in use. A sensor with no calibration gets a comment, not a value."""
        return "\n".join(
            (
                _settings_line(
                    "gyro", GYRO_BIAS_SETTING, self.gyro_bias, self.gyro_source, 2
                ),
                _settings_line(
                    "magnetometer",
                    MAGNETOMETER_OFFSET_SETTING,
                    self.magnetometer_offset,
                    self.magnetometer_source,
                    1,
                ),
            )
        )

    def calibrate(self, magnetometer_seconds=30.0):
        """Walk through both calibrations one step at a time, then show the
        settings.toml lines. Each step waits for Enter (or any key), and a
        step that fails can be tried again or skipped."""
        print()
        print("=" * 60)
        print("CALIBRATION")
        print("=" * 60)

        def measure_gyro():
            print("  Measuring for 3 seconds - don't touch the board...")
            return self.calibrate_gyro(seconds=3.0, force=True)

        print()
        print("Step 1 of 2: gyro")
        print("  Put the board down and take your hands off it.")
        self._step("gyro", "Press Enter when it is still (s to skip).", measure_gyro)

        print()
        print("Step 2 of 2: magnetometer")
        print("  Pick the board up, away from laptops, phones and magnets.")
        print("  When you press Enter, turn it slowly through every direction")
        print(
            f"  for {magnetometer_seconds:.0f} seconds: roll it, tip it, flip it over."
        )
        self._step(
            "magnetometer",
            "Press Enter to start turning (s to skip).",
            lambda: self.calibrate_magnetometer(seconds=magnetometer_seconds),
        )

        print()
        print("Results")
        print("  Paste these lines into settings.toml on the CIRCUITPY drive to")
        print("  keep this calibration after a restart:")
        print()
        print(self.settings_lines())
        wait_for_key("Press Enter when you have copied them.")

    def _step(self, sensor, prompt, measure):
        """Repeat one calibration until it succeeds or is skipped."""
        while True:
            if wait_for_key(prompt) == "s":
                print("  Skipped. " + self._in_use(sensor))
                return
            succeeded = measure()
            print("  " + self.message)
            if succeeded:
                return
            print("  Let's try that again.")

    def _in_use(self, sensor):
        if sensor == "gyro":
            value, source = _join(self.gyro_bias, 2) + " deg/s", self.gyro_source
        else:
            value = _join(self.magnetometer_offset, 1) + " uT"
            source = self.magnetometer_source
        if source == "none":
            return f"No {sensor} calibration is in use."
        return f"Still using {value} ({source})."


# ---- Measuring ------------------------------------------------------------


class GyroBias:
    """What the gyro reads while still."""

    def __init__(self, bias, noise, samples):
        #: ``(x, y, z)`` average reading, in degrees per second.
        self.bias = bias
        #: ``(x, y, z)`` standard deviation of the readings.
        self.noise = noise
        self.samples = samples

    def correct(self, rotation):
        """Remove the bias from a gyro reading ``(x, y, z)``."""
        return tuple(r - b for r, b in zip(rotation, self.bias))


def measure_gyro_bias(gyro, seconds=2.0):
    """Average the gyro for ``seconds`` while the board sits still.

    ``gyro`` is anything with a ``rotation`` property. Returns a GyroBias.
    """
    readings = []
    end = time.monotonic_ns() + int(seconds * 1e9)
    while time.monotonic_ns() < end:
        readings.append(gyro.rotation)
        time.sleep(0.01)
    return GyroBias(_mean(readings), _spread(readings), len(readings))


class MagnetometerOffsets:
    """How far each magnetometer axis swung while the board was turned."""

    def __init__(self, minimum, maximum):
        #: ``(x, y, z)`` middle of each axis' swing: the offset to subtract.
        self.offset = tuple((hi + lo) / 2 for lo, hi in zip(minimum, maximum))
        #: ``(x, y, z)`` half of each axis' swing, in microtesla.
        self.half_range = tuple((hi - lo) / 2 for lo, hi in zip(minimum, maximum))

    @property
    def field_estimate(self):
        """Earth's field strength as measured by the swing, in microtesla.

        An axis only swings the full amount if it was pointed straight along
        the field and straight against it. Missing some directions can make
        a swing smaller, never bigger, so the biggest swing is the best guess.
        """
        return max(self.half_range)


def measure_magnetometer_offsets(magnetometer, seconds=20.0, show_progress=True):
    """Record the smallest and largest reading on each axis while the board
    is turned slowly through every direction.

    ``magnetometer`` is anything with a ``magnetic`` property.
    Returns a MagnetometerOffsets.
    """
    minimum = [float("inf")] * 3
    maximum = [float("-inf")] * 3
    measured = False
    start = time.monotonic_ns()
    end = start + int(seconds * 1e9)
    next_report = start
    while time.monotonic_ns() < end:
        reading = magnetometer.magnetic
        if not any(math.isnan(v) for v in reading):  # nan means saturated
            measured = True
            for axis in range(3):
                minimum[axis] = min(minimum[axis], reading[axis])
                maximum[axis] = max(maximum[axis], reading[axis])
        now = time.monotonic_ns()
        if show_progress and measured and now >= next_report:
            swing = ", ".join(f"{hi - lo:5.1f}" for lo, hi in zip(minimum, maximum))
            print(f"  {(end - now) / 1e9:4.1f} s left   swing so far (uT): {swing}")
            next_report = now + 2 * 1000000000
        time.sleep(0.02)
    if not measured:
        raise RuntimeError(
            "Every magnetometer reading was off the scale. Move the board "
            "away from magnets and motors and try again."
        )
    return MagnetometerOffsets(minimum, maximum)


# ---- Helpers --------------------------------------------------------------


def wait_for_key(prompt, quiet_seconds=0.3):
    """Show a prompt and wait for a reply. Returns it in lower case.

    Any key or message counts as Enter. ``input()`` is not used because some
    serial monitors send no newline, which it would wait for forever. A reply
    ends at a newline, or when no key arrives for ``quiet_seconds``.
    """
    _discard_pending_keys()
    print()
    print(">>> " + prompt)
    reply = sys.stdin.read(1)
    last_key = time.monotonic_ns()
    while reply[-1] not in "\r\n":
        if supervisor.runtime.serial_bytes_available:
            reply += sys.stdin.read(1)
            last_key = time.monotonic_ns()
        elif time.monotonic_ns() - last_key > quiet_seconds * 1e9:
            break
    time.sleep(0.05)  # time for the second half of a "\r\n" to arrive
    _discard_pending_keys()
    return reply.strip().lower()


def _discard_pending_keys():
    while supervisor.runtime.serial_bytes_available:
        sys.stdin.read(1)


def _starting_value(given, setting):
    """Return ((x, y, z), source) for one correction."""
    if given is not None:
        return _xyz(given, "The value given"), "given"
    text = os.getenv(setting)
    if text is None or text == "":
        return (0.0, 0.0, 0.0), "none"
    return (
        _xyz(str(text).split(","), f"{setting} in settings.toml"),
        "settings.toml",
    )


def _settings_line(sensor, setting, value, source, digits):
    if source == "none":
        return f"# {setting}: the {sensor} is not calibrated"
    return f'{setting} = "{_join(value, digits)}"'


def _xyz(values, where):
    try:
        numbers = tuple(float(v) for v in values)
    except (TypeError, ValueError):
        numbers = ()
    if len(numbers) != 3:
        raise ValueError(
            f'{where} should be three numbers like "0.1, -0.2, 0.3", not {values!r}.'
        )
    return numbers


def _join(values, digits):
    return ", ".join(f"{v:.{digits}f}" for v in values)


def _mean(readings):
    n = len(readings)
    return tuple(sum(r[axis] for r in readings) / n for axis in range(3))


def _spread(readings):
    """Standard deviation of each axis."""
    n = len(readings)
    if n < 2:
        return (0.0, 0.0, 0.0)
    mean = _mean(readings)
    return tuple(
        math.sqrt(sum((r[axis] - mean[axis]) ** 2 for r in readings) / (n - 1))
        for axis in range(3)
    )
