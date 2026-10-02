# SPDX-License-Identifier: MIT
"""
All four sensors on the DFRobot SEN0140 "Fermion 10 DOF IMU" as one object.

    import imu

    imu.scan()                 # list every chip on the I2C bus

    sensors = imu.IMU()        # uses the board's default I2C pins
    sensors.report()           # which chips were found, and their IDs
    print(sensors.acceleration)    # (x, y, z) in m/s^2
    print(sensors.rotation)        # (x, y, z) in degrees per second
    print(sensors.magnetic)        # (x, y, z) in microtesla
    print(sensors.field_strength)  # microtesla
    print(sensors.pressure)        # hPa
    print(sensors.temperature)     # degrees C

The accelerometer and barometer use Adafruit's libraries (adafruit_adxl34x,
adafruit_bmp280). The gyro and magnetometer use itg3205.py and qmc5883.py.
For calibrated readings, see imu_calibration.py.

On a board with no default I2C pins (a Raspberry Pi Pico, for example), say
which pins the sensor is on:

    sensors = imu.IMU(imu.open_i2c("GP5", "GP4"))   # SCL, SDA
"""

import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice

import itg3205
import qmc5883

__version__ = "0.2.0"

# What each address on a SEN0140 can be. Used by scan().
KNOWN_DEVICES = (
    (0x0C, "magnetometer (VCM5883L)"),
    (0x0D, "magnetometer (QMC5883L)"),
    (0x1D, "accelerometer (ADXL345, other address)"),
    (0x1E, "magnetometer (HMC5883L)"),
    (0x53, "accelerometer (ADXL345)"),
    (0x68, "gyroscope (ITG-3205 / ITG-3200)"),
    (0x69, "gyroscope (ITG-3205 / ITG-3200, other address)"),
    (0x76, "barometer (BMP280)"),
    (0x77, "barometer (BMP280)"),
)

_ADXL345_ADDRESSES = (0x53, 0x1D)
_ADXL345_ID_REGISTER = 0x00
_ADXL345_ID = 0xE5
_BMP_ADDRESSES = (0x77, 0x76)
_BMP_ID_REGISTER = 0xD0
_BMP280_ID = 0x58

_WIRING_HINT = (
    "Check that 3V3 and GND are connected, and that SDA and SCL go to the "
    "pins the code uses."
)

_bus = None


def open_i2c(scl=None, sda=None):
    """Start the I2C bus, with a readable error if it will not start.

    With no pins given, uses the board's default I2C pins. Otherwise give the
    two pins, as ``board`` pins or as names: ``open_i2c("GP5", "GP4")``.
    Calling this again returns the bus that is already open.
    """
    global _bus
    if _bus is not None:
        return _bus
    use_default = scl is None and sda is None
    if use_default and not hasattr(board, "I2C"):
        raise RuntimeError(
            "This board has no default I2C pins. Say which pins the sensor "
            'is on, for example imu.open_i2c("GP5", "GP4") for SCL on GP5 '
            "and SDA on GP4."
        )
    if not use_default:
        scl, sda = _pin(scl), _pin(sda)
    try:
        _bus = board.I2C() if use_default else busio.I2C(scl, sda)
    except RuntimeError:
        # CircuitPython raises this when nothing pulls SDA and SCL high.
        raise RuntimeError(
            "The I2C bus would not start. That usually means the sensor board "
            "has no power or a wire is loose. " + _WIRING_HINT
        )
    return _bus


def _pin(pin):
    if pin is None:
        raise ValueError("Give both pins: open_i2c(scl, sda).")
    if isinstance(pin, str):
        if not hasattr(board, pin):
            raise ValueError(f"This board has no pin called {pin}.")
        return getattr(board, pin)
    return pin


def scan(i2c=None, show=True):
    """Find every chip on the I2C bus and guess what each one is.

    Prints a table unless ``show`` is False. Returns the list of addresses.
    """
    if i2c is None:
        i2c = open_i2c()
    while not i2c.try_lock():
        pass
    try:
        addresses = i2c.scan()
    finally:
        i2c.unlock()

    if show:
        if not addresses:
            print("No chips answered on the I2C bus. " + _WIRING_HINT)
        else:
            print(f"Found {len(addresses)} chip(s) on the I2C bus:")
            for address in addresses:
                print(f"  0x{address:02X}  {guess(address)}")
    return addresses


def guess(address):
    """Best guess at what's at an I2C address on a SEN0140."""
    for known_address, description in KNOWN_DEVICES:
        if known_address == address:
            return description
    return "unknown - not a chip that belongs on the SEN0140"


def format_chip_id(chip_id):
    """Turn a chip ID (a number or some bytes) into readable text."""
    if isinstance(chip_id, int):
        return f"0x{chip_id:02X}"
    text = " ".join(f"0x{b:02X}" for b in chip_id)
    if all(32 < b < 127 for b in chip_id):
        letters = "".join(chr(b) for b in chip_id)
        text += f' ("{letters}")'
    return text


class IMU:
    """All four SEN0140 sensors as one object.

    :param i2c: an I2C bus. Leave as ``None`` to use the board's default
        I2C pins; see ``open_i2c`` for boards that have none.
    :param strict: if True (the default), stop with an error when any sensor
        is missing. If False, carry on: missing sensors are ``None`` and the
        reasons are listed in ``problems``.
    """

    def __init__(self, i2c=None, strict=True):
        self.i2c = open_i2c() if i2c is None else i2c

        #: One entry per chip found: (job, chip name, I2C address, chip ID).
        self.chips = []
        #: Why each missing sensor is missing.
        self.problems = []

        self.accelerometer = None
        self.gyro = None
        self.magnetometer = None
        self.barometer = None

        if not scan(self.i2c, show=False):
            self.problems.append(
                "No chips answered on the I2C bus at all. " + _WIRING_HINT
            )
        else:
            self.accelerometer = self._try(self._start_accelerometer)
            self.gyro = self._try(self._start_gyro)
            self.magnetometer = self._try(self._start_magnetometer)
            self.barometer = self._try(self._start_barometer)

        if strict and self.problems:
            raise RuntimeError("\n".join(self.problems))

    # ---- Readings ---------------------------------------------------------

    @property
    def acceleration(self):
        """``(x, y, z)`` in m/s^2. About 9.8 straight down when still."""
        return self._need(self.accelerometer, "accelerometer").acceleration

    @property
    def rotation(self):
        """``(x, y, z)`` turning speed in degrees per second (uncalibrated)."""
        return self._need(self.gyro, "gyro").rotation

    @property
    def magnetic(self):
        """``(x, y, z)`` magnetic field in microtesla (uncalibrated)."""
        return self._need(self.magnetometer, "magnetometer").magnetic

    @property
    def field_strength(self):
        """Size of the magnetic field in microtesla (uncalibrated)."""
        return self._need(self.magnetometer, "magnetometer").field_strength

    @property
    def pressure(self):
        """Air pressure in hPa. About 1013 hPa at sea level."""
        return self._need(self.barometer, "barometer").pressure

    @property
    def temperature(self):
        """Temperature in degrees C, from the barometer chip."""
        return self._need(self.barometer, "barometer").temperature

    # ---- Chip identity ----------------------------------------------------

    def report(self):
        """Print every chip that was found, with the ID it reported."""
        for job, chip, address, chip_id in self.chips:
            print(
                f"  {job:<14} {chip:<22} at 0x{address:02X}, "
                f"ID {format_chip_id(chip_id)}"
            )
        for problem in self.problems:
            print("  PROBLEM: " + problem)

    def read_register(self, address, register):
        """Read one byte from any register of any chip on the bus."""
        result = bytearray(1)
        with I2CDevice(self.i2c, address) as device:
            device.write_then_readinto(bytes((register,)), result)
        return result[0]

    # ---- Start-up for each sensor -----------------------------------------

    def _try(self, start):
        try:
            return start()
        except RuntimeError as error:
            self.problems.append(str(error))
            return None

    def _start_accelerometer(self):
        import adafruit_adxl34x

        address = self._find(_ADXL345_ADDRESSES, "accelerometer")
        chip_id = self.read_register(address, _ADXL345_ID_REGISTER)
        if chip_id != _ADXL345_ID:
            raise RuntimeError(
                f"Found something at 0x{address:02X}, but it is not an ADXL345 "
                f"accelerometer (its ID is 0x{chip_id:02X}, expected "
                f"0x{_ADXL345_ID:02X})."
            )
        sensor = adafruit_adxl34x.ADXL345(self.i2c, address)
        self.chips.append(("accelerometer", "ADXL345", address, chip_id))
        return sensor

    def _start_gyro(self):
        sensor = itg3205.ITG3205(self.i2c)
        self.chips.append(("gyroscope", sensor.chip, sensor.address, sensor.chip_id))
        return sensor

    def _start_magnetometer(self):
        sensor = qmc5883.QMC5883(self.i2c)
        self.chips.append(("magnetometer", sensor.chip, sensor.address, sensor.chip_id))
        return sensor

    def _start_barometer(self):
        import adafruit_bmp280

        address = self._find(_BMP_ADDRESSES, "barometer")
        chip_id = self.read_register(address, _BMP_ID_REGISTER)
        if chip_id != _BMP280_ID:
            raise RuntimeError(
                f"Found something at 0x{address:02X}, but it is not a BMP280 "
                f"barometer (its ID is 0x{chip_id:02X}, expected 0x{_BMP280_ID:02X})."
            )
        sensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c, address)
        self.chips.append(("barometer", "BMP280", address, chip_id))
        return sensor

    def _find(self, addresses, job):
        for address in addresses:
            try:
                I2CDevice(self.i2c, address)
                return address
            except ValueError:
                pass  # nothing answered at this address
        tried = " or ".join(f"0x{a:02X}" for a in addresses)
        raise RuntimeError(f"No {job} found at {tried} - check the SDA and SCL wires.")

    @staticmethod
    def _need(sensor, job):
        if sensor is None:
            raise RuntimeError(
                f"There is no {job} to read: it was not found when the IMU "
                "started. Run imu.scan() to see what is connected."
            )
        return sensor
