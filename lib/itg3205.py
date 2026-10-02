# SPDX-License-Identifier: MIT
"""
CircuitPython driver for the InvenSense ITG-3205 / ITG-3200 gyroscope,
as found on the DFRobot SEN0140 "Fermion 10 DOF IMU".

    import board
    from itg3205 import ITG3205

    gyro = ITG3205(board.I2C())
    print(gyro.rotation)      # (x, y, z) in degrees per second
    print(gyro.temperature)   # degrees C

Settings:

    gyro.filter_bandwidth = 20   # Hz; one of gyro.filter_bandwidths
    gyro.sample_rate = 50        # new readings per second

Register values and scale factors are from the ITG-3200 Product Specification
(PS-ITG-3200A). The ITG-3205 has no public datasheet and reports the same
WHO_AM_I value, so the two parts are treated as one.
"""

import struct
import time

from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

__version__ = "0.2.0"

ADDRESSES = (0x68, 0x69)

_WHO_AM_I = const(0x00)
_SMPLRT_DIV = const(0x15)
_DLPF_FS = const(0x16)
_TEMP_OUT_H = const(0x1B)
_GYRO_XOUT_H = const(0x1D)
_PWR_MGM = const(0x3E)

# WHO_AM_I holds the I2C address in bits 6..1. Bits 7 and 0 are undefined.
_WHO_AM_I_MASK = const(0x7E)
_WHO_AM_I_VALUE = const(0x68)

# DLPF_FS bits 4..3 (FS_SEL) must be 3, the only range the chip supports:
# +/-2000 deg/s. Bits 7..5 are undefined.
_FS_SEL_2000DPS = const(0x03 << 3)
_DLPF_FS_MASK = const(0x1F)
# DLPF_FS bits 2..0 select the low-pass filter. The position in this list is
# the code written to the chip.
_FILTER_BANDWIDTHS = (256, 188, 98, 42, 20, 10, 5)
# The chip samples at 8 kHz with the 256 Hz filter and 1 kHz with the others.
_INTERNAL_RATE_256HZ = 8000
_INTERNAL_RATE = 1000
_MAX_DIVIDER = 255

_DEFAULT_BANDWIDTH = 42
_DEFAULT_SAMPLE_RATE = 100

_H_RESET = const(0x80)
# Clock from the X gyro's PLL, which is steadier than the internal oscillator.
_CLK_SEL_PLL_X = const(0x01)

_LSB_PER_DPS = 14.375
# The temperature output reads -13200 at 35 C and changes by 280 per degree.
_TEMP_LSB_PER_C = 280.0
_TEMP_OFFSET_LSB = 13200
_TEMP_REFERENCE_C = 35.0

_RESET_S = 0.020
# A new DLPF_FS value only takes effect at the chip's next sample.
_DLPF_LATCH_S = 0.005
_SETTLE_S = 0.050


class ITG3205:
    """ITG-3205 / ITG-3200 3-axis gyroscope.

    :param i2c: the ``busio.I2C`` bus the gyro is on (usually ``board.I2C()``)
    :param address: I2C address. Leave as ``None`` to try 0x68 then 0x69.
    """

    chip = "ITG-3205 / ITG-3200"

    #: The low-pass filter bandwidths the chip offers, in Hz.
    filter_bandwidths = tuple(sorted(_FILTER_BANDWIDTHS))

    def __init__(self, i2c, address=None):
        self._buffer = bytearray(6)
        self.i2c_device, self.address = _find(i2c, address)

        #: The WHO_AM_I value the chip reported.
        self.chip_id = self.read_register(_WHO_AM_I)
        if self.chip_id & _WHO_AM_I_MASK != _WHO_AM_I_VALUE:
            raise RuntimeError(
                f"Found something at 0x{self.address:02X}, but it is not an "
                f"ITG-3205/3200 gyro (it says its ID is 0x{self.chip_id:02X}, "
                "expected 0x68 or 0x69). Run the I2C scan to see what is on the bus."
            )

        self._write_register(_PWR_MGM, _H_RESET)
        time.sleep(_RESET_S)
        self._write_register(_PWR_MGM, _CLK_SEL_PLL_X)

        self._bandwidth = _DEFAULT_BANDWIDTH
        dlpf_fs = _FS_SEL_2000DPS | _FILTER_BANDWIDTHS.index(_DEFAULT_BANDWIDTH)
        self._write_register(_DLPF_FS, dlpf_fs)
        time.sleep(_DLPF_LATCH_S)
        # Without FS_SEL = 3 every reading is wrong, so check that it stuck.
        if self.read_register(_DLPF_FS) & _DLPF_FS_MASK != dlpf_fs:
            raise RuntimeError(
                f"The gyro at 0x{self.address:02X} did not accept its settings. "
                "Unplug the board, plug it back in, and try again."
            )
        self.sample_rate = _DEFAULT_SAMPLE_RATE
        time.sleep(_SETTLE_S)

    @property
    def rotation(self):
        """How fast the board is turning, as ``(x, y, z)`` in degrees per
        second. Uncalibrated: each axis can read up to +/-40 deg/s when still.
        See ``imu_calibration``."""
        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytes((_GYRO_XOUT_H,)), self._buffer)
        x, y, z = struct.unpack(">hhh", self._buffer)
        return (x / _LSB_PER_DPS, y / _LSB_PER_DPS, z / _LSB_PER_DPS)

    @property
    def temperature(self):
        """Temperature of the gyro chip in degrees C. The chip warms itself a
        little, so this reads slightly above the room."""
        raw = struct.unpack(">h", self.read_registers(_TEMP_OUT_H, 2))[0]
        return _TEMP_REFERENCE_C + (raw + _TEMP_OFFSET_LSB) / _TEMP_LSB_PER_C

    # ---- Settings ---------------------------------------------------------

    @property
    def filter_bandwidth(self):
        """Bandwidth of the chip's low-pass filter in Hz. Must be one of
        ``filter_bandwidths``. Lower is smoother but slower to respond.
        Changing it keeps ``sample_rate`` as close as the chip allows."""
        return self._bandwidth

    @filter_bandwidth.setter
    def filter_bandwidth(self, hertz):
        if hertz not in _FILTER_BANDWIDTHS:
            choices = ", ".join(str(b) for b in self.filter_bandwidths)
            raise ValueError(
                f"The gyro can't use a filter bandwidth of {hertz} Hz. "
                f"Choose one of: {choices}"
            )
        rate = self.sample_rate
        self._write_register(
            _DLPF_FS, _FS_SEL_2000DPS | _FILTER_BANDWIDTHS.index(hertz)
        )
        # Reading DLPF_FS straight back returns the old value (see
        # _DLPF_LATCH_S), so the bandwidth is remembered, not read.
        self._bandwidth = hertz
        # The internal rate may have changed, so set the divider again.
        divider = round(self._internal_rate() / rate) - 1
        self._write_register(_SMPLRT_DIV, min(max(divider, 0), _MAX_DIVIDER))

    @property
    def sample_rate(self):
        """How many new readings the chip makes per second (Hz). The chip
        divides its internal rate by a whole number, so the rate you get back
        is the nearest one it can make. Reading faster returns repeats."""
        return self._internal_rate() / (self.read_register(_SMPLRT_DIV) + 1)

    @sample_rate.setter
    def sample_rate(self, hertz):
        internal = self._internal_rate()
        slowest = internal / (_MAX_DIVIDER + 1)
        if not slowest <= hertz <= internal:
            raise ValueError(
                f"With this filter the gyro's sample rate must be between "
                f"{slowest:g} and {internal:g} Hz, not {hertz}."
            )
        self._write_register(_SMPLRT_DIV, round(internal / hertz) - 1)

    def _internal_rate(self):
        return _INTERNAL_RATE_256HZ if self._bandwidth == 256 else _INTERNAL_RATE

    # ---- Register access --------------------------------------------------

    def read_register(self, register):
        """Read one byte from a register. Returns an int from 0 to 255."""
        return self.read_registers(register, 1)[0]

    def read_registers(self, register, count):
        """Read ``count`` bytes starting at ``register``. Returns bytes."""
        result = bytearray(count)
        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytes((register,)), result)
        return bytes(result)

    def _write_register(self, register, value):
        with self.i2c_device as i2c:
            i2c.write(bytes((register, value)))


def _find(i2c, address):
    candidates = ADDRESSES if address is None else (address,)
    for candidate in candidates:
        try:
            return I2CDevice(i2c, candidate), candidate
        except ValueError:
            pass  # nothing answered at this address
    tried = " or ".join(f"0x{a:02X}" for a in candidates)
    raise RuntimeError(f"No gyro found at {tried} - check the SDA and SCL wires.")
