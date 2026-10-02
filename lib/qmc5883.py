# SPDX-License-Identifier: MIT
"""
CircuitPython driver for the magnetometer on the DFRobot SEN0140
"Fermion 10 DOF IMU". The board has shipped with three different magnetometer
chips, so this driver works out which one is fitted:

    ============  =============  ==========================
    Chip          I2C address    Identified by
    ============  =============  ==========================
    HMC5883L      0x1E           ID registers read "H43"
    QMC5883L      0x0D           chip ID register reads 0xFF
    VCM5883L      0x0C           chip ID register reads 0x82
    ============  =============  ==========================

    import board
    from qmc5883 import QMC5883

    mag = QMC5883(board.I2C())
    print(mag.chip)            # which part was found, e.g. "VCM5883L"
    print(mag.magnetic)        # (x, y, z) in microtesla
    print(mag.field_strength)  # size of the field in microtesla

Settings (the choices depend on which chip was found):

    print(mag.data_rates)      # e.g. (10, 50, 100, 200) readings per second
    mag.data_rate = 100
    print(mag.ranges)          # e.g. (800,) full-scale range in microtesla
    mag.range = 800

Chip detection and register handling are ported from DFRobot_QMC5883
(https://github.com/DFRobot/DFRobot_QMC5883, MIT license, Copyright 2010
DFRobot Co.Ltd). Scale factors are from the Honeywell HMC5883L, QST QMC5883L
and Voltafield VCM5883 datasheets.
"""

import math
import struct
import time

from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.2.0"

_UT_PER_GAUSS = 100.0

# Each chip class below has the same shape:
#   data_rates, ranges   the choices, where the position in the tuple is the
#                        code written to the chip
#   data_rate, range     the choice in use
#   ut_per_lsb           microtesla per count at that range
#   identify(device)     -> (chip ID bytes, whether they are the right ones)
#   configure(device)    one-time setup, ending with apply()
#   apply(device)        write data_rate and range to the chip
#   read_raw(device)     -> (x, y, z, saturated) in counts


class _HMC5883L:
    name = "HMC5883L"
    address = 0x1E

    _CONFIG_A = 0x00  # output rate in bits 4-2; the rest stay 0
    _CONFIG_B = 0x01  # gain in bits 7-5
    _MODE = 0x02
    _DATA_X_MSB = 0x03
    _IDENT_A = 0x0A

    data_rates = (0.75, 1.5, 3, 7.5, 15, 30, 75)
    ranges = (88, 130, 190, 250, 400, 470, 560, 810)
    _LSB_PER_GAUSS = (1370, 1090, 820, 660, 440, 390, 330, 230)
    _MODE_CONTINUOUS = 0x00
    # An axis that overflowed reads exactly this.
    _OVERFLOW = -4096

    def __init__(self):
        self.data_rate = 15
        self.range = 130

    def identify(self, device):
        chip_id = _read(device, self._IDENT_A, 3)
        return chip_id, chip_id == b"H43"

    def configure(self, device):
        self.apply(device)

    def apply(self, device):
        gain = self.ranges.index(self.range)
        _write(device, self._CONFIG_A, self.data_rates.index(self.data_rate) << 2)
        _write(device, self._CONFIG_B, gain << 5)
        _write(device, self._MODE, self._MODE_CONTINUOUS)
        self.ut_per_lsb = _UT_PER_GAUSS / self._LSB_PER_GAUSS[gain]
        # The first measurement after a gain change still uses the old gain.
        time.sleep(2.0 / self.data_rate)

    def read_raw(self, device):
        # The registers are in the order X, Z, Y.
        x, z, y = struct.unpack(">hhh", _read(device, self._DATA_X_MSB, 6))
        return x, y, z, self._OVERFLOW in (x, y, z)


class _QMC5883L:
    name = "QMC5883L"
    address = 0x0D

    _DATA_X_LSB = 0x00
    # Oversampling in bits 7-6 (left at 0, the quietest), range in bits 5-4,
    # output rate in bits 3-2, mode in bits 1-0.
    _CONTROL_1 = 0x09
    _SET_RESET_PERIOD = 0x0B
    _CHIP_ID = 0x0D

    _CHIP_ID_VALUE = 0xFF
    _SET_RESET_PERIOD_VALUE = 0x01
    _MODE_CONTINUOUS = 0x01
    data_rates = (10, 50, 100, 200)
    ranges = (200, 800)
    _LSB_PER_GAUSS = (12000, 3000)
    # The status register follows the data registers.
    _STATUS_OVERFLOW = 0x02

    # DFRobot's library writes these two registers, which the QMC5883L
    # datasheet does not list.
    _UNDOCUMENTED_SETUP = ((0x20, 0x40), (0x21, 0x01))

    def __init__(self):
        self.data_rate = 50
        self.range = 800

    def identify(self, device):
        chip_id = _read(device, self._CHIP_ID, 1)
        if chip_id[0] != self._CHIP_ID_VALUE:
            return chip_id, False
        # 0xFF is also what an empty bus reads, so check that a register
        # keeps a value written to it.
        _write(device, self._SET_RESET_PERIOD, self._SET_RESET_PERIOD_VALUE)
        kept = _read(device, self._SET_RESET_PERIOD, 1)[0]
        return chip_id, kept == self._SET_RESET_PERIOD_VALUE

    def configure(self, device):
        _write(device, self._SET_RESET_PERIOD, self._SET_RESET_PERIOD_VALUE)
        for register, value in self._UNDOCUMENTED_SETUP:
            _write(device, register, value)
        self.apply(device)

    def apply(self, device):
        range_code = self.ranges.index(self.range)
        rate_code = self.data_rates.index(self.data_rate)
        _write(
            device,
            self._CONTROL_1,
            range_code << 4 | rate_code << 2 | self._MODE_CONTINUOUS,
        )
        self.ut_per_lsb = _UT_PER_GAUSS / self._LSB_PER_GAUSS[range_code]
        time.sleep(2.0 / self.data_rate)  # let fresh measurements arrive

    def read_raw(self, device):
        x, y, z, status = struct.unpack("<hhhB", _read(device, self._DATA_X_LSB, 7))
        return x, y, z, bool(status & self._STATUS_OVERFLOW)


class _VCM5883L:
    name = "VCM5883L"
    address = 0x0C

    _DATA_X_LSB = 0x00
    # Bits 7-4 must be 0100, output rate in bits 3-2, bit 0 = measuring.
    _CONTROL_2 = 0x0A
    _CONTROL_1 = 0x0B
    _CHIP_ID = 0x0C

    _CHIP_ID_VALUE = 0x82
    _SOFT_RESET = 0x80
    _AUTO_SET_RESET = 0x00
    _CONTROL_2_FIXED = 0x40
    _MODE_NORMAL = 0x01
    data_rates = (200, 100, 50, 10)
    ranges = (800,)
    ut_per_lsb = _UT_PER_GAUSS / 3000.0
    # Outputs saturate at the ends of the 16-bit range.
    _LIMITS = (-32768, 32767)

    def __init__(self):
        self.data_rate = 50
        self.range = 800

    def identify(self, device):
        chip_id = _read(device, self._CHIP_ID, 1)
        return chip_id, chip_id[0] == self._CHIP_ID_VALUE

    def configure(self, device):
        _write(device, self._CONTROL_1, self._SOFT_RESET)
        time.sleep(0.01)
        # The reset leaves the chip in standby, where it does not acknowledge
        # the I2C access that wakes it.
        _wake(device)
        _write(device, self._CONTROL_1, self._AUTO_SET_RESET)
        self.apply(device)

    def apply(self, device):
        rate_code = self.data_rates.index(self.data_rate)
        _write(
            device,
            self._CONTROL_2,
            self._CONTROL_2_FIXED | rate_code << 2 | self._MODE_NORMAL,
        )
        time.sleep(2.0 / self.data_rate)  # let fresh measurements arrive

    def read_raw(self, device):
        # All six bytes must be read together: the chip holds the data still
        # until the last one has been read.
        x, y, z = struct.unpack("<hhh", _read(device, self._DATA_X_LSB, 6))
        saturated = any(v in self._LIMITS for v in (x, y, z))
        # The signs are flipped to match DFRobot's library. This has not been
        # checked against a known field direction.
        return -x, -y, -z, saturated


# The order the addresses are tried in.
_CHIPS = (_HMC5883L, _QMC5883L, _VCM5883L)

ADDRESSES = tuple(chip.address for chip in _CHIPS)


class QMC5883:
    """Magnetometer on the SEN0140. Finds out which chip is fitted.

    :param i2c: the ``busio.I2C`` bus (usually ``board.I2C()``)
    :param address: leave as ``None`` to search 0x1E, 0x0D, then 0x0C.
    """

    def __init__(self, i2c, address=None):
        chips = [c for c in _CHIPS if address is None or c.address == address]
        if not chips:
            valid = _hex(ADDRESSES, ", ")
            raise ValueError(
                f"0x{address:02X} is not a magnetometer address. Use one of: {valid}"
            )

        problems = []
        for chip_class in chips:
            device = _open(i2c, chip_class.address)
            if device is None:
                continue
            chip = chip_class()
            chip_id, ok = chip.identify(device)
            if ok:
                break
            problems.append(
                f"Something answered at 0x{chip.address:02X}, where a {chip.name} "
                f"would be, but its ID is {_hex(chip_id)} instead."
            )
        else:
            if problems:
                raise RuntimeError(
                    " ".join(problems) + " Run the I2C scan to see what is on the bus."
                )
            tried = _hex((c.address for c in chips), " or ")
            raise RuntimeError(
                f"No magnetometer found at {tried} - check the SDA and SCL wires."
            )

        self._chip = chip
        self.i2c_device = device
        self.address = chip.address
        #: "HMC5883L", "QMC5883L" or "VCM5883L".
        self.chip = chip.name
        #: The ID bytes the chip reported.
        self.chip_id = chip_id
        #: True if the last reading was beyond the chip's range (a strong
        #: magnet or motor nearby). Such a reading is returned as nan.
        self.saturated = False

        chip.configure(device)

    @property
    def magnetic(self):
        """Magnetic field as ``(x, y, z)`` in microtesla."""
        x, y, z, self.saturated = self._chip.read_raw(self.i2c_device)
        if self.saturated:
            nan = float("nan")
            return (nan, nan, nan)
        scale = self._chip.ut_per_lsb
        return (x * scale, y * scale, z * scale)

    @property
    def field_strength(self):
        """Size of the magnetic field in microtesla. Earth's field is 25 to
        65 uT everywhere on the planet."""
        x, y, z = self.magnetic
        return math.sqrt(x * x + y * y + z * z)

    # ---- Settings ---------------------------------------------------------

    @property
    def data_rates(self):
        """The output rates this chip offers, in readings per second (Hz)."""
        return tuple(sorted(self._chip.data_rates))

    @property
    def data_rate(self):
        """How many new readings the chip makes per second (Hz). Must be one
        of ``data_rates``. Reading faster than this returns repeats."""
        return self._chip.data_rate

    @data_rate.setter
    def data_rate(self, hertz):
        self._set("data_rate", hertz, self._chip.data_rates, "Hz")

    @property
    def ranges(self):
        """The full-scale ranges this chip offers, in microtesla."""
        return tuple(sorted(self._chip.ranges))

    @property
    def range(self):
        """Largest field the chip can measure on each axis, in microtesla
        (plus or minus). Must be one of ``ranges``. A smaller range gives
        finer steps but saturates sooner."""
        return self._chip.range

    @range.setter
    def range(self, microtesla):
        self._set("range", microtesla, self._chip.ranges, "uT")

    def _set(self, name, value, choices, unit):
        if value not in choices:
            setting = name.replace("_", " ")
            listed = ", ".join(str(c) for c in sorted(choices))
            raise ValueError(
                f"The {self.chip} can't use a {setting} of {value} {unit}. "
                f"Choose one of: {listed}"
            )
        # Store the listed value, so 50.0 and 50 behave the same.
        setattr(self._chip, name, choices[choices.index(value)])
        self._chip.apply(self.i2c_device)

    # ---- Register access --------------------------------------------------

    def read_register(self, register):
        """Read one byte from a register. Returns an int from 0 to 255."""
        return _read(self.i2c_device, register, 1)[0]

    def read_registers(self, register, count):
        """Read ``count`` bytes starting at ``register``. Returns bytes."""
        return _read(self.i2c_device, register, count)


def _open(i2c, address):
    # Two tries: a chip in standby may ignore the first one (see _wake).
    for _ in range(2):
        try:
            return I2CDevice(i2c, address)
        except ValueError:
            time.sleep(0.005)
    return None


def _wake(device, tries=10):
    for _ in range(tries):
        try:
            _read(device, 0x00, 1)
            return
        except OSError:
            time.sleep(0.002)
    raise RuntimeError(
        "The magnetometer stopped answering after it was reset. Unplug the "
        "board, plug it back in, and try again."
    )


def _read(device, register, count):
    result = bytearray(count)
    with device as i2c:
        i2c.write_then_readinto(bytes((register,)), result)
    return bytes(result)


def _write(device, register, value):
    with device as i2c:
        i2c.write(bytes((register, value)))


def _hex(values, separator=" "):
    return separator.join(f"0x{v:02X}" for v in values)
