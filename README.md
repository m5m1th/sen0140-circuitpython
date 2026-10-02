# CircuitPython drivers for the DFRobot SEN0140 10-DOF IMU

Drivers for the gyroscope and magnetometer on the DFRobot SEN0140 "Fermion
10 DOF IMU", a wrapper that presents all four sensors as one object, and a
verification script. The accelerometer (ADXL345) and barometer (BMP280) use
Adafruit's libraries.

| File | What it is |
|---|---|
| `lib/itg3205.py` | Gyroscope driver (ITG-3205 / ITG-3200) |
| `lib/qmc5883.py` | Magnetometer driver. Detects HMC5883L, QMC5883L or VCM5883L at runtime |
| `lib/imu.py` | All four sensors as one object, plus an I2C scan helper |
| `lib/imu_calibration.py` | `CalibratedIMU`: readings with gyro bias and magnetometer offset removed, plus elevation change |
| `test_sensors.py` | Guided verification with PASS/FAIL results |
| `code.py` | Live readout of every sensor; press `g` for the guided test |

**Status:** tested on one board (VCM5883L magnetometer, BMP280 barometer) on
a Seeed XIAO RP2350 and a Raspberry Pi Pico 2 W with CircuitPython 10.3.1. The HMC5883L and QMC5883L
paths have never run on real chips. See [Known limits](#known-limits).

## Install

1. Copy the four files in `lib/` to `CIRCUITPY/lib/`.
2. Install the Adafruit libraries:

   ```bash
   circup install --py adafruit_adxl34x adafruit_bmp280 adafruit_bus_device
   ```

3. Optional: copy `test_sensors.py` and `code.py` to the root of `CIRCUITPY`.

Then connect the sensor's SDA, SCL, 3V3 and GND. See
[Example wiring](#example-wiring) for two boards.

## Example wiring

`imu.IMU()` uses the board's default I2C pins. A board without default pins
needs them named, as (SCL, SDA).

| SEN0140 | Seeed XIAO RP2350 | Raspberry Pi Pico 2 W |
|---|---|---|
| SDA | D4 | GP4 (pin 6) |
| SCL | D5 | GP5 (pin 7) |
| 3V3 | 3V3 | 3V3(OUT) (pin 36) |
| GND | GND | GND (pin 38) |
| In code | `imu.IMU()` | `imu.IMU(imu.open_i2c("GP5", "GP4"))` |
| In `code.py` and `test_sensors.py` | `I2C_PINS = None` | `I2C_PINS = ("GP5", "GP4")` |

Both were tested with CircuitPython 10.3.1

## Use

```python
import imu

imu.scan()  # list every chip on the bus

sensors = imu.IMU()
sensors.report()  # chips found, addresses, chip IDs
sensors.acceleration  # (x, y, z) m/s^2
sensors.rotation  # (x, y, z) degrees per second
sensors.magnetic  # (x, y, z) microtesla
sensors.field_strength  # microtesla
sensors.pressure  # hPa
sensors.temperature  # degrees C, from the barometer
sensors.gyro.temperature  # degrees C, from the gyro chip
```

Each driver also works on its own: `itg3205.ITG3205(i2c)` and
`qmc5883.QMC5883(i2c)`. `imu.IMU(strict=False)` starts with sensors missing;
they become `None` and the reasons are listed in `sensors.problems`.

### Settings

| Setting | Choices | Default |
|---|---|---|
| `sensors.gyro.filter_bandwidth` | 5, 10, 20, 42, 98, 188, 256 Hz | 42 |
| `sensors.gyro.sample_rate` | 3.9 to 1000 Hz (31.25 to 8000 with the 256 Hz filter) | 100 |
| `sensors.magnetometer.data_rate` | see `.data_rates` | 50 (HMC: 15) |
| `sensors.magnetometer.range` | see `.ranges`, in microtesla | 800 (HMC: 130) |

| Magnetometer | `data_rates` (Hz) | `ranges` (µT) |
|---|---|---|
| HMC5883L | 0.75, 1.5, 3, 7.5, 15, 30, 75 | 88, 130, 190, 250, 400, 470, 560, 810 |
| QMC5883L | 10, 50, 100, 200 | 200, 800 |
| VCM5883L | 10, 50, 100, 200 | 800 |

A value the chip doesn't offer raises `ValueError` listing the choices. The
gyro's full-scale range is fixed at ±2000 °/s by the chip.

### Chip identity and raw registers

```python
sensors.magnetometer.chip  # "VCM5883L", "QMC5883L" or "HMC5883L"
sensors.magnetometer.chip_id  # b'\x82', b'\xff' or b'H43'
sensors.gyro.chip_id  # 0x68
sensors.gyro.read_register(0x00)  # one register of one chip
sensors.read_register(0x53, 0x00)  # any register of any chip on the bus
```

### Calibration

The drivers return what the chips report. `CalibratedIMU` wraps an `IMU` and
gives the same readings with the corrections applied:

```python
import imu, imu_calibration

sensors = imu_calibration.CalibratedIMU(imu.IMU())
sensors.rotation  # gyro bias removed
sensors.magnetic  # magnetometer offset removed
sensors.field_strength  # size of the corrected field
sensors.elevation_change  # meters since zero_elevation()
sensors.zero_elevation()
sensors.raw.rotation  # the uncorrected reading
```

The corrections come from values you pass in (`gyro_bias=`,
`magnetometer_offset=`), otherwise from `settings.toml`, otherwise none.
`sensors.gyro_source` and `sensors.magnetometer_source` say which.

To calibrate, run `sensors.calibrate()` (or press `c` in `code.py`). It goes
one step at a time and outputs the lines to paste into `settings.toml`.

```toml
SEN0140_GYRO_BIAS = "-0.41, 0.36, -0.77"
SEN0140_MAG_OFFSET = "4.8, 66.4, -14.1"
# My settings from a Xiao RP2350 on a breadboard. Your values will be different!
```

- **Gyro bias drifts with temperature.** `sensors.calibrate_gyro()` refreshes
  it in 2 seconds and is safe to call at every start-up: it returns `False`
  and keeps the value in use if the board was moving, or if the result is more
  than 3 °/s from that value. `sensors.message` says what happened.
  `force=True` skips the comparison, for when the stored value is wrong.
- **The magnetometer offset depends on what is near the sensor.** Measure it
  again after mounting the board somewhere new.
  `sensors.calibrate_magnetometer()` keeps the old offset if the board was
  not turned through enough directions.
- **Elevation is zeroed at start-up**, never stored, because air pressure
  changes with the weather. It is corrected for air temperature using the
  barometer's thermometer. One reading is good to about 0.3 m; average
  several for better.

`magnetometer.saturated` is `True`, and the readings are `nan`, when the
field exceeds the chip's range.

## Verify a board

At the REPL, `import test_sensors` runs six guided checks (about two minutes)
and prints PASS, WARN or FAIL for each with the expected value. Set
`EXPECTED_FIELD_UT` at the top of the file to the Earth's field where you are
(eg 48 µT in Santa Cruz, CA). Any key or message answers a prompt.

| Check | Expected |
|---|---|
| All four chips found and identified | found |
| Accelerometer at rest | 8.8 to 10.8 m/s² |
| Gyro at rest | bias ≤ 40 °/s, noise ≤ 1 °/s |
| Gyro quarter turn, integrated | 80° to 100° |
| Magnetometer at rest | 25 to 65 µT |
| Magnetometer rotated through every direction | Earth's field ± 20% |
| Barometer lifted and lowered | 0.08 to 0.17 hPa per meter |

An out-of-range magnetometer at rest becomes a WARN, not a FAIL, when the rotation
check passes: the scale is right and the board has a fixed magnetic offset. This can happen when your
breadboard clips are slightly magnetic...whoops.

`code.py` streams acceleration, turn angle, calibrated and raw field
strength, elevation and temperature continuously. It uses the calibration in
`settings.toml` and measures nothing at start-up. Keys: `z` zero the angles,
`h` zero the elevation, `c` calibrate step by step, `p` show the calibration
in use, `g` run the guided test.

### Results on the test board

| Check | Result |
|---|---|
| gravity | 9.64 m/s² |
| gyro bias / noise | 0.80 / 0.05 °/s |
| gyro quarter turn | 89.16° |
| field strength at rest | 119.36 µT |
| field strength after rotation | 44.28 µT |
| pressure change per meter | 0.13 hPa/m |

The field at rest is far outside Earth's range because something fixed to
the sensor adds an offset of about 110 µT. The offset changed when the same
sensor was moved to the Pico, so it comes from the surroundings. Use
`CalibratedIMU` wherever the field strength matters.

## Known limits

- **HMC5883L and QMC5883L are untested on hardware.** Their register values
  and scale factors come from the datasheets and were checked only against
  mock chips written from those same datasheets. They work, _in theory..._
- **Axes are not aligned between chips.** Each driver reports its own chip's
  axes, and the VCM5883L's sign flip is copied from DFRobot without a hardware
  check. Magnitudes are unaffected; headings would be.
- **No compass heading helper.**
- **VCM5883L data rate is only partly confirmed.** The setting is accepted and
  changes how often readings update, but the measured update rate does not
  match the nominal value cleanly.
- **The ITG-3205 is treated as an ITG-3200.** No ITG-3205 datasheet was
  found, and the two cannot be told apart in software.
- **The QMC5883L setup writes registers 0x20 and 0x21**, as DFRobot's code
  does. They are not in the QMC5883L datasheet.
- **The accelerometer reads about 2% high**, because Adafruit's library uses
  4 mg per count where the datasheet gives 3.9.

Two chip behaviors found on hardware and handled in the drivers: the
VCM5883L does not acknowledge the first I2C access after a soft reset, and the
ITG-3205 takes up a new filter setting only at its next sample, so reading the
register straight back returns the old value.

## License and credits

MIT license, Copyright (c) 2026 Matt E Smith. See [LICENSE](LICENSE).

Magnetometer chip detection and register handling are ported from DFRobot's
Arduino library [DFRobot_QMC5883](https://github.com/DFRobot/DFRobot_QMC5883)
(commit 0787570), MIT license, Copyright 2010 DFRobot Co.Ltd.

This port differs from it where the datasheets disagree with its code:

- Readings are converted to microtesla. DFRobot's library returns raw counts,
  and its one QMC5883L scale constant (4.35 mG per count) would give readings
  13 times too large.
- The VCM5883L and QMC5883L chip IDs are checked before the chip is used.
- QMC5883L oversampling is 512. DFRobot's setting selects 64.
- The range is written to the register the QMC5883L datasheet names.
- No write is made to registers the VCM5883L does not define.

Datasheets used: InvenSense ITG-3200 (PS-ITG-3200A rev 1.4 and 1.7),
Honeywell HMC5883L, QST QMC5883L rev 1.0, Voltafield VCM5883. Register
values and scale factors in the drivers come from these.
