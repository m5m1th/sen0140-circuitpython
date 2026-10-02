# Brief: CircuitPython drivers for the DFRobot SEN0140 (10-DOF IMU)

## Context

I teach a Python course to high school students (9th grade and up) in a robotics club
that builds an underwater ROV for the MATE ROV competition. Lessons run on
**CircuitPython**. We are adding a sensor unit, and each student will have:

- A **Seeed Studio XIAO RP2350** running CircuitPython
- A **DFRobot SEN0140 "Fermion 10 DOF IMU"** breakout, wired over I2C with
  pre-soldered Dupont leads (SDA -> D4, SCL -> D5, 3V3, GND)

The SEN0140 carries four separate chips on one flat I2C bus (no auxiliary-bus
bypass needed, unlike the MPU6050-based GY boards):

| Chip | Function | I2C address | Status |
|---|---|---|---|
| ADXL345 | accelerometer | 0x53 | **Adafruit library exists — do not write one** |
| BMP280 | barometer | 0x76 or 0x77 | **Adafruit library exists — do not write one** |
| ITG3205 / ITG3200 | gyroscope | 0x68 (0x69 alt) | needs a driver |
| VCM5883L | magnetometer | varies — see below | needs a driver |

## What I need

CircuitPython drivers for the **gyroscope** and **magnetometer**, plus a small
wrapper that presents all four sensors as one object, plus a verification script.

### Important: board revisions vary

DFRobot has silently changed parts on this board over time. Some retailers still
list it with an **HMC5883L**; current listings say **VCM5883L**; older units had a
BMP180 instead of the BMP280. Batches are not consistent.

DFRobot's own Arduino library `DFRobot_QMC5883` handles **QMC5883, HMC5883 and
VCM5883** with runtime chip detection. Please fetch that library from GitHub and
port its detection logic along with the register handling, so the driver
identifies whichever part actually shipped and configures it accordingly. Do not
hardcode one chip.

Same defensiveness for the gyro (ITG3205 vs ITG3200) and the barometer
(BMP280 vs BMP180) — detect, report, and fail with a clear message rather than
returning plausible-looking garbage.

## Deliverables

1. `itg3205.py` — gyroscope driver
2. `qmc5883.py` — magnetometer driver with runtime chip detection
3. `imu.py` — a wrapper exposing all four sensors through one object
4. `test_sensors.py` — verification script (see below)
5. `README.md` — wiring, install steps, expected output

Plain `.py` files only. **Do not produce `.mpy`.** The RP2350 has flash to spare,
and `mpy-cross` output is tied to the CircuitPython major version, which turns
every firmware update into a recompile we'd forget about.

## API style

This matters — it has to match the rest of the course.

- **Hide the registers.** Students should never see a hex constant.
- **Return real objects in real units**, as properties:
  - gyro: degrees per second, tuple `(x, y, z)`
  - magnetometer: microtesla, tuple `(x, y, z)`
  - accelerometer: m/s^2 (whatever `adafruit_adxl34x` gives)
  - barometer: hPa and degrees C
- **Leave a low-level escape hatch.** I teach an "ask the chip what it is before
  you trust it" lesson, so expose the detected chip IDs and a raw register read.
- Include an **I2C scan helper** that prints every address found on the bus with a
  guess at what's there. This is its own lesson.
- Error messages get read by 14-year-olds. "No gyro found at 0x68 — check the SDA
  and SCL wires" beats a traceback.

Also expose a **field magnitude** helper for the magnetometer, `sqrt(x^2+y^2+z^2)`.
We use it for a lesson on detecting magnetic interference from the ROV's thrusters:
Earth's field is 25-65 uT everywhere on the planet, so a magnitude far outside that
range means the sensor is being lied to, without needing any ground truth.

## Known specifics for the ITG3205

Please verify these against the datasheet rather than trusting me:

- Single fixed full-scale range of +/-2000 deg/s
- Sensitivity 14.375 LSB per deg/s
- `FS_SEL` bits in the `DLPF_FS` register (0x16) **must** be set to `0b11` or the
  part does not work at all
- Data is 16-bit **big-endian signed**, starting at register 0x1D
- `WHO_AM_I` is at 0x00
- There is an on-die temperature sensor; expose it if it's cheap to do

## Verification script

I have **no hardware in hand yet** — I am ordering one board with fast shipping to
test before buying five more. So the risk here is not a crash, it's silently wrong
scaling: readings that look plausible and are 2x off. `test_sensors.py` should make
that detectable:

- **I2C scan** — print all addresses found, and which chip was detected at each
- **Accelerometer at rest** — magnitude should read ~9.8 m/s^2
- **Gyro at rest** — all three axes should read near zero; print the noise spread
- **Gyro integration** — prompt the user to rotate the board a quarter turn slowly,
  integrate the z axis, and print the result; it should land near 90 degrees
- **Magnetometer magnitude** — should read roughly 48 uT (I'm in Santa Cruz, CA).
  Anything near 480 or 4.8 means the scale factor is wrong.
- **Barometer** — print pressure continuously; lifting the board 1-2 m should move
  it by roughly 0.12 hPa per meter

Make the expected value and a pass/fail verdict part of the printed output, so I
can run it once and know whether the port is good.

## Ground rules

- Target current CircuitPython on the XIAO RP2350. Use `busio.I2C` and
  `adafruit_bus_device.i2c_device` in the normal Adafruit idiom.
- Do not reimplement ADXL345 or BMP280 — import the Adafruit libraries.
- **Flag anything you are guessing at.** If a register value or scale factor comes
  from inference rather than a datasheet or DFRobot's source, say so in a comment
  and list it in the README. I would rather hand-verify five uncertain constants
  than discover them in front of six students.
