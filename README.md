# STM32 IMU Attitude Filters: Complementary vs EKF

Roll and pitch estimation from an MPU6050 on an STM32F401RE (Nucleo), written in C with the HAL.
Both filters run in the same loop on the same sensor samples, so they can be compared fairly on accuracy, noise, and cost.

| Filter | Status |
|---|---|
| Complementary filter with `\|a\|`-adaptive gain | Done (`v0.1-complementary`) |
| Extended Kalman filter (roll, pitch, gyro bias) | In progress |
| Side-by-side results | After EKF |

<!-- Add a GIF or demo clip here: docs/demo.gif -->

## Why

A single IMU sensor is not enough. The gyroscope is smooth but drifts. The accelerometer has no drift but is noisy
and reads hand or vehicle acceleration as tilt. Fusing the two gives an angle that is both stable and responsive.
This project builds that fusion twice, with a classic complementary filter and an EKF, and measures the difference.

## Hardware

- STM32 Nucleo-F401RE (Cortex-M4F, 84 MHz)
- MPU6050 breakout (accelerometer + gyroscope)
- I2C1 at 100 kHz, UART2 at 115200 8N1 over the ST-LINK virtual COM port

Pin assignments are defined in `firmware/mpu_test.ioc`. A wiring diagram will be added under `docs/`.

## How the complementary filter works

Accelerometer angles (valid when the board is not accelerating):

```
roll_acc  = atan2(ay, az)
pitch_acc = atan2(-ax, sqrt(ay^2 + az^2))
```

Fusion, run at a fixed 200 Hz with a measured `dt` (DWT cycle counter):

```
roll  = a * (roll  + gx * dt) + (1 - a) * roll_acc
pitch = a * (pitch + gy * dt) + (1 - a) * pitch_acc
```

`a` is the gyro weight. It is 0.98 normally. When total acceleration `|a|` strays more than 0.03 g from 1 g,
the accelerometer angle is unreliable, so `a` rises to 0.998 and the filter leans on the gyro.
Without this gate, shaking the board leaked about 10 degrees of false tilt into the estimate.

Other details:
- Gyro bias is calibrated at startup from 400 samples (board must be still), and `c` over serial recalibrates.
- The filter is seeded from the accelerometer angle, so it does not converge from zero.
- MPU6050 is configured for a 42 Hz low-pass filter, 200 Hz sample rate, +/-250 dps, +/-2 g.

## Results

Not measured yet. Every cell below is `TBD`. The numbers will be computed from the logs in `data/` with an analysis script that is still to be written, and the EKF column fills in once the EKF is done.

| Metric | Accel only | Complementary | EKF |
|---|---|---|---|
| Rest noise, roll (std, deg) | TBD | TBD | TBD |
| Rest noise, pitch (std, deg) | TBD | TBD | TBD |
| Shake ripple, pitch (peak-to-peak, deg) | TBD | TBD | TBD |
| Static error at 15 / 30 / 45 deg (deg) | TBD | TBD | TBD |
| Filter cost per update (us) | n/a | TBD | TBD |
| Flash / RAM (bytes) | n/a | TBD | TBD |

Plots of the still, tilt and shake tests will be added to `docs/`.

## Run it

1. Open `firmware/` in STM32CubeIDE, build, and flash the Nucleo.
2. Close any serial terminal (only one program can hold the COM port).
3. Install the plotting tools and start the live plot:

```
py -m pip install pyserial matplotlib
py tools/imu_plot.py
```

Edit `PORT` at the top of `tools/imu_plot.py` if your board is not on COM9.
The plot also writes `imu_log.csv` with timestamps for later analysis.

## Serial output format

One CSV line per print, angles in centi-degrees:

```
roll_acc, roll_filtered, pitch_acc, pitch_filtered
```

## Repo layout

```
firmware/   STM32CubeIDE project (.ioc, Core/, Drivers/)
tools/      live plotter and analysis scripts
data/       recorded logs (still, tilt, shake)
docs/       plots, wiring diagram, demo (added as they are produced)
```

## Known limits

- No yaw. Gyro-only yaw drifts, and the MPU6050 has no magnetometer.
- Sustained acceleration (a steady turn or ramp) still corrupts the estimate, because gravity and acceleration cannot be separated with these two sensors alone.
- MPU6050 accelerometer offsets are not calibrated, so a resting angle can be off by a degree or two.
- The 0.03 g gating tolerance was tuned by hand on shake tests. It has not yet been checked against the sensor's resting `|a|`; if that reads outside 0.97 to 1.03 g, the filter stays in its gyro-heavy mode even when still.

## Roadmap

- [x] Complementary filter with adaptive gain
- [x] Live plotting and CSV logging
- [ ] Analysis script for the logs in `data/`
- [ ] Wiring diagram and demo clip in `docs/`
- [ ] EKF with gyro-bias state and `|a|`-scaled measurement noise
- [ ] Static error test against known angles
- [ ] Cost and memory comparison
- [ ] Fault-detection classifier on top of the estimate

## License

MIT for the code in this repository. Files under `firmware/Drivers/` are from STMicroelectronics and carry their own licenses.
