"""Live plot + CSV log of the STM32 attitude filter output.
Close PuTTY first (only one program can hold the COM port).
pip install pyserial matplotlib

Expected line (7 values): roll_acc, roll_cf, pitch_acc, pitch_cf, roll_ekf, pitch_ekf, accel_mg
Angles are centi-degrees; the last value is total acceleration in milli-g.
Lines that do not have 7 values (status messages like "read fail") are skipped.
"""
import collections
import csv
import time

import matplotlib.pyplot as plt
import serial

PORT = "COM9"
BAUD = 115200
N = 500  # samples shown (500 / 50 Hz = 10 s)

names = ["roll_acc", "roll_cf", "pitch_acc", "pitch_cf", "roll_ekf", "pitch_ekf", "accel_g"]
buf = [collections.deque([0.0] * N, maxlen=N) for _ in names]

ser = serial.Serial(PORT, BAUD, timeout=0.05)
log_name = time.strftime("imu_log_%Y%m%d_%H%M%S.csv")
f = open(log_name, "w", newline="")
w = csv.writer(f)
w.writerow(["t"] + names)
print("logging to", log_name)

plt.ion()
fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
groups = [("Roll", [0, 1, 4], "deg"), ("Pitch", [2, 3, 5], "deg"), ("|a|", [6], "g")]
lines = {}
for ax, (title, idxs, unit) in zip(axs, groups):
    for k in idxs:
        (l,) = ax.plot(list(buf[k]), label=names[k])
        lines[k] = l
    ax.set_title(title)
    ax.set_ylabel(unit)
    ax.legend(loc="upper left")

t0 = time.time()
last = 0.0
while plt.fignum_exists(fig.number):
    raw = ser.readline().decode(errors="ignore").strip()
    try:
        vals = [int(x) for x in raw.split(",")]
        if len(vals) != 7:
            raise ValueError
        v = [x / 100 for x in vals[:6]] + [vals[6] / 1000]
    except ValueError:
        v = None

    if v:
        w.writerow([round(time.time() - t0, 3)] + v)
        for b, x in zip(buf, v):
            b.append(x)

    if time.time() - last > 0.05:
        for k, l in lines.items():
            l.set_ydata(list(buf[k]))
        for ax in axs:
            ax.relim()
            ax.autoscale_view()
        plt.pause(0.001)
        last = time.time()

f.close()
ser.close()