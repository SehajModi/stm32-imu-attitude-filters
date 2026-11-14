"""Live plot + CSV log of roll_acc, roll_cf, pitch_acc, pitch_cf from the STM32.
Close PuTTY first (only one program can hold the COM port).
pip install pyserial matplotlib
"""
import collections
import csv
import time

import matplotlib.pyplot as plt
import serial

PORT = "COM9"
BAUD = 115200
N = 500  # samples shown (500 / 50 Hz = 10 s)

names = ["roll_acc", "roll_cf", "pitch_acc", "pitch_cf"]
buf = [collections.deque([0.0] * N, maxlen=N) for _ in names]

ser = serial.Serial(PORT, BAUD, timeout=0.05)
f = open("imu_log.csv", "w", newline="")
w = csv.writer(f)
w.writerow(["t"] + names)

plt.ion()
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
lines = []
for ax, (i, j), title in zip(axs, [(0, 1), (2, 3)], ["Roll", "Pitch"]):
    for k in (i, j):
        (l,) = ax.plot(list(buf[k]), label=names[k])
        lines.append(l)
    ax.set_ylabel("deg")
    ax.set_title(title)
    ax.legend(loc="upper left")

t0 = time.time()
last = 0.0
while plt.fignum_exists(fig.number):
    raw = ser.readline().decode(errors="ignore").strip()
    try:
        v = [int(x) / 100 for x in raw.split(",")]
        if len(v) != 4:
            raise ValueError
    except ValueError:
        v = None

    if v:
        w.writerow([round(time.time() - t0, 3)] + v)
        for b, x in zip(buf, v):
            b.append(x)

    if time.time() - last > 0.05:
        for l, b in zip(lines, buf):
            l.set_ydata(list(b))
        for ax in axs:
            ax.relim()
            ax.autoscale_view()
        plt.pause(0.001)
        last = time.time()

f.close()
ser.close()
