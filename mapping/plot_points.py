import math
import os
import matplotlib.pyplot as plt

MAX_RANGE = 200

xs = []
ys = []

BASE = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(BASE, "serial_capture.txt")) as f:
    for line in f:
        if not line.startswith("Angle:"):
            continue
        parts = line.split("|")
        if len(parts) != 2:
            continue
        angle = float(parts[0].replace("Angle:", "").replace("degrees", ""))
        distance = float(parts[1].replace("Distance:", "").replace("cm", ""))
        if distance > MAX_RANGE:
            continue
        theta = math.radians(angle)
        xs.append(distance * math.cos(theta))
        ys.append(distance * math.sin(theta))

plt.scatter(xs, ys)
plt.scatter([0], [0], marker="^", color="red")
plt.axis("equal")
plt.grid(True)
plt.title("Occupancy map (real serial data)")
plt.xlabel("x (cm)")
plt.ylabel("y (cm)")
plt.show()