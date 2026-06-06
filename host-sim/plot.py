"""plot.py -- plot the host-sim step-response CSV

Usage:
    ./motor_sim > response.csv
    python3 plot.py
"""

import csv, sys
import matplotlib.pyplot as plt

fname = "response.csv"
t, sp, w = [], [], []

with open(fname) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["t"]))
        sp.append(float(row["setpoint"]))
        w.append(float(row["omega"]))

fig, ax = plt.subplots(figsize=(9, 4))
ax.plot(t, sp, "k--", linewidth=1.2, label="Setpoint (rad/s)")
ax.plot(t, w,  "tab:orange", linewidth=1.8, label="Motor speed (rad/s)")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Angular velocity [rad/s]")
ax.set_title("DC-Motor Closed-Loop Step Response  (host reference sim)")
ax.legend()
ax.grid(True, alpha=0.3)
fig.tight_layout()
plt.savefig("step_response.png", dpi=150)
print("Saved step_response.png")
plt.show()
