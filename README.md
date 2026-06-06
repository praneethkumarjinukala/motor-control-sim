# Real-Time Motor Control System (simulation-only)

> Closed-loop DC-motor speed control for an ARM Cortex-M — built and demonstrated  
> entirely in simulation, no hardware required.

[![Live Demo](https://img.shields.io/badge/Live%20Demo-Wokwi-brightgreen)](https://wokwi.com/projects/466051495487088641)

## Project structure

| Folder | Tool | What it shows |
|---|---|---|
| `wokwi/` | **Wokwi** (browser) | Live interactive demo on STM32 Blue Pill (Cortex-M3). Turn the potentiometer, watch the speed track on the Serial Plotter. |
| `renode/` | **Renode** (desktop) | Bare-metal STM32F407 firmware: PID in a SysTick ISR, GDB debugging (replaces JTAG), deterministic latency profiling. |
| `host-sim/` | **C + matplotlib** | The "golden" reference: control law + RK4 motor model, step-response plot used to validate the firmware ports. |

---

## Quick start — live demo (Wokwi)

1. Click the live link: **<https://wokwi.com/projects/466051495487088641>**
2. Press ▶ Play — the STM32 boots and starts the closed-loop control.
3. Open **Tools → Serial Plotter** (baud 115200).
4. Drag the potentiometer knob — target speed changes; the orange speed trace chases it.

The servo needle sweeps 0 → 180° as a live speedometer.

---

## Run the firmware (Renode)

See [`renode/README.md`](renode/README.md) — build with `arm-none-eabi-gcc`, load into Renode,
attach GDB on `:3333`, and capture the UART CSV to compare against the host curve.

---

## Reference model (host)

```bash
cd host-sim
gcc -O2 motor_sim.c -lm -o motor_sim
./motor_sim > response.csv
python3 plot.py
```

---

## Motor model (DC machine physics)

Coupled electrical + mechanical ODE, integrated with RK4:

```
di/dt = (V - R·i - Ke·ω) / L       # armature circuit
dω/dt = (Kt·i - Bv·ω) / J          # Newton's 2nd (rotational)
```

Parameters: R=1 Ω, L=0.5 mH, Kt=Ke=0.023 N·m/A, J=10 µkg·m², Bv=0.1 mN·m·s/rad  
PID: Kp=0.08, Ki=6.0, Kd=0.0008 — derivative on measurement, back-calculation anti-windup

---

## What this maps to on a résumé

- **Closed-loop control firmware** — PWM/ADC/DMA-style I/O, PID, anti-windup
- **Debugging & latency analysis** — GDB + Renode deterministic timing (replaces JTAG/ICD)
- **Ethernet / SCADA-HMI** — next milestone: lwIP + Modbus TCP in Renode
