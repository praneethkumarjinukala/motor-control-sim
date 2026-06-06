# Real-Time Motor Control System (simulation-only)

> Closed-loop DC-motor speed control for an ARM Cortex-M — built and demonstrated  
> entirely in simulation, no hardware required.

[![Live Demo](https://img.shields.io/badge/Live%20Demo-Wokwi-brightgreen)](https://wokwi.com/projects/466051495487088641)

## Project structure

| Folder | Tool | What it shows |
|---|---|---|
| `wokwi/` | **Wokwi** (browser) | Live interactive demo on STM32 Blue Pill (Cortex-M3). Turn the potentiometer, watch the speed track on the Serial Plotter. |
| `renode/` | **Renode** (desktop) | Bare-metal STM32F407 firmware: PID in SysTick ISR, UART CSV telemetry, **lwIP + Modbus TCP** (register 40001 = omega), GDB on :3333. |
| `host-sim/` | **C + matplotlib** | The "golden" reference: control law + RK4 motor model, step-response plot used to validate the firmware ports. |

---

## Quick start — live demo (Wokwi)

1. Click the live link: **<https://wokwi.com/projects/466051495487088641>**
2. Press ▶ Play — the STM32 boots and starts the control loop.
3. Open **Tools → Serial Plotter** (baud 115200).
4. Drag the potentiometer knob — target speed changes and the orange speed trace chases it.

The servo needle sweeps 0 → 180° as a live speedometer.

---

## Run the firmware (Renode)

See [`renode/README.md`](renode/README.md) for the full quick-start.

```bash
cd renode
make                  # build firmware.elf with arm-none-eabi-gcc
make renode           # launch Renode + GDB server on :3333
telnet localhost 3456 # read UART CSV: t,setpoint,omega
```

**Modbus TCP** — read live motor speed from any SCADA/HMI tool:

```bash
# After TAP interface setup (see renode/README.md)
python3 -c "
import socket, struct
s = socket.create_connection(('192.168.0.10', 502))
s.sendall(struct.pack('>HHHBBHH', 1,0,6,1,3,0,1))
print('omega:', struct.unpack('>H', s.recv(11)[9:11])[0] / 10.0, 'rad/s')
"
```

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
- **Ethernet / SCADA-HMI** — lwIP + Modbus TCP: motor speed visible as holding register 40001
