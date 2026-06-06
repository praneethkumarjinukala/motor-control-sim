# Renode — Bare-Metal STM32F407 Firmware + lwIP Modbus TCP

This layer runs a real compiled ELF binary on a simulated **STM32F407** inside
[Renode](https://renode.io) — the same PID control law and RK4 motor model as
the host-sim and Wokwi layers, now as production-style embedded firmware.

## File map

| File | Purpose |
|------|---------|
| `main.c` | System init, SysTick ISR (1 kHz PID tick), idle loop (lwIP poll) |
| `pid.c / pid.h` | Portable PID module — derivative-on-measurement, anti-windup |
| `motor_model.c / .h` | RK4 DC-motor ODE plant (same parameters as host-sim) |
| `modbus_tcp.c` | lwIP raw-API Modbus TCP server; register 40001 = omega x10 |
| `motor_control.resc` | Renode script: load ELF, UART telnet, Ethernet TAP, GDB :3333 |
| `Makefile` | One-command build with `arm-none-eabi-gcc` |
| `STM32F407VGTx_FLASH.ld` | Linker script (1 MB Flash, SRAM1/2, CCM) |

## Prerequisites

```bash
# Toolchain
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

# Renode (Linux AppImage)
wget https://github.com/renode/renode/releases/latest/download/renode-*.linux-portable.tar.gz
tar xf renode-*.linux-portable.tar.gz
export PATH=$PATH:$PWD/renode_*

# lwIP source tree (2.1.x)
git clone https://github.com/lwip-tcpip/lwip.git /opt/lwip
```

## Quick start

```bash
# 1. Build firmware
cd renode
make

# 2. Launch Renode
make renode
# Renode monitor opens. Type:
(monitor) start

# 3. Read UART CSV stream
telnet localhost 3456
# Output: t,setpoint,omega  (one row per 10 ms)

# 4. Attach GDB (in another terminal)
make gdb
(gdb) break SysTick_Handler    # breakpoint in ISR
(gdb) watch g_omega            # watchpoint on motor speed
(gdb) continue
```

## Modbus TCP — read motor speed from any SCADA tool

The STM32 gets IP **192.168.0.10** via the Renode TAP adapter.

```bash
# Set up TAP interface (Linux, run once)
sudo ip tuntap add tap0 mode tap
sudo ip addr add 192.168.0.1/24 dev tap0
sudo ip link set tap0 up

# Read holding register 40001 (omega * 10) with Python
python3 - <<'EOF'
import socket, struct
s = socket.create_connection(('192.168.0.10', 502), timeout=5)
# Modbus TCP: TID=1, PID=0, len=6, unit=1, FC=03, addr=0, count=1
req = struct.pack('>HHHBBHH', 1, 0, 6, 1, 0x03, 0, 1)
s.sendall(req)
resp = s.recv(64)
val = struct.unpack('>H', resp[9:11])[0]
print(f"Motor speed: {val/10:.1f} rad/s")
s.close()
EOF
```

## GDB latency profiling

Measure exact ISR execution time using Renode's cycle-accurate timer:

```
(monitor) cpu ExecutionMode SingleStep
(monitor) cpu PC                      # check program counter
(gdb) break SysTick_Handler
(gdb) commands
  silent
  printf "tick %d: omega=%.1f\n", g_tick, g_omega
  continue
end
(gdb) continue
```

## Architecture notes

- **SysTick @ 1 kHz** drives the entire control loop in IRQ context
- **ADC1 CH0** (PA0) sampled each tick → 0-300 rad/s setpoint
- **TIM4 CH1** (PD12) PWM @ 10 kHz → motor driver duty cycle
- **USART2** (PA2/PA3) @ 115200 → CSV telemetry (every 10 ticks = 10 ms)
- **Ethernet MAC** → lwIP → Modbus TCP on port 502 → SCADA visibility
- **Software plant** (motor_model.c) runs inside the ISR: 100 x RK4 micro-steps
  per control tick = 1 ms of accurate motor physics every real millisecond
