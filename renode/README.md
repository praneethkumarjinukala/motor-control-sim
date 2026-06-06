# Renode — Bare-Metal STM32F407 Firmware

> Status: **coming soon** — firmware scaffolding in progress.

This layer runs the same PID control law on a real STM32F407 ELF binary inside
[Renode](https://renode.io), a deterministic embedded-systems emulator.

## What it demonstrates

- **PID in a SysTick ISR** — 1 kHz control tick, same gains as the host reference
- **ADC + PWM peripheral emulation** — Renode models the STM32 peripherals without hardware
- **GDB over JTAG (simulated)** — attach on `:3333`, set breakpoints, inspect registers
- **Deterministic latency profiling** — Renode's cycle-accurate timer measures ISR jitter

## Planned quick-start

```bash
# 1. Build firmware
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
  -o firmware.elf main.c pid.c motor_model.c startup_stm32f407.s \
  -T STM32F407VGTx_FLASH.ld -lm

# 2. Launch Renode
renode motor_control.resc

# 3. Attach GDB
arm-none-eabi-gdb firmware.elf
(gdb) target remote :3333
(gdb) monitor start
(gdb) continue

# 4. Capture UART output -> compare with host-sim/response.csv
```

## Files (planned)

| File | Purpose |
|------|---------|
| `main.c` | SysTick ISR, ADC read, PWM output |
| `pid.c / pid.h` | Portable PID module (shared with host-sim) |
| `motor_model.c` | Optional software plant for HIL testing |
| `motor_control.resc` | Renode script: load ELF, configure peripherals, start GDB server |
| `STM32F407VGTx_FLASH.ld` | Linker script |
