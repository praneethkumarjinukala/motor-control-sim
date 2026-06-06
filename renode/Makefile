# Makefile - STM32F407 motor controller firmware
# Requires: arm-none-eabi-gcc toolchain + CMSIS headers + lwIP source tree
#
# Usage:
#   make          - build firmware.elf
#   make clean    - remove build artefacts
#   make renode   - launch Renode with motor_control.resc
#   make gdb      - attach GDB to running Renode session

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
GDB     = arm-none-eabi-gdb

# --- paths (adjust to your system) ---
CMSIS_INC  = /opt/CMSIS/Include
CMSIS_DEV  = /opt/CMSIS/Device/ST/STM32F4xx/Include
LWIP_SRC   = /opt/lwip/src
LWIP_PORT  = lwip_port

# --- source files ---
SRCS = main.c pid.c motor_model.c modbus_tcp.c
SRCS += startup_stm32f407xx.s

# lwIP core (minimal subset)
LWIP_SRCS  = $(LWIP_SRC)/core/init.c
LWIP_SRCS += $(LWIP_SRC)/core/tcp.c
LWIP_SRCS += $(LWIP_SRC)/core/tcp_in.c
LWIP_SRCS += $(LWIP_SRC)/core/tcp_out.c
LWIP_SRCS += $(LWIP_SRC)/core/pbuf.c
LWIP_SRCS += $(LWIP_SRC)/core/netif.c
LWIP_SRCS += $(LWIP_SRC)/core/timers.c
LWIP_SRCS += $(LWIP_SRC)/core/ipv4/ip4.c
LWIP_SRCS += $(LWIP_SRC)/core/ipv4/ip4_addr.c
LWIP_SRCS += $(LWIP_SRC)/core/ipv4/etharp.c
LWIP_SRCS += $(LWIP_SRC)/netif/ethernet.c
LWIP_SRCS += $(LWIP_PORT)/ethernetif.c   # STM32F4 MAC driver

# --- flags ---
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
OPT_FLAGS = -O2 -ffunction-sections -fdata-sections
DBG_FLAGS = -g3 -gdwarf-2

CFLAGS  = $(CPU_FLAGS) $(OPT_FLAGS) $(DBG_FLAGS)
CFLAGS += -DSTM32F407xx -DUSE_HAL_DRIVER
CFLAGS += -I. -I$(CMSIS_INC) -I$(CMSIS_DEV)
CFLAGS += -I$(LWIP_SRC)/include -I$(LWIP_PORT)
CFLAGS += -Wall -Wextra

LDFLAGS = $(CPU_FLAGS) -T STM32F407VGTx_FLASH.ld
LDFLAGS += -Wl,--gc-sections -lm -lc -lnosys
LDFLAGS += -specs=nano.specs -specs=nosys.specs

# --- build rules ---
TARGET = firmware

all: $(TARGET).elf $(TARGET).bin

$(TARGET).elf: $(SRCS) $(LWIP_SRCS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@arm-none-eabi-size $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(TARGET).elf $(TARGET).bin *.o

renode:
	renode motor_control.resc

gdb:
	$(GDB) $(TARGET).elf -ex "target remote :3333" -ex "monitor start"

.PHONY: all clean renode gdb
