/* STM32F407VGTx_FLASH.ld - Linker script for STM32F407VGT6
 * Flash: 1 MB @ 0x08000000
 * SRAM1: 112 KB @ 0x20000000
 * SRAM2: 16  KB @ 0x2001C000
 * CCM:   64  KB @ 0x10000000  (CPU core-coupled, no DMA)
 */

ENTRY(Reset_Handler)

MEMORY {
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  SRAM1 (rwx) : ORIGIN = 0x20000000, LENGTH = 112K
  SRAM2 (rwx) : ORIGIN = 0x2001C000, LENGTH = 16K
  CCM   (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

_estack = ORIGIN(SRAM1) + LENGTH(SRAM1);
_Min_Heap_Size  = 0x800;
_Min_Stack_Size = 0x800;

SECTIONS {

  .isr_vector : {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text : {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .data : AT(_etext) {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > SRAM1

  .bss : {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > SRAM1

  /* lwIP pbuf pool in SRAM2 */
  .lwip_mem (NOLOAD) : {
    . = ALIGN(4);
    *(.lwip_mem)
  } > SRAM2

  ._user_heap_stack : {
    . = ALIGN(8);
    PROVIDE(end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > SRAM1

  /DISCARD/ : {
    libc.a (*)
    libm.a (*)
    libgcc.a (*)
  }
}
