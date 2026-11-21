/*
 * Linker script for the STM32F405xG (1 MB flash, 128 KB SRAM, 64 KB CCM).
 *
 * Flash sectors 0-3 are 16 KB each, sector 4 is 64 KB, 5-11 are 128 KB.
 *   sector 0     0x08000000  vector table only (the core boots from here)
 *   sectors 1-2  0x08004000  flash store, two 16 KB A/B slots; never part of
 *                            the image, so a reflash keeps saved settings
 *   sector 3+    0x0800C000  code, constants and .data's load image
 * Erasing a small sector stalls the bus for less time than a 128 KB one,
 * which is why the store lives in sectors 1-2 rather than at the end.
 *
 * SRAM (SRAM1 + SRAM2, contiguous): .data, .dma, .bss, .noinit, heap, stack.
 * CCM is CPU-only (no DMA) and holds only symbols marked HAL_CCM, in .ccm.
 *
 * libopencm3's reset_handler copies .data from _data_loadaddr to
 * _data.._edata, then zeroes up to _ebss, then runs .preinit_array,
 * .init_array and main(). hal/stm32/memory.cpp zeroes .ccm and paints the
 * stack from .preinit_array.
 *
 * tools/check_stm32_layout.py checks the linked result.
 */

MEMORY
{
    VECTORS (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
    STORE   (r)   : ORIGIN = 0x08004000, LENGTH = 32K
    FLASH   (rx)  : ORIGIN = 0x0800C000, LENGTH = 1024K - 48K
    SRAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCM     (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Stack reserved at the top of SRAM; the heap may not grow into it. */
_min_stack_size = 8K;

EXTERN(vector_table)
ENTRY(reset_handler)

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.vectors))
    } > VECTORS

    _flash_store_start = ORIGIN(STORE);
    _flash_store_end = ORIGIN(STORE) + LENGTH(STORE);

    .text :
    {
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    .preinit_array :
    {
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
    } > FLASH

    .init_array :
    {
        . = ALIGN(4);
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
    } > FLASH

    .fini_array :
    {
        . = ALIGN(4);
        __fini_array_start = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end = .;
    } > FLASH

    /* Unwind tables, still emitted by prebuilt libstdc++/libgcc objects. */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    . = ALIGN(4);
    _etext = .;

    .data :
    {
        _data = .;
        *(.data*)
        *(.ramtext*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT > FLASH
    _data_loadaddr = LOADADDR(.data);

    /* DMA buffers: SRAM only, since DMA cannot reach CCM. Zeroed with .bss. */
    .dma (NOLOAD) :
    {
        . = ALIGN(4);
        *(.dma*)
        . = ALIGN(4);
    } > SRAM

    .bss (NOLOAD) :
    {
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM

    /* Survives a warm reset: past _ebss, so startup never zeroes it. */
    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit*)
        . = ALIGN(8);
    } > SRAM

    _heap_start = .;
    _stack = ORIGIN(SRAM) + LENGTH(SRAM);
    _heap_end = _stack - _min_stack_size;
    ASSERT(_heap_start <= _heap_end, "SRAM full: no room left for the minimum stack")

    .ccm (NOLOAD) :
    {
        . = ALIGN(4);
        _ccm_start = .;
        *(.ccm*)
        . = ALIGN(4);
        _ccm_end = .;
    } > CCM
}
