#include "hal/stm32/memory.h"

#include <cerrno>
#include <cstddef>

// Symbols from hal/stm32/stm32f405.ld.
extern "C" char _heap_start, _heap_end, _stack, _ccm_start, _ccm_end;

namespace
{
constexpr uint32_t STACK_PAINT = 0xA5A5A5A5u;
// Left unpainted below the live stack pointer while painting runs.
constexpr uintptr_t PAINT_MARGIN_BYTES = 64;

char* heap_break = &_heap_start;
char* heap_peak  = &_heap_start;

uintptr_t stackPointer()
{
    uintptr_t sp;
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    return sp;
}

// Runs from .preinit_array, after .data/.bss are set up and before any constructor.
void startupInit()
{
    for (char* p = &_ccm_start; p < &_ccm_end; ++p)
    {
        *p = 0;
    }
    uint32_t* const end = reinterpret_cast<uint32_t*>(stackPointer() - PAINT_MARGIN_BYTES);
    for (uint32_t* p = reinterpret_cast<uint32_t*>(&_heap_end); p < end; ++p)
    {
        *p = STACK_PAINT;
    }
}

__attribute__((section(".preinit_array"), used)) void (*const startup_init_entry)() = startupInit;
} // namespace

// newlib's malloc grows the heap through _sbrk. Refuse anything past
// _heap_end so the heap can never run into the reserved stack.
extern "C" void* _sbrk(ptrdiff_t increment)
{
    if (increment > &_heap_end - heap_break || increment < &_heap_start - heap_break)
    {
        errno = ENOMEM;
        return reinterpret_cast<void*>(-1);
    }
    char* const previous = heap_break;
    heap_break += increment;
    if (heap_break > heap_peak)
    {
        heap_peak = heap_break;
    }
    return previous;
}

// The C++ ABI's handle for this image, which __cxa_atexit records with each
// static destructor. libstdc++'s stream error categories, reached from
// common/log.h's std::ostringstream, register theirs; the firmware never
// exits, so none run.
extern "C"
{
    void* __dso_handle = nullptr;
}

namespace hal::stm32
{
MemoryUsage memoryUsage()
{
    const uint32_t* p   = reinterpret_cast<const uint32_t*>(&_heap_end);
    const uint32_t* top = reinterpret_cast<const uint32_t*>(&_stack);
    while (p < top && *p == STACK_PAINT)
    {
        ++p;
    }

    MemoryUsage usage;
    usage.heap_peak_bytes   = static_cast<uint32_t>(heap_peak - &_heap_start);
    usage.heap_limit_bytes  = static_cast<uint32_t>(&_heap_end - &_heap_start);
    usage.stack_peak_bytes  = static_cast<uint32_t>(reinterpret_cast<const char*>(top) -
                                                    reinterpret_cast<const char*>(p));
    usage.stack_limit_bytes = static_cast<uint32_t>(&_stack - &_heap_end);
    return usage;
}
} // namespace hal::stm32
