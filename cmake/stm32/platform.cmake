# STM32F405 build, included from the top-level CMakeLists.txt when
# JURABABA_PLATFORM=stm32. Use its own build directory:
#   cmake -S . -B build-stm32 -G Ninja -DJURABABA_PLATFORM=stm32 && ninja -C build-stm32
cmake_minimum_required(VERSION 3.24)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/arm-none-eabi.cmake)
endif()

project(Jurababa C CXX ASM)

include(${CMAKE_CURRENT_LIST_DIR}/libopencm3.cmake)

set(STM32_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/firmware/hal/stm32/stm32f405.ld)

set(STM32_HAL_SOURCES
    firmware/hal/console.cpp
    firmware/hal/stm32/clock.cpp
    firmware/hal/stm32/console.cpp
    firmware/hal/stm32/flash.cpp
    firmware/hal/stm32/memory.cpp
    firmware/hal/stm32/quad_counter.cpp
    firmware/hal/stm32/spi.cpp
    firmware/hal/stm32/system.cpp
    firmware/hal/stm32/tick.cpp
    firmware/hal/stm32/time.cpp
    firmware/hal/stm32/uid.cpp
    firmware/hal/stm32/watchdog.cpp
)

add_executable(Jurababa_stm32
    ${STM32_HAL_SOURCES}
    firmware/common/board_identity.cpp
    firmware/common/breadcrumb.cpp
    firmware/common/flash_store.cpp
    firmware/common/reset_report.cpp
    firmware/control/run_log.cpp
    firmware/drivers/lsm6dsr.cpp
    firmware/main_stm32.cpp
)
set_target_properties(Jurababa_stm32 PROPERTIES SUFFIX .elf LINK_DEPENDS ${STM32_LINKER_SCRIPT})

target_include_directories(Jurababa_stm32 PRIVATE ${CMAKE_SOURCE_DIR}/firmware)
target_compile_definitions(Jurababa_stm32 PRIVATE STM32_BUILD)

set(STM32_CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
target_compile_options(Jurababa_stm32 PRIVATE
    ${STM32_CPU_FLAGS}
    -O2 -g
    -ffunction-sections -fdata-sections
    -Wall -Wextra -Wdouble-promotion
    # The FPU is single precision; an unsuffixed literal must not drag a
    # float expression into software double maths.
    -fsingle-precision-constant
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics>
)
target_link_options(Jurababa_stm32 PRIVATE
    ${STM32_CPU_FLAGS}
    -T${STM32_LINKER_SCRIPT}
    -nostartfiles # libopencm3's reset_handler is the startup code
    --specs=nano.specs --specs=nosys.specs
    -u _printf_float # newlib-nano drops %f from printf without it
    -Wl,--gc-sections
    -Wl,-Map=$<TARGET_FILE_DIR:Jurababa_stm32>/Jurababa_stm32.map
    -Wl,--print-memory-usage
    # .preinit_array is writable data next to .text; flash has no MMU to care.
    -Wl,--no-warn-rwx-segments
)
target_link_libraries(Jurababa_stm32 PRIVATE opencm3_stm32f4)

# Every build checks the memory layout, then emits a flashable image. Intel
# HEX, not a raw .bin: a .bin would fill the gap between the vectors and
# .text with zeros and wipe the flash store on every flash.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(TARGET Jurababa_stm32 POST_BUILD
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/check_stm32_layout.py
        $<TARGET_FILE:Jurababa_stm32> ${CMAKE_OBJDUMP}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:Jurababa_stm32>
        $<TARGET_FILE_DIR:Jurababa_stm32>/Jurababa_stm32.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:Jurababa_stm32>
    VERBATIM
)
