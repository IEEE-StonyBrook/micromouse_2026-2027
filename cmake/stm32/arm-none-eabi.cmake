# Toolchain file for the STM32F405 build (Cortex-M4F, bare metal).
#
# Finds arm-none-eabi-gcc in, in order: ARM_TOOLCHAIN_PATH or
# PICO_TOOLCHAIN_PATH (a toolchain root with bin/), as a CMake or environment
# variable, then PATH. The toolchain must ship newlib-nano (the Arm GNU Toolchain does;
# Homebrew's bare arm-none-eabi-gcc does not).
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(_jurababa_hints)
foreach(_var ARM_TOOLCHAIN_PATH PICO_TOOLCHAIN_PATH)
    if(DEFINED ${_var})
        list(APPEND _jurababa_hints "${${_var}}/bin")
    endif()
    if(DEFINED ENV{${_var}})
        list(APPEND _jurababa_hints "$ENV{${_var}}/bin")
    endif()
endforeach()
find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc HINTS ${_jurababa_hints} NO_CACHE REQUIRED)
get_filename_component(_jurababa_bin "${ARM_NONE_EABI_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER "${_jurababa_bin}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${_jurababa_bin}/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "${_jurababa_bin}/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY "${_jurababa_bin}/arm-none-eabi-objcopy" CACHE FILEPATH "")
set(CMAKE_OBJDUMP "${_jurababa_bin}/arm-none-eabi-objdump" CACHE FILEPATH "")
set(CMAKE_SIZE "${_jurababa_bin}/arm-none-eabi-size" CACHE FILEPATH "")

# There is no OS to run a test executable on, so compiler checks build a library.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
