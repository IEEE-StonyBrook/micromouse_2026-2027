# libopencm3, pinned to one commit and checked by hash, built for STM32F4 only.
# Defines the imported library `opencm3_stm32f4`.
#
# To move the pin: change LIBOPENCM3_COMMIT, then update LIBOPENCM3_SHA256
# with the hash of
# https://github.com/libopencm3/libopencm3/archive/<commit>.tar.gz.
include(ExternalProject)

set(LIBOPENCM3_COMMIT a3977e14519e4cff8d1feb96445655465797908b) # master, 2026-11-06
set(LIBOPENCM3_SHA256 7d9c78b85f0896cbf72b03dab49c027e7a0d6770813dae9cb86fcd9dca9560ec)

set(LIBOPENCM3_DIR ${CMAKE_BINARY_DIR}/libopencm3)
set(LIBOPENCM3_LIB ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f4.a)

# libopencm3's make calls $(PREFIX)gcc, so point it at the compiler CMake found.
string(REGEX REPLACE "gcc$" "" LIBOPENCM3_PREFIX "${CMAKE_C_COMPILER}")

find_program(MAKE_EXECUTABLE NAMES gmake make REQUIRED)

# The build runs scripts/irq2nvic_h, so python3 must be on PATH.
ExternalProject_Add(libopencm3_build
    URL https://github.com/libopencm3/libopencm3/archive/${LIBOPENCM3_COMMIT}.tar.gz
    URL_HASH SHA256=${LIBOPENCM3_SHA256}
    SOURCE_DIR ${LIBOPENCM3_DIR}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    BUILD_IN_SOURCE TRUE
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${MAKE_EXECUTABLE} -j4 TARGETS=stm32/f4 PREFIX=${LIBOPENCM3_PREFIX}
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${LIBOPENCM3_LIB}
)

# The include directory only appears after the download; CMake wants it to exist now.
file(MAKE_DIRECTORY ${LIBOPENCM3_DIR}/include)

add_library(opencm3_stm32f4 STATIC IMPORTED GLOBAL)
set_target_properties(opencm3_stm32f4 PROPERTIES
    IMPORTED_LOCATION ${LIBOPENCM3_LIB}
    INTERFACE_INCLUDE_DIRECTORIES ${LIBOPENCM3_DIR}/include
    INTERFACE_COMPILE_DEFINITIONS STM32F4
)
add_dependencies(opencm3_stm32f4 libopencm3_build)
