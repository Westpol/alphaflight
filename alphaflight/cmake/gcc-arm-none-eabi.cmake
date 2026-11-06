set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ------------------------------------------------
# Toolchain
# ------------------------------------------------

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)

set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE    ${TOOLCHAIN_PREFIX}size)

# Do not try to link test executables when checking the cross compiler.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ------------------------------------------------
# Target CPU: STM32H723 (Cortex-M7, hard-float)
# ------------------------------------------------

set(TARGET_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

# ------------------------------------------------
# Compiler flags
# ------------------------------------------------

# C
set(CMAKE_C_FLAGS_INIT
    "${TARGET_FLAGS} -Wall -ffunction-sections -fdata-sections"
)

set(CMAKE_C_FLAGS_DEBUG_INIT   "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE_INIT "-Os -g0")

# C++
set(CMAKE_CXX_FLAGS_INIT
    "${TARGET_FLAGS} -Wall -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics"
)

set(CMAKE_CXX_FLAGS_DEBUG_INIT   "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Os -g0")

# Assembly
set(CMAKE_ASM_FLAGS_INIT
    "${TARGET_FLAGS} -x assembler-with-cpp"
)

# ------------------------------------------------
# Linker
# ------------------------------------------------

# Use the C++ compiler driver for the final link when the executable
# contains C++ sources. CMake selects this automatically.
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${TARGET_FLAGS} --specs=nano.specs -T${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections -Wl,--print-memory-usage -u _printf_float"
)

# Math library. The C++ compiler driver supplies the C++ runtime
# automatically when linking a C++ executable.
set(TOOLCHAIN_LINK_LIBRARIES m)
