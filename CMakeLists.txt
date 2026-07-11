cmake_minimum_required(VERSION 3.22)
# Setup C
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set project name
set(CMAKE_PROJECT_NAME BRUSHED_DC_MOTOR)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add easy framework
add_subdirectory(cmake/easy_frame)

set(SYSCONFIG_SRC
    ./sysconfig/ti_msp_dl_config.c
)
set(SYSCONFIG_INC
    ./sysconfig
)

# Config
set(Config_Src
./User/Config/Src/mcu_device.c
)

# BSP
set(BSP_Src 
    ./User/BSP/Src/bsp_time.c
    ./User/BSP/Src/bsp_mspm0g_tim_base.c
    ./User/BSP/Src/bsp_can.c
    ./User/BSP/Src/bsp_mspm0g_it.c
    ./User/BSP/Src/bsp_mspm0g_usart.c
    ./User/BSP/Src/bsp_mspm0g_dma.c
    ./User/BSP/Src/bsp_mspm0g_gpio.c
    ./User/BSP/Src/bsp_mspm0g_i2c.c
)

# Device
set(Device_Src 
    ./User/Device/Src/qei_encoder.c
    ./User/Device/Src/brushed_motor.c
    ./User/Device/Src/at24cxx.c
    ./User/Device/Src/comm_led.c

)

# Algorithm
set(Algorithm_Src
    ./User/Algorithm/Src/crc_ref.c
    ./User/Algorithm/Src/pid.c
)

#Application 
set(Application_Src 
    ./User/Application/Src/motor_task.c
    ./User/Application/Src/comm_task.c
    ./User/Application/Src/detect_task.c
    ./User/Application/Src/motor_runner.c
)

# Protocol
set(Protocol_Src 
    ./User/Protocol/Src/master_slave_codec.c
    ./User/Protocol/Src/motor_can_control.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${SYSCONFIG_SRC}
    ${Config_Src}
    ${BSP_Src}
    ${Device_Src}
    ${Protocol_Src}
    ${Application_Src}
    ${Algorithm_Src}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${SYSCONFIG_INC}
    ./User/Config/Inc
    ./User/BSP/Inc
    ./User/Device/Inc
    ./User/Algorithm/Inc
    ./User/Application/Inc
    ./User/Protocol/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    mspm0g_easy_frame

    # Add user defined libraries
)

#Add Segger RTT
set(RTT_Src
    ./Middlewares/Segger/RTT/SEGGER_RTT.c
    ./Middlewares/Segger/RTT/SEGGER_RTT_ASM_ARMv7M.S
    ./Middlewares/Segger/RTT/SEGGER_RTT_printf.c
    ./Middlewares/Segger/RTT/SEGGER_RTT_Syscalls_GCC.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${RTT_Src}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ./Middlewares/Segger/RTT
)

