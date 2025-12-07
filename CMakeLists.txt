# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================


cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(Temp_Press_DMA C CXX ASM)
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Temp_Press_DMA Temp_Press_DMA.c lib/aht20.c
        lib/bmp280.c
        lib/ssd1306.c)

pico_set_program_name(Temp_Press_DMA "Temp_Press_DMA")
pico_set_program_version(Temp_Press_DMA "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Temp_Press_DMA 1)
pico_enable_stdio_usb(Temp_Press_DMA 1)

# Add the standard library to the build
target_link_libraries(Temp_Press_DMA
        pico_stdlib
        pico_stdlib
        hardware_i2c
        hardware_dma)

# Add the standard include files to the build
target_include_directories(Temp_Press_DMA PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

# Add any user requested libraries
target_link_libraries(Temp_Press_DMA
        
        )

pico_add_extra_outputs(Temp_Press_DMA)

