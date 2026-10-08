cmake_minimum_required(VERSION 3.22)

project(rtt_log)
add_library(rtt_log INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(rtt_log INTERFACE 
    
)

target_include_directories(rtt_log INTERFACE
    .
)

target_sources(rtt_log INTERFACE
    ./SEGGER_RTT_printf.c
    ./SEGGER_RTT.c
)

target_link_directories(rtt_log INTERFACE
)

target_link_libraries(rtt_log INTERFACE
)

# Validate that fwlibs code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()

