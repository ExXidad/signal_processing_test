cmake_minimum_required(VERSION 3.0)
project(signal_processing_test)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party)
add_subdirectory(src)
if (PC)
    add_compile_definitions(PC)
elseif (MCU)
    add_compile_definitions(MCU)
endif ()
enable_testing()
add_subdirectory(tests)