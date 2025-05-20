cmake_minimum_required(VERSION 3.10)
project(ReaderVM64E VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/inc)
set(serial_SRCS
    src/serial/serial.cpp
    inc/serial/serial.h
    inc/serial/v8stdint.h
    src/serial/impl/list_ports/list_ports_win.cc
    src/serial/impl/list_ports/list_ports_linux.cc
)

list(APPEND serial_SRCS src/serial/impl/win.cc)
set( Reader_SRCS
    src/reader/ReaderMethod.cpp
    inc/reader/ReaderMethod.h
)

add_library(serial ${serial_SRCS})
add_library(reader ${Reader_SRCS})


add_executable(ReaderVM64E main.cpp)
target_link_libraries(ReaderVM64E serial)