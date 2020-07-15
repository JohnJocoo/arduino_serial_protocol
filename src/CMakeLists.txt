cmake_minimum_required(VERSION 3.2)

find_library(C_LIB c)
message(STATUS "avr-libc: ${C_LIB}")

set(LIB_HEADERS
      "arduino_serial_protocol.h")

set(LIB_SRC
      "arduino_serial_protocol.cpp")

add_avr_library(arduino_serial_protocol
                ${LIB_SRC}
                ${LIB_HEADERS})
                
avr_target_link_libraries(arduino_serial_protocol
                          "${C_LIB}")

add_custom_command(TARGET arduino_serial_protocol
                   POST_BUILD
                   COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_BINARY_DIR}/include"
                   COMMAND "${CMAKE_COMMAND}" -E copy ${LIB_HEADERS} "${CMAKE_BINARY_DIR}/include")
