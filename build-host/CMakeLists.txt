cmake_minimum_required(VERSION 3.2)

project(ArduinoSerialProtocol LANGUAGES CXX)


if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

add_definitions("-Wall")
# TODO turn on -Werror by ignoring all False Positives
#add_definitions("-Werror")
add_definitions("-pedantic")
add_definitions("-pedantic-errors")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-fno-trapv")
add_definitions("-fno-exceptions")
add_definitions("-fno-rtti")
add_definitions("-std=c++11")

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")

set(LIB_HEADERS
        "${SRC_DIR}/arduino_serial_protocol.h")

set(LIB_SRC
        "${SRC_DIR}/arduino_serial_protocol.cpp")

add_library(arduino_serial_protocol STATIC
        ${LIB_SRC}
        ${LIB_HEADERS})

add_custom_command(TARGET arduino_serial_protocol
        POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_BINARY_DIR}/include"
        COMMAND "${CMAKE_COMMAND}" -E copy ${LIB_HEADERS} "${CMAKE_BINARY_DIR}/include")

add_subdirectory("../external/gtest" "${CMAKE_CURRENT_BINARY_DIR}/gtest")

enable_testing()

# Include the gtest library. gtest_SOURCE_DIR is available due to
# 'project(gtest)' above.
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

##############
# Unit Tests
##############
add_executable(arduino_serial_protocol_test
        ${SRC_DIR}/arduino_serial_protocol_test.cpp)

# Standard linking to gtest stuff.
target_link_libraries(arduino_serial_protocol_test
        arduino_serial_protocol
        gtest gtest_main)

# This is so you can do 'make test' to see all your tests run, instead of
# manually running the executable runUnitTests to see those specific tests.
add_test(NAME runTests COMMAND arduino_serial_protocol_test)
