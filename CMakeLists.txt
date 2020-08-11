cmake_minimum_required(VERSION 3.2)


set(BUILD_AVR FALSE CACHE STRING "Build version for AVR")

if(BUILD_AVR)
    add_subdirectory(build-avr)
else(BUILD_AVR)
    add_subdirectory(build-host)
endif(BUILD_AVR)
