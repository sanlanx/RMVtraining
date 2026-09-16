cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(rmv_serial LANGUAGES CXX)
endif()

add_library(serialPort STATIC
    src/SerialPort.cpp
)

target_include_directories(serialPort
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(serialPort PUBLIC cxx_std_17)

add_library(rmv::serial ALIAS serialPort)
