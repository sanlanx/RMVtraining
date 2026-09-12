cmake_minimum_required(VERSION 3.0.0)
project(serialPort VERSION 0.1.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

aux_source_directory(./src ALL_SRCS)
add_library(serialPort STATIC ${ALL_SRCS})

