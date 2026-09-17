cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(rmv_web_debug LANGUAGES CXX)
endif()

find_package(Threads REQUIRED)

# 链接此目标即可获得实现、头文件以及 WEB_DEBUG 宏定义。
add_library(web_debug STATIC
    src/WebLogger.cpp
)

target_compile_features(web_debug PUBLIC cxx_std_17)
target_include_directories(web_debug
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)
target_compile_definitions(web_debug
    PUBLIC WEB_DEBUG
    PRIVATE WEB_DEBUG_STATIC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/static"
)
target_link_libraries(web_debug PUBLIC Threads::Threads)
