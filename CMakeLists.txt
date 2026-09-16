cmake_minimum_required(VERSION 3.16)
project(RMVTraining LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_WEB_DEBUG "Build the scalar Web Debug dashboard" ON)
option(BUILD_TESTING "Build the starter-project tests" ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core)
find_package(Threads REQUIRED)

add_subdirectory(serialPort)
if(ENABLE_WEB_DEBUG)
    add_subdirectory(web_debug)
endif()

add_library(algorithm_support STATIC
    armor/src/BulletModel.cpp
    armor/src/KalmanFilter.cpp
)
target_include_directories(algorithm_support PUBLIC armor/include)
target_link_libraries(algorithm_support PUBLIC Eigen3::Eigen)

add_library(training_core STATIC
    src/AppSupport.cpp
    src/AutoAimTypes.cpp
    student/src/StudentTasks.cpp
)
target_include_directories(training_core
    PUBLIC
        include
        student/include
        serialPort/include
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(training_core PUBLIC serialPort ${OpenCV_LIBS})

add_executable(rmv_training main.cpp)
target_include_directories(rmv_training PRIVATE web_debug/include)
target_link_libraries(rmv_training
    PRIVATE
        training_core
        algorithm_support
        Threads::Threads
)
if(ENABLE_WEB_DEBUG)
    target_link_libraries(rmv_training PRIVATE web_debug)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_executable(test_scaffold tests/test_scaffold.cpp)
    target_link_libraries(test_scaffold PRIVATE training_core)
    add_test(NAME scaffold_safety COMMAND test_scaffold)
    add_test(NAME cli_help COMMAND rmv_training --help)
endif()
