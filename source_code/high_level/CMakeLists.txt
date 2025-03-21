cmake_minimum_required(VERSION 3.10)

project(xuanwu VERSION 1.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SOURCES
    xuanwu.cpp
    devices/src/CAN.cpp
    devices/src/STM32_protocol.cpp
    devices/src/IMU.cpp
    algorithms/src/user_math.cpp
    algorithms/src/kinematics.cpp
    algorithms/src/dynamics.cpp
    algorithms/src/walking_patterns.cpp
    algorithms/src/controls.cpp
    algorithms/src/estimations.cpp
    algorithms/src/robot.cpp
    data_plotter/data_exporter.cpp
)

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(third_party/Fusion)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        pthread
        Fusion
)

target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/algorithms/inc
        ${PROJECT_SOURCE_DIR}/devices/inc
        ${PROJECT_SOURCE_DIR}/third_party
        ${PROJECT_SOURCE_DIR}/data_plotter
)