cmake_minimum_required(VERSION 3.10)

project(xuanwu VERSION 1.2)

set(SOURCES
    xuanwu.cpp
    devices/src/CAN.cpp
    devices/src/STM32_protocol.cpp
    devices/src/IMU.cpp
    algorithms/src/user_math.cpp
    algorithms/src/kinematics.cpp
    algorithms/src/dynamics.cpp
    algorithms/src/robot.cpp
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
)