cmake_minimum_required(VERSION 3.10)

project(xuanwu VERSION 1.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)

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
list(APPEND CMAKE_PREFIX_PATH "/usr/src/pytorch/torch/share/cmake/Torch")
set(Boost_USE_MULTITHREADED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

add_subdirectory(third_party/Fusion)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        pthread
        Fusion
        ${TORCH_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/algorithms/inc
        ${PROJECT_SOURCE_DIR}/devices/inc
        ${PROJECT_SOURCE_DIR}/third_party
        ${PROJECT_SOURCE_DIR}/data_plotter
)