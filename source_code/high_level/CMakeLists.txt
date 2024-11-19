cmake_minimum_required(VERSION 3.10)

project(xuanwu VERSION 1.2)

set(SOURCES
    xuanwu.cpp
    src/CAN.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/inc
)