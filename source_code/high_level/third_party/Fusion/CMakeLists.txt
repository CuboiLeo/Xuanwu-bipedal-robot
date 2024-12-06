add_library(Fusion STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FusionAhrs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FusionCompass.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FusionOffset.c
)

target_include_directories(Fusion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(Fusion m)