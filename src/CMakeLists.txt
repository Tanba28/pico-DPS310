add_library(DPS310 DPS310.cpp)

target_link_libraries(DPS310 pico_stdlib hardware_i2c)

target_include_directories(DPS310
    PUBLIC ${PROJECT_SOURCE_DIR}/include/
    )

