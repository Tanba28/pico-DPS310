add_library(pico-DPS310 DPS310.cpp)

target_link_libraries(pico-DPS310 pico-I2C pico_stdlib hardware_i2c)

target_include_directories(pico-DPS310
    PUBLIC ${PROJECT_SOURCE_DIR}/include/
    )

