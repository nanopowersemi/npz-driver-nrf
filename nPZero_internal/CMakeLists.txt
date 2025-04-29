target_include_directories(app PRIVATE .)

target_sources(app PRIVATE np0_protected_registers.c)
target_sources(app PRIVATE np0_temp_sensor.c)