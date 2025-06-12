target_include_directories(app PRIVATE .)

target_sources(app PRIVATE npz_protected_registers.c)
target_sources(app PRIVATE npz_temp_sensor.c)