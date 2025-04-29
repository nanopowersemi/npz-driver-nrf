target_include_directories(app PRIVATE .)

target_sources(app PRIVATE np0_hal.c)
target_sources(app PRIVATE np0.c)
target_sources(app PRIVATE np0_device_control.c)
target_sources(app PRIVATE np0_logs.c)