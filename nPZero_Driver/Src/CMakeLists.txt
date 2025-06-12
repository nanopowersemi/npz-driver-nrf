target_include_directories(app PRIVATE .)

target_sources(app PRIVATE npz_hal.c)
target_sources(app PRIVATE npz.c)
target_sources(app PRIVATE npz_device_control.c)
target_sources(app PRIVATE npz_logs.c)