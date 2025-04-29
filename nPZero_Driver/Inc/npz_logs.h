/**
 * @file npz_logs.h
 * @brief This file contains functions for reading and logging configurations of the npz device, including global
 * settings, peripheral configurations, and ADC. The logging functions are designed to ensure that all configurations
 * are read correctly and presented in a standardized format for easier debugging and verification.
 */

#ifndef __NPZ_LOGS_H
#define __NPZ_LOGS_H

/** @cond */
#include "npz.h"
/** @endcond */

/**
 * @brief Reads and logs the npz device configuration settings.
 *
 * This function reads the global settings, peripheral configurations,
 * and ADC of the npz device, and logs the data in a structured format.
 * It helps in verifying the device configuration and debugging.
 *
 * @param [in] device_config Pointer to the device configuration structure.
 */
void npz_log_configurations(npz_device_config_s *device_config);
#endif /* __NPZ_LOGS_H */
