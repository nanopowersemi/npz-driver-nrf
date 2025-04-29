/**
 * @file np0_logs.h
 * @brief This file contains functions for reading and logging configurations of the NP0 device, including global
 * settings, peripheral configurations, and ADC. The logging functions are designed to ensure that all configurations
 * are read correctly and presented in a standardized format for easier debugging and verification.
 */

#ifndef NPO_LOGS_H
#define NPO_LOGS_H

/** @cond */
#include "np0.h"
/** @endcond */

/**
 * @brief Reads and logs the NP0 device configuration settings.
 *
 * This function reads the global settings, peripheral configurations,
 * and ADC of the NP0 device, and logs the data in a structured format.
 * It helps in verifying the device configuration and debugging.
 *
 * @param [in] device_config Pointer to the device configuration structure.
 */
void np0_log_configurations(np0_device_config_s *device_config);
#endif /* NPO_LOGS_H */
