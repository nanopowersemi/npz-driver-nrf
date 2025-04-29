/**
 * @file np0_device_control.h
 * @brief Header file for functionsto read and handle status registers (STA1 and STA2), put nP0 device in sleep
 * mode, performe a software reset, and set a wake-up timeout.
 *
 * The registers in STA1 and STA2 can be read to determine what was the cause of wake-up.
 */

#ifndef NPO_DEVICE_CONTROL_H
#define NPO_DEVICE_CONTROL_H

/** @cond */
#include "np0.h"
#include <stdbool.h>
#include <stdint.h>
/** @endcond */

/**
 * @brief Reads the value from a specified peripheral.
 *
 * @param [in]  psw_lp          The low power switch indicates which peripheral that will be written.
 * @param [in]  index           Index of the peripheral.
 * @param [out] peripheral_value Pointer to store the value read from the peripheral.
 *
 * @return True if the peripheral value was successfully read, otherwise false.
 */
bool np0_device_read_peripheral_value(np0_psw_e psw_lp, int index, int *peripheral_value);

/**
 * @brief Handles the internal ADC and retrieves the relevant value.
 *
 * @return True if the internal ADC was successfully handled, otherwise false.
 */
bool  np0_device_handle_adc_internal(void);

/**
 * @brief Handles the external ADC and retrieves the relevant value.
 *
 * @return True if the external ADC was successfully handled, otherwise false.
 */
bool np0_device_handle_adc_external(void);

/**
 * @brief Put the device into sleep mode.
 *
 * @param [in] device Pointer to the device structure.
 */
void np0_device_go_to_sleep(void);

/**
 * @brief Reset the device by software.
 *
 * @param [in] device Pointer to the device structure.
 */
void np0_device_soft_reset(void);

/**
 * @brief Setup np0 device configuration.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */
void np0_device_configure(np0_device_config_s *device_config);

#endif /* NPO_DEVICE_CONTROL_H */
