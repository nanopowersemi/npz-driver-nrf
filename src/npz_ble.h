/**
 * @file np0_ble.h
 * @brief Bluetooth Low Energy (BLE) handling for NP0 device.
 *
 * This header file contains functions for initializing BLE,
 * updating, and advertising temperature data.
 */

#ifndef __NPZ_BLE_H
#define __NPZ_BLE_H

/** @cond */
#include <stdbool.h>
#include <stdint.h>
/** @endcond */

/**
 * @brief Initialize the Bluetooth subsystem.
 *
 * This function initializes the Bluetooth subsystem and registers a callback
 * for when the Bluetooth stack is ready.
 *
 * @return 0 on success, a negative error code on failure.
 */
int npz_ble_init(void);

/**
 * @brief Update the BLE advertisement data with the latest read values from the peripheral.
 *
 * This function updates the valp data in the advertisement packet
 * and restarts the BLE advertising to broadcast the new data.
 *
 * @param [in] peripheral_value  The last read value from the peripheral.
 * @return 0 on success, a negative error code on failure.
 */
int npz_ble_update_valp_advertisement_data(int peripheral_value);

/**
 * @brief Update the BLE advertisement data with the latest adc internal values.
 *
 * This function updates the internal adc data in the advertisement packet
 * and restarts the BLE advertising to broadcast the new data.
 *
 * @param [in] int_adc_value  The internal value of the adc.
 * @return 0 on success, a negative error code on failure.
 */
int npz_ble_update_adc_int_advertisement_data(uint8_t int_adc_value);

/**
 * @brief Update the BLE advertisement data with the latest adc external values.
 *
 * This function updates the external adc data in the advertisement packet
 * and restarts the BLE advertising to broadcast the new data.
 *
 * @param [in] ext_adc_value  The external value of the adc.
 * @return 0 on success, a negative error code on failure.
 */
int npz_ble_update_adc_ext_advertisement_data(uint8_t ext_adc_value);
#endif /* __NPZ_BLE_H */