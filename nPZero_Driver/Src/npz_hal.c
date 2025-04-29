/**
 * @file np0_hal.c
 *
 * @brief Source file for nPZero Hardware Abstraction Layer (HAL)
 *
 * This file contains the hardware-specific functions and configurations for
 * the I2C communication interface. Users are required to customize these
 * functions according to their target hardware and MCU setup.
 *
 * The functions in this file serve as an abstraction layer between the nPZero
 * driver and the hardware-specific I2C implementation on the target system.
 * Users should modify these functions to match the I2C peripheral and
 * communication protocol used in their system.
 *
 * @note This file provides placeholder functions and configurations that need
 * to be adapted to the specific hardware and MCU setup. Users must replace
 * these placeholders with the actual implementations relevant to their system.
 *
 * @warning Incorrect configuration or improper implementation of these functions
 * may result in I2C communication failures or unexpected behavior.
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/

#include "../Inc/npz_hal.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define I2C_DELAY_MS 5
#define MY_TWIM DT_NODELABEL(arduino_i2c)

/*****************************************************************************
 * Data
 *****************************************************************************/

static const struct device *m_nrfx_twis_dev;

/*****************************************************************************
 * Private Methods
 *****************************************************************************/

/*****************************************************************************
 * Public Methods
 *****************************************************************************/

/**
 * @brief Function to read registers over I2C.
 */
npz_status_e npz_hal_read(uint8_t slave_address, uint8_t slave_register, uint8_t *pData, uint16_t size,
                          uint32_t timeout)
{
    uint8_t transmitData[] = {slave_register};
    if (i2c_write_read(m_nrfx_twis_dev, slave_address, transmitData, sizeof(transmitData), pData, size) != 0)
    {
        return ERR;
    }

    return OK;
}

/**
 * @brief Function to write to registers over I2C.
 */
npz_status_e npz_hal_write(uint8_t slave_address, uint8_t *pData, uint16_t size, uint32_t timeout)
{
    if (i2c_write(m_nrfx_twis_dev, pData, size, slave_address) != 0)
    {
        return ERR;
    }

    return OK;
}

/**
 * @brief Function to initialize I2C instance that will communicate with nPZero.
 */
npz_status_e npz_hal_init()
{
    int error_code = 0;
    m_nrfx_twis_dev = DEVICE_DT_GET(MY_TWIM);
    if (m_nrfx_twis_dev == NULL)
    {
        return ERR;
    }

    printk("I2C device: %s\r\n", DT_PROP(DT_NODELABEL(twis_device), label));

    error_code = i2c_configure(m_nrfx_twis_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
    if (error_code != 0)
    {
        return ERR;
    }

    pm_device_action_run(m_nrfx_twis_dev, PM_DEVICE_ACTION_SUSPEND);

    return OK;
}
