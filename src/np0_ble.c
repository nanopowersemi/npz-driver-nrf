
/**
 * @file np0_ble.c
 * @brief Bluetooth Low Energy (BLE) handling for NP0 device.
 *
 * This file contains the implementation of BLE functions for initializing,
 * updating, and advertising temperature data.
 */
/*****************************************************************************
 * Includes
 *****************************************************************************/

#include "np0_ble.h"
#include <stddef.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_MIN_5000_MS 8000
#define ADV_MAX_5200_MS 8320
#define BLE_ADV_NCONN_NSCAN BT_LE_ADV_PARAM(0, ADV_MIN_5000_MS, ADV_MAX_5200_MS, NULL)

/*****************************************************************************
 * Data
 *****************************************************************************/

// Advertisement data array
static uint8_t m_eddystone_data[] = {
    0xaa, // Eddystone UUID (LSB)
    0xfe, // Eddystone UUID (MSB)
    0x20, // Eddystone-TLM frame type
    0x00, // TLM Version
    0x00, // adc_core (LSB)
    0x00, // adc_ext  (MSB)
    0x00, // VALP_L (LSB) of the read value from the peripheral
    0x80  // VALP_H (MSB) of the read value from the peripheral
};

/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static struct bt_data m_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
    BT_DATA(BT_DATA_SVC_DATA16, m_eddystone_data, sizeof(m_eddystone_data)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/*****************************************************************************
 * Private Methods
 *****************************************************************************/

static int start_advertising(void)
{
    int err = bt_le_adv_start(BLE_ADV_NCONN_NSCAN, m_ad, ARRAY_SIZE(m_ad), NULL, 0);
    if (err < 0)
    {
        printk("Advertising failed to start (err %d)\r\n", err);
    }

    return err;
}

/**
 * @brief This function is called when the Bluetooth subsystem is initialized.
 * It starts advertising and logs the beacon's status.
 *
 * @param err Error code from Bluetooth initialization
 */
static void bt_ready(int err)
{
    char addr_s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t addr = {0};
    size_t count = 1;

    if (err)
    {
        printk("Bluetooth init failed (err %d)\r\n", err);
        return;
    }

    printk("Bluetooth initialized\r\n");

    /* Start advertising */
    err = start_advertising();
    if (err)
    {
        return;
    }

    printk("Advertising successfully started\r\n");

    /* For connectable advertising you would use
     * bt_le_oob_get_local().  For non-connectable non-identity
     * advertising an non-resolvable private address is used;
     * there is no API to retrieve that.
     */
    bt_id_get(&addr, &count);
    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

    printk("Beacon started, advertising as %s\r\n", addr_s);
}

/*****************************************************************************
 * Public Methods
 *****************************************************************************/

int np0_ble_init(void)
{
    //  Initialize the Bluetooth Subsystem
    int err = bt_enable(bt_ready);
    if (err < 0)
    {
        printk("Bluetooth enable failed (err %d)\r\n", err);
    }

    return err;
}

int np0_ble_update_valp_advertisement_data(int peripheral_value)
{
    uint8_t valp_low = peripheral_value & 0xFF;         // Lower byte
    uint8_t valp_high = (peripheral_value >> 8) & 0xFF; // Upper byte

    // Update the valp bytes in the modifiable advertisement data array
    m_eddystone_data[6] = valp_low;
    m_eddystone_data[7] = valp_high;

    // Restart advertising with updated data
    bt_le_adv_stop();
    int err = start_advertising();
    if (err < 0)
    {
        printk("Failed to update VALP advertisement data (err %d)\r\n", err);
    }

    return err;
}

int np0_ble_update_adc_int_advertisement_data(uint8_t int_adc_value)
{
    // Update the adc_core bytes in the modifiable advertisement data array
    m_eddystone_data[4] = int_adc_value;

    // Restart advertising with updated data
    bt_le_adv_stop();
    int err = start_advertising();
    if (err < 0)
    {
        printk("Failed to update ADC core advertisement data (err %d)\r\n", err);
    }

    return err;
}

int np0_ble_update_adc_ext_advertisement_data(uint8_t ext_adc_value)
{
    // Update the adc_ext bytes in the modifiable advertisement data array
    m_eddystone_data[5] = ext_adc_value;

    // Restart advertising with updated data
    bt_le_adv_stop();
    int err = start_advertising();
    if (err < 0)
    {
        printk("Failed to update ADC ext advertisement data (err %d)\r\n", err);
    }

    return err;
}