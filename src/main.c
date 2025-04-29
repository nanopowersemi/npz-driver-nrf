#include "np0.h"
#include "np0_hal.h"
#include "np0_device_control.h"
#include <zephyr/kernel.h>
#include <stdio.h>

//[CFGSTRUCT_START]
np0_peripheral_config_s peripheral_3 = {
    .power_mode = POWER_MODE_PERIODIC,
    .communication_protocol = COM_SPI,
    .polling_mode = POLLING_MODE_PERIODIC_READ_COMPARE_THRESHOLD,
    .power_switch_mode = POWER_SWITCH_MODE_STANDARD,
    .interrupt_pin_mode = INTERRUPT_PIN_MODE_INPUT_ACTIVE_HIGH,
    .comparison_mode = COMPARISON_MODE_INSIDE_THRESHOLD,
    .sensor_data_type = DATA_TYPE_INT16,
    .multi_byte_transfer_enable = 0,
    .swap_registers = 0,
    .spi_cfg.bytes_from_sram_num = 2,
    .spi_cfg.bytes_from_sram = {0x20, 0x10},
    .spi_cfg.bytes_from_sram_read_num = 1,
    .spi_cfg.bytes_from_sram_read = {0xA8},
    .spi_cfg.mode = SPIMOD_SPI_MODE_0,
    .polling_period = 10,
    .pre_wait_time = PRE_WAIT_TIME_EXTEND_256,
    .post_wait_time = POST_WAIT_TIME_EXTEND_256,
    .time_to_wait = 156,
    .threshold_over = 1000,
    .threshold_under = 64536,
};

np0_peripheral_config_s peripheral_4 = {
    .power_mode = POWER_MODE_PERIODIC,
    .communication_protocol = COM_I2C,
    .polling_mode = POLLING_MODE_PERIODIC_READ_COMPARE_THRESHOLD,
    .power_switch_mode = POWER_SWITCH_MODE_STANDARD,
    .interrupt_pin_mode = INTERRUPT_PIN_MODE_INPUT_ACTIVE_HIGH,
    .comparison_mode = COMPARISON_MODE_INSIDE_THRESHOLD,
    .sensor_data_type = DATA_TYPE_INT16,
    .multi_byte_transfer_enable = 1,
    .swap_registers = 1,
    .i2c_cfg.command_num = 0,
    .i2c_cfg.bytes_from_sram = {0},
    .i2c_cfg.sensor_address = 0x49,
    .i2c_cfg.reg_address_value = 0x00,
    .i2c_cfg.num_of_retries_on_nak = 0,
    .i2c_cfg.wake_on_nak = 0,
    .polling_period = 100,
    .pre_wait_time = PRE_WAIT_TIME_EXTEND_256,
    .post_wait_time = POST_WAIT_TIME_EXTEND_256,
    .time_to_wait = 93,
    .threshold_over = 3840,
    .threshold_under = 2560,
};

np0_adc_config_channels_s np0_adc_internal_config = {
    .wakeup_enable = 0,
    .over_threshold = 0,
    .under_threshold = 0,
};

np0_adc_config_channels_s np0_adc_external_config = {
    .wakeup_enable = 0,
    .over_threshold = 0,
    .under_threshold = 0,
};

np0_device_config_s np0_configuration = {
    .host_power_mode = HOST_POWER_MODE_SWITCH,
    .power_switch_normal_mode_per1 = 0,
    .power_switch_normal_mode_per2 = 0,
    .power_switch_normal_mode_per3 = 0,
    .power_switch_normal_mode_per4 = 0,
    .system_clock_divider = SCLK_DIV_DISABLE,
    .system_clock_source = SYS_CLOCK_10HZ,
    .io_strength = IO_STR_NORMAL,
    .i2c_pull_mode = I2C_PULL_DISABLE,
    .spi_auto = SPI_PINS_AUTO_DISABLE,
    .xo_clock_out_sel = XO_CLK_OFF,
    .wake_up_per1 = 0,
    .wake_up_per2 = 0,
    .wake_up_per3 = 1,
    .wake_up_per4 = 1,
    .wake_up_any_or_all = WAKEUP_ANY,
    .global_timeout = 65535,
    .interrupt_pin_pull_up_pin1 = INT_PIN_PULL_LOW,
    .interrupt_pin_pull_up_pin2 = INT_PIN_PULL_LOW,
    .interrupt_pin_pull_up_pin3 = INT_PIN_PULL_LOW,
    .interrupt_pin_pull_up_pin4 = INT_PIN_PULL_LOW,
    .adc_ext_sampling_enable = 0,
    .adc_clock_sel = ADC_CLK_SC,
    .adc_channels = {&np0_adc_internal_config, &np0_adc_external_config},
    .peripherals = {0, 0, &peripheral_3, &peripheral_4},
};
//[CFGSTRUCT_END]

static void read_peripheral_temp(int peripheral_value)
{
    // Calculate the temperature in degrees Celsius
    float temperature = peripheral_value * 0.0078125; //     // AS6212 temperature sensor resolution is 0.0078125°C.
    printk("Calculated temperature: %d.%03d °C\r\n", (int) temperature,
           (int) ((temperature - (int) temperature) * 1000));
}

static void np0_read_status_registers(np0_status_s *status)
{
    // Read the first status register
    if (np0_read_STA1(&status->status1) != OK)
    {
        return;
    }

    // Handle status1
    if (status->status1.reset_source == RESETSOURCE_NONE)
    {
        printk("Reset source is None\r\n");
    }
    else if (status->status1.reset_source == RESETSOURCE_PWR_RESET)
    {
        printk("Power-on reset triggered\r\n");
    }
    else if (status->status1.reset_source == RESETSOURCE_SOFT_RESET)
    {
        printk("Soft reset triggered (via I2C command)\r\n");
    }
    else if (status->status1.reset_source == RESETSOURCE_EXT_RESET)
    {
        printk("External reset triggered (via RST pin)\r\n");
    }

    if (status->status1.ext_adc_triggered == 1)
    {
        if (!np0_device_handle_adc_external())
        {
            return;
        }
    }

    if (status->status1.int_adc_triggered == 1)
    {
        if (!np0_device_handle_adc_internal())
        {
            return;
        }
    }

    if (status->status1.global_timeout_triggered == 1)
    {
        printk("Global Timeout triggered before any wake up source triggered\r\n");
    }

    // Read the second status register
    if (np0_read_STA2(&status->status2) != OK)
    {
        return;
    }

    // Handle status2
    // Arrays to map peripherals and switches
    np0_psw_e switches[4] = {PSW_LP1, PSW_LP2, PSW_LP3, PSW_LP4};
    uint8_t triggered[4] = {status->status2.per1_triggered, status->status2.per2_triggered,
                            status->status2.per3_triggered, status->status2.per4_triggered};
    uint8_t timeouts[4] = {status->status2.per1_global_timeout, status->status2.per2_global_timeout,
                           status->status2.per3_global_timeout, status->status2.per4_global_timeout};

    // Iterate over each peripheral to check for triggers and timeouts
    for (int i = 0; i < 4; i++)
    {
        if (triggered[i]) // Check if peripheral is triggered
        {
            int peripheral_value = 0;

            np0_device_read_peripheral_value(switches[i], i,
                                             &peripheral_value); // Read the peripheral value based on the switch

            if (np0_configuration.peripherals[i]->communication_protocol == COM_I2C &&
                (np0_configuration.peripherals[i]->polling_mode == POLLING_MODE_PERIODIC_READ_COMPARE_THRESHOLD ||
                 np0_configuration.peripherals[i]->polling_mode ==
                     POLLING_MODE_PERIODIC_WAIT_INTERRUPT_COMPARE_THRESHOLD))
            {
                read_peripheral_temp(peripheral_value);
            }
        }

        if (timeouts[i]) // Check if global timeout is triggered for this peripheral
        {
            printk("Peripheral %d global timeout was triggered\r\n", i + 1); // Log the timeout event
        }
    }
}

int main(void)
{
    // Print welcome message
    printk("nPZero Host is active.........\r\n");

    // Initialize the nP0 interface
    np0_hal_init();
  
    // Read the status registers of the nP0 device after every reset
    np0_status_s np0_status = {0};
    np0_read_status_registers(&np0_status);

    // Send the configuration to the device
    np0_device_configure(&np0_configuration);

    // Add a delay at the start of main, to give the user time to flash the MCU before it enters sleep
    // This delay should be removed in production code  
    k_msleep(1000);

    // At the end of your operations, put the device into sleep mode
    np0_device_go_to_sleep();
}
