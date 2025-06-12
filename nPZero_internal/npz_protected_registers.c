/**
 * @brief Header file for nPZero protected registers.
 *
 * This header file contains declarations and definitions for controlling and managing the nPZero protected registers.
 * @see npz_protected_registers.h for the corresponding header file providing function prototypes and definitions.
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/

#include "npz_protected_registers.h"
#include "npz_registers.h"
#include "npz_hal.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define NPZ_CHIP_A 0x56
#define NPZ_CHIP_B 0x57
#define NPZ_CHIP_C 0x58
#define NPZ_CHIP_D 0x59

#define REG_TUNLOCK 0x64      /**< Test Unlock Register*/
#define REG_TMUX 0x65         /**< Test MUX Register*/
#define REG_RCP_TRIM 0x66     /**< Recursive Charge Pump and ADC Trim Register*/
#define REG_VDDD_TRIM 0x67    /**< VDDD Reference Trim Register*/
#define REG_VREF_TRIM 0x68    /**< Auxiliary Reference Trim Register*/
#define REG_VREF_TC_TRIM 0x69 /**< Voltage Reference Temperature Coefficient Trim Register*/
#define REG_IREF_TRIM1 0x6A   /**< Current Reference Trim 1 Register*/
#define REG_IREF_TRIM2 0x6B   /**< Current Reference Trim 2 Register*/
#define REG_IREF_SEL 0x6C     /**< Reference Selection and Routing Register*/
#define REG_FOSC_TRIM 0x6D    /**< Fast Oscillator Trim Register*/
#define REG_SOSC_TRIM 0x6E    /**< Slow Oscillator Trim Register*/
#define REG_PSW_TRIM 0x6F     /**< Power Switch Trim Register*/
#define REG_PSW_VON 0x70      /**< Power Switch Voltage On Register*/
#define REG_XO1 0x71          /**< Crystal Oscillator Test 1 Register*/
#define REG_XO2 0x72          /**< Crystal Oscillator Test 2 Register*/
#define REG_TEST 0x7F         /**< Test Mode Register*/

/*****************************************************************************
 * Private Methods
 *****************************************************************************/

/*****************************************************************************
 * Public Methods
 *****************************************************************************/

npz_status_e npz_write_TUNLOCK(const npz_register_tunlock_s tunlock)
{
    uint8_t transmitData[2] = {REG_TUNLOCK, tunlock.tunlock};

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_TUNLOCK(npz_register_tunlock_s *tunlock)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_TUNLOCK, (uint8_t *) tunlock, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_TMUX(const npz_register_tmux_s tmux)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_TMUX;
    transmitData[1] |= tmux.t_io_addr;
    transmitData[1] |= tmux.t_blk_addr << 3;
    transmitData[1] |= tmux.t_en_buf << 7;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_TMUX(npz_register_tmux_s *tmux)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_TMUX, (uint8_t *) tmux, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_RCP_TRIM(const npz_register_rcp_trim_s rcp_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_RCP_TRIM;
    transmitData[1] |= rcp_trim.adc_vtrim;
    transmitData[1] |= rcp_trim.fb_iosc_trim << 2;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_RCP_TRIM(npz_register_rcp_trim_s *rcp_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_RCP_TRIM, (uint8_t *) rcp_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_VDDD_TRIM(const npz_register_vddd_trim_s vddd_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_VDDD_TRIM;
    transmitData[1] |= vddd_trim.vddd_trim;
    transmitData[1] |= vddd_trim.vddd_tc_trim << 3;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_VDDD_TRIM(npz_register_vddd_trim_s *vddd_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_VDDD_TRIM, (uint8_t *) vddd_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_VREF_TRIM(const npz_register_vref_trim_s vref_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_VREF_TRIM;
    transmitData[1] |= vref_trim.vref_0v25_trim;
    transmitData[1] |= vref_trim.vref_0v5_trim << 3;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_VREF_TRIM(npz_register_vref_trim_s *vref_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_VREF_TRIM, (uint8_t *) vref_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_VREF_TC_TRIM(const npz_register_vref_tc_trim_s vref_tc_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_VREF_TC_TRIM;
    transmitData[1] |= vref_tc_trim.vref_tc_trim;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_VREF_TC_TRIM(npz_register_vref_tc_trim_s *vref_tc_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_VREF_TC_TRIM, (uint8_t *) vref_tc_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_IREF_TRIM1(const npz_register_iref_trim1_s iref_trim1)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_IREF_TRIM1;
    transmitData[1] |= iref_trim1.iref_trim;
    transmitData[1] |= iref_trim1.iref_vsp_trim << 3;
    transmitData[1] |= iref_trim1.iref_vgp_trim << 6;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_IREF_TRIM1(npz_register_iref_trim1_s *iref_trim1)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_IREF_TRIM1, (uint8_t *) iref_trim1, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_IREF_TRIM2(const npz_register_iref_trim2_s iref_trim2)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_IREF_TRIM2;
    transmitData[1] |= iref_trim2.iref_v1_trim;
    transmitData[1] |= iref_trim2.iref_v2_trim << 3;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_IREF_TRIM2(npz_register_iref_trim2_s *iref_trim2)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_IREF_TRIM2, (uint8_t *) iref_trim2, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_IREF_SEL(const npz_register_iref_sel_s iref_sel)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_IREF_SEL;
    transmitData[1] |= iref_sel.sel_bypass;
    transmitData[1] |= iref_sel.sel_irefint << 1;
    transmitData[1] |= iref_sel.sel_irefresint << 2;
    transmitData[1] |= iref_sel.en_alliref;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_IREF_SEL(npz_register_iref_sel_s *iref_sel)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_IREF_SEL, (uint8_t *) iref_sel, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_FOSC_TRIM(const npz_register_fosc_trim_s fosc_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_FOSC_TRIM;
    transmitData[1] |= fosc_trim.fosc_trim;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_FOSC_TRIM(npz_register_fosc_trim_s *fosc_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_FOSC_TRIM, (uint8_t *) fosc_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_SOSC_TRIM(const npz_register_sosc_trim_s sosc_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_SOSC_TRIM;
    transmitData[1] |= sosc_trim.freq_trim;
    transmitData[1] |= sosc_trim.ipoa_trim << 4;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_SOSC_TRIM(npz_register_sosc_trim_s *sosc_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_SOSC_TRIM, (uint8_t *) sosc_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_PSW_TRIM(const npz_register_psw_trim_s psw_trim)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_PSW_TRIM;
    transmitData[1] |= psw_trim.psw_sw_trim;
    transmitData[1] |= psw_trim.psw_ib_trim << 2;
    transmitData[1] |= psw_trim.psw_en << 4;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_PSW_TRIM(npz_register_psw_trim_s *psw_trim)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_PSW_TRIM, (uint8_t *) psw_trim, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_PSW_VON(npz_register_psw_von_s *psw_von)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_PSW_VON, (uint8_t *) psw_von, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_XO1(const npz_register_xo1_s xo1)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_XO1;
    transmitData[1] |= xo1.xo_en;
    transmitData[1] |= xo1.en_div << 1;
    transmitData[1] |= xo1.sel_div << 2;
    transmitData[1] |= xo1.en_div_rchg << 4;
    transmitData[1] |= xo1.sel_rchg << 5;
    transmitData[1] |= xo1.en_iext << 7;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_XO1(npz_register_xo1_s *xo1)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_XO1, (uint8_t *) xo1, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_XO2(const npz_register_xo2_s xo2)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_XO2;
    transmitData[1] |= xo2.xo_sel_cap;
    transmitData[1] |= xo2.xo_aws << 2;
    transmitData[1] |= xo2.xo_swc_b << 3;
    transmitData[1] |= xo2.xo_test5 << 4;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_XO2(npz_register_xo2_s *xo2)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_XO2, (uint8_t *) xo2, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_write_TEST(const npz_register_test_s test)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_TEST;
    transmitData[1] |= test.test_en;
    transmitData[1] |= test.osc_mux << 1;
    transmitData[1] |= test.sosc_en << 2;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

npz_status_e npz_read_TEST(npz_register_test_s *test)
{
    return npz_hal_read(NPZ_I2C_ADDRESS, REG_TEST, (uint8_t *) test, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

static npz_status_e npz_write_unlock_registers(void)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_TUNLOCK;
    transmitData[1] = 0x59;

    return npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

void npz_set_iref_gen0(void)
{
    uint8_t reg_iref;
    uint8_t rd_data;

    npz_write_unlock_registers();

    npz_hal_read(NPZ_I2C_ADDRESS, REG_IREF_SEL, &reg_iref, 1, I2C_TRANSMISSION_TIMEOUT_MS);

    if (reg_iref != 0x0C)
    {
        npz_write_unlock_registers();
        uint8_t transmitData[2] = {REG_IREF_SEL, reg_iref | 0x08};

        npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);

        npz_write_unlock_registers();

        npz_hal_read(NPZ_I2C_ADDRESS, REG_IREF_SEL, &rd_data, 1, I2C_TRANSMISSION_TIMEOUT_MS);
    }
}

void npz_disable_clkoutput_gen0(void)
{
    uint8_t reg_clkoutput;
    uint8_t rd_data;

    npz_write_unlock_registers();

    npz_hal_read(NPZ_I2C_ADDRESS, REG_XO1, &reg_clkoutput, 1, I2C_TRANSMISSION_TIMEOUT_MS);

    if (reg_clkoutput != 0x01)
    {
        npz_write_unlock_registers();

        uint8_t transmitData[2] = {REG_XO1, 0x01};
        npz_hal_write(NPZ_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);

        npz_write_unlock_registers();

        npz_hal_read(NPZ_I2C_ADDRESS, REG_XO1, &rd_data, 1, I2C_TRANSMISSION_TIMEOUT_MS);
    }
}
