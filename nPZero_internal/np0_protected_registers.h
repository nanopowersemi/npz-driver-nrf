/**
 * @file np0_protected_registers.h
 *
 * @brief Header file for nPZero protected registers.
 *
 * This header file contains declarations and definitions for controlling and managing the nPZero protected registers.
 */

#ifndef NP0_PROTECTED_REGISTERS_H_
#define NP0_PROTECTED_REGISTERS_H_

/** @cond */
#include "np0.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/** @endcond */

/** Constants and Macros. */

/**  Test Unlock, Protected register. */
typedef struct
{
    uint8_t
        tunlock; /**< When set to the same value as ID, the protected registers will be unlocked and can then be written
                  * or read. The registers can be locked again by writing any other value to this register. */
} np0_register_tunlock_s;

/**  Test Mux, Protected register*/
typedef struct
{
    uint8_t t_io_addr : 3;  /**< Test MUX IO Address. */
    uint8_t t_blk_addr : 3; /**< Test MUX Block Address. */
    uint8_t padding : 1;    /**< Not Used. */
    uint8_t t_en_buf : 1; /**< Enable test mux output buffer. With the buffer disabled the selected signal is routed to
                           * the AMO pin, when the buffer is enabled it is routed only to the buffer and its out. */
} np0_register_tmux_s;

/**  Test Recursive Charge Pump, Protected register. */
typedef struct
{
    uint8_t adc_vtrim : 2;    /**< ADC Vref buffer trim, see @np0_adc_vtrim_e. */
    uint8_t fb_iosc_trim : 2; /**< RPC feedback bias current trim, see @FB_IOSC_TRIM. */
    uint8_t padding : 4;      /**< Not used. */
} np0_register_rcp_trim_s;

/**  Test VDDD reference trim, Protected register. */
typedef struct
{
    uint8_t vddd_trim : 3;    /**< VDDD offset trim, see @nP0_vddd_trim_e. */
    uint8_t vddd_tc_trim : 3; /**< VDDD temperature coefficient trim, see @nP0_vddd_tc_trim_e. */
    uint8_t padding : 2;      /**< Not used. */
} np0_register_vddd_trim_s;

/**  Test VREF trim, Protected register. */
typedef struct
{
    uint8_t vref_0v25_trim : 3; /**< Voltage reference 250mV offset trim. */
    uint8_t vref_0v5_trim : 3;  /**< Voltage reference 500 mV offset trim, see @nP0_vref_0v5_trim_e. */
    uint8_t padding : 2;        /**< Not used. */
} np0_register_vref_trim_s;

/**  Test VREF Temperature Coefficient trim, Protected register. */
typedef struct
{
    uint8_t vref_tc_trim : 4; /**< System voltage references 500 mV and 250 mV temperature coefficient trim. */
    uint8_t padding : 4;      /**< Not used. */
} np0_register_vref_tc_trim_s;

/**  Test IREF trim 1, Protected register. */
typedef struct
{
    uint8_t iref_trim : 3;     /**< System current bias multiplier trim, see @nP0_vref_iref_trim_e. */
    uint8_t iref_vsp_trim : 3; /**< PMOS source voltage offset trim of internal current reference. This trim is
                                * dependent of IREF_V1_TRIM and IREF_V2_TRIM. */
    uint8_t iref_vgp_trim : 2; /**< PMOS gate voltage trim of internal current reference. */
} np0_register_iref_trim1_s;

/**  Test IREF trim 2, Protected register. */
typedef struct
{
    uint8_t iref_v1_trim : 3; /**< Voltage reference 500 mV offset trim. */
    uint8_t iref_v2_trim : 3; /**< Voltage reference 750 mV offset trim. This trim is dependent of IREF_V1_TRIM. */
    uint8_t padding : 2;      /**< Not Used. */
} np0_register_iref_trim2_s;

/**  Test IREF selection and routing, Protected register. */
typedef struct
{
    uint8_t sel_bypass : 1; /**< Allows to bypass the high impedance voltage dividers of voltage references. When active
                             * the offset trimming is not available. */
    uint8_t sel_irefint : 1;    /**< Selects current reference to be used by the system. 0 = Selects voltage to current
                                 * reference w/ resistance., 1 = Selects internal current reference. */
    uint8_t sel_irefresint : 1; /**< Selects voltage to current reference resistance source. 0 = Selects external
                                 * resistance at EXT_RES pin. 1 = Selects internal resistance. */
    uint8_t en_alliref : 1; /**< Enables only the current reference selected in SEL_IREFINT. After selection of current
                             * reference this should be high to save power. */
    uint8_t padding : 4;    /**< Not Used. */
} np0_register_iref_sel_s;

/**  Test Fast Oscillator Frequency Trim, Protected register. */
typedef struct
{
    uint8_t fosc_trim : 5; /**< Fast Oscillator Frequency Trim. */
    uint8_t padding : 3;   /**< Not used. */
} np0_register_fosc_trim_s;

/**  Test Slow Oscillator Frequency Trim, Protected register. */
typedef struct
{
    uint8_t freq_trim : 4; /**< Slow Oscillator Frequency Trim. */
    uint8_t ipoa_trim : 3; /**< Slow oscillator bias current trim used in OpAmp and comparator. */
    uint8_t padding : 1;   /**< Not used. */
} np0_register_sosc_trim_s;

/**  Test Power Switch Trim, Protected register. */
typedef struct
{
    uint8_t psw_sw_trim : 2; /**< Slew rate control capacitor trimming [1pF-4pF]. */
    uint8_t psw_ib_trim : 2; /**< Slew ate bias current trimming [5nA-11nA]. */
    uint8_t psw_en : 1;      /**< Enables gate control. If set low, gate is left floating. */
    uint8_t padding : 3;     /**< Not used. */
} np0_register_psw_trim_s;

/** (Read) Test Power Switch VON, Protected register. */
typedef struct
{
    uint8_t voh_lp1 : 1; /**< Detects when Low Power Switch 1 output almost reaches its final value (VBAT-100mV). */
    uint8_t voh_lp2 : 1; /**< Detects when Low Power Switch 2 output almost reaches its final value (VBAT-100mV). */
    uint8_t voh_lp3 : 1; /**< Detects when Low Power Switch 3 output almost reaches its final value (VBAT-100mV). */
    uint8_t voh_lp4 : 1; /**< Detects when Low Power Switch 4 output almost reaches its final value (VBAT-100mV). */
    uint8_t voh_hp : 1;  /**< Detects when High Power Switch 1 output almost reaches its final value (VBAT-100mV). */
    uint8_t padding : 3; /**< Not used. */
} np0_register_psw_von_s;

/**  Test Crystal Oscillator, Protected register. */
typedef struct
{
    uint8_t xo_en : 1;       /**< Enable crystal oscillator. */
    uint8_t en_div : 1;      /**< Enable clock divided output (CLK_OUT pin). */
    uint8_t sel_div : 2;     /**< Selects the division value of CLK_OUT pin, see @nP0_sclk_div_sel_e. */
    uint8_t en_div_rchg : 1; /**< Enables the charge injection rate division. */
    uint8_t sel_rchg : 2;    /**< Selects charge injection rate of clock periods. */
    uint8_t en_iext : 1;     /**< Enables XO_IBIAS input 100nA bias current for text purposes. */
} np0_register_xo1_s;

/**  Test Crystal Oscillator, Protected register. */
typedef struct
{
    uint8_t xo_sel_cap : 2; /**< Adds extra internal cap on the vxtal node. */
    uint8_t xo_aws : 1;     /**< Forces startup mode always on. */
    uint8_t xo_swc_b : 2;   /**< Forces startup without the OK comparator. */
    uint8_t xo_test5 : 1;   /**< Turns on BETAM needed for test5. */
    uint8_t padding : 2;    /**< Not Used. */
} np0_register_xo2_s;

/**  Test mode, Protected register. */
typedef struct
{
    uint8_t test_en : 1; /**< Enables test mode. UVLO bypass. */
    uint8_t osc_mux : 1; /**< Selects fast and slow system clock sources. 0 = Internal clock oscillators. 1 = External
                          * clocks from INT3 and INT4 pins. */
    uint8_t sosc_en : 1; /**< Enables slow oscillator, Slow oscillator can be off if the SCLK_SEL is 1 as XO is the
                          * clock source. */
    uint8_t padding : 5; /**< Not Used. */
} np0_register_test_s;

/** Protected registers struct that holds all protected registers in the nPZero
 * 	Registers in this region should be used for testing & debugging only.
 */
typedef struct
{
    np0_register_tunlock_s tunlock;
    np0_register_tmux_s tmux;
    np0_register_rcp_trim_s rcp_trim;
    np0_register_vddd_trim_s vddd_trim;
    np0_register_vref_trim_s vref_trim;
    np0_register_vref_tc_trim_s vref_tc_trim;
    np0_register_iref_trim1_s iref_trim1;
    np0_register_iref_trim2_s iref_trim2;
    np0_register_iref_sel_s iref_sel;
    np0_register_fosc_trim_s fosc_trim;
    np0_register_sosc_trim_s sosc_trim;
    np0_register_psw_trim_s psw_trim;
    np0_register_psw_von_s psw_von;
    np0_register_xo1_s xo1;
    np0_register_xo2_s xo2;
    np0_register_test_s test;
} np0_protected_registers_s;

/* Function Prototypes */

/**
 * @brief Writes the tunlock struct to the tunlock register.
 * @brief When set to the same value as ID, the protected registers will be unlocked and can then be written or
 * read.
 * @brief The registers can be locked again by writing any other value to this register.
 *
 *
 * @param [in] tunlock Test Unlock register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_TUNLOCK(const np0_register_tunlock_s tunlock);

/**
 * @brief Reads the tunlock register and writes it to np0_register_tunlock_s struct.
 *
 *
 * @param [in] sw Low power switch indicates which peripheral will be read.
 * @param [out] tunlock Pointer to Test Unlock register where value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_TUNLOCK(np0_register_tunlock_s *tunlock);

/**
 * @brief Writes the tmux struct to the tmux register.
 *
 *
 * @param [in] tmux Test MUX register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_TMUX(const np0_register_tmux_s tmux);

/**
 * @brief Reads the tmux register and writes it to np0_register_tmux_s struct.
 *
 *
 * @param [out] tmux Pointer to Test MUX register where value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_TMUX(np0_register_tmux_s *tmux);

/**
 * @brief Writes the rcp_trim struct to the rcp_trim register.
 *
 *
 * @param [in] rcp_trim Recursive Charge Pump/ADC Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_RCP_TRIM(const np0_register_rcp_trim_s rcp_trim);

/**
 * @brief Reads the rcp_trim register and writes it to np0_register_rcp_trim_s struct.
 *
 *
 * @param [out] rcp_trim Pointer to Recursive Charge Pump/ADC Trim register where value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_RCP_TRIM(np0_register_rcp_trim_s *rcp_trim);

/**
 * @brief Writes the vddd_trim struct to the vddd_trim register.
 *
 *
 * @param [in] vddd_trim VDDD Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_VDDD_TRIM(const np0_register_vddd_trim_s vddd_trim);

/**
 * @brief Reads the vddd_trim register and writes it to np0_register_vddd_trim_s struct.
 *
 *
 * @param [out] vddd_trim Pointer to VDDD Trim register where value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_VDDD_TRIM(np0_register_vddd_trim_s *vddd_trim);

/**
 * @brief Writes the vref_trim struct to the vref_trim register.
 *
 *
 * @param [in] vref_trim VREF Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_VREF_TRIM(const np0_register_vref_trim_s vref_trim);

/**
 * @brief Reads the vref_trim register and writes it to np0_register_vref_trim_s struct.
 *
 *
 * @param [out] vref_trim Pointer to VREF Trim register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_VREF_TRIM(np0_register_vref_trim_s *vref_trim);

/**
 * @brief Writes the vref_tc_trim struct to the vref_tc_trim register.
 *
 *
 * @param [in] vref_tc_trim VREF Temperature Coefficient Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_VREF_TC_TRIM(const np0_register_vref_tc_trim_s vref_tc_trim);

/**
 * @brief Reads the VREF_RC_TRIM register and writes it to np0_register_vref_tc_trim_s struct.
 *
 *
 * @param [out] vref_tc_trim Pointer to VREF Temperature Coefficient Trim register where the value will be
 * stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_VREF_TC_TRIM(np0_register_vref_tc_trim_s *vref_tc_trim);

/**
 * @brief Writes the iref_trim1 struct to the iref_trim1 register.
 *
 *
 * @param [in] iref_trim1 Current Reference Trim 1 register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_IREF_TRIM1(const np0_register_iref_trim1_s iref_trim1);

/**
 * @brief Reads the iref_trim1 register and writes it to np0_register_iref_trim1_s struct.
 *
 *
 * @param [out] iref_trim1 Pointer to Current Reference Trim 1 register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_IREF_TRIM1(np0_register_iref_trim1_s *iref_trim1);

/**
 * @brief Writes the iref_trim2 struct to the iref_trim2 register.
 *
 *
 * @param [in] iref_trim2 Current Reference Trim 2 register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_IREF_TRIM2(const np0_register_iref_trim2_s iref_trim2);

/**
 * @brief Reads the iref_trim2 register and writes it to np0_register_iref_trim2_s struct.
 *
 *
 * @param [out] iref_trim2 Pointer to Current Reference Trim 2 register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_IREF_TRIM2(np0_register_iref_trim2_s *iref_trim2);

/**
 * @brief Writes the iref_sel struct to the iref_sel register.
 *
 *
 * @param [in] iref_sel Current Reference Selection register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_IREF_SEL(const np0_register_iref_sel_s iref_sel);

/**
 * @brief Reads the iref_sel register and writes it to np0_register_iref_sel_s struct.
 *
 *
 * @param [out] iref_sel Current Reference Selection register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_IREF_SEL(np0_register_iref_sel_s *iref_sel);

/**
 * @brief Writes the fosc_trim struct to the fosc_trim register.
 *
 *
 * @param [in] fosc_trim Fast Oscillator Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_FOSC_TRIM(const np0_register_fosc_trim_s fosc_trim);

/**
 * @brief Reads the fosc_trim register and writes it to np0_register_fosc_trim_s struct.
 *
 *
 * @param [out] fosc_trim Pointer to Fast Oscillator Trim register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_FOSC_TRIM(np0_register_fosc_trim_s *fosc_trim);

/**
 * @brief Writes the sosc_trim struct to the sosc_trim register.
 *
 *
 * @param [in] sosc_trim Slow Oscillator Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_SOSC_TRIM(const np0_register_sosc_trim_s sosc_trim);

/**
 * @brief Reads the sosc_trim register and writes it to np0_register_sosc_trim_s struct.
 *
 *
 * @param [out] sosc_trim Slow Oscillator Trim register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_SOSC_TRIM(np0_register_sosc_trim_s *sosc_trim);

/**
 * @brief Writes the psw_trim struct to the psw_trim register.
 *
 *
 * @param [in] psw_trim Power Switch Trim register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_PSW_TRIM(const np0_register_psw_trim_s psw_trim);

/**
 * @brief Reads the psw_trim register and writes it to np0_register_psw_trim_s struct.
 *
 *
 * @param [out] psw_trim Pointer to Power Switch Trim register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_PSW_TRIM(np0_register_psw_trim_s *psw_trim);

/**
 * @brief Reads the psw_von register and writes it to np0_register_psw_von_s struct.
 *
 *
 * @param [out] psw_von Pointer to Power Switch Voltage On register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_PSW_VON(np0_register_psw_von_s *psw_von);

/**
 * @brief Writes the xo1 struct to the xo1 register.
 *
 *
 * @param [in] xo1 Crystal Oscillator 1 register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_XO1(const np0_register_xo1_s xo1);

/**
 * @brief Reads the xo1 register and writes it to np0_register_xo1_s struct.
 *
 *
 * @param [out] xo1 Pointer to Crystal Oscillator 1 register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_XO1(np0_register_xo1_s *xo1);

/**
 * @brief Writes the xo2 struct to the xo2 register.
 *
 *
 * @param [in] xo2 Crystal Oscillator 2 register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_XO2(const np0_register_xo2_s xo2);

/**
 * @brief Reads the xo2 register and writes it to np0_register_xo2_s struct.
 *
 *
 *
 * @param [in] sw Low power switch indicates which peripheral will be read..
 * @param [out] xo2 Pointer to Crystal Oscillator 2 register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_XO2(np0_register_xo2_s *xo2);

/**
 * @brief Writes the test struct to the test register.
 *
 *
 * @param [in] test Test register that holds the value to be written.
 * @return np0_status_e Status
 */
np0_status_e np0_write_TEST(const np0_register_test_s test);

/**
 * @brief Reads the test register and writes it to np0_register_test_s struct.
 *
 *
 * @param [out] test Pointer to Test register where the value will be stored.
 * @return np0_status_e Status
 */
np0_status_e np0_read_TEST(np0_register_test_s *test);

void np0_set_iref_gen0(void);
void np0_disable_clkoutput_gen0(void);

#endif /* NP0_PROTECTED_REGISTERS_H_ */
