/**
 * @file np0.c
 *
 * @brief Source file for nPZero Driver.
 * 
 * This file contains the implementation of the nPZero driver for controlling and managing 
 * the nPZero power-saving IC.
 * It provides functions to interact with the nPZero on a register level, 
 * enabling extensive configuration and features, such as periodic power switching Peripherals for value comparison.
 *  
 * This driver is designed to work with the nPZero power-saving IC from Nanopower Semiconductor. 
 * It uitilizes the I2C communication protocol for interfacing with the host MCU.
 * 
 * @note This driver assumes proper initialization of the hardware abstraction layer for I2C is present in the np0_hal.c file.
 * 
 * @see np0.h for the corresponding header file providing function prototypes and definitions.
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/

#include "../Inc/np0.h"
#include "../Inc/np0_registers.h"

/*****************************************************************************
 * Private Methods
 *****************************************************************************/

/*****************************************************************************
 * Public Methods
 *****************************************************************************/

np0_status_e np0_write_SLEEP_RST(uint8_t sleep_rst_value)
{
    uint8_t transmitData[] = {REG_SLEEP_RST, sleep_rst_value};

    return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_SLEEP_RST(uint8_t *sleep_rst_value)
{
    return np0_hal_read(NP0_I2C_ADDRESS, REG_SLEEP_RST, sleep_rst_value, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_ID(uint8_t *id)
{
    return np0_hal_read(NP0_I2C_ADDRESS, REG_ID, id, 1, I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_STA1(np0_register_sta1_s *sta1) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_STA1, (uint8_t*) sta1, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_STA2(np0_register_sta2_s *sta2) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_STA2, (uint8_t*) sta2, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_PSWCTL(const np0_register_pswctl_s pswctl) {
	uint8_t transmitData[2] = { 0 };

	transmitData[0] = REG_PSWCTL;
	transmitData[1] |= pswctl.pswint_p1;
	transmitData[1] |= pswctl.pswint_p2 << 1;
	transmitData[1] |= pswctl.pswint_p3 << 2;
	transmitData[1] |= pswctl.pswint_p4 << 3;
	transmitData[1] |= pswctl.pswh_mode << 4;
	transmitData[1] |= pswctl.psw_en_vn << 6;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_PSWCTL(np0_register_pswctl_s *pswctl) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_PSWCTL, (uint8_t*) pswctl, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_SYSCFG1(const np0_register_syscfg1_s syscfg1) 
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_SYSCFG1;

    transmitData[1] |= syscfg1.wup1;
    transmitData[1] |= syscfg1.wup2 << 1;
    transmitData[1] |= syscfg1.wup3 << 2;
    transmitData[1] |= syscfg1.wup4 << 3;
    transmitData[1] |= syscfg1.adc_int_wakeup_enable << 4;
    transmitData[1] |= syscfg1.adc_ext_wakeup_enable << 5;
    transmitData[1] |= syscfg1.wake_up_any_or_all << 6;

    return np0_hal_write(
        NP0_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_SYSCFG1(np0_register_syscfg1_s *syscfg1) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_SYSCFG1, (uint8_t*) syscfg1, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_SYSCFG2(const np0_register_syscfg2_s syscfg2)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_SYSCFG2;
    transmitData[1] |= syscfg2.sclk_div_en;
    transmitData[1] |= syscfg2.sclk_div_sel << 1;
    transmitData[1] |= syscfg2.sclk_sel << 3;	
    transmitData[1] |= syscfg2.adc_ext_on << 4;
    transmitData[1] |= syscfg2.adc_clk_sel << 5;

    return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_SYSCFG2(np0_register_syscfg2_s *syscfg2) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_SYSCFG2, (uint8_t*) syscfg2, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_SYSCFG3(const np0_register_syscfg3_s syscfg3)
{
    uint8_t transmitData[2] = {0};
    transmitData[0] = REG_SYSCFG3;
    transmitData[1] |= syscfg3.io_str;
    transmitData[1] |= syscfg3.i2c_pup_en << 1;
    transmitData[1] |= syscfg3.i2c_pup_auto << 2;
    transmitData[1] |= syscfg3.spi_auto << 3;
    transmitData[1] |= syscfg3.xo_clkout_div << 4;
    transmitData[1] |= syscfg3.sclk_sel_status << 7;

    return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_SYSCFG3(np0_register_syscfg3_s *syscfg3) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_SYSCFG3, (uint8_t*) syscfg3, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_TOUT(const np0_register_tout_s tout) {
	uint8_t transmitData[2] = { REG_TOUT_L, tout.tout_l };

	if (np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS) != OK) {
		return ERR;
	} else {
		transmitData[0] = REG_TOUT_H;
		transmitData[1] = tout.tout_h;

		return np0_hal_write(NP0_I2C_ADDRESS, transmitData,
				sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
	}
}

np0_status_e np0_read_TOUT(np0_register_tout_s *tout) {
	np0_status_e success = ERR;

	success = np0_hal_read(NP0_I2C_ADDRESS, REG_TOUT_L, &tout->tout_l, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);

	if (success == OK) {
		success = np0_hal_read(NP0_I2C_ADDRESS, REG_TOUT_H, &tout->tout_h, 1,
				I2C_TRANSMISSION_TIMEOUT_MS);
	}

	return success;
}

np0_status_e np0_write_INTCFG(const np0_register_intcfg_s intcfg) {
	uint8_t transmitData[2] = { 0 };
	transmitData[0] = REG_INTCFG;
	transmitData[1] |= intcfg.pu_int1;
	transmitData[1] |= intcfg.pu_s_int1 << 1;
	transmitData[1] |= intcfg.pu_int2 << 2;
	transmitData[1] |= intcfg.pu_s_int2 << 3;
	transmitData[1] |= intcfg.pu_int3 << 4;
	transmitData[1] |= intcfg.pu_s_int3 << 5;
	transmitData[1] |= intcfg.pu_int4 << 6;
	transmitData[1] |= intcfg.pu_s_int4 << 7;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_INTCFG(np0_register_intcfg_s *intcfg) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_INTCFG, (uint8_t*) intcfg, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_THROVA1(const np0_register_throva1_s throva1) {
	uint8_t transmitData[2] = { 0 };
	transmitData[0] = REG_THROVA1;
	transmitData[1] = throva1.throva;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_THROVA1(np0_register_throva1_s *throva1) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_THROVA1, (uint8_t*) throva1, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_THROVA2(const np0_register_throva2_s throva2) {
	uint8_t transmitData[2] = { 0 };
	transmitData[0] = REG_THROVA2;
	transmitData[1] = throva2.throva;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_THROVA2(np0_register_throva2_s *throva2) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_THROVA2, (uint8_t*) throva2, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_THRUNA1(const np0_register_thruna1_s thruna1) {
	uint8_t transmitData[2] = { 0 };
	transmitData[0] = REG_THRUNA1;
	transmitData[1] = thruna1.thruna;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_THRUNA1(np0_register_thruna1_s *thruna1) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_THRUNA1, (uint8_t*) thruna1, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_THRUNA2(const np0_register_thruna2_s thruna2) {
	uint8_t transmitData[2] = { 0 };
	transmitData[0] = REG_THRUNA2;
	transmitData[1] = thruna2.thruna;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_THRUNA2(np0_register_thruna2_s *thruna2) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_THRUNA2, (uint8_t*) thruna2, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_ADC_CORE(np0_register_adc_core_s *adc_core) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_ADC_CORE, &adc_core->adc_core, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_ADC_EXT(np0_register_adc_ext_s *adc_ext) {
	return np0_hal_read(NP0_I2C_ADDRESS, REG_ADC_EXT, &adc_ext->adc_ext, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_SRAM(const uint8_t sram_reg, const uint8_t SRAM)
{
	uint8_t transmitData[2] = { 0 };
	transmitData[0] = sram_reg;
	transmitData[1] = SRAM;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_SRAM(const uint8_t sram_reg, np0_register_sram_s *SRAM) {
	return np0_hal_read(NP0_I2C_ADDRESS, sram_reg, (uint8_t*) SRAM, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_CFGP(const np0_psw_e sw, const np0_register_cfgp_s cfgp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_CFGP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_CFGP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_CFGP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_CFGP4;
		break;
	}

	transmitData[1] |= cfgp.pwmod;
	transmitData[1] |= cfgp.tmod << 2;
	transmitData[1] |= cfgp.pswmod << 4;
	transmitData[1] |= cfgp.intmod << 6;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_CFGP(const np0_psw_e sw, np0_register_cfgp_s *cfgp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_CFGP1;
		break;

	case PSW_LP2:
		reg = REG_CFGP2;
		break;

	case PSW_LP3:
		reg = REG_CFGP3;
		break;

	case PSW_LP4:
		reg = REG_CFGP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) cfgp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_MODP(const np0_psw_e sw, const np0_register_modp_s modp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_MODP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_MODP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_MODP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_MODP4;
		break;
	}

	transmitData[1] |= modp.cmod;
	transmitData[1] |= modp.dtype << 1;
	transmitData[1] |= modp.seqrw << 3;
	transmitData[1] |= modp.wunak << 4;
	transmitData[1] |= modp.swprreg << 5;
	transmitData[1] |= modp.spimod << 6;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_MODP(const np0_psw_e sw, np0_register_modp_s *modp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_MODP1;
		break;

	case PSW_LP2:
		reg = REG_MODP2;
		break;

	case PSW_LP3:
		reg = REG_MODP3;
		break;

	case PSW_LP4:
		reg = REG_MODP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) modp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_PERP(const np0_psw_e sw, const np0_register_perp_s perp) {
	uint8_t reg_h = 0;
	uint8_t transmitData[2] = { 0 };

	if (perp.perp_l == 0 && perp.perp_h == 0) {
		return INVALID_PARAM;
	}

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_PERP1_L;
		transmitData[1] = perp.perp_l;

		reg_h = REG_PERP1_H;
		break;

	case PSW_LP2:
		transmitData[0] = REG_PERP2_L;
		transmitData[1] = perp.perp_l;

		reg_h = REG_PERP2_H;
		break;

	case PSW_LP3:
		transmitData[0] = REG_PERP3_L;
		transmitData[1] = perp.perp_l;

		reg_h = REG_PERP3_H;
		break;

	case PSW_LP4:
		transmitData[0] = REG_PERP4_L;
		transmitData[1] = perp.perp_l;

		reg_h = REG_PERP4_H;
		break;
	}

	if (np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS) != OK) {
		return ERR;
	}

	transmitData[0] = reg_h;
	transmitData[1] = perp.perp_h;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_PERP(const np0_psw_e sw, np0_register_perp_s *perp) {
	uint8_t reg_l = 0, reg_h = 0;
	np0_status_e success = ERR;

	switch (sw) {
	case PSW_LP1:
		reg_l = REG_PERP1_L;
		reg_h = REG_PERP1_H;
		break;

	case PSW_LP2:
		reg_l = REG_PERP2_L;
		reg_h = REG_PERP2_H;
		break;

	case PSW_LP3:
		reg_l = REG_PERP3_L;
		reg_h = REG_PERP3_H;
		break;

	case PSW_LP4:
		reg_l = REG_PERP4_L;
		reg_h = REG_PERP4_H;
		break;
	}

	success = np0_hal_read(NP0_I2C_ADDRESS, reg_l, &perp->perp_l, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);

	if (success == OK) {
		success = np0_hal_read(NP0_I2C_ADDRESS, reg_h, &perp->perp_h, 1,
				I2C_TRANSMISSION_TIMEOUT_MS);
	}

	return success;
}

np0_status_e np0_write_NCMDP(const np0_psw_e sw,
		const np0_register_ncmdp_s ncmdp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_NCMDP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_NCMDP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_NCMDP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_NCMDP4;
		break;
	}

	transmitData[1] |= ncmdp.ncmdp;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_NCMDP(const np0_psw_e sw, np0_register_ncmdp_s *ncmdp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_NCMDP1;
		break;

	case PSW_LP2:
		reg = REG_NCMDP2;
		break;

	case PSW_LP3:
		reg = REG_NCMDP3;
		break;

	case PSW_LP4:
		reg = REG_NCMDP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) ncmdp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_ADDRP(const np0_psw_e sw,
		const np0_register_addrp_s addrp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_ADDRP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_ADDRP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_ADDRP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_ADDRP4;
		break;
	}

	transmitData[1] |= addrp.addrp;
	transmitData[1] |= addrp.spi_en << 7;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_ADDRP(const np0_psw_e sw, np0_register_addrp_s *addrp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_ADDRP1;
		break;

	case PSW_LP2:
		reg = REG_ADDRP2;
		break;

	case PSW_LP3:
		reg = REG_ADDRP3;
		break;

	case PSW_LP4:
		reg = REG_ADDRP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) addrp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_RREGP(const np0_psw_e sw,
		const np0_register_rregp_s rregp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_RREGP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_RREGP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_RREGP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_RREGP4;
		break;
	}

	transmitData[1] |= rregp.rregp;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_RREGP(const np0_psw_e sw, np0_register_rregp_s *rregp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_RREGP1;
		break;

	case PSW_LP2:
		reg = REG_RREGP2;
		break;

	case PSW_LP3:
		reg = REG_RREGP3;
		break;

	case PSW_LP4:
		reg = REG_RREGP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) rregp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_THROVP(const np0_psw_e sw,
		const np0_register_throvp_s throvp) {
	uint8_t reg_h = 0;
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_THROVP1_L;

		reg_h = REG_THROVP1_H;
		break;

	case PSW_LP2:
		transmitData[0] = REG_THROVP2_L;
		reg_h = REG_THROVP2_H;
		break;

	case PSW_LP3:
		transmitData[0] = REG_THROVP3_L;
		reg_h = REG_THROVP3_H;
		break;

	case PSW_LP4:
		transmitData[0] = REG_THROVP4_L;
		reg_h = REG_THROVP4_H;
		break;
	}

	transmitData[1] = throvp.throvp_l;

	if (np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS) != OK) {
		return ERR;
	}

	transmitData[0] = reg_h;
	transmitData[1] = throvp.throvp_h;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_THROVP(const np0_psw_e sw, np0_register_throvp_s *throvp) {
	uint8_t reg_l = 0, reg_h = 0;
	np0_status_e success = ERR;

	switch (sw) {
	case PSW_LP1:
		reg_l = REG_THROVP1_L;
		reg_h = REG_THROVP1_H;
		break;

	case PSW_LP2:
		reg_l = REG_THROVP2_L;
		reg_h = REG_THROVP2_H;
		break;

	case PSW_LP3:
		reg_l = REG_THROVP3_L;
		reg_h = REG_THROVP3_H;
		break;

	case PSW_LP4:
		reg_l = REG_THROVP4_L;
		reg_h = REG_THROVP4_H;
		break;
	}

	success = np0_hal_read(NP0_I2C_ADDRESS, reg_l, &throvp->throvp_l, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);

	if (success == OK) {
		success = np0_hal_read(NP0_I2C_ADDRESS, reg_h, &throvp->throvp_h, 1,
				I2C_TRANSMISSION_TIMEOUT_MS);
	}

	return success;
}

np0_status_e np0_write_THRUNP(const np0_psw_e sw,
		const np0_register_thrunp_s thrunp) {
	uint8_t reg_h = 0;
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_THRUNP1_L;

		reg_h = REG_THRUNP1_H;
		break;

	case PSW_LP2:
		transmitData[0] = REG_THRUNP2_L;
		reg_h = REG_THRUNP2_H;
		break;

	case PSW_LP3:
		transmitData[0] = REG_THRUNP3_L;
		reg_h = REG_THRUNP3_H;
		break;

	case PSW_LP4:
		transmitData[0] = REG_THRUNP4_L;
		reg_h = REG_THRUNP4_H;
		break;
	}

	transmitData[1] = thrunp.thrunp_l;

	if (np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS) != OK) {
		return ERR;
	}

	transmitData[0] = reg_h;
	transmitData[1] = thrunp.thrunp_h;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_THRUNP(const np0_psw_e sw, np0_register_thrunp_s *thrunp) {
	uint8_t reg_l = 0, reg_h = 0;
	np0_status_e success = ERR;

	switch (sw) {
	case PSW_LP1:
		reg_l = REG_THRUNP1_L;
		reg_h = REG_THRUNP1_H;
		break;

	case PSW_LP2:
		reg_l = REG_THRUNP2_L;
		reg_h = REG_THRUNP2_H;
		break;

	case PSW_LP3:
		reg_l = REG_THRUNP3_L;
		reg_h = REG_THRUNP3_H;
		break;

	case PSW_LP4:
		reg_l = REG_THRUNP4_L;
		reg_h = REG_THRUNP4_H;
		break;
	}

	success = np0_hal_read(NP0_I2C_ADDRESS, reg_l, &thrunp->thrunp_l, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);

	if (success == OK) {
		success = np0_hal_read(NP0_I2C_ADDRESS, reg_h, &thrunp->thrunp_h, 1,
				I2C_TRANSMISSION_TIMEOUT_MS);
	}

	return success;
}

np0_status_e np0_write_TWTP(const np0_psw_e sw, const np0_register_twtp_s twtp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_TWTP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_TWTP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_TWTP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_TWTP4;
		break;
	}

	transmitData[1] |= twtp.twtp;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_TWTP(const np0_psw_e sw, np0_register_twtp_s *twtp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_TWTP1;
		break;

	case PSW_LP2:
		reg = REG_TWTP2;
		break;

	case PSW_LP3:
		reg = REG_TWTP3;
		break;

	case PSW_LP4:
		reg = REG_TWTP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) twtp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_write_TCFGP(const np0_psw_e sw,
		const np0_register_tcfgp_s tcfgp) {
	uint8_t transmitData[2] = { 0 };

	switch (sw) {
	case PSW_LP1:
		transmitData[0] = REG_TCFGP1;
		break;

	case PSW_LP2:
		transmitData[0] = REG_TCFGP2;
		break;

	case PSW_LP3:
		transmitData[0] = REG_TCFGP3;
		break;

	case PSW_LP4:
		transmitData[0] = REG_TCFGP4;
		break;
	}

	transmitData[1] |= tcfgp.twt_en;
	transmitData[1] |= tcfgp.twt_ext << 1;
	transmitData[1] |= tcfgp.tinit_en << 2;
	transmitData[1] |= tcfgp.tinit_ext << 3;
	transmitData[1] |= tcfgp.i2cret << 4;

	return np0_hal_write(NP0_I2C_ADDRESS, transmitData, sizeof(transmitData),
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_TCFGP(const np0_psw_e sw, np0_register_tcfgp_s *tcfgp) {
	uint8_t reg = 0;

	switch (sw) {
	case PSW_LP1:
		reg = REG_TCFGP1;
		break;

	case PSW_LP2:
		reg = REG_TCFGP2;
		break;

	case PSW_LP3:
		reg = REG_TCFGP3;
		break;

	case PSW_LP4:
		reg = REG_TCFGP4;
		break;
	}

	return np0_hal_read(NP0_I2C_ADDRESS, reg, (uint8_t*) tcfgp, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);
}

np0_status_e np0_read_VALP(const np0_psw_e sw, np0_register_valp_s *valp) {
	uint8_t reg_l = 0, reg_h = 0;
	np0_status_e success = ERR;

	switch (sw) {
	case PSW_LP1:
		reg_l = REG_VALP1_L;
		reg_h = REG_VALP1_H;
		break;

	case PSW_LP2:
		reg_l = REG_VALP2_L;
		reg_h = REG_VALP2_H;
		break;

	case PSW_LP3:
		reg_l = REG_VALP3_L;
		reg_h = REG_VALP3_H;
		break;

	case PSW_LP4:
		reg_l = REG_VALP4_L;
		reg_h = REG_VALP4_H;
		break;
	}

	success = np0_hal_read(NP0_I2C_ADDRESS, reg_l, &valp->valp_l, 1,
			I2C_TRANSMISSION_TIMEOUT_MS);

	if (success == OK) {
		success = np0_hal_read(NP0_I2C_ADDRESS, reg_h, &valp->valp_h, 1,
				I2C_TRANSMISSION_TIMEOUT_MS);
	}

	return success;
}

np0_status_e np0_read_register(uint8_t register_address, void *buffer, size_t size)
{
    return np0_hal_read(NP0_I2C_ADDRESS, register_address, (uint8_t *) buffer, size, I2C_TRANSMISSION_TIMEOUT_MS);
}
