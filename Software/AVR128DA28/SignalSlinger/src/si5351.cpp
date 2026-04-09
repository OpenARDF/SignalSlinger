/*
 *  MIT License
 *
 *  Copyright (c) 2026 DigitalConfections
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 */

/*
 * Si5351 clock-generator control for firmware RF outputs.
 *
 * This module holds the low-level Si5351 register sequencing, cached local
 * state, and the helper math that converts requested output frequencies into
 * PLL and multisynth register values.
 *
 * It stays intentionally compact: callers are expected to choose legal clock
 * plans, while this module focuses on programming the device efficiently.
 */

#include "defs.h"

#ifdef INCLUDE_SI5351_SUPPORT

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <util/twi.h>

#include "include/i2c.h"
#include "include/si5351.h"

#define SI5351_I2C_SLAVE_ADDR 0xC0 /* I2C slave address */

/* Cached calibration, PLLB VCO state, and last-requested output frequencies. */
static int32_t g_si5351_ref_correction = EEPROM_SI5351_CALIBRATION_DEFAULT;
static Frequency_Hz freqVCOB = 0;

static Frequency_Hz xtal_freq = SI5351_XTAL_FREQ;
static uint8_t enabledClocksMask = 0x00;
static Frequency_Hz clock_out[3] = {0, 0, 0};

#ifdef SUPPORT_STATUS_READS
Si5351Status dev_status;
Si5351IntStatus dev_int_status;
#endif

/* Internal helpers used to translate requested clock plans into register writes. */

void pll_reset(Si5351_pll);

#ifdef DEBUGGING_ONLY
uint32_t pll_calc(Frequency_Hz, Union_si5351_regs *, int32_t);
#else
#ifdef APPLY_XTAL_CALIBRATION_VALUE
bool pll_calc(Frequency_Hz, Union_si5351_regs *, int32_t);
#else
bool pll_calc(Frequency_Hz, Union_si5351_regs *);
#endif
#endif

#ifdef SUPPORT_FOUT_BELOW_1024KHZ
uint8_t select_r_div(Frequency_Hz *);
#endif

#ifdef DEBUGGING_ONLY
uint32_t multisynth_calc(Frequency_Hz, Union_si5351_regs *, bool *, bool *, uint32_t *);
#else
uint32_t multisynth_calc(Frequency_Hz, Union_si5351_regs *, bool *, bool *);
#endif

#ifdef DEBUGGING_ONLY
uint32_t set_pll(Frequency_Hz, Si5351_pll);
#else
bool set_pll(Frequency_Hz, Si5351_pll);
#endif

Frequency_Hz multisynth_estimate(Frequency_Hz freq_Fout, Union_si5351_regs *reg, bool *int_mode, bool *divBy4);
bool set_multisynth_registers_source(Si5351_clock, Si5351_pll);
//	bool set_multisynth_registers(Si5351_clock, Union_si5351_regs, uint8_t, uint8_t, bool);
bool set_multisynth_registers(Si5351_clock clk, Union_si5351_regs ms_reg, bool int_mode, uint8_t r_div, bool div_by_4);
uint32_t calc_gcd(uint32_t, uint32_t);
void reduce_by_gcd(uint32_t *, uint32_t *);
bool si5351_write_bulk(uint8_t, uint8_t *, uint8_t);
bool si5351_read_bulk(uint8_t, uint8_t *, uint8_t);
bool set_integer_mode(Si5351_clock, bool);
bool ms_div(Si5351_clock, uint8_t, bool);

#ifdef SUPPORT_STATUS_READS
bool si5351_read_sys_status(Si5351Status *);
bool si5351_read_int_status(Si5351IntStatus *);
#endif

bool g_si5351_initialized = false;

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */
/* Public functions */
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void si5351_shutdown_comms(void)
{
	I2C_0_Shutdown();
}

void si5351_start_comms(void)
{
	I2C_0_Init();
}

/**
 * Initialize the Si5351 communication path and baseline device state.
 *
 * The routine disables all outputs, powers down the three used clock drivers,
 * applies the requested crystal load setting, and optionally updates the
 * expected reference-clock frequency used by later PLL calculations.
 *
 * @param xtal_load_c Crystal load capacitance selection.
 * @param ref_osc_freq Reference oscillator frequency in Hz, or 0 for the default crystal.
 * @return true on failure, false on success.
 */
bool si5351_init(Si5351_Xtal_load_pF xtal_load_c, Frequency_Hz ref_osc_freq)
{
	static bool err = false;
	uint8_t data[2];

#ifndef DEBUG_WITHOUT_I2C
	/* Start I2C comms */
	I2C_0_Init();
#endif

#ifdef DEBUGGING_ONLY
	uint8_t result_val = 0x00;
	bool result = false;
#endif
	uint8_t reg_val;

	freqVCOB = 0;
	xtal_freq = SI5351_XTAL_FREQ;
	enabledClocksMask = 0x00;

	/* Disable Outputs */
	/* Set CLKx_DIS high; Reg. 3 = 0xFF */
	data[0] = 0xFF;
	err = si5351_write_bulk(0x03, data, 1);

	if(err)
		return err;
	/* Power down clocks */
	data[0] = 0xCC;
	err |= si5351_write_bulk(0x10, data, 1);
	err |= si5351_write_bulk(0x11, data, 1);
	err |= si5351_write_bulk(0x12, data, 1);

	/* Set crystal load capacitance */
	reg_val = 0x12; /* 0b010010 reserved value bits */
	reg_val |= xtal_load_c;

	data[0] = reg_val;
	err |= si5351_write_bulk(SI5351_CRYSTAL_LOAD, data, 1);

	if(!ref_osc_freq)
	{
		ref_osc_freq = SI5351_XTAL_FREQ;
	}

	/* Change the ref osc freq if different from default */
	if(ref_osc_freq != xtal_freq)
	{
		if(si5351_read_bulk(SI5351_PLL_INPUT_SOURCE, data, 1))
		{
			return true;
		}

		reg_val = data[0];

		/* Clear the bits first */
		reg_val &= ~(SI5351_CLKIN_DIV_MASK);

		xtal_freq = ref_osc_freq;
		reg_val |= SI5351_CLKIN_DIV_1;

#ifdef DIVIDE_XTAL_FREQ_IF_NEEDED
		/* Divide down if greater than 30 MHz */

		if(ref_osc_freq > 30000000UL && ref_osc_freq <= 60000000UL)
		{
			xtal_freq /= 2;
			reg_val |= SI5351_CLKIN_DIV_2;
		}
		else if(ref_osc_freq > 60000000UL && ref_osc_freq <= 100000000UL)
		{
			xtal_freq /= 4;
			reg_val |= SI5351_CLKIN_DIV_4;
		}

#endif /* #ifndef DIVIDE_XTAL_FREQ_IF_NEEDED */

		data[0] = reg_val;
		err |= si5351_write_bulk(SI5351_PLL_INPUT_SOURCE, data, 1);
	}

	g_si5351_initialized = !err;

	return err;
}

/**
 * Program one Si5351 output clock to the requested frequency.
 *
 * CLK0 always uses PLLA. CLK1 and CLK2 share PLLB, so the first configured
 * PLLB-backed clock establishes the VCO unless the caller already fixed it
 * with si5351_set_vcoB_freq().
 *
 * @param freq_Fout Requested output frequency in Hz.
 * @param clk Output clock to configure.
 * @param clocksOff true to keep the enabled outputs disabled after programming.
 * @return true on failure, false on success.
 */
bool si5351_set_freq(Frequency_Hz freq_Fout, Si5351_clock clk, bool clocksOff)
{
	Union_si5351_regs ms_reg;
	Frequency_Hz freq_VCO = 0;
	Si5351_pll target_pll;
	uint8_t clock_ctrl_addr;
	uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;
	bool int_mode = false;
	bool div_by_4 = false;
	uint8_t data[2];

#ifdef DEBUGGING_ONLY
	uint32_t div = 0;
#endif
	if(!g_si5351_initialized)
		return (true);

#ifdef DO_BOUNDS_CHECKING
	if(freq_Fout < SI5351_CLKOUT_MIN_FREQ)
	{
		return (true);
	}

	if(freq_Fout > SI5351_CLKOUT_MAX_FREQ)
	{
		return (true);
	}
#endif

#ifdef SUPPORT_FOUT_BELOW_1024KHZ
	/* Select the proper R div value used for Fout frequencies below 1.024 MHz */
	r_div = select_r_div(&freq_Fout);
#endif

#ifdef PREVENT_UNACHIEVABLE_FREQUENCIES
	/* Prevent unachievable frequencies from being entered. The Si5351 will accept these, but some may result */
	/* in no clock output. */
	if(freq_Fout > 999999)
	{
		freq_Fout /= 100;
		freq_Fout *= 100;
	}
#endif

	/* CLK1 and CLK2 share PLLB, so only the first configured one gets to choose its VCO. */
	switch(clk)
	{
		case SI5351_CLK0:
		{
			enabledClocksMask |= 0x01;
			clock_ctrl_addr = 16;

			/* Block 1: Disable Outputs */
			/* Set CLKx_DIS high; Reg. 3 = 0xFF */
			// 				data[0] = ~enabledClocksMask | 0xF9;
			/*			si5351_write_bulk(0x03, data, 1); // only disable CLK0 */

			target_pll = SI5351_PLLA;
			clock_out[SI5351_CLK0] = freq_Fout; /* store the value for reference */
		}
		break;

		case SI5351_CLK1:
		{
			/* No checking is performed to ensure that PLLB is not unavailable due to other output being < 1.024 MHz or >= 112.5 MHz */
			/* User must ensure the clock design is valid before setting clocks */

			enabledClocksMask |= 0x02;
			clock_ctrl_addr = 17;

			/* Block 1: Disable Outputs */
			/* Set CLKx_DIS high; Reg. 3 = 0xFF */
			data[0] = ~enabledClocksMask | 0xFA;
			if(si5351_write_bulk(0x03, data, 1)) /* only disable CLK1 */
			{
				return true;
			}

			target_pll = SI5351_PLLB;
			clock_out[SI5351_CLK1] = freq_Fout; /* store the value for reference */
		}
		break;

		case SI5351_CLK2:
		{
			/* No checking is performed to ensure that PLLB is not unavailable due to other output being < 1.024 MHz or >= 112.5 MHz */
			/* User must ensure the clock design is valid before setting clocks */

			enabledClocksMask |= 0x04;
			clock_ctrl_addr = 18;

			/* Block 1: Disable Outputs */
			/* Set CLKx_DIS high; Reg. 3 = 0xFF */
			data[0] = ~enabledClocksMask | 0xFC; /* only disable CLK2 */
			if(si5351_write_bulk(0x03, data, 1)) /* only disable CLK1 */
			{
				return true;
			}

			target_pll = SI5351_PLLB;
			clock_out[SI5351_CLK2] = freq_Fout; /* store the value for reference */
		}
		break;

		default:
		{
			return (true);
		}
		break;
	}

		/* http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351-B.pdf */
		/* Page 19 */
		/* I2C Programming Procedure */
		/* */

		/* Block 1 handled above */

		/* Block 2: */
		/* Power down all output drivers */
		/* Reg. 16, 17, 18, 19, 20, 21, 22, 23 = 0x80 */
		//  	data[0] = 0xCC; /* only disable CLK2 */
		/*	si5351_write_bulk(clock_ctrl_addr, data, 1); // power down only clock being set, leaving that clock configured as follows: */
		/*   o Drive strength = 2 mA */
		/*   o Input source = multisynth */
		/*   o Output clock not inverted */
		/*   o PLLA is multisynth source */
		/*   o Integer mode set */
		/*   o Clock powered down */
		/* If any of the above settings is incorrect, it needs to be set correctly in the code that follows. */

		/* Block 3: */
		/* Set interrupt masks */
		/* Ignore this for Si5153A */

		/* Block 4: */
		/* Write new configuration to device using */
		/* the contents of the register map */
		/* generated in software above. This */
		/* step also needs to power up the output drivers */
		/* (Registers 15-92 and 149-170 and 183) */

#ifdef DEBUGGING_ONLY
	Frequency_Hz freq_VCO_calc;
	Frequency_Hz fout_calc;
	int32_t f_err;
#endif

	if((target_pll == SI5351_PLLA) || !freqVCOB)
	{
#ifdef DEBUGGING_ONLY
		freq_VCO = multisynth_calc(freq_Fout, &ms_reg, &int_mode, &div_by_4, &div);
#else
		freq_VCO = multisynth_calc(freq_Fout, &ms_reg, &int_mode, &div_by_4);
#endif
	}
	else
	{
#ifdef DEBUGGING_ONLY
		fout_calc = freq_Fout; /* save the intended output frequency */
#endif
		freq_Fout = multisynth_estimate(freq_Fout, &ms_reg, &int_mode, &div_by_4);
	}

	/* Set multisynth registers (MS must be set before PLL) */
	if(set_multisynth_registers_source(clk, target_pll))
	{
		return (true);
	}

	if(set_multisynth_registers(clk, ms_reg, int_mode, r_div, div_by_4))
	{
		return (true);
	}

	/* Set PLL if necessary */
#ifdef DEBUGGING_ONLY
	if(freq_VCO)
	{
		freq_VCO_calc = set_pll(freq_VCO, target_pll);
		fout_calc = freq_VCO_calc / div;
	}

	f_err = freq_Fout - fout_calc;
#else
	if(freq_VCO)
	{
		set_pll(freq_VCO, target_pll);
	}
#endif

	/* Block 5: */
	/* Apply PLLA or PLLB soft reset */
	/* Reg. 177 = 0xAC */
	/*	pll_reset(target_pll); */

	/* Block 6: */
	/* Enable desired outputs */
	/* (see Register 3) */
	if(clocksOff)
	{
		data[0] = enabledClocksMask;
		if(si5351_write_bulk(0x03, data, 1)) /* disable clock(s) in use */
		{
			return true;
		}
	}
	else
	{
		data[0] = ~enabledClocksMask;
		if(si5351_write_bulk(0x03, data, 1)) /* only enable clock(s) in use */
		{
			return true;
		}
	}

	/* power up the clock */
	if(target_pll == SI5351_PLLA)
	{
		data[0] = 0x4C;
		if(si5351_write_bulk(clock_ctrl_addr, data, 1)) /* power up only clock being set, leaving that clock configured as follows: */
		{
			return true;
		}
		/*   o Drive strength = 2 mA */
		/*   o Input source = multisynth */
		/*   o Output clock not inverted */
		/*   o PLLA is multisynth source */
		/*   o Integer mode set */
		/*   o Clock powered up */
	}
	else
	{
		if(int_mode)
		{
			data[0] = 0x6C;
			if(si5351_write_bulk(clock_ctrl_addr, data, 1)) /* power up only clock being set, leaving that clock configured as follows: */
			{
				return true;
			}
			/*   o Drive strength = 2 mA */
			/*   o Input source = multisynth */
			/*   o Output clock not inverted */
			/*   o PLLB is multisynth source */
			/*   o Integer mode set */
			/*   o Clock powered up */
		}
		else
		{
			data[0] = 0x2C;
			if(si5351_write_bulk(clock_ctrl_addr, data, 1)) /* power up only clock being set, leaving that clock configured as follows: */
			{
				return true;
			}
			/*   o Drive strength = 2 mA */
			/*   o Input source = multisynth */
			/*   o Output clock not inverted */
			/*   o PLLB is multisynth source */
			/*   o Integer mode cleared */
			/*   o Clock powered up */
		}

		if(freq_VCO)
		{
			freqVCOB = freq_VCO;
		}
	}

	return (false);
}

/**
 * Return the last frequency requested for a clock output.
 *
 * This is the library's cached value, not a hardware readback from the Si5351.
 *
 * @param clock Output clock to query.
 * @return Cached frequency in Hz.
 */
Frequency_Hz si5351_get_frequency(Si5351_clock clock)
{
	return (clock_out[clock]);
}

/**
 * Enable or disable one Si5351 output through the output-enable register.
 *
 * @param clk Output clock to update.
 * @param enable true to enable the output, false to disable it.
 * @return true on success, false on I2C failure.
 */
bool si5351_clock_enable(Si5351_clock clk, bool enable)
{
	uint8_t reg_val;
	uint8_t data[2];

	if(si5351_read_bulk(SI5351_OUTPUT_ENABLE_CTRL, data, 1))
	{
		return false;
	}

	reg_val = data[0];

	if(enable)
	{
		reg_val &= ~(1 << (uint8_t)clk);
	}
	else
	{
		reg_val |= (1 << (uint8_t)clk);
	}

	data[0] = reg_val;
	if(si5351_write_bulk(SI5351_OUTPUT_ENABLE_CTRL, data, 1))
		return false;

	return true;
}

/**
 * Set the configured output drive strength for one clock pin.
 *
 * @param clk Output clock to update.
 * @param drive Requested output drive level.
 * @return true on success, false on I2C failure.
 */
bool si5351_drive_strength(Si5351_clock clk, Si5351_drive drive)
{
	uint8_t reg_val;
	uint8_t data[2];
	const uint8_t mask = 0x03;

	if(si5351_read_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
	{
		return false;
	}

	reg_val = data[0];

	switch(drive)
	{
		case SI5351_DRIVE_2MA:
		{
			reg_val &= ~(mask);
			reg_val |= 0x00;
		}
		break;

		case SI5351_DRIVE_4MA:
		{
			reg_val &= ~(mask);
			reg_val |= 0x01;
		}
		break;

		case SI5351_DRIVE_6MA:
		{
			reg_val &= ~(mask);
			reg_val |= 0x02;
		}
		break;

		case SI5351_DRIVE_8MA:
		{
			reg_val &= ~(mask);
			reg_val |= 0x03;
		}
		break;

		default:
		{
		}
		break;
	}

	data[0] = reg_val;
	if(si5351_write_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
		return false;

	return true;
}

/**
 * Write the raw phase-offset register for one output clock.
 *
 * @param clk Output clock to update.
 * @param phase Raw 7-bit phase-offset value.
 * @return true on success, false on I2C failure.
 */
bool si5351_set_phase(Si5351_clock clk, uint8_t phase)
{
	uint8_t reg_val;
	uint8_t data[2];
	const uint8_t mask = 0x7F;

	reg_val = phase & mask;
	data[0] = reg_val;
	if(si5351_write_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
		return false;

	return true;
}

/**
 * Read the raw phase-offset register for one output clock.
 *
 * @param clk Output clock to query.
 * @param phase Output pointer for the returned register value.
 * @return true on success, false on I2C failure.
 */
bool si5351_get_phase(Si5351_clock clk, uint8_t *phase)
{
	uint8_t data[2];

	if(si5351_read_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
	{
		return false;
	}

	if(phase)
		*phase = data[0];

	return true;
}

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */
/* Private functions */
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

/**
 * Apply a soft reset to one or both PLLs.
 *
 * @param target_pll PLL selection mask indicating PLLA, PLLB, or both.
 */
void pll_reset(Si5351_pll target_pll)
{
	uint8_t data[1];

	if(target_pll & SI5351_PLLA)
	{
		data[0] = SI5351_PLL_RESET_A;
		si5351_write_bulk(SI5351_PLL_RESET, data, 1);
	}

	if(target_pll & SI5351_PLLB)
	{
		data[0] = SI5351_PLL_RESET_B;
		si5351_write_bulk(SI5351_PLL_RESET, data, 1);
	}
}

/**
 * Choose the output R-divider needed for very low frequencies.
 *
 * The Si5351 multisynth math is performed on an internally scaled-up frequency.
 * This helper multiplies `*freq` accordingly and returns the divider setting
 * that must later be programmed into the output path.
 *
 * @param freq In/out frequency value in Hz.
 * @return Encoded R-divider setting for the target output.
 */
#ifdef SUPPORT_FOUT_BELOW_1024KHZ
uint8_t select_r_div(Frequency_Hz *freq)
{
	uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

	Frequency_Hz max_freq = SI5351_CLKOUT_MIN_FREQ * 128;

	if(*freq < max_freq)
	{
		/* Choose the correct R divider for output frequencies between 8 kHz and 1.024 MHz */
		if((*freq >= SI5351_CLKOUT_MIN_FREQ) && (*freq < SI5351_CLKOUT_MIN_FREQ * 2))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_128;
			*freq *= 128UL;
		}
		else if((*freq >= SI5351_CLKOUT_MIN_FREQ * 2) && (*freq < SI5351_CLKOUT_MIN_FREQ * 4))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_64;
			*freq *= 64UL;
		}
		else if((*freq >= SI5351_CLKOUT_MIN_FREQ * 4) && (*freq < SI5351_CLKOUT_MIN_FREQ * 8))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_32;
			*freq *= 32UL;
		}
		else if((*freq >= SI5351_CLKOUT_MIN_FREQ * 8) && (*freq < SI5351_CLKOUT_MIN_FREQ * 16))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_16;
			*freq *= 16UL;
		}
		else if((*freq >= SI5351_CLKOUT_MIN_FREQ * 16) && (*freq < SI5351_CLKOUT_MIN_FREQ * 32))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_8;
			*freq *= 8UL;
		}
		else if((*freq >= SI5351_CLKOUT_MIN_FREQ * 32) && (*freq < SI5351_CLKOUT_MIN_FREQ * 64))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_4;
			*freq *= 4UL;
		}
		else if((*freq >= SI5351_CLKOUT_MIN_FREQ * 64) && (*freq < SI5351_CLKOUT_MIN_FREQ * 128))
		{
			r_div = SI5351_OUTPUT_CLK_DIV_2;
			*freq *= 2UL;
		}
	}

	return (r_div);
}
#endif /*#ifdef SUPPORT_FOUT_BELOW_1024KHZ */

/**
 * Program PLLA or PLLB for the requested VCO frequency.
 *
 * The helper derives the packed P1/P2/P3 register values and writes them to
 * the correct PLL register block.
 *
 * @param freq_VCO Desired PLL VCO frequency in Hz.
 * @param target_pll PLL to program.
 * @return true on failure, false on success.
 */
#ifdef DEBUGGING_ONLY
uint32_t set_pll(Frequency_Hz freq_VCO, Si5351_pll target_pll)
#else
bool set_pll(Frequency_Hz freq_VCO, Si5351_pll target_pll)
#endif
{
	Union_si5351_regs pll_reg;
	uint8_t params[10];

	/* Output Multisynth Settings (Synthesis Stage 2) */
#ifdef DEBUGGING_ONLY
	Frequency_Hz result = pll_calc(freq_VCO, &pll_reg, g_si5351_ref_correction);
	Frequency_Hz pll_error = freq_VCO - result;
#else
#ifdef APPLY_XTAL_CALIBRATION_VALUE
	if(pll_calc(freq_VCO, &pll_reg, g_si5351_ref_correction))
	{
		return (true);
	}
#else
	if(pll_calc(freq_VCO, &pll_reg))
	{
		return (true);
	}
#endif
#endif

	/* Derive the register values to write */

	/* Prepare an array for parameters to be written to */
	uint8_t i = 0;

	/* Registers 26-27 */
	params[i++] = pll_reg.reg.p3_1;
	params[i++] = pll_reg.reg.p3_0;

	/* Register 28 */
	params[i++] = pll_reg.reg.p1_2 & 0x03;

	/* Registers 29-30 */
	params[i++] = pll_reg.reg.p1_1;
	params[i++] = pll_reg.reg.p1_0;

	/* Register 31 */
	params[i] = pll_reg.reg.p3_2 << 4;
	params[i++] += pll_reg.reg.p2_2 & 0x0F;

	/* Registers 32-33 */
	params[i++] = pll_reg.reg.p2_1;
	params[i++] = pll_reg.reg.p2_0;

	/* Write the parameters */
	if(target_pll == SI5351_PLLA)
	{
		if(si5351_write_bulk(SI5351_PLLA_PARAMETERS, params, i))
		{
			return (true);
		}
	}
	else /* if(target_pll == SI5351_PLLB) */
	{
		if(si5351_write_bulk(SI5351_PLLB_PARAMETERS, params, i))
		{
			return (true);
		}
	}

#ifdef DEBUGGING_ONLY
	return (result);
#else
	return (false);
#endif
}

#ifdef SUPPORT_STATUS_READS
/**
 * Refresh the cached device and interrupt status snapshots from the Si5351.
 */
void si5351_read_status(void)
{
	si5351_read_sys_status(&dev_status);
	si5351_read_int_status(&dev_int_status);
}
#endif

/**
 * Update the reference-oscillator correction value used by PLL calculations.
 *
 * The caller is responsible for persisting this value elsewhere if it should
 * survive reset. The library simply applies it to later frequency calculations.
 *
 * @param corr Signed reference correction term.
 */
void si5351_set_correction(int32_t corr)
{
	g_si5351_ref_correction = corr;
}

/**
 * Return the currently configured reference-oscillator correction value.
 *
 * @return Signed reference correction term.
 */
int32_t si5351_get_correction(void)
{
	return (g_si5351_ref_correction);
}

/**
 * Convert a target VCO frequency into packed Si5351 PLL register fields.
 *
 * The result is expressed in the device's P1/P2/P3 form derived from the
 * crystal reference and optional correction value.
 *
 * @param vco_freq Requested PLL VCO frequency in Hz.
 * @param reg Output register bundle.
 * @return true on failure, false on success.
 */
#ifdef DEBUGGING_ONLY
Frequency_Hz pll_calc(Frequency_Hz vco_freq, Union_si5351_regs *reg, int32_t correction)
#else
#ifdef APPLY_XTAL_CALIBRATION_VALUE
bool pll_calc(Frequency_Hz vco_freq, Union_si5351_regs *reg, int32_t correction)
#else
bool pll_calc(Frequency_Hz vco_freq, Union_si5351_regs *reg)
#endif
#endif
{
#ifdef DEBUGGING_ONLY
	Frequency_Hz result = 0;
#endif
	Frequency_Hz ref_freq = xtal_freq;
	uint32_t a, b, c;

#ifdef APPLY_XTAL_CALIBRATION_VALUE
	/* Factor calibration value into nominal crystal frequency */
	/* Measured in parts-per-billion */
	if(correction)
	{
		ref_freq += (int32_t)((((((int64_t)correction) << 31) / 1000000000LL) * ref_freq) >> 31);
	}
#endif

#ifdef DO_BOUNDS_CHECKING
	if(vco_freq < SI5351_PLL_VCO_MIN)
	{
		return (true);
	}
	if(vco_freq > SI5351_PLL_VCO_MAX)
	{
		return (true);
	}
#endif

	/* Determine integer part of feedback equation */
	a = vco_freq / ref_freq;

#ifdef DO_BOUNDS_CHECKING
	if(a < SI5351_PLL_A_MIN)
	{
		return (true);
	}
	if(a > SI5351_PLL_A_MAX)
	{
		return (true);
	}
#endif

	/* Find best approximation for b/c = fVCO mod fIN */
	b = vco_freq % ref_freq;
	c = ref_freq;

	reduce_by_gcd(&b, &c);

	uint32_t bx128 = b << 7;
	uint32_t bx128overc = bx128 / c;
	reg->ms.p1 = (uint32_t)((a << 7) + bx128overc) - 512; /* 128 * a + floor((128 * b) / c) - 512 */
	reg->ms.p2 = (uint32_t)bx128 - (c * bx128overc);      /* 128 * b - c * floor((128 * b) / c) */
	reg->ms.p3 = c;

#ifdef DEBUGGING_ONLY

	/* Recalculate frequency as fIN * (a + b/c) */
	if(a)
	{
		uint64_t temp = (uint64_t)ref_freq * (uint64_t)b;
		temp /= c;
		temp += ref_freq * a;
		result = temp;
	}

	return (result);

#else

	return (false);

#endif
}

/**
 * Reduce a fraction in place by dividing numerator and denominator by their GCD.
 *
 * @param m In/out numerator.
 * @param n In/out denominator.
 */
void reduce_by_gcd(uint32_t *m, uint32_t *n)
{
	uint32_t r;
	uint32_t b = *m;
	uint32_t c = *n;

	if(!b || !c)
	{
		return;
	}

	for(r = b % c; r; b = c, c = r, r = b % c)
	{
		;
	}

	if(c > 1)
	{
		*m /= c;
		*n /= c;
	}

	return;
}

/**
 * Choose a PLL VCO frequency and multisynth settings for a requested output.
 *
 * This implementation intentionally restricts itself to even integer divider
 * solutions, which keeps the generated clocks simple and predictable at the
 * cost of excluding some otherwise legal Si5351 fractional combinations.
 *
 * @param freq_Fout Requested output frequency in Hz.
 * @param reg Output register bundle for the selected multisynth settings.
 * @param int_mode Output flag indicating whether integer mode should be enabled.
 * @param divBy4 Output flag indicating whether divide-by-4 mode is required.
 * @return Selected PLL VCO frequency in Hz, or 0 on failure.
 */
#ifdef DEBUGGING_ONLY
Frequency_Hz multisynth_calc(Frequency_Hz freq_Fout, Union_si5351_regs *reg, bool *int_mode, bool *divBy4, uint32_t *div)
#else
Frequency_Hz multisynth_calc(Frequency_Hz freq_Fout, Union_si5351_regs *reg, bool *int_mode, bool *divBy4)
#endif
{
	uint32_t a = 0;
	Frequency_Hz freq_VCO = 0;

	*int_mode = true; /* assumed */

#ifdef DO_BOUNDS_CHECKING
	/* Multisynth bounds checking */
	if(freq_Fout > SI5351_MULTISYNTH_MAX_FREQ)
	{
		return (0);
	}
	if(freq_Fout < SI5351_MULTISYNTH_MIN_FREQ)
	{
		return (0);
	}
#endif /* DO_BOUNDS_CHECKING */

	/* All frequencies above 150 MHz must use divide by 4 */
	if(freq_Fout >= SI5351_MULTISYNTH_DIVBY4_FREQ)
	{
		a = 4;
		freq_VCO = a * freq_Fout;
	}
	else
	{
		uint32_t temp;
		uint8_t done = false;
		uint8_t success = false;
		uint8_t count = 0;

		/* Search downward from the maximum VCO until an even integer divider is found. */
		while(!done)
		{
			temp = SI5351_PLL_VCO_MAX - (count * freq_Fout); /* SI5351_PLL_VCO_MAX assumed even */
			count += 2;

			if(temp >= SI5351_PLL_VCO_MIN)
			{
				temp /= freq_Fout;

				if(temp >= 4) /* accepts only even integers of 4 or greater */
				{
					done = true;
					success = true;
					a = temp;
				}
			}
			else
			{
				done = true;
			}
		}

		if(success)
		{
			freq_VCO = a * freq_Fout;
		}
	}

	*divBy4 = (a == 4);
	reg->ms.p1 = (uint32_t)(a << 7) - 512; /* 128 * a + floor((128 * b) / c) - 512 */
	reg->ms.p2 = 0;                        /* 128 * b - c * floor((128 * b) / c) */
	reg->ms.p3 = 1;

#ifdef DEBUGGING_ONLY
	*div = a;
#endif

	return (freq_VCO);
}

/**
 * Force PLLB to a caller-selected VCO frequency.
 *
 * This is useful when CLK1 and CLK2 must share a specific precomputed PLLB
 * plan rather than letting the first configured output choose the VCO.
 *
 * @param freq_VCO Desired PLLB VCO frequency in Hz.
 */
void si5351_set_vcoB_freq(Frequency_Hz freq_VCO)
{
	freqVCOB = freq_VCO;
	set_pll(freq_VCO, SI5351_PLLB);
	return;
}

/**
 * Derive multisynth settings for CLK1 or CLK2 from an already-fixed PLLB VCO.
 *
 * The returned frequency may differ slightly from the requested frequency when
 * PLLB cannot realize it exactly with the available divider resolution.
 *
 * @param freq_Fout Requested output frequency in Hz.
 * @param reg Output register bundle for the derived multisynth settings.
 * @param int_mode Output flag indicating whether integer mode should be enabled.
 * @param divBy4 Output flag indicating whether divide-by-4 mode is required.
 * @return Achievable output frequency in Hz, or 0 on failure.
 */
Frequency_Hz multisynth_estimate(Frequency_Hz freq_Fout, Union_si5351_regs *reg, bool *int_mode, bool *divBy4)
{
	uint32_t a, b, c;

#ifdef DO_BOUNDS_CHECKING
	if(freqVCOB < 600000000)
	{
		return (0);
	}

	if(freqVCOB > 900000000)
	{
		return (0);
	}

	if(freq_Fout > SI5351_MULTISYNTH_MAX_FREQ)
	{
		return (0);
	}

	if(freq_Fout < SI5351_MULTISYNTH_MIN_FREQ)
	{
		return (0);
	}
#endif /* DO_BOUNDS_CHECKING */

	/* Determine integer part of feedback equation */
	a = freqVCOB / freq_Fout;
	b = freqVCOB % freq_Fout;
	c = freq_Fout;
	/* Reduce the fraction before packing to avoid overflow and match ClockBuilder math. */
	reduce_by_gcd(&b, &c);

	/* Calculate the approximated output frequency given by fOUT = fvco / (a + b/c) */
	freq_Fout = freqVCOB;
	freq_Fout /= (a * c + b);
	freq_Fout *= c;

	*int_mode = (b == 0) && !(a % 2);
	*divBy4 = (a == 4) && *int_mode;

	/* Calculate parameters */
	if(*divBy4)
	{
		reg->ms.p1 = 0;
		reg->ms.p2 = 0;
		reg->ms.p3 = 1;
	}
	else
	{
		uint32_t bx128 = b << 7;
		uint32_t bx128overc = bx128 / c;
		reg->ms.p1 = (uint32_t)((a << 7) + bx128overc) - 512; /* 128 * a + floor((128 * b) / c) - 512 */
		reg->ms.p2 = (uint32_t)bx128 - (c * bx128overc);      /* 128 * b - c * floor((128 * b) / c) */
		reg->ms.p3 = c;
	}

	return (freq_Fout);
}

/**
 * Write a contiguous block of Si5351 registers over I2C with simple retries.
 *
 * @param regAddr First register address to write.
 * @param data Pointer to the source bytes.
 * @param bytes Number of bytes to write.
 * @return true on failure, false on success.
 */
bool si5351_write_bulk(uint8_t regAddr, uint8_t *data, uint8_t bytes)
{
	uint8_t tries = 5;
	bool fail = true;
	while(--tries && (fail = I2C_0_SendData(SI5351_I2C_SLAVE_ADDR, regAddr, data, bytes) != bytes))
		;
	return (fail);
}

/**
 * Read a contiguous block of Si5351 registers over I2C with simple retries.
 *
 * @param regAddr First register address to read.
 * @param data Destination buffer for the returned bytes.
 * @param bytes Number of bytes to read.
 * @return true on failure, false on success.
 */
bool si5351_read_bulk(uint8_t regAddr, uint8_t *data, uint8_t bytes)
{
	uint8_t tries = 5;
	bool fail = true;
	while(--tries && (fail = I2C_0_GetData(SI5351_I2C_SLAVE_ADDR, regAddr, data, bytes) != bytes))
		;
	return (fail);
}

#ifdef SUPPORT_STATUS_READS
/**
 * Read and decode the current device-status register.
 *
 * @param status Output structure to receive the decoded bits.
 * @return true on failure, false on success.
 */
bool si5351_read_sys_status(Si5351Status *status)
{
	uint8_t reg_val = 0;
	uint8_t data[2];

	if(si5351_read_bulk(SI5351_DEVICE_STATUS, data, 1))
	{
		return (true);
	}

	reg_val = data[0];

	/* Parse the register */
	status->SYS_INIT = (reg_val >> 7) & 0x01;
	status->LOL_B = (reg_val >> 6) & 0x01;
	status->LOL_A = (reg_val >> 5) & 0x01;
	status->LOS = (reg_val >> 4) & 0x01;
	status->REVID = reg_val & 0x03;

	return (false);
}

/**
 * Read and decode the interrupt-status register.
 *
 * @param status Output structure to receive the decoded bits.
 * @return true on failure, false on success.
 */
bool si5351_read_int_status(Si5351IntStatus *status)
{
	uint8_t reg_val = 0;
	uint8_t data[2];

	if(si5351_read_bulk(SI5351_DEVICE_STATUS, data, 1))
	{
		return (true);
	}

	reg_val = data[0];

	/* Parse the register */
	status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
	status->LOL_B_STKY = (reg_val >> 6) & 0x01;
	status->LOL_A_STKY = (reg_val >> 5) & 0x01;
	status->LOS_STKY = (reg_val >> 4) & 0x01;

	return (false);
}
#endif /* #ifdef SUPPORT_STATUS_READS */

/**
 * Select whether a clock's multisynth is fed from PLLA or PLLB.
 *
 * @param clk Output clock to update.
 * @param pll PLL to select as the multisynth source.
 * @return true on failure, false on success.
 */
bool set_multisynth_registers_source(Si5351_clock clk, Si5351_pll pll)
{
	uint8_t reg_val;
	uint8_t data[2];

	if(si5351_read_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
	{
		return (true);
	}

	reg_val = data[0];

	if(pll == SI5351_PLLA)
	{
		reg_val &= ~(SI5351_CLK_PLL_SELECT);
	}
	else if(pll == SI5351_PLLB)
	{
		reg_val |= SI5351_CLK_PLL_SELECT;
	}

	data[0] = reg_val;
	if(si5351_write_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
	{
		return (true);
	}

	return (false);
}

/**
 * Write the packed multisynth register block for one output clock.
 *
 * The helper writes P1/P2/P3, then applies the associated integer-mode and
 * output-divider settings stored in separate control bits.
 *
 * @param clk Output clock to configure.
 * @param ms_reg Packed multisynth register values.
 * @param int_mode true to enable integer mode.
 * @param r_div Encoded output R-divider setting.
 * @param div_by_4 true to enable the dedicated divide-by-4 mode.
 * @return true on failure, false on success.
 */
bool set_multisynth_registers(Si5351_clock clk, Union_si5351_regs ms_reg, bool int_mode, uint8_t r_div, bool div_by_4)
{
	uint8_t params[11];
	uint8_t i = 0;
	uint8_t reg_val;
	uint8_t data[2];

	/* Registers 42-43 for CLK0; 50-51 for CLK1 */
	params[i++] = ms_reg.reg.p3_1;
	params[i++] = ms_reg.reg.p3_0;

	/* Register 44 for CLK0; 52 for CLK1 */
	if(si5351_read_bulk((SI5351_CLK0_PARAMETERS + 2) + (clk * 8), data, 1))
	{
		return (true);
	}

	reg_val = data[0];

	reg_val &= 0xFC; /*~(0x03); */
	params[i++] = reg_val | (ms_reg.reg.p1_2 & 0x03);

	/* Registers 45-46 for CLK0 */
	params[i++] = ms_reg.reg.p1_1;
	params[i++] = ms_reg.reg.p1_0;

	/* Register 47 for CLK0 */
	params[i] = (ms_reg.reg.p3_2 << 4);
	params[i++] += (ms_reg.reg.p2_2 & 0x0F);

	/* Registers 48-49 for CLK0 */
	params[i++] = ms_reg.reg.p2_1;
	params[i++] = ms_reg.reg.p2_0;

	/* Write the parameters */
	switch(clk)
	{
		case SI5351_CLK0:
		{
			if(si5351_write_bulk(SI5351_CLK0_PARAMETERS, params, i))
			{
				return (true);
			}
		}
		break;

		case SI5351_CLK1:
		{
			if(si5351_write_bulk(SI5351_CLK1_PARAMETERS, params, i))
			{
				return (true);
			}
		}
		break;

		case SI5351_CLK2:
		{
			if(si5351_write_bulk(SI5351_CLK2_PARAMETERS, params, i))
			{
				return (true);
			}
		}
		break;

		default:
		{
		}
		break;
	}

	if(set_integer_mode(clk, int_mode))
	{
		return (true);
	}

	if(ms_div(clk, r_div, div_by_4))
	{
		return (true);
	}

	return (false);
}

/**
 * Enable or disable integer mode for one output clock's multisynth.
 *
 * @param clk Output clock to update.
 * @param enable true to enable integer mode, false to clear it.
 * @return true on failure, false on success.
 */
bool set_integer_mode(Si5351_clock clk, bool enable)
{
	uint8_t reg_val;
	uint8_t data[2];

	if(si5351_read_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1))
	{
		return (true);
	}

	reg_val = data[0];

	if(enable)
	{
		reg_val |= (SI5351_CLK_INTEGER_MODE);
	}
	else
	{
		reg_val &= ~(SI5351_CLK_INTEGER_MODE);
	}

	data[0] = reg_val;
	return (si5351_write_bulk(SI5351_CLK0_CTRL + (uint8_t)clk, data, 1));
}

/**
 * Apply the output divider and divide-by-4 bits for one multisynth.
 *
 * @param clk Output clock to update.
 * @param r_div Encoded R-divider value.
 * @param div_by_4 true to enable divide-by-4 mode.
 * @return true on failure, false on success.
 */
bool ms_div(Si5351_clock clk, uint8_t r_div, bool div_by_4)
{
	uint8_t reg_val, reg_addr;
	uint8_t data[2];

	switch(clk)
	{
		case SI5351_CLK0:
		{
			reg_addr = SI5351_CLK0_PARAMETERS + 2;
		}
		break;

		case SI5351_CLK1:
		{
			reg_addr = SI5351_CLK1_PARAMETERS + 2;
		}
		break;

		case SI5351_CLK2:
		{
			reg_addr = SI5351_CLK2_PARAMETERS + 2;
		}
		break;

		default:
			return (true);
	}

	if(si5351_read_bulk(reg_addr, data, 1))
	{
		return (true);
	}

	reg_val = data[0];

	/* Clear the appropriate bits */
	reg_val &= ~(0x7c);

	if(div_by_4)
	{
		reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
	}
	else
	{
		reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
	}

	reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);

	data[0] = reg_val;
	return (si5351_write_bulk(reg_addr, data, 1));
}

#endif /* #ifdef INCLUDE_SI5351_SUPPORT */
