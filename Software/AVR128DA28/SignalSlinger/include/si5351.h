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
 * This module provides a compact AVR-oriented interface to the Si5351A. It
 * owns the I2C register programming, PLL configuration, and multisynth helper
 * math used to derive output clocks from the crystal reference.
 *
 * The library intentionally assumes that higher-level callers are responsible
 * for selecting legal clock plans. This module focuses on translating those
 * plans into Si5351 register updates with minimal code size.
 */

#include "defs.h"

#ifndef SI5351_H_
#define SI5351_H_

//#include <avr/eeprom.h>

#ifdef __cplusplus
extern "C" {
#endif


/********************************************************************************************************************
 *  Notes:
 *      Only one hardware version of the Si5351 device is supported by this library:
 *          Si5351A MSOP10 (3 output clocks, XTAL input)
 *          PLL usage is as follows, change code to modify this behavior:
 *              CLK0 uses PLLA,
 *              CLK1 uses PLLB
 *              CLK2 uses PLLB
 *
 *
 *  CODE EXAMPLES:
 *
 *     Example 1: Set CLK0 to 144.571 MHz
 *        si5351_init(SI5351_CRYSTAL_LOAD_10PF, 0);
 *        si5351_set_freq(144571000, SI5351_CLK0);
 *
 *     Example 2: Set CLK0 to 133.3 MHz, then use PLL VCO of 700 MHz to set CLK2 to 70MHz, and CLK1 to 10.705 MHz
 *        si5351_init(SI5351_CRYSTAL_LOAD_10PF, 0);
 *        si5351_set_freq(133300000, SI5351_CLK0);
 *        si5351_set_vcoB_freq(700000000);
 *        si5351_set_freq(70000000, SI5351_CLK2);
 *        si5351_set_freq(10705000, SI5351_CLK1);
 *
 *  The Si5351 needs only to be initialized once at power up. Initialization leaves all clocks powered down.
 *
 *  Calls to si5351_set_freq() set one output clock only: it must be called once for each clock that will be used.
 *  Unused clocks will remain powered down. Setting the frequency of a clock also enables that clock's output.
 *
 *  si5351_set_vcoB_freq() should only be called once, and it must be called prior to setting CLK1 or CLK2 output
 *  frequencies. Calling si5351_set_vcoB_freq() after CLK1 or CLK2 has been set will change the output frequencies
 *  of both those clocks to a indeterminate values.
 *
 *  The Si5351 integrated circuit places certain limitations on the ranges and combinations of frequencies that be
 *  assigned to each of the three clock outputs. It is the library user's responsibility to be sufficiently familiar
 *  with the Si5351 specification so as to avoid illegal settings.
 *
 ****************************************************************************************************************/

/***************************************************************************************************************
 * These definitions affect compiled code size as well as debugging and error checking functionality.
 *
 * Debug statements should be commented out in production code
 #define DEBUG_WITHOUT_I2C
 #define DEBUGGING_ONLY
 *
 * Enable the following definitions as needed, but check program memory usage.
 *
#define SUPPORT_FOUT_BELOW_1024KHZ
#define DO_BOUNDS_CHECKING
#define DIVIDE_XTAL_FREQ_IF_NEEDED
#define APPLY_XTAL_CALIBRATION_VALUE
#define SUPPORT_STATUS_READS
*/
#define PREVENT_UNACHIEVABLE_FREQUENCIES
/*
 * The following flag is used to disable GCC compiler optimizations in code regions where the optimizer has
 * been found to introduce run-time problems. Comment out the following #define if your compiler does not support
 * the syntax used in this library.
 */
#ifndef SELECTIVELY_DISABLE_OPTIMIZATION
	#define SELECTIVELY_DISABLE_OPTIMIZATION
#endif
/*
 ****************************************************************************************************************/

#define SI5351_XTAL_FREQ                                25000000UL
#define SI5351_PLL_FIXED                                900000000UL

#define SI5351_PLL_VCO_MIN                              600000000UL
#define SI5351_PLL_VCO_MAX                              900000000UL /* This must be defined as an even number to support frequency-setting algorithm */
#define SI5351_MULTISYNTH_MIN_FREQ                      1000000UL
#define SI5351_MULTISYNTH_DIVBY4_FREQ                   150000000UL
#define SI5351_MULTISYNTH_MAX_FREQ                      160000000UL
#define SI5351_MULTISYNTH_SHARE_MAX                     112500000UL
#define SI5351_MULTISYNTH67_MAX_FREQ                    SI5351_MULTISYNTH_DIVBY4_FREQ
#define SI5351_CLKOUT_MIN_FREQ                          8000UL
#define SI5351_CLKOUT_MAX_FREQ                          SI5351_MULTISYNTH_MAX_FREQ
#define SI5351_CLKOUT67_MAX_FREQ                        SI5351_MULTISYNTH67_MAX_FREQ

#define SI5351_PLL_A_MIN                                15
#define SI5351_PLL_A_MAX                                90
#define SI5351_PLL_B_MAX                                (SI5351_PLL_C_MAX - 1)
#define SI5351_PLL_C_MAX                                1048575
#define SI5351_MULTISYNTH_A_MIN                         6
#define SI5351_MULTISYNTH_A_MAX                         1800
#define SI5351_MULTISYNTH67_A_MAX                       254
#define SI5351_MULTISYNTH_B_MAX                         (SI5351_MULTISYNTH_C_MAX - 1)
#define SI5351_MULTISYNTH_C_MAX                         1048575
#define SI5351_MULTISYNTH_P1_MAX                        ((1 << 18) - 1)
#define SI5351_MULTISYNTH_P2_MAX                        ((1 << 20) - 1)
#define SI5351_MULTISYNTH_P3_MAX                        ((1 << 20) - 1)

#define SI5351_DEVICE_STATUS                            0
#define SI5351_INTERRUPT_STATUS                         1
#define SI5351_INTERRUPT_MASK                           2
#define SI5351_STATUS_SYS_INIT                          (1 << 7)
#define SI5351_STATUS_LOL_B                             (1 << 6)
#define SI5351_STATUS_LOL_A                             (1 << 5)
#define SI5351_STATUS_LOS                               (1 << 4)
#define SI5351_OUTPUT_ENABLE_CTRL                       3
#define SI5351_OEB_PIN_ENABLE_CTRL                      9
#define SI5351_PLL_INPUT_SOURCE                         15
#define SI5351_CLKIN_DIV_MASK                           (3 << 6)
#define SI5351_CLKIN_DIV_1                              (0 << 6)
#define SI5351_CLKIN_DIV_2                              (1 << 6)
#define SI5351_CLKIN_DIV_4                              (2 << 6)
#define SI5351_CLKIN_DIV_8                              (3 << 6)
#define SI5351_PLLB_SOURCE                              (1 << 3)
#define SI5351_PLLA_SOURCE                              (1 << 2)

#define SI5351_CLK0_CTRL                                16
#define SI5351_CLK1_CTRL                                17
#define SI5351_CLK2_CTRL                                18
#define SI5351_CLK_POWERDOWN                            (1 << 7)
#define SI5351_CLK_INTEGER_MODE                         (1 << 6)
#define SI5351_CLK_PLL_SELECT                           (1 << 5)
#define SI5351_CLK_INVERT                               (1 << 4)
#define SI5351_CLK_INPUT_MASK                           (3 << 2)
#define SI5351_CLK_INPUT_XTAL                           (0 << 2)
#define SI5351_CLK_INPUT_CLKIN                          (1 << 2)
#define SI5351_CLK_INPUT_MULTISYNTH_0_4                 (2 << 2)
#define SI5351_CLK_INPUT_MULTISYNTH_N                   (3 << 2)
#define SI5351_CLK_DRIVE_STRENGTH_MASK                  (3 << 0)
#define SI5351_CLK_DRIVE_STRENGTH_2MA                   (0 << 0)
#define SI5351_CLK_DRIVE_STRENGTH_4MA                   (1 << 0)
#define SI5351_CLK_DRIVE_STRENGTH_6MA                   (2 << 0)
#define SI5351_CLK_DRIVE_STRENGTH_8MA                   (3 << 0)

#define SI5351_CLK3_0_DISABLE_STATE                     24
#define SI5351_CLK7_4_DISABLE_STATE                     25
#define SI5351_CLK_DISABLE_STATE_MASK                   3
#define SI5351_CLK_DISABLE_STATE_LOW                    0
#define SI5351_CLK_DISABLE_STATE_HIGH                   1
#define SI5351_CLK_DISABLE_STATE_FLOAT                  2
#define SI5351_CLK_DISABLE_STATE_NEVER                  3

#define SI5351_PARAMETERS_LENGTH                        8
#define SI5351_PLLA_PARAMETERS                          26
#define SI5351_PLLB_PARAMETERS                          34
#define SI5351_CLK0_PARAMETERS                          42
#define SI5351_CLK1_PARAMETERS                          50
#define SI5351_CLK2_PARAMETERS                          58
#define SI5351_CLK3_PARAMETERS                          66
#define SI5351_CLK4_PARAMETERS                          74
#define SI5351_CLK5_PARAMETERS                          82
#define SI5351_CLK6_PARAMETERS                          90
#define SI5351_CLK7_PARAMETERS                          91
#define SI5351_CLK6_7_OUTPUT_DIVIDER                    92
#define SI5351_OUTPUT_CLK_DIV_MASK                      (7 << 4)
#define SI5351_OUTPUT_CLK6_DIV_MASK                     (7 << 0)
#define SI5351_OUTPUT_CLK_DIV_SHIFT                     4
#define SI5351_OUTPUT_CLK_DIV6_SHIFT                    0
#define SI5351_OUTPUT_CLK_DIV_1                         0
#define SI5351_OUTPUT_CLK_DIV_2                         1
#define SI5351_OUTPUT_CLK_DIV_4                         2
#define SI5351_OUTPUT_CLK_DIV_8                         3
#define SI5351_OUTPUT_CLK_DIV_16                        4
#define SI5351_OUTPUT_CLK_DIV_32                        5
#define SI5351_OUTPUT_CLK_DIV_64                        6
#define SI5351_OUTPUT_CLK_DIV_128                       7
#define SI5351_OUTPUT_CLK_DIVBY4                        (3 << 2)

#define SI5351_SSC_PARAM0                               149
#define SI5351_SSC_PARAM1                               150
#define SI5351_SSC_PARAM2                               151
#define SI5351_SSC_PARAM3                               152
#define SI5351_SSC_PARAM4                               153
#define SI5351_SSC_PARAM5                               154
#define SI5351_SSC_PARAM6                               155
#define SI5351_SSC_PARAM7                               156
#define SI5351_SSC_PARAM8                               157
#define SI5351_SSC_PARAM9                               158
#define SI5351_SSC_PARAM10                              159
#define SI5351_SSC_PARAM11                              160
#define SI5351_SSC_PARAM12                              161

#define SI5351_VXCO_PARAMETERS_LOW                      162
#define SI5351_VXCO_PARAMETERS_MID                      163
#define SI5351_VXCO_PARAMETERS_HIGH                     164

#define SI5351_CLK0_PHASE_OFFSET                        165
#define SI5351_CLK1_PHASE_OFFSET                        166
#define SI5351_CLK2_PHASE_OFFSET                        167
#define SI5351_CLK3_PHASE_OFFSET                        168
#define SI5351_CLK4_PHASE_OFFSET                        169
#define SI5351_CLK5_PHASE_OFFSET                        170

#define SI5351_PLL_RESET                                177
#define SI5351_PLL_RESET_B                              (1 << 7)
#define SI5351_PLL_RESET_A                              (1 << 5)

#define SI5351_CRYSTAL_LOAD                             183
#define SI5351_CRYSTAL_LOAD_MASK                        (3 << 6)

#define SI5351_FANOUT_ENABLE                            187
#define SI5351_CLKIN_ENABLE                             (1 << 7)
#define SI5351_XTAL_ENABLE                              (1 << 6)
#define SI5351_MULTISYNTH_ENABLE                        (1 << 4)

/* Macro definitions */

#define RFRAC_DENOM                                     ((1L << 20) - 1)

#ifdef DEBUGGING_ONLY

#define NUM_REGS_MAX 100

	typedef struct Reg_Data
	{
		unsigned char Reg_Addr;
		unsigned char Reg_Val;
	} Reg_Data;

#endif  /* #ifdef DEBUGGING_ONLY */


/********************************************************************************************************************
 * Typedefs
 ******************************************************************************************************************** */


#ifndef Frequency_Hz
	typedef uint32_t Frequency_Hz;
#endif

#define FREQUENCY_NOT_SPECIFIED 0

	typedef enum si5351_xtal_load_pF
	{
		SI5351_CRYSTAL_LOAD_6PF = (uint8_t)(1 << 6),
		SI5351_CRYSTAL_LOAD_8PF = (uint8_t)(2 << 6),
		SI5351_CRYSTAL_LOAD_10PF = (uint8_t)(3 << 6)
	} Si5351_Xtal_load_pF;

	typedef enum si5351_clock
	{
		SI5351_CLK0 = 0,
		SI5351_CLK1 = 1,
		SI5351_CLK2 = 2,
		NUMBER_OF_SI5351_CLOCKS,
		SI5351_NO_CLOCK
	} Si5351_clock;

	typedef enum si5351_clk_enable
	{
		SI5351_CLK_DISABLED = 0,
		SI5351_CLK_ENABLED,
		SI5351_ENABLE_NOT_SPECIFIED
	} Si5351_clock_enable;

	typedef enum si5351_pll
	{
		SI5351_PLLA = 1,
		SI5351_PLLB = 2,
		SI5351_PLLA_B = 3
	} Si5351_pll;

	typedef enum si5351_drive
	{
		SI5351_DRIVE_2MA = 0,
		SI5351_DRIVE_4MA,
		SI5351_DRIVE_6MA,
		SI5351_DRIVE_8MA,
		SI5351_DRIVE_NOT_SPECIFIED
	} Si5351_drive;

	typedef struct si5351RegSet
	{
		uint32_t p1;
		uint32_t p2;
		uint32_t p3;
	} Si5351RegSet;

	typedef struct si5351RegBytes
	{
		uint8_t p1_0;
		uint8_t p1_1;
		uint8_t p1_2;
		uint8_t p1_3;
		uint8_t p2_0;
		uint8_t p2_1;
		uint8_t p2_2;
		uint8_t p2_3;
		uint8_t p3_0;
		uint8_t p3_1;
		uint8_t p3_2;
		uint8_t p3_3;
	} Si5351RegBytes;

	typedef union u_si5351_regs
	{
		Si5351RegBytes reg;
		Si5351RegSet ms;
	} Union_si5351_regs;

	typedef struct si5351Status
	{
		uint8_t SYS_INIT;
		uint8_t LOL_B;
		uint8_t LOL_A;
		uint8_t LOS;
		uint8_t REVID;
	} Si5351Status;

	typedef struct si5351IntStatus
	{
		uint8_t SYS_INIT_STKY;
		uint8_t LOL_B_STKY;
		uint8_t LOL_A_STKY;
		uint8_t LOS_STKY;
	} Si5351IntStatus;

	typedef struct
	{
		Frequency_Hz freq;
		bool enabled;
		Si5351_drive drive;
	} ClockPin;

/*
 *  SI5351Device type:
 *  Defines all the parameters that a library user might want to keep track of. Usage:
 *
 *       SI5351 clock_chip;
 *       clock_chip.clk0 = 145520000;
 *
 *  TODO: The library might make use of this for certain function calls, such as initialization,
 *  to support efficient passing of all needed parameters.
 */
	typedef struct
	{
		ClockPin clk0;
		ClockPin clk1;
		ClockPin clk2;
		Si5351_Xtal_load_pF xtal_load;
		int32_t correction;
		Frequency_Hz vcoB;
	} SI5351Device;

/********************************************************************************************************************
 * Public Function Prototypes
 ******************************************************************************************************************** */
#ifdef INCLUDE_SI5351_SUPPORT

/**
 * Shut down the I2C peripheral used to communicate with the Si5351.
 *
 * This does not change any programmed Si5351 clock state; it only releases the
 * local AVR-side communications interface.
 */
void si5351_shutdown_comms(void);

/**
 * Initialize the I2C peripheral used to communicate with the Si5351.
 */
void si5351_start_comms(void);

/**
 * Initialize the Si5351 and apply baseline device configuration.
 *
 * Initialization resets the cached library state, disables all outputs, powers
 * down the device clocks, and programs the crystal load capacitance. When
 * `ref_osc_freq` is zero, the default crystal frequency is used.
 *
 * @param xtal_load_c Crystal load capacitance setting.
 * @param ref_osc_freq Reference crystal or clock input frequency in Hz, or 0 for default.
 * @return true on I2C/configuration failure, false on success.
 */
bool si5351_init(Si5351_Xtal_load_pF, Frequency_Hz);

/**
 * Program one Si5351 output clock.
 *
 * CLK0 is generated from PLLA. CLK1 and CLK2 share PLLB, so the first one
 * configured establishes PLLB's VCO frequency unless the caller has already
 * fixed it with si5351_set_vcoB_freq().
 *
 * @param freq_Fout Desired output frequency in Hz.
 * @param clk Output clock to configure.
 * @param clocksOff true to leave the enabled outputs disabled after programming.
 * @return true on failure, false on success.
 */
bool si5351_set_freq(Frequency_Hz, Si5351_clock, bool clocksOff);

/**
 * Report the last frequency value requested for a clock output.
 *
 * The returned value is the library's cached setting, not a readback from the
 * Si5351 hardware registers.
 *
 * @param clock Output clock to query.
 * @return Cached output frequency in Hz.
 */
Frequency_Hz si5351_get_frequency(Si5351_clock clock);

/**
 * Enable or disable one output through the Si5351 output-enable register.
 *
 * @param clk Output clock to update.
 * @param enable true to enable the clock output, false to disable it.
 * @return true on success, false on I2C failure.
 */
bool si5351_clock_enable(Si5351_clock, bool);

/**
 * Set the configured output drive strength for one clock pin.
 *
 * @param clk Output clock to update.
 * @param drive Requested output drive level.
 * @return true on success, false on I2C failure.
 */
bool si5351_drive_strength(Si5351_clock, Si5351_drive);

/**
 * Program the raw phase-offset register for one output clock.
 *
 * @param clk Output clock to update.
 * @param phase Raw 7-bit phase offset value to write.
 * @return true on success, false on I2C failure.
 */
bool si5351_set_phase(Si5351_clock clk, uint8_t phase);

/**
 * Read the raw phase-offset register for one output clock.
 *
 * @param clk Output clock to query.
 * @param phase Output pointer for the returned 8-bit register value.
 * @return true on success, false on I2C failure.
 */
bool si5351_get_phase(Si5351_clock clk, uint8_t* phase);

/**
 * Store the crystal correction factor used by the PLL calculations.
 *
 * The value is a signed parts-per-billion style correction term used only by
 * this library's internal math. Persisting it in EEPROM is the caller's job.
 *
 * @param correction Signed reference-frequency correction value.
 */
void si5351_set_correction(int32_t);

/**
 * Return the currently configured crystal correction factor.
 *
 * @return Signed reference-frequency correction value.
 */
int32_t si5351_get_correction(void);

/**
 * Force PLLB to a specific VCO frequency before configuring CLK1 or CLK2.
 *
 * This is useful when the caller wants a particular PLLB clock plan rather
 * than letting the first configured PLLB-backed output choose the VCO.
 *
 * @param freq_VCO Desired PLLB VCO frequency in Hz.
 */
void si5351_set_vcoB_freq(Frequency_Hz);

/**
 * Soft-reset one or both PLLs.
 *
 * @param target_pll PLL selection mask indicating PLLA, PLLB, or both.
 */
void pll_reset(Si5351_pll);

#ifdef SUPPORT_STATUS_READS
/**
 * Refresh the cached device and interrupt status structures from the Si5351.
 */
void si5351_read_status(void);
#endif  /* #ifdef SUPPORT_STATUS_READS */

#ifdef DEBUGGING_ONLY

/**
 * Compare the library's expected register map against the live device state.
 *
 * Debug-only helper used while validating the tuning algorithm.
 */
bool compare_with_register_map(void);

/**
 * Dump Si5351 register contents for debugging.
 */
void dump_registers(void);

#endif  /* #ifdef DEBUGGING_ONLY */

#endif  /* #ifdef INCLUDE_SI5351_SUPPORT */

#ifdef __cplusplus
}
#endif

#endif  /* SI5351_H_ */
