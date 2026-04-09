/**
 * \file
 *
 * \brief RSTCTRL
 *
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/*
 * Reset-controller helpers for software reset and reset-cause handling.
 *
 * This header wraps the AVR reset controller registers with small inline
 * helpers so the rest of the firmware can trigger a software reset or inspect
 * reset-cause flags without duplicating register details.
 */

/**
 * \defgroup doc_driver_system_rstctrl Reset Controller
 * \ingroup doc_driver_system
 *
 * \section doc_driver_rstctrl_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#ifndef RSTCTRL_INCLUDED
#define RSTCTRL_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <ccp.h>

/**
 * Request a software reset through the reset controller.
 *
 * The SWRR register is CCP-protected, so the helper routes the write through
 * the shared CCP wrapper.
 */
static inline void RSTCTRL_reset(void)
{
	/* SWRR is protected with CCP */
	ccp_write_io((void *)&RSTCTRL.SWRR, 0x1);
}

/**
 * Read the raw reset-cause flags latched by the AVR reset controller.
 *
 * @return Current contents of the reset flag register.
 */
static inline uint8_t RSTCTRL_get_reset_cause(void)
{
	return RSTCTRL.RSTFR;
}

/**
 * Clear all reset-cause flags currently tracked by the AVR reset controller.
 */
static inline void RSTCTRL_clear_reset_cause(void)
{
	RSTCTRL.RSTFR
	    = RSTCTRL_UPDIRF_bm | RSTCTRL_SWRF_bm | RSTCTRL_WDRF_bm | RSTCTRL_EXTRF_bm | RSTCTRL_BORF_bm | RSTCTRL_PORF_bm;
}

#ifdef __cplusplus
}
#endif

#endif /* RSTCTRL_INCLUDED */
