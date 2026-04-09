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
 *  SOFTWARE.
 */

/*
 * Helpers for writing CCP-protected AVR registers.
 *
 * The AVR128 DA family guards certain control registers behind Configuration
 * Change Protection (CCP). These wrappers provide the small, intention-revealing
 * entry points used by the rest of the firmware to perform those protected
 * writes through the shared assembly helper.
 */



#ifndef CPU_CCP_H
#define CPU_CCP_H

#include <compiler.h>
#include <protected_io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Write one byte to an I/O register protected by the standard CCP key.
 *
 * @param addr Address of the protected I/O register.
 * @param value Value to write after unlocking the register.
 */
static inline void ccp_write_io(void *addr, uint8_t value)
{
	protected_write_io(addr, CCP_IOREG_gc, value);
}

/** @} */

/**
 * Write one byte to an SPM-protected register using the CCP SPM key.
 *
 * @param addr Address of the protected SPM register.
 * @param value Value to write after unlocking the register.
 */
static inline void ccp_write_spm(void *addr, uint8_t value)
{
	protected_write_io(addr, CCP_SPM_gc, value);
}

	/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_CCP_H */
