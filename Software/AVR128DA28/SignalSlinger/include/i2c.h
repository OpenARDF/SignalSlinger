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
 * Low-level I2C/TWI host helpers.
 *
 * This module contains support functions for:
 * - initializing the primary TWI host peripheral
 * - sending register-addressed writes and reads
 * - ending or shutting down an I2C session cleanly
 *
 * Device-specific register maps and higher-level retry policy belong elsewhere.
 */

#include "defs.h"

#ifndef I2C_H_INCLUDED
#define	I2C_H_INCLUDED

//#include <xc.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
	#endif /* __cplusplus */

	/**
	 * Initialize the primary I2C/TWI host interface.
	 */
	void I2C_0_Init(void);

	/**
	 * Write one or more bytes to a register-addressed I2C peripheral.
	 *
	 * @param slaveAddr 7-bit slave address in write form.
	 * @param regAddr Register address to send before the payload.
	 * @param pData Pointer to the bytes to send.
	 * @param len Number of payload bytes to send.
	 * @return Number of payload bytes acknowledged, or `0xFF` if the slave address is NACKed.
	 */
	uint8_t I2C_0_SendData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint8_t len);

	/**
	 * Read one or more bytes from a register-addressed I2C peripheral.
	 *
	 * @param slaveAddr 7-bit slave address in write form.
	 * @param regAddr Register address to read from.
	 * @param pData Destination buffer for received bytes.
	 * @param len Number of bytes to receive.
	 * @return Number of bytes received, or `0xFF` if the slave address is NACKed.
	 */
	uint8_t I2C_0_GetData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint8_t len);

	/**
	 * Send a STOP condition to end the current I2C transaction.
	 */
	void I2C_0_EndSession(void);

	/**
	 * Disable the primary I2C/TWI host interface.
	 */
	void I2C_0_Shutdown(void);

// 	void    I2C_1_Init(void);
// 	uint8_t I2C_1_SendData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint8_t len); // returns how many bytes have been sent, -1 means NACK at address
// 	uint8_t I2C_1_GetData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint8_t len); // returns how many bytes have been received, -1 means NACK at address
// 	void    I2C_1_EndSession(void);
// 	void	I2C_1_Shutdown(void);

	#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* I2C_H_INCLUDED */
