/*
 *  MIT License
 *
 *  Copyright (c) 2022 DigitalConfections
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


#include <compiler.h>
#include <clock_config.h>
#include <usart_basic.h>
#include <atomic.h>

/*****************************************************************************************
USART1
******************************************************************************************/

/**
 * \brief Initialize USART interface
 * If module is configured to disabled state, the clock to the USART is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the USART init was successful
 * \retval 1 the USART init was not successful
 */
int8_t USART1_init(uint32_t baud, bool autobaud)
{
	DISABLE_INTERRUPTS();
	
	USART1.BAUD = USART1_BAUD_RATE(baud); /* set baud rate register */

	USART1.CTRLA = 0 << USART_ABEIE_bp /* Auto-baud Error Interrupt Enable: disabled */
			 | 0 << USART_LBME_bp /* Loop-back Mode Enable: disabled */
			 | USART_RS485_DISABLE_gc /* RS485 Mode disabled */
			 | 0 << USART_RXSIE_bp /* Receiver Start Frame Interrupt Enable: disabled */
			 | 0 << USART_DREIE_bp /* Data Register Empty Interrupt Enable: disabled */
			 | 0 << USART_TXCIE_bp /* Transmit Complete Interrupt Enable: disable */
			 | 1 << USART_RXCIE_bp; /* Receive Complete Interrupt Enable: enable */

    USART1.CTRLC = USART_CHSIZE0_bm
                 | USART_CHSIZE1_bm;                    /* set the data format to 8-bit*/

	if(autobaud)
	{
		USART1.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
					   | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
					   | USART_RXMODE_GENAUTO_gc /* Generic autobaud mode */
					   | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
					   | 1 << USART_TXEN_bp    /* Transmitter Enable: enable */
					   | 1 << USART_RXEN_bp;     /* Receiver Enable: enable */
	}
	else
	{
		USART1.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
					   | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
					   | USART_RXMODE_NORMAL_gc /* Normal mode */
					   | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
					   | 1 << USART_TXEN_bp    /* Transmitter Enable: enable */
					   | 1 << USART_RXEN_bp;     /* Receiver Enable: enable */
	}

	// USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART1.DBGCTRL = 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART1.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART1.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART1.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */
	
	ENABLE_INTERRUPTS();

	return 0;
}

/**
 * \brief Enable RX and TX in USART1
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX and TX enable-bits in the USART control register
 *
 * \return Nothing
 */
void USART1_enable()
{
	USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

/**
 * \brief Enable RX in USART1
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX enable-bit in the USART control register
 *
 * \return Nothing
 */
void USART1_enable_rx()
{
	USART1.CTRLB |= USART_RXEN_bm;
}

/**
 * \brief Enable TX in USART1
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the TX enable-bit in the USART control register
 *
 * \return Nothing
 */
void USART1_enable_tx()
{
	USART1.CTRLB |= USART_TXEN_bm;
	USART1.CTRLA |= 1 << USART_DREIE_bp; /* Transmit Data Ready Interrupt Enable: enable */
//	USART1.STATUS |= 1 << USART_TXEN_bp; /* Clear any existing flag setting */
}

/**
 * \brief Disable USART1
 * 1. Disables the USART module by clearing the enable-bit(s) in the USART control register
 * 2. If supported by the clock system, disables the clock to the USART
 *
 * \return Nothing
 */
void USART1_disable()
{
	USART1.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/**
 * \brief Get received data from USART1
 *
 * \return Data register from USART1 module
 */
uint8_t USART1_get_data()
{
	return USART1.RXDATAL;
}

/**
 * \brief Check if the usart can accept data to be transmitted
 *
 * \return The status of USART TX data ready check
 * \retval false The USART can not receive data to be transmitted
 * \retval true The USART can receive data to be transmitted
 */
bool USART1_is_tx_ready()
{
	return (USART1.STATUS & USART_DREIF_bm);
}

/**
 * \brief Check if the USART has received data
 *
 * \return The status of USART RX data ready check
 * \retval true The USART has received data
 * \retval false The USART has not received data
 */
bool USART1_is_rx_ready()
{
	return (USART1.STATUS & USART_RXCIF_bm);
}

/**
 * \brief Check if USART1 data is transmitted
 *
 * \return Receiver ready status
 * \retval true  Data is not completely shifted out of the shift register
 * \retval false Data completely shifted out if the USART shift register
 */
bool USART1_is_tx_busy()
{
	return (!(USART1.STATUS & USART_TXCIF_bm));
}

/**
 * \brief Read one character from USART1
 *
 * Function will block if a character is not available.
 *
 * \return Data read from the USART1 module
 */
uint8_t USART1_read()
{
	while (!(USART1.STATUS & USART_RXCIF_bm))
		;
	return USART1.RXDATAL;
}

/**
 * \brief Write one character to USART1
 *
 * Function will block until a character can be accepted.
 *
 * \param[in] data The character to write to the USART
 *
 * \return Nothing
 */
void USART1_write(const uint8_t data)
{
	while (!(USART1.STATUS & USART_DREIF_bm))
		;
	USART1.TXDATAL = data;
}



/*****************************************************************************************
USART0
******************************************************************************************/

/**
 * \brief Initialize USART interface
 * If module is configured to disabled state, the clock to the USART is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the USART init was successful
 * \retval 1 the USART init was not successful
 */
int8_t USART0_init(uint32_t baud, bool autobaud)
{
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(baud); /* set baud rate register */

	USART0.CTRLA = 0 << USART_ABEIE_bp /* Auto-baud Error Interrupt Enable: disabled */
			 | 0 << USART_LBME_bp /* Loop-back Mode Enable: disabled */
			 | USART_RS485_DISABLE_gc /* RS485 Mode disabled */
			 | 0 << USART_RXSIE_bp /* Receiver Start Frame Interrupt Enable: disabled */
			 | 0 << USART_DREIE_bp /* Data Register Empty Interrupt Enable: disabled */
			 | 0 << USART_TXCIE_bp /* Transmit Complete Interrupt Enable: disable */
			 | 1 << USART_RXCIE_bp; /* Receive Complete Interrupt Enable: enable */

	if(autobaud)
	{
		USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
					   | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
					   | USART_RXMODE_GENAUTO_gc /* Generic autobaud mode */
					   | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
					   | 1 << USART_TXEN_bp    /* Transmitter Enable: enable */
					   | 1 << USART_RXEN_bp;     /* Receiver Enable: enable */
	}
	else
	{
		USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
					   | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
					   | USART_RXMODE_NORMAL_gc /* Normal mode */
					   | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
					   | 1 << USART_TXEN_bp    /* Transmitter Enable: enable */
					   | 1 << USART_RXEN_bp;     /* Receiver Enable: enable */
	}

	// USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART0.DBGCTRL = 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART0.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART0.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART0.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */

	return 0;
}

/**
 * \brief Enable RX and TX in USART0
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX and TX enable-bits in the USART control register
 *
 * \return Nothing
 */
void USART0_enable()
{
	USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

/**
 * \brief Enable RX in USART0
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX enable-bit in the USART control register
 *
 * \return Nothing
 */
void USART0_enable_rx()
{
	USART0.CTRLB |= USART_RXEN_bm;
}

/**
 * \brief Enable TX in USART0
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the TX enable-bit in the USART control register
 *
 * \return Nothing
 */
void USART0_enable_tx()
{
	USART0.CTRLB |= USART_TXEN_bm;
	USART0.CTRLA |= 1 << USART_DREIE_bp; /* Transmit Data Ready Interrupt Enable: enable */
//	USART0.STATUS |= 1 << USART_TXEN_bp; /* Clear any existing flag setting */
}

/**
 * \brief Disable USART0
 * 1. Disables the USART module by clearing the enable-bit(s) in the USART control register
 * 2. If supported by the clock system, disables the clock to the USART
 *
 * \return Nothing
 */
void USART0_disable()
{
	USART0.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/**
 * \brief Get recieved data from USART0
 *
 * \return Data register from USART0 module
 */
uint8_t USART0_get_data()
{
	return USART0.RXDATAL;
}

/**
 * \brief Check if the usart can accept data to be transmitted
 *
 * \return The status of USART TX data ready check
 * \retval false The USART can not receive data to be transmitted
 * \retval true The USART can receive data to be transmitted
 */
bool USART0_is_tx_ready()
{
	return (USART0.STATUS & USART_DREIF_bm);
}

/**
 * \brief Check if the USART has received data
 *
 * \return The status of USART RX data ready check
 * \retval true The USART has received data
 * \retval false The USART has not received data
 */
bool USART0_is_rx_ready()
{
	return (USART0.STATUS & USART_RXCIF_bm);
}

/**
 * \brief Check if USART0 data is transmitted
 *
 * \return Receiver ready status
 * \retval true  Data is not completely shifted out of the shift register
 * \retval false Data completely shifted out if the USART shift register
 */
bool USART0_is_tx_busy()
{
	return (!(USART0.STATUS & USART_TXCIF_bm));
}

/**
 * \brief Read one character from USART0
 *
 * Function will block if a character is not available.
 *
 * \return Data read from the USART0 module
 */
uint8_t USART0_read()
{
	while (!(USART0.STATUS & USART_RXCIF_bm))
		;
	return USART0.RXDATAL;
}

/**
 * \brief Write one character to USART0
 *
 * Function will block until a character can be accepted.
 *
 * \param[in] data The character to write to the USART
 *
 * \return Nothing
 */
void USART0_write(const uint8_t data)
{
	while (!(USART0.STATUS & USART_DREIF_bm))
		;
	USART0.TXDATAL = data;
}

