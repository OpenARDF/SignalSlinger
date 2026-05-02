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
 * Simple serial inter-processor messaging helpers.
 *
 * This module contains support functions for:
 * - managing UART-backed transmit and receive buffers for serial commands
 * - queueing prompt, reply, and echo traffic
 * - exposing parsed receive buffers to foreground code
 *
 * Command semantics and application-level message handling belong elsewhere.
 */

#include "defs.h"
#include "serialbus.h"
#include "usart_basic.h"
#include "globals.h"
#include <atomic.h>
#include "shared_state.h"
#include <string.h>
#include <stdio.h>
#include "atmel_start_pins.h"

static const uint16_t SERIALBUS_DEFAULT_TIMEOUT_TICKS = 200;
static const uint16_t SERIALBUS_TX_BUFFER_RETRY_COUNT = 200;
static const uint16_t SERIALBUS_TX_BUFFER_WAIT_SPINS = 500;
static const uint32_t SERIALBUS_TX_COMPLETION_SPIN_GUARD = 120000UL;

volatile uint16_t g_serial_timeout_ticks = SERIALBUS_DEFAULT_TIMEOUT_TICKS;
volatile USART_Number_t g_serialbus_usart_number = USART_NOT_SET;
static volatile bool g_serialbus_disabled = true;
static const char crlf[] = "\n";
static char lineTerm[8] = "\n";
static const char textPrompt[] = "> ";
#define SERIALBUS_ISR_ECHO_FIFO_SIZE 16
static volatile uint8_t g_isr_echo_head = 0;
static volatile uint8_t g_isr_echo_tail = 0;
static volatile char g_isr_echo_fifo[SERIALBUS_ISR_ECHO_FIFO_SIZE];

/* Local function prototypes */
/**
 * Begin transmission of the next queued serial-bus message if the UART is idle.
 *
 * @return true when transmission was started, false if a transmission is already active.
 */
bool serialbus_start_tx(void);

/**
 * Queue a null-terminated text string into the transmit-buffer pool.
 *
 * @param text Null-terminated string to enqueue.
 * @return true on error, false on success.
 */
bool serialbus_send_text(char *text);
static void USART1_initialization(uint32_t baud);
static void USART0_initialization(uint32_t baud);

/**
 * Reset the ISR-side echo FIFO.
 */
static inline void serialbus_echo_fifo_reset(void)
{
	g_isr_echo_head = 0;
	g_isr_echo_tail = 0;
}

/* Module global variables */
static volatile bool serialbus_tx_active = false; /* volatile is required to ensure optimizer handles this properly */
static volatile SerialbusTxBuffer tx_buffer[SERIALBUS_NUMBER_OF_TX_MSG_BUFFERS];
static volatile SerialbusRxBuffer rx_buffer[SERIALBUS_NUMBER_OF_RX_MSG_BUFFERS];

typedef bool (*buffer_index_match_t)(uint8_t index);

/**
 * Scan a circular index until a matching buffer slot is found.
 *
 * @param bufferIndex Starting index, updated in place to the matching slot.
 * @param bufferCount Number of buffers in the ring.
 * @param matches Predicate that identifies an acceptable slot.
 * @return true when a matching slot is found.
 */
static bool find_matching_buffer_index(uint8_t *bufferIndex, uint8_t bufferCount, buffer_index_match_t matches)
{
	uint8_t count = 0;

	while(!matches(*bufferIndex))
	{
		if(++count >= bufferCount)
		{
			return false;
		}

		(*bufferIndex)++;
		if(*bufferIndex >= bufferCount)
		{
			*bufferIndex = 0;
		}
	}

	return true;
}

/**
 * Test whether a transmit buffer currently holds queued data.
 */
static bool tx_buffer_is_full(uint8_t index)
{
	return tx_buffer[index][0] != '\0';
}

/**
 * Test whether a transmit buffer slot is empty.
 */
static bool tx_buffer_is_empty(uint8_t index)
{
	return tx_buffer[index][0] == '\0';
}

/**
 * Test whether a receive buffer currently holds a parsed message.
 */
static bool rx_buffer_is_full(uint8_t index)
{
	return rx_buffer[index].id != SB_MESSAGE_EMPTY;
}

/**
 * Test whether a receive buffer slot is empty.
 */
static bool rx_buffer_is_empty(uint8_t index)
{
	return rx_buffer[index].id == SB_MESSAGE_EMPTY;
}

/*
 * Locate the next transmit buffer that contains a queued message.
 * Returns a pointer to the buffer or null if no messages are waiting.
 */
SerialbusTxBuffer *nextFullSBTxBuffer(void)
{
	static uint8_t bufferIndex = 0;

	if(find_matching_buffer_index(&bufferIndex, SERIALBUS_NUMBER_OF_TX_MSG_BUFFERS, tx_buffer_is_full))
	{
		return ((SerialbusTxBuffer *)&tx_buffer[bufferIndex]);
	}

	return (null);
}

/*
 * Find the next available empty transmit buffer for queuing data.
 * Returns a pointer to the buffer or null if all buffers are in use.
 */
SerialbusTxBuffer *nextEmptySBTxBuffer(void)
{
	static uint8_t bufferIndex = 0;

	if(find_matching_buffer_index(&bufferIndex, SERIALBUS_NUMBER_OF_TX_MSG_BUFFERS, tx_buffer_is_empty))
	{
		return ((SerialbusTxBuffer *)&tx_buffer[bufferIndex]);
	}

	return (null);
}

/*
 * Return a pointer to the next empty receive buffer so incoming data
 * can be stored without overwriting unread messages.
 */
SerialbusRxBuffer *nextEmptySBRxBuffer(void)
{
	static uint8_t bufferIndex = 0;

	if(find_matching_buffer_index(&bufferIndex, SERIALBUS_NUMBER_OF_RX_MSG_BUFFERS, rx_buffer_is_empty))
	{
		for(int i = 0; i < SERIALBUS_MAX_MSG_NUMBER_OF_FIELDS; i++)
		{
			rx_buffer[bufferIndex].fields[i][0] = '\0';
		}

		return ((SerialbusRxBuffer *)&rx_buffer[bufferIndex]);
	}

	return (null);
}

/*
 * Fetch the next receive buffer that has been filled with a message
 * from the remote device.
 */
SerialbusRxBuffer *nextFullSBRxBuffer(void)
{
	static uint8_t bufferIndex = 0;

	if(find_matching_buffer_index(&bufferIndex, SERIALBUS_NUMBER_OF_RX_MSG_BUFFERS, rx_buffer_is_full))
	{
		return ((SerialbusRxBuffer *)&rx_buffer[bufferIndex]);
	}

	return (null);
}

/***********************************************************************
 * serialbusTxInProgress(void)
 ************************************************************************/
bool serialbusTxInProgress(void)
{
	return (serialbus_tx_active);
}

/*
 * Begin UART transmission if the interface is idle.  The first byte will
 * be loaded by the interrupt handler once the transmitter is enabled.
 */
bool serialbus_start_tx(void)
{
	bool success = !serialbus_tx_active;

	if(success) /* message will be lost if transmit is busy */
	{
		serialbus_tx_active = true;

		if(g_serialbus_usart_number == USART_0)
		{
			USART0_enable_tx();
		}
		else
		{
			USART1_enable_tx();
		}
	}

	return (success);
}

void serialbus_end_tx(void)
{
	if(serialbus_tx_active)
	{
		if(g_serialbus_usart_number == USART_0)
		{
			USART0.CTRLA &= ~(1 << USART_DREIE_bp); /* Transmit Data Register Empty Interrupt Enable: disable */
		}
		else
		{
			USART1.CTRLA &= ~(1 << USART_DREIE_bp); /* Transmit Data Register Empty Interrupt Enable: disable */
		}

		serialbus_tx_active = false;
	}
}

/* configure the pins and initialize the registers */
/**
 * Configure the USART1 pins and initialize the USART peripheral.
 *
 * @param baud Requested baud rate.
 */
void USART1_initialization(uint32_t baud)
{
	// Set Rx pin direction to input
	PC1_set_dir(PORT_DIR_IN);
	PC1_set_pull_mode(PORT_PULL_OFF);

	// Set Tx pin direction to output
	PC0_set_dir(PORT_DIR_OUT);
	PC0_set_level(HIGH);

	USART1_init(baud, false);
}

/* configure the pins and initialize the registers */
/**
 * Configure the USART0 pins and initialize the USART peripheral.
 *
 * @param baud Requested baud rate.
 */
void USART0_initialization(uint32_t baud)
{
	// Set Rx pin direction to input
	PA5_set_dir(PORT_DIR_IN);
	PA5_set_pull_mode(PORT_PULL_OFF);

	// Set Tx pin direction to output
	PA4_set_dir(PORT_DIR_OUT);
	PA4_set_level(HIGH);

	USART0_init(baud, false);
}

/**
 * Configure the serial bus with a baud rate and USART instance.
 *
 * @param baud Requested baud rate.
 * @param usart USART instance selection.
 */
void serialbus_init(uint32_t baud, USART_Number_t usart)
{
	memset((void *)rx_buffer, 0, sizeof(rx_buffer));
	serialbus_set_rx_accepting_input(false);
	serialbus_end_tx();
	serialbus_echo_fifo_reset();

	for(int bufferIndex = 0; bufferIndex < SERIALBUS_NUMBER_OF_TX_MSG_BUFFERS; bufferIndex++)
	{
		tx_buffer[bufferIndex][0] = '\0';
	}

	if((usart != USART_DO_NOT_CHANGE) || (g_serialbus_usart_number == USART_NOT_SET))
	{
		if(usart == USART_0)
		{
			USART0_initialization(baud);
		}
		else
		{
			USART1_initialization(baud);
		}

		g_serialbus_usart_number = usart;
	}

	g_serialbus_disabled = false;
	serialbus_flush_rx();
}

/**
 * Discard any unread bytes currently waiting in the active USART RX register path.
 */
void serialbus_flush_rx(void)
{
	if(g_serialbus_disabled)
		return;

	if(g_serialbus_usart_number == USART_0)
	{
		while(USART0_is_rx_ready())
			(void)USART0_get_data();
	}
	else
	{
		while(USART1_is_rx_ready())
			(void)USART1_get_data();
	}

	return;
}

/**
 * Immediately disable the serial bus and clear queued RX/TX state.
 */
void serialbus_disable(void)
{
	uint8_t bufferIndex;

	g_serialbus_disabled = true;
	serialbus_end_tx();

	if(g_serialbus_usart_number == USART_0)
	{
		USART0_disable();
	}
	else
	{
		USART1_disable();
	}

	memset((void *)rx_buffer, 0, sizeof(rx_buffer));
	serialbus_set_rx_accepting_input(false);
	serialbus_echo_fifo_reset();

	for(bufferIndex = 0; bufferIndex < SERIALBUS_NUMBER_OF_TX_MSG_BUFFERS; bufferIndex++)
	{
		tx_buffer[bufferIndex][0] = '\0';
	}
}

/*
 * Queue a null-terminated string for transmission.  The routine will block
 * briefly if no buffer is available, returning true on error.
 */
bool serialbus_send_text(char *text)
{
	bool err = true;

	if(g_serialbus_disabled)
	{
		return (err);
	}

	uint16_t tries;
	if(text)
	{
		SerialbusTxBuffer local_copy;
		snprintf(local_copy, SERIALBUS_MAX_TX_MSG_LENGTH, "%s", text);
		SerialbusTxBuffer *buff = nextEmptySBTxBuffer();
		tries = SERIALBUS_TX_BUFFER_RETRY_COUNT;

		while(!buff && tries--)
		{
			uint16_t spin = SERIALBUS_TX_BUFFER_WAIT_SPINS;
			while(serialbusTxInProgress() && spin--)
				; /* Wait for previous transmission to complete */
			buff = nextEmptySBTxBuffer();
		}

		if(buff)
		{
			/* Publish buffer atomically with first byte written last so the DRE ISR
			 * does not observe a partially formatted message as ready. */
			ENTER_CRITICAL(serialbus_tx_publish);
			(*buff)[0] = '\0';
			for(uint8_t i = 1; i < SERIALBUS_MAX_TX_MSG_LENGTH; i++)
			{
				(*buff)[i] = local_copy[i];
				if(local_copy[i] == '\0')
				{
					break;
				}
			}
			(*buff)[0] = local_copy[0];
			EXIT_CRITICAL(serialbus_tx_publish);
			serialbus_start_tx();
			err = false;
		}
	}

	return (err);
}

/***********************************************************************************
 *  Support for creating and sending various Terminal Mode Serialbus messages is provided below.
 ************************************************************************************/

void sb_send_NewPrompt(void)
{
	if(g_isMaster || g_cloningInProgress)
		return;

	if(g_serialbus_disabled)
	{
		return;
	}

	while(serialbus_send_text((char *)textPrompt))
	{
		;
	}
}

void sb_send_NewLine(void)
{
	if(g_isMaster || g_cloningInProgress)
		return;

	if(g_serialbus_disabled)
	{
		return;
	}
	serialbus_send_text((char *)crlf);
}

void sb_echo_char(uint8_t c)
{
	if(g_isMaster || g_cloningInProgress)
		return;

	if(g_serialbus_disabled)
	{
		return;
	}
	char echo[2];
	echo[0] = c;
	echo[1] = '\0';
	serialbus_send_text(echo);
}

bool sb_echo_char_isr(uint8_t c)
{
	uint8_t next_head;

	if(g_isMaster || g_cloningInProgress)
		return false;
	if(g_serialbus_disabled)
		return false;

	next_head = (uint8_t)((g_isr_echo_head + 1U) % SERIALBUS_ISR_ECHO_FIFO_SIZE);
	if(next_head == g_isr_echo_tail)
	{
		return false; /* FIFO full: drop echo char rather than block in ISR */
	}

	g_isr_echo_fifo[g_isr_echo_head] = (char)c;
	g_isr_echo_head = next_head;
	serialbus_start_tx(); /* Safe kick: returns false if already active */
	return true;
}

/**
 * Pop one pending ISR-queued echo character.
 *
 * @param c Destination for the queued character.
 * @return true when a character was returned.
 */
bool serialbus_echo_try_get_isr(uint8_t *c)
{
	if(!c)
		return false;
	if(g_isr_echo_head == g_isr_echo_tail)
		return false;

	*c = (uint8_t)g_isr_echo_fifo[g_isr_echo_tail];
	g_isr_echo_tail = (uint8_t)((g_isr_echo_tail + 1U) % SERIALBUS_ISR_ECHO_FIFO_SIZE);
	return true;
}

/**
 * Queue a string for transmission using normal caller routing.
 *
 * @param str Null-terminated string to send.
 * @return true on error, false on success.
 */
bool sb_send_string(char *str)
{
	if(g_isMaster)
		return true;

	return sb_send_master_string(str);
}

/**
 * Queue a string for transmission without the non-master forwarding shortcut.
 *
 * The string is chunked into serial-bus-sized packets and each packet waits for
 * transmit completion before the next chunk is queued.
 *
 * @param str Null-terminated string to send.
 * @return true on error, false on success.
 */
bool sb_send_master_string(char *str)
{
	char buf[SERIALBUS_MAX_TX_MSG_LENGTH + 1];
	bool err = false;
	uint16_t length, lengthToSend, lengthSent = 0;
	bool done = false;

	if(g_serialbus_disabled)
	{
		return (true);
	}

	if(str == NULL)
	{
		return (true);
	}

	if(!*str)
	{
		return (true);
	}

	length = strlen(str);

	do
	{
		lengthToSend = MIN(length - lengthSent, (uint16_t)(SERIALBUS_MAX_TX_MSG_LENGTH - 1));
		strncpy(buf, &str[lengthSent], lengthToSend);

		buf[lengthToSend] = '\0';
		err = serialbus_send_text(buf);

		atomic_write_u16(&g_serial_timeout_ticks, SERIALBUS_DEFAULT_TIMEOUT_TICKS);
		if(!err)
		{
			uint32_t spin_guard = SERIALBUS_TX_COMPLETION_SPIN_GUARD;
			while(serialbusTxInProgress() && atomic_read_u16(&g_serial_timeout_ticks) && spin_guard--)
			{
				;
			}

			if(serialbusTxInProgress() && (!spin_guard || !atomic_read_u16(&g_serial_timeout_ticks)))
				err = true;
		}

		lengthSent += lengthToSend;
		done = err || (lengthSent >= length);
	} while(!done);

	return (err);
}

/**
 * Send an ASCII label followed by a numeric value.
 *
 * @param value Numeric value to send.
 * @param label ASCII label to prefix before the value.
 */
void sb_send_value(uint16_t value, char *label)
{
	bool err;
	char tempMsgBuff[SERIALBUS_MAX_MSG_LENGTH];

	if(g_serialbus_disabled)
	{
		return;
	}

	sprintf(tempMsgBuff, "> %s=%d%s", label, value, lineTerm);
	while((err = serialbus_send_text(tempMsgBuff)))
	{
		;
	}

	atomic_write_u16(&g_serial_timeout_ticks, SERIALBUS_DEFAULT_TIMEOUT_TICKS);
	{
		uint32_t spin_guard = SERIALBUS_TX_COMPLETION_SPIN_GUARD;
		while(!err && serialbusTxInProgress() && atomic_read_u16(&g_serial_timeout_ticks) && spin_guard--)
		{
			;
		}
		if(!err && serialbusTxInProgress() && (!spin_guard || !atomic_read_u16(&g_serial_timeout_ticks)))
		{
			err = true;
		}
	}
}

/**
 * Report whether the serial bus has been initialized and enabled.
 *
 * @return true when the serial bus is enabled.
 */
bool sb_enabled(void)
{
	return (!g_serialbus_disabled);
}
