#include "bootloader_config.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "ccp.h"

#ifndef USART_CHSIZE0_bm
#define USART_CHSIZE0_bm USART_CHSIZE_0_bm
#define USART_CHSIZE1_bm USART_CHSIZE_1_bm
#endif

#define USART1_BAUD_VALUE(baud) ((uint16_t)((F_CPU * 64UL / (16UL * (baud))) + 0.5))

static void clock_init(void)
{
	ccp_write_io((void *)&CLKCTRL.OSCHFCTRLA,
	             CLKCTRL_FRQSEL_24M_gc |
	                 (1 << CLKCTRL_AUTOTUNE_bp) |
	                 (0 << CLKCTRL_RUNSTDBY_bp));
}

static void pins_init(void)
{
	/* Front-panel switch on PD3, active low. */
	PORTD.DIRCLR = PIN3_bm;
	PORTD.PIN3CTRL = PORT_PULLUPEN_bm;

	/* USART1 on PC0 TX, PC1 RX. */
	PORTC.DIRSET = PIN0_bm;
	PORTC.OUTSET = PIN0_bm;
	PORTC.DIRCLR = PIN1_bm;
	PORTC.PIN1CTRL = 0;
}

static bool switch_is_held(void)
{
	return (PORTD.IN & PIN3_bm) == 0;
}

static void usart_init(void)
{
	USART1.BAUD = USART1_BAUD_VALUE(SIGNALSLINGER_BOOT_USART_BAUD);
	USART1.CTRLA = 0;
	USART1.CTRLC = USART_CHSIZE0_bm | USART_CHSIZE1_bm;
	USART1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
}

static void usart_disable(void)
{
	USART1.CTRLB = 0;
	USART1.CTRLA = 0;
}

static bool usart_rx_ready(void)
{
	return (USART1.STATUS & USART_RXCIF_bm) != 0;
}

static char usart_read(void)
{
	return USART1.RXDATAL;
}

static void usart_write(char value)
{
	while((USART1.STATUS & USART_DREIF_bm) == 0)
	{
	}
	USART1.TXDATAL = value;
}

static void usart_write_text(const char *text)
{
	while(*text)
	{
		usart_write(*text++);
	}
}

static bool app_vector_looks_programmed(void)
{
	uint16_t reset_word = pgm_read_word_far(SIGNALSLINGER_APP_START_BYTES);
	return reset_word != 0xFFFFU;
}

static void jump_to_application(void)
{
	cli();
	usart_disable();

	/*
	 * AVR program-counter addresses are word addresses. The initial
	 * bootloader allocation puts the application at byte 0x4000, word 0x2000.
	 */
	asm volatile("jmp 0x2000");
}

static void send_banner(void)
{
	usart_write_text("\r\nSignalSlinger ");
	usart_write_text(SIGNALSLINGER_BOOTLOADER_VERSION);
	usart_write_text(" app=0x4000 page=512\r\n");
}

static bool serial_entry_requested(void)
{
	for(uint16_t elapsed_ms = 0; elapsed_ms < SIGNALSLINGER_BOOT_ENTRY_WINDOW_MS; elapsed_ms++)
	{
		if(usart_rx_ready())
		{
			char command = usart_read();
			if(command == SIGNALSLINGER_UPDATE_REQUEST_CHAR)
			{
				usart_write_text("BOOT\r\n");
				return true;
			}
			if(command == SIGNALSLINGER_INFO_CHAR)
			{
				send_banner();
			}
			if(command == SIGNALSLINGER_RUN_APP_CHAR)
			{
				return false;
			}
		}
		_delay_ms(1);
	}

	return false;
}

int main(void)
{
	cli();
	clock_init();
	pins_init();
	usart_init();

	bool stay_in_bootloader = switch_is_held() || !app_vector_looks_programmed() || serial_entry_requested();

	if(!stay_in_bootloader)
	{
		jump_to_application();
	}

	send_banner();
	usart_write_text("Waiting for updater\r\n");

	for(;;)
	{
		if(usart_rx_ready())
		{
			char command = usart_read();
			if(command == SIGNALSLINGER_INFO_CHAR)
			{
				send_banner();
			}
			else if(command == SIGNALSLINGER_RUN_APP_CHAR && app_vector_looks_programmed())
			{
				jump_to_application();
			}
			else
			{
				usart_write_text("ERR unsupported\r\n");
			}
		}
	}
}
