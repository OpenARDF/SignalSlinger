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

static uint8_t page_buffer[SIGNALSLINGER_FLASH_PAGE_BYTES];
static uint8_t last_nvm_error;
static uint8_t last_usart_error;

#define SIGNALSLINGER_USART_RX_ERROR_MASK (USART_PERR_bm | USART_FERR_bm | USART_BUFOVF_bm)

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

static bool usart_read_byte(uint8_t *value)
{
	uint8_t status = USART1.RXDATAH;
	*value = USART1.RXDATAL;
	last_usart_error = status & SIGNALSLINGER_USART_RX_ERROR_MASK;
	return last_usart_error == 0U;
}

static bool usart_read_timeout(uint8_t *value, uint16_t timeout_ms)
{
	uint32_t timeout_ticks = (uint32_t)timeout_ms * 100U;
	last_usart_error = 0;
	while(timeout_ticks--)
	{
		if(usart_rx_ready())
		{
			return usart_read_byte(value);
		}
		_delay_us(10);
	}

	return false;
}

static void usart_flush_rx(void)
{
	uint8_t discarded;
	while(usart_rx_ready())
	{
		(void)usart_read_byte(&discarded);
	}
	last_usart_error = 0;
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

static void usart_write_hex_nibble(uint8_t value)
{
	value &= 0x0FU;
	usart_write((char)(value < 10U ? ('0' + value) : ('A' + value - 10U)));
}

static void usart_write_hex8(uint8_t value)
{
	usart_write_hex_nibble(value >> 4);
	usart_write_hex_nibble(value);
}

static void send_ok(const char *detail)
{
	usart_write_text("OK ");
	usart_write_text(detail);
	usart_write_text("\r\n");
}

static void send_error(const char *detail)
{
	usart_write_text("ERR ");
	usart_write_text(detail);
	usart_write_text("\r\n");
}

static void send_usart_error(void)
{
	usart_write_text("ERR serial ");
	usart_write_hex8(last_usart_error);
	usart_write_text("\r\n");
}

static uint16_t crc16_update(uint16_t crc, uint8_t value)
{
	crc ^= (uint16_t)value << 8;
	for(uint8_t bit = 0; bit < 8; bit++)
	{
		if(crc & 0x8000U)
		{
			crc = (uint16_t)((crc << 1) ^ 0x1021U);
		}
		else
		{
			crc <<= 1;
		}
	}

	return crc;
}

static bool read_crc_byte(uint8_t *value, uint16_t *crc)
{
	if(!usart_read_timeout(value, SIGNALSLINGER_BOOT_FRAME_BYTE_TIMEOUT_MS))
	{
		return false;
	}

	*crc = crc16_update(*crc, *value);
	return true;
}

static bool read_u32_le(uint32_t *value, uint16_t *crc)
{
	*value = 0;
	for(uint8_t shift = 0; shift < 32; shift += 8)
	{
		uint8_t byte;
		if(!read_crc_byte(&byte, crc))
		{
			return false;
		}
		*value |= (uint32_t)byte << shift;
	}

	return true;
}

static bool read_expected_crc(uint16_t *value)
{
	uint8_t low;
	uint8_t high;
	if(!usart_read_timeout(&low, SIGNALSLINGER_BOOT_FRAME_BYTE_TIMEOUT_MS) ||
	   !usart_read_timeout(&high, SIGNALSLINGER_BOOT_FRAME_BYTE_TIMEOUT_MS))
	{
		return false;
	}

	*value = (uint16_t)low | ((uint16_t)high << 8);
	return true;
}

static bool read_page_payload(uint16_t *crc)
{
	for(uint16_t index = 0; index < SIGNALSLINGER_FLASH_PAGE_BYTES; index++)
	{
		if(!read_crc_byte(&page_buffer[index], crc))
		{
			return false;
		}
	}

	return true;
}

static bool app_page_address_is_writable(uint32_t address)
{
	if((address % SIGNALSLINGER_FLASH_PAGE_BYTES) != 0U)
	{
		return false;
	}
	if(address < SIGNALSLINGER_APP_START_BYTES)
	{
		return false;
	}
	if(address > (SIGNALSLINGER_FLASH_BYTES - SIGNALSLINGER_FLASH_PAGE_BYTES))
	{
		return false;
	}

	return true;
}

static void nvm_wait_flash(void)
{
	while(NVMCTRL.STATUS & NVMCTRL_FBUSY_bm)
	{
	}
}

static void nvm_command(NVMCTRL_CMD_t command)
{
	ccp_write_spm((void *)&NVMCTRL.CTRLA, command);
}

static uint8_t nvm_error_bits(void)
{
	return NVMCTRL.STATUS & NVMCTRL_ERROR_gm;
}

static void send_nvm_error(void)
{
	usart_write_text("ERR nvm ");
	usart_write_hex8(last_nvm_error);
	usart_write_text("\r\n");
}

static void pgm_word_write(uint32_t byte_address, uint16_t value)
{
	asm volatile(
	    "movw r30, %A0\n\t"
	    "sts %1, %C0\n\t"
	    "movw r0, %2\n\t"
	    "spm\n\t"
	    "clr r1\n\t"
	    :
	    : "r"(byte_address),
	      "i"(_SFR_MEM_ADDR(RAMPZ)),
	      "r"(value)
	    : "r0", "r30", "r31", "memory");
}

static bool erase_app_page(uint32_t address)
{
	if(!app_page_address_is_writable(address))
	{
		return false;
	}

	nvm_wait_flash();
	nvm_command(NVMCTRL_CMD_NONE_gc);
	nvm_command(NVMCTRL_CMD_FLPER_gc);
	pgm_word_write(address, 0x0000U);
	nvm_wait_flash();
	last_nvm_error = nvm_error_bits();
	nvm_command(NVMCTRL_CMD_NONE_gc);

	return last_nvm_error == NVMCTRL_ERROR_NOERROR_gc;
}

static bool write_app_page(uint32_t address)
{
	if(!app_page_address_is_writable(address))
	{
		return false;
	}

	nvm_wait_flash();
	nvm_command(NVMCTRL_CMD_NONE_gc);
	nvm_command(NVMCTRL_CMD_FLWR_gc);

	for(uint16_t index = 0; index < SIGNALSLINGER_FLASH_PAGE_BYTES; index += 2)
	{
		uint16_t word = (uint16_t)page_buffer[index] | ((uint16_t)page_buffer[index + 1] << 8);
		pgm_word_write(address + index, word);
	}

	nvm_wait_flash();
	last_nvm_error = nvm_error_bits();
	nvm_command(NVMCTRL_CMD_NONE_gc);

	return last_nvm_error == NVMCTRL_ERROR_NOERROR_gc;
}

static bool receive_page_address_frame(char command, uint32_t *address)
{
	uint16_t crc = crc16_update(0xFFFFU, (uint8_t)command);
	uint16_t expected_crc;

	if(!read_u32_le(address, &crc) || !read_expected_crc(&expected_crc))
	{
		if(last_usart_error != 0U)
		{
			send_usart_error();
		}
		else
		{
			send_error("timeout");
		}
		return false;
	}
	if(crc != expected_crc)
	{
		send_error("crc");
		return false;
	}
	if(!app_page_address_is_writable(*address))
	{
		send_error("address");
		return false;
	}

	return true;
}

static bool receive_page_write_frame(char command, uint32_t *address)
{
	uint16_t crc = crc16_update(0xFFFFU, (uint8_t)command);
	uint16_t expected_crc;

	if(!read_u32_le(address, &crc) ||
	   !read_page_payload(&crc) ||
	   !read_expected_crc(&expected_crc))
	{
		if(last_usart_error != 0U)
		{
			send_usart_error();
		}
		else
		{
			send_error("timeout");
		}
		return false;
	}
	if(crc != expected_crc)
	{
		send_error("crc");
		return false;
	}
	if(!app_page_address_is_writable(*address))
	{
		send_error("address");
		return false;
	}

	return true;
}

static void handle_erase_frame(char command)
{
	uint32_t address;
	if(!receive_page_address_frame(command, &address))
	{
		return;
	}

	if(erase_app_page(address))
	{
		send_ok("erase");
	}
	else
	{
		send_nvm_error();
	}
}

static void handle_write_frame(char command)
{
	uint32_t address;
	if(!receive_page_write_frame(command, &address))
	{
		return;
	}

	if(write_app_page(address))
	{
		send_ok("write");
	}
	else
	{
		send_nvm_error();
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

	asm volatile("jmp 0x4000");
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
			uint8_t command_byte;
			if(!usart_read_byte(&command_byte))
			{
				continue;
			}
			char command = (char)command_byte;
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

	usart_flush_rx();
	send_banner();
	usart_write_text("Waiting for updater\r\n");

	for(;;)
	{
		if(usart_rx_ready())
		{
			uint8_t command_byte;
			if(!usart_read_byte(&command_byte))
			{
				send_usart_error();
				continue;
			}
			char command = (char)command_byte;
			if(command == SIGNALSLINGER_INFO_CHAR)
			{
				send_banner();
			}
			else if(command == SIGNALSLINGER_RUN_APP_CHAR && app_vector_looks_programmed())
			{
				jump_to_application();
			}
			else if(command == SIGNALSLINGER_ERASE_PAGE_CHAR)
			{
				handle_erase_frame(command);
			}
			else if(command == SIGNALSLINGER_WRITE_PAGE_CHAR)
			{
				handle_write_frame(command);
			}
			else
			{
				send_error("unsupported");
			}
		}
	}
}
