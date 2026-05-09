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

#define SIGNALSLINGER_POWER_ENABLE_PIN PIN3_bm
#define SIGNALSLINGER_RED_LED_PIN PIN2_bm
#define SIGNALSLINGER_GREEN_LED_PIN PIN4_bm
#define SIGNALSLINGER_STARTUP_LED_PINS (SIGNALSLINGER_RED_LED_PIN | SIGNALSLINGER_GREEN_LED_PIN)
#define SIGNALSLINGER_STARTUP_LED_BLINK_HALF_PERIOD_MS 50U
#define SIGNALSLINGER_BOOT_HANDOFF_POWER_BUTTON_HELD 0x53U
#define SIGNALSLINGER_BOOT_APP_UPDATE_REQUEST 0xA5U
#define SIGNALSLINGER_BOOT_HANDOFF_INFO_MAGIC 0xB0U
#define SIGNALSLINGER_BOOT_HANDOFF_INFO_PROTOCOL_MASK 0x0FU
#define SIGNALSLINGER_USART_RX_ERROR_MASK (USART_PERR_bm | USART_FERR_bm | USART_BUFOVF_bm)
#define SIGNALSLINGER_RESET_FLAGS_MASK \
	(RSTCTRL_UPDIRF_bm | RSTCTRL_SWRF_bm | RSTCTRL_WDRF_bm | RSTCTRL_EXTRF_bm | RSTCTRL_BORF_bm | RSTCTRL_PORF_bm)

static uint8_t read_and_clear_reset_flags(void)
{
	uint8_t reset_flags = RSTCTRL.RSTFR;
	RSTCTRL.RSTFR = SIGNALSLINGER_RESET_FLAGS_MASK;
	return reset_flags;
}

static bool reset_was_power_start(uint8_t reset_flags)
{
	return (reset_flags & (RSTCTRL_PORF_bm | RSTCTRL_BORF_bm)) != 0U;
}

static bool app_requested_bootloader(uint8_t reset_flags)
{
	bool requested = ((reset_flags & RSTCTRL_SWRF_bm) != 0U) &&
	                 (GPR.GPR1 == SIGNALSLINGER_BOOT_APP_UPDATE_REQUEST);
	GPR.GPR1 = 0U;
	return requested;
}

static void clock_init(void)
{
	ccp_write_io((void *)&CLKCTRL.OSCHFCTRLA,
	             CLKCTRL_FRQSEL_24M_gc |
	                 (1 << CLKCTRL_AUTOTUNE_bp) |
	                 (0 << CLKCTRL_RUNSTDBY_bp));
}

static void latch_power_on(void)
{
	PORTA.OUTSET = SIGNALSLINGER_POWER_ENABLE_PIN;
	PORTA.DIRSET = SIGNALSLINGER_POWER_ENABLE_PIN;
}

static void pins_init(void)
{
	/* Immediate user feedback during the bootloader startup window. */
	PORTD.OUTSET = SIGNALSLINGER_STARTUP_LED_PINS;
	PORTD.DIRSET = SIGNALSLINGER_STARTUP_LED_PINS;

	/* Front-panel switch on PD3, active low. */
	PORTD.DIRCLR = PIN3_bm;
	PORTD.PIN3CTRL = PORT_PULLUPEN_bm;

	/* USART1 on PC0 TX, PC1 RX. */
	PORTC.DIRSET = PIN0_bm;
	PORTC.OUTSET = PIN0_bm;
	PORTC.DIRCLR = PIN1_bm;
	PORTC.PIN1CTRL = 0;
}

static void startup_leds_off(void)
{
	PORTD.OUTCLR = SIGNALSLINGER_STARTUP_LED_PINS;
}

static void startup_leds_on(void)
{
	PORTD.OUTSET = SIGNALSLINGER_STARTUP_LED_PINS;
}

static void startup_leds_toggle(void)
{
	PORTD.OUTTGL = SIGNALSLINGER_STARTUP_LED_PINS;
}

static void update_flash_activity_led(void)
{
	PORTD.OUTTGL = SIGNALSLINGER_RED_LED_PIN;
}

static void update_payload_activity_led(void)
{
	PORTD.OUTTGL = SIGNALSLINGER_GREEN_LED_PIN;
}

static void update_error_leds(void)
{
	PORTD.OUTSET = SIGNALSLINGER_RED_LED_PIN;
	PORTD.OUTCLR = SIGNALSLINGER_GREEN_LED_PIN;
}

static bool switch_is_held(void)
{
	return (PORTD.IN & PIN3_bm) == 0;
}

static void service_startup_led_feedback(uint16_t elapsed_ms)
{
	if(!switch_is_held())
	{
		startup_leds_on();
		return;
	}

	if((elapsed_ms > 0U) && ((elapsed_ms % SIGNALSLINGER_STARTUP_LED_BLINK_HALF_PERIOD_MS) == 0U))
	{
		startup_leds_toggle();
	}
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

static void usart_write_hex16(uint16_t value)
{
	usart_write_hex8((uint8_t)(value >> 8));
	usart_write_hex8((uint8_t)value);
}

static void usart_write_hex32(uint32_t value)
{
	usart_write_hex16((uint16_t)(value >> 16));
	usart_write_hex16((uint16_t)value);
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
	update_error_leds();
}

static void send_usart_error(void)
{
	usart_write_text("ERR serial ");
	usart_write_hex8(last_usart_error);
	usart_write_text("\r\n");
	update_error_leds();
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
		if((index % 64U) == 0U)
		{
			update_payload_activity_led();
		}
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
	update_error_leds();
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

static uint16_t crc_app_page(uint32_t address)
{
	uint16_t crc = 0xFFFFU;

	nvm_wait_flash();
	for(uint16_t index = 0; index < SIGNALSLINGER_FLASH_PAGE_BYTES; index++)
	{
		crc = crc16_update(crc, pgm_read_byte_far(address + index));
	}

	return crc;
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
	update_flash_activity_led();
	uint32_t address;
	if(!receive_page_address_frame(command, &address))
	{
		return;
	}

	if(erase_app_page(address))
	{
		update_flash_activity_led();
		send_ok("erase");
	}
	else
	{
		send_nvm_error();
	}
}

static void handle_write_frame(char command)
{
	update_flash_activity_led();
	uint32_t address;
	if(!receive_page_write_frame(command, &address))
	{
		return;
	}

	if(write_app_page(address))
	{
		update_flash_activity_led();
		send_ok("write");
	}
	else
	{
		send_nvm_error();
	}
}

static void handle_crc_frame(char command)
{
	uint32_t address;
	if(!receive_page_address_frame(command, &address))
	{
		return;
	}

	uint16_t page_crc = crc_app_page(address);
	usart_write_text("OK crc 0x");
	usart_write_hex32(address);
	usart_write(' ');
	usart_write_hex16(page_crc);
	usart_write_text("\r\n");
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

	asm volatile("jmp 0x2000");
}

static void set_app_handoff_power_button_state(bool button_held)
{
	GPR.GPR0 = button_held ? SIGNALSLINGER_BOOT_HANDOFF_POWER_BUTTON_HELD : 0U;
}

static void set_app_handoff_bootloader_info(void)
{
	GPR.GPR1 = SIGNALSLINGER_BOOT_HANDOFF_INFO_MAGIC |
	           (SIGNALSLINGER_BOOT_PROTOCOL_VERSION & SIGNALSLINGER_BOOT_HANDOFF_INFO_PROTOCOL_MASK);
	GPR.GPR2 = SIGNALSLINGER_BOOTLOADER_VERSION_MAJOR;
	GPR.GPR3 = SIGNALSLINGER_BOOTLOADER_VERSION_MINOR;
}

static void send_banner(void)
{
	usart_write_text("\r\nSignalSlinger ");
	usart_write_text(SIGNALSLINGER_BOOTLOADER_VERSION);
	usart_write_text(" proto=1 minproto=1 maxproto=1 app=0x2000 page=512 flash=131072 baud=115200 boot=16 write=0x2000-0x1FFFF features=appmark,pagecrc,resetlast cmds=U,R,?,E,W,C\r\n");
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
		service_startup_led_feedback(elapsed_ms);
		_delay_ms(1);
	}

	return false;
}

int main(void)
{
	cli();
	uint8_t reset_flags = read_and_clear_reset_flags();
	bool app_update_requested = app_requested_bootloader(reset_flags);
	latch_power_on();
	clock_init();
	pins_init();
	usart_init();

	bool switch_entry_allowed = !reset_was_power_start(reset_flags);
	bool stay_in_bootloader = app_update_requested ||
	                          (switch_entry_allowed && switch_is_held()) ||
	                          !app_vector_looks_programmed() ||
	                          serial_entry_requested();

	if(!stay_in_bootloader)
	{
		bool handoff_power_button_held = reset_was_power_start(reset_flags) && switch_is_held();
		set_app_handoff_power_button_state(handoff_power_button_held);
		set_app_handoff_bootloader_info();
		if(handoff_power_button_held)
		{
			startup_leds_on();
		}
		else
		{
			startup_leds_off();
		}
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
			else if(command == SIGNALSLINGER_UPDATE_REQUEST_CHAR)
			{
				usart_write_text("BOOT\r\n");
			}
			else if(command == SIGNALSLINGER_RUN_APP_CHAR && app_vector_looks_programmed())
			{
				set_app_handoff_power_button_state(false);
				set_app_handoff_bootloader_info();
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
			else if(command == SIGNALSLINGER_CRC_PAGE_CHAR)
			{
				handle_crc_frame(command);
			}
			else
			{
				send_error("unsupported");
			}
		}
	}
}
