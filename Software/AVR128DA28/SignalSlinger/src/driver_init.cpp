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


#include "defs.h"
#include "driver_init.h"
#include <system.h>
#include "usart_basic.h"
#include "serialbus.h"
#include "dac0.h"
#include "binio.h"
#include "rtc.h"


/**
 * \brief System initialization
 */
void system_init()
{
	mcu_init();

	CLKCTRL_init(); /* Set CPU clock speed appropriately */
	TIMERB_init(); /* Timers must be initialized before utility_delay functions will work */
	RTC_init(); /* Start 1-second timer and interrupts */
	CPUINT_init(); /* Interrupts must also be enabled before timer interrupts will function */
	BINIO_init();

	SLPCTRL_init();
	
	// DAC0_init();

	serialbus_init(SB_BAUD, SERIALBUS_USART);

	BOD_init();
}

void system_sleep_config()
{
	mcu_init();

//	CLKCTRL_init(); /* Set CPU clock speed appropriately */
	TIMERB_sleep(); /* Timers must be initialized before utility_delay functions will work */
//	CPUINT_init(); /* Interrupts must also be enabled before timer interrupts will function */
//	BINIO_init();

	LED_set_RED_dir(PORT_DIR_OUT);
	LED_set_RED_level(OFF);
	LED_set_GREEN_dir(PORT_DIR_OUT);
	LED_set_GREEN_level(OFF);

	PORTA_set_pin_dir(FAN_CONTR, PORT_DIR_OUT);
	PORTA_set_pin_level(FAN_CONTR, LOW);
	PORTA_set_pin_dir(FET_DRIVER_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(FET_DRIVER_ENABLE, LOW);
	PORTA_set_pin_dir(V3V3_PWR_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(V3V3_PWR_ENABLE, LOW);
// 	PORTA_set_pin_dir(POWER_ENABLE, PORT_DIR_OUT); /* Must not disable battery power if running off of battery */
// 	PORTA_set_pin_level(POWER_ENABLE, LOW); /* Must not disable battery power if running off of battery */
	PORTA_set_pin_dir(TO_WIFI_RX, PORT_DIR_OUT);
	PORTA_set_pin_level(TO_WIFI_RX, LOW);
	PORTA_set_pin_dir(TO_WIFI_TX, PORT_DIR_OUT);
	PORTA_set_pin_level(TO_WIFI_TX, LOW);
	PORTA_set_pin_dir(PS_5V_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(PS_5V_ENABLE, LOW);
	PORTA_set_pin_dir(WIFI_ENABLE, PORT_DIR_OUT);
	PORTA_set_pin_level(WIFI_ENABLE, LOW);

	PORTC_set_pin_dir(SERIAL_TX, PORT_DIR_OUT);
	PORTC_set_pin_level(SERIAL_TX, LOW); /* Leave port serial line low */
	PORTC_set_pin_dir(SERIAL_RX, PORT_DIR_OUT);
	PORTC_set_pin_level(SERIAL_RX, LOW); /* Leave port serial line low */

	PORTC_set_pin_dir(SI5351_SDA, PORT_DIR_OUT);
	PORTC_set_pin_level(SI5351_SDA, LOW);
	PORTC_set_pin_dir(SI5351_SCL, PORT_DIR_OUT);
	PORTC_set_pin_level(SI5351_SCL, LOW);
	
	PORTD_set_pin_dir(unusedD5, PORT_DIR_OUT);
	PORTD_set_pin_level(unusedD5, LOW);
	PORTD_set_pin_dir(unusedD7, PORT_DIR_OUT);
	PORTD_set_pin_level(unusedD7, LOW);

	PORTF_set_pin_dir(unusedF1, PORT_DIR_OUT);
	PORTF_set_pin_level(unusedF1, LOW);
	PORTF_set_pin_dir(unusedF6, PORT_DIR_OUT);
	PORTF_set_pin_level(unusedF6, LOW);
}

