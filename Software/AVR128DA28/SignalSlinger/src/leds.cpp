/*
 * leds.cpp
 *
 * Created: 5/5/2022 10:27:49 AM
 * Author: charl
 */

#include "defs.h"
#include "atmel_start_pins.h"
#include "leds.h"
#include "CircularStringBuff.h"
#include "globals.h"
#include <atomic.h>
#include <string.h>

#define FAST_ON 25
#define FAST_OFF 25
#define WAKE_AUTH_ON 12
#define SLOW_ON 250
#define SLOW_OFF 250
#define BRIEF_ON 15
#define BRIEF_OFF 50
#define LED_TIMEOUT_DELAY 600000

static volatile bool timer_red_blink_inhibit = false;   /* disable blinking by timer */
static volatile bool timer_green_blink_inhibit = false; /* disable blinking by timer */
static volatile bool force_wake_authorization_blink = false;
static volatile Blink_t lastRedBlinkSetting = LEDS_OFF;
static volatile Blink_t lastGreenBlinkSetting = LEDS_OFF;
static volatile Blink_t lastBothBlinkSetting = LEDS_OFF;
static volatile uint32_t led_timeout_count = LED_TIMEOUT_DELAY;
static volatile int16_t red_blink_on_period = 0;
static volatile int16_t red_blink_off_period = 0;
static volatile int16_t green_blink_on_period = 0;
static volatile int16_t green_blink_off_period = 0;
static volatile int16_t red_blink_count = 0;
static volatile int16_t green_blink_count = 0;

static uint32_t led_timeout_count_read_atomic(void)
{
	uint32_t value;
	ENTER_CRITICAL(leds_timeout_read);
	value = led_timeout_count;
	EXIT_CRITICAL(leds_timeout_read);
	return value;
}

static void led_timeout_count_write_atomic(uint32_t value)
{
	ENTER_CRITICAL(leds_timeout_write);
	led_timeout_count = value;
	EXIT_CRITICAL(leds_timeout_write);
}

static void text_buff_reset_and_disable_manual_atomic(void)
{
	ENTER_CRITICAL(leds_text_buff_reset);
	g_text_buff.reset();
	g_enable_manual_transmissions = false;
	EXIT_CRITICAL(leds_text_buff_reset);
}

/* LED enunciation timer interrupt.  This handler manages the blink state
 * machines for the red and green LEDs and enforces the overall timeout that
 * turns the indicators off after a period of inactivity.
 */
ISR(TCB1_INT_vect)
{
	uint8_t x = TCB1.INTFLAGS;

	if(x & TCB_CAPT_bm)
	{
		if(force_wake_authorization_blink)
		{
			LED_set_RED_level(ON);
			LED_set_GREEN_level(ON);

			TCB1.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* clear interrupt flag */
			return;
		}

		if(led_timeout_count)
		{
			led_timeout_count--;

			if(!led_timeout_count)
			{
				LED_set_RED_level(OFF);
				LED_set_GREEN_level(OFF);
			}
		}

		if(led_timeout_count)
		{
			if(red_blink_count && !timer_red_blink_inhibit)
			{
				if(red_blink_count > 1)
				{
					LED_set_RED_level(ON);
					red_blink_count--;
				}
				else if(red_blink_count < -1)
				{
					LED_set_RED_level(OFF);
					red_blink_count++;
				}

				if(red_blink_count == 1)
				{
					if(red_blink_off_period)
					{
						red_blink_count = -red_blink_off_period;
					}
					else /* constantly on */
					{
						red_blink_count = red_blink_on_period;
					}
				}
				else if(red_blink_count == -1)
				{
					red_blink_count = red_blink_on_period;
				}
			}
			if(green_blink_count && !timer_green_blink_inhibit)
			{
				if(green_blink_count > 1)
				{
					LED_set_GREEN_level(ON);
					green_blink_count--;
				}
				else if(green_blink_count < -1)
				{
					LED_set_GREEN_level(OFF);
					green_blink_count++;
				}

				if(green_blink_count == 1)
				{
					if(green_blink_off_period)
					{
						green_blink_count = -green_blink_off_period;
					}
					else /* constantly on */
					{
						green_blink_count = green_blink_on_period;
					}
				}
				else if(green_blink_count == -1)
				{
					green_blink_count = green_blink_on_period;
				}
			}
		}
	}

	TCB1.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* clear interrupt flag */
}

bool leds::active(void)
{
	return (led_timeout_count_read_atomic() && (TCB1.INTCTRL & (1 << TCB_CAPT_bp)));
}

void leds::deactivate(void)
{
	ENTER_CRITICAL(leds_deactivate);
	TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Disable timer interrupt */
	LED_set_RED_level(OFF);
	LED_set_GREEN_level(OFF);
	text_buff_reset_and_disable_manual_atomic();
	timer_red_blink_inhibit = timer_green_blink_inhibit = true; /* Disable timer LED control */
	lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS;
	led_timeout_count_write_atomic(0);
	EXIT_CRITICAL(leds_deactivate);
}

void leds::setRed(bool on)
{
	if(!led_timeout_count_read_atomic())
		return;

	//	TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Disable timer interrupt */
	ENTER_CRITICAL(leds_set_red);
	timer_red_blink_inhibit = true;
	lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS;

	if(on)
	{
		LED_set_RED_level(ON);
	}
	else
	{
		LED_set_RED_level(OFF);
	}
	EXIT_CRITICAL(leds_set_red);
}

void leds::setGreen(bool on)
{
	if(!led_timeout_count_read_atomic())
		return;

	//	TCB1.INTCTRL &= ~TCB_CAPT_bm;   /* Capture or Timeout: disabled */
	ENTER_CRITICAL(leds_set_green);
	timer_green_blink_inhibit = true;

	if(on)
	{
		LED_set_GREEN_level(ON);
	}
	else
	{
		LED_set_GREEN_level(OFF);
	}
	EXIT_CRITICAL(leds_set_green);
}

/* Turns off LEDs, resets the text buffer, and disables LED character transmissions. Re-enables LED timer blink functionality. */
void leds::reset(void)
{
	ENTER_CRITICAL(leds_reset);
	blink(LEDS_OFF);
	text_buff_reset_and_disable_manual_atomic();
	timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
	lastRedBlinkSetting = lastGreenBlinkSetting = lastBothBlinkSetting = LEDS_NUMBER_OF_SETTINGS;
	led_timeout_count_write_atomic(LED_TIMEOUT_DELAY);
	EXIT_CRITICAL(leds_reset);
}

/* Disables LED timer while resetting settings for interrupt safety. */
void leds::init(void)
{
	init(LEDS_OFF);
}

void leds::init(Blink_t setBlink)
{
	if(force_wake_authorization_blink)
	{
		if(setBlink != LEDS_OFF)
		{
			led_timeout_count_write_atomic(LED_TIMEOUT_DELAY);
		}
		return;
	}

	ENTER_CRITICAL(leds_init);
	TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Disable timer interrupt */
	reset();
	TCB1.INTCTRL |= TCB_CAPT_bm; /* Capture or Timeout: enabled */
	if(setBlink != LEDS_OFF)
		blink(setBlink, true);
	EXIT_CRITICAL(leds_init);
}

void leds::sendCode(char *str)
{
	if(!led_timeout_count_read_atomic())
		return;

	if(!str || !strlen(str))
	{
		return;
	}

	int lenstr = strlen(str);
	ENTER_CRITICAL(leds_send_code_text_buff);
	{
		int i = 0;
		bool holdMan = g_enable_manual_transmissions;
		g_enable_manual_transmissions = false; /* Prevent the TCB0 ISR manual-transmission path from consuming while mutating the shared buffer */

		while(!g_text_buff.full() && i < lenstr && i < TEXT_BUFF_SIZE)
		{
			g_text_buff.put(str[i++]);
		}

		timer_red_blink_inhibit = true; /* Prevent timer from controlling LED */
		lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS;
		g_enable_manual_transmissions = holdMan;
	}
	EXIT_CRITICAL(leds_send_code_text_buff);
}

void leds::setWakeAuthorizationBlink(bool active)
{
	ENTER_CRITICAL(leds_wake_auth_blink);
	bool was_active = force_wake_authorization_blink;
	force_wake_authorization_blink = active;

	if(active)
	{
		led_timeout_count_write_atomic(LED_TIMEOUT_DELAY);
		if(!was_active)
		{
			LED_set_RED_level(ON);
			LED_set_GREEN_level(ON);
		}
	}

	EXIT_CRITICAL(leds_wake_auth_blink);
}

/* Public wrapper that leaves the LED timeout unchanged. */
void leds::blink(Blink_t blinkMode)
{
	blink(blinkMode, false);
}

/* Core blink routine.  Selects the blink pattern and optionally resets
 * the automatic timeout that turns off the LEDs after inactivity.
 */
void leds::blink(Blink_t blinkMode, bool resetTimeout)
{
	if(resetTimeout)
	{
		led_timeout_count_write_atomic(LED_TIMEOUT_DELAY);
	}

	ENTER_CRITICAL(leds_blink);
	if(blinkMode == LEDS_NO_CHANGE)
	{
		EXIT_CRITICAL(leds_blink);
		return;
	}

	/* While wake authorization is active, the steady red+green indication is the
	 * only LED state that should be visible. Ignore all competing LED mode
	 * changes until the authorization flow explicitly clears the override.
	 */
	if(force_wake_authorization_blink)
	{
		EXIT_CRITICAL(leds_blink);
		return;
	}

	if(!led_timeout_count_read_atomic() && (blinkMode != LEDS_OFF))
	{
		EXIT_CRITICAL(leds_blink);
		return;
	}

	bool isRed = ((blinkMode == LEDS_RED_OFF) || (blinkMode == LEDS_RED_BLINK_FAST) || (blinkMode == LEDS_RED_BLINK_SLOW) || (blinkMode == LEDS_RED_ON_CONSTANT));
	bool isGreen = ((blinkMode == LEDS_GREEN_OFF) || (blinkMode == LEDS_GREEN_BLINK_FAST) || (blinkMode == LEDS_GREEN_BLINK_SLOW) || (blinkMode == LEDS_GREEN_ON_CONSTANT));
	bool isBoth = !isRed && !isGreen;

	if(isRed && resetTimeout)
	{
		lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS; // A user action should always reset the red LED behavior
	}

	if((isRed && (blinkMode != lastRedBlinkSetting)) || (isGreen && (blinkMode != lastGreenBlinkSetting)) || (isBoth && (blinkMode != lastBothBlinkSetting)))
	{
		TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Capture or Timeout: disabled */

		switch(blinkMode)
		{
			case LEDS_OFF:
			{
				red_blink_count = 0;
				green_blink_count = 0;
				LED_set_RED_level(OFF);
				LED_set_GREEN_level(OFF);
			}
			break;

			case LEDS_RED_OFF:
			{
				LED_set_RED_level(OFF);
				red_blink_count = 0;
				g_enable_manual_transmissions = false; /* Only the red LED can send individual characters */
			}
			break;

			case LEDS_GREEN_OFF:
			{
				LED_set_GREEN_level(OFF);
				green_blink_count = 0;
			}
			break;

			case LEDS_RED_BLINK_FAST:
			{
				red_blink_on_period = BRIEF_ON;
				red_blink_off_period = BRIEF_OFF;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_GREEN_BLINK_FAST:
			{
				green_blink_on_period = BRIEF_ON;
				green_blink_off_period = BRIEF_OFF;
				green_blink_count = green_blink_on_period;
				timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_BLINK_SLOW:
			{
				red_blink_on_period = FAST_ON;
				red_blink_off_period = SLOW_OFF;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_GREEN_BLINK_SLOW:
			{
				green_blink_on_period = SLOW_ON;
				green_blink_off_period = SLOW_OFF;
				green_blink_count = green_blink_on_period;
				timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_THEN_GREEN_BLINK_SLOW:
			{
				green_blink_on_period = SLOW_ON;
				green_blink_off_period = SLOW_OFF;
				green_blink_count = -green_blink_on_period;
				red_blink_on_period = SLOW_ON;
				red_blink_off_period = SLOW_OFF;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_THEN_GREEN_BLINK_FAST:
			{
				green_blink_on_period = FAST_ON;
				green_blink_off_period = FAST_OFF;
				green_blink_count = -green_blink_on_period;
				red_blink_on_period = FAST_ON;
				red_blink_off_period = FAST_OFF;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_AND_GREEN_BLINK_SLOW:
			{
				green_blink_on_period = SLOW_ON;
				green_blink_off_period = SLOW_OFF;
				green_blink_count = green_blink_on_period;
				red_blink_on_period = SLOW_ON;
				red_blink_off_period = SLOW_OFF;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_AND_GREEN_BLINK_FAST:
			{
				green_blink_on_period = FAST_ON;
				green_blink_off_period = FAST_OFF;
				green_blink_count = green_blink_on_period;
				red_blink_on_period = FAST_ON;
				red_blink_off_period = FAST_OFF;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_AND_GREEN_BLINK_WAKE_AUTH:
			{
				/* Wake authorization now uses a steady simultaneous red+green indication. */
				green_blink_on_period = WAKE_AUTH_ON;
				green_blink_off_period = 0;
				green_blink_count = green_blink_on_period;
				red_blink_on_period = WAKE_AUTH_ON;
				red_blink_off_period = 0;
				red_blink_count = red_blink_on_period;
				LED_set_RED_level(ON);
				LED_set_GREEN_level(ON);
				timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_RED_ON_CONSTANT:
			{
				red_blink_on_period = SLOW_ON;
				red_blink_off_period = 0;
				red_blink_count = red_blink_on_period;
				timer_red_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			case LEDS_GREEN_ON_CONSTANT:
			{
				green_blink_on_period = SLOW_ON;
				green_blink_off_period = 0;
				green_blink_count = green_blink_on_period;
				timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;

			default:
			{
			}
			break;
		}

		TCB1.INTCTRL |= TCB_CAPT_bm; /* Capture or Timeout: enabled */
	}

	if(isRed)
	{
		lastRedBlinkSetting = blinkMode;
	}
	else if(isGreen)
	{
		lastGreenBlinkSetting = blinkMode;
	}
	else
	{
		lastBothBlinkSetting = lastRedBlinkSetting = lastGreenBlinkSetting = blinkMode;
	}
	EXIT_CRITICAL(leds_blink);
}
