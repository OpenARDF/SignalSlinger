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
#include <string.h>

#define FAST_ON 25
#define FAST_OFF 25
#define SLOW_ON 250
#define SLOW_OFF 250
#define BRIEF_ON 15
#define BRIEF_OFF 50
#define LED_TIMEOUT_DELAY 60000

extern volatile bool g_event_enabled;
extern volatile bool g_enable_manual_transmissions;
extern CircularStringBuff g_text_buff;
extern Enunciation_t g_enunciator;

static volatile bool timer_red_blink_inhibit = false; /* disable blinking by timer */
static volatile bool timer_green_blink_inhibit = false; /* disable blinking by timer */
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
static volatile bool red_led_configured = false;
static volatile bool green_led_configured = false;

// default constructor
leds::leds()
{
} //leds

// default destructor
leds::~leds()
{
} //~leds

/* LED enunciation timer interrupt */
ISR(TCB1_INT_vect)
{
	uint8_t x = TCB1.INTFLAGS;
	
	if(x & TCB_CAPT_bm)
	{
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
// 			else if(red_led_configured)
// 			{
// 				LED_set_RED_level(OFF);
// 			}
			
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
// 			else if(green_led_configured)
// 			{
// 				LED_set_GREEN_level(OFF);
// 			}
		}
	}
		
	TCB1.INTFLAGS =  (TCB_CAPT_bm | TCB_OVF_bm); /* clear interrupt flag */
}


bool leds::active(void)
{
	return(led_timeout_count && (TCB1.INTCTRL & (1 << TCB_CAPT_bp)));
}

void leds::deactivate(void)
{
	TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Disable timer interrupt */
	LED_set_RED_level(OFF);
	LED_set_GREEN_level(OFF);
	g_text_buff.reset();
	g_enable_manual_transmissions = false;
	timer_red_blink_inhibit = timer_green_blink_inhibit = true; /* Disable timer LED control */
	lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS;
	led_timeout_count = 0;
}

void leds::setRed(bool on)
{
	if(!led_timeout_count) return;

//	TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Disable timer interrupt */

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
}

void leds::setGreen(bool on)
{
	if(!led_timeout_count) return;

//	TCB1.INTCTRL &= ~TCB_CAPT_bm;   /* Capture or Timeout: disabled */

	timer_green_blink_inhibit = true;

	if(on)
	{
		LED_set_GREEN_level(ON);
	}
	else
	{
		LED_set_GREEN_level(OFF);
	}
}

/* Turns off LEDs, resets the text buffer, and disables LED character transmissions. Re-enables LED timer blink functionality. */
void leds::reset(void)
{
	blink(LEDS_OFF);
	g_text_buff.reset();
	g_enable_manual_transmissions = false;
	timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
	lastRedBlinkSetting = lastGreenBlinkSetting = lastBothBlinkSetting = LEDS_NUMBER_OF_SETTINGS; 
	led_timeout_count = LED_TIMEOUT_DELAY;
}

/* Disables LED timer while resetting settings for interrupt safety. */
void leds::init(void)
{
	init(LEDS_OFF);
}

void leds::init(Blink_t setBlink)
{
	TCB1.INTCTRL &= ~TCB_CAPT_bm; /* Disable timer interrupt */
	reset();
	TCB1.INTCTRL |= TCB_CAPT_bm;   /* Capture or Timeout: enabled */
	if(setBlink != LEDS_OFF) blink(setBlink, true);
}

void leds::sendCode(char* str)
{
	if(!led_timeout_count) return;
	
	if(!str || !strlen(str))
	{
		return;
	}

	int lenstr = strlen(str);					
	int i = 0;

	bool holdMan = g_enable_manual_transmissions;
	g_enable_manual_transmissions = false; /* simple thread collision avoidance */
	
	while(!g_text_buff.full() && i<lenstr && i<TEXT_BUFF_SIZE)
	{
		g_text_buff.put(str[i++]);
	}
	
	timer_red_blink_inhibit = true; /* Prevent timer from controlling LED */
	lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS;
	g_enable_manual_transmissions = holdMan;
}

void leds::blink(Blink_t blinkMode)
{
	blink(blinkMode, false);
}

void leds::blink(Blink_t blinkMode, bool resetTimeout)
{
	if(resetTimeout)
	{
		led_timeout_count = LED_TIMEOUT_DELAY;
	}
	
	if(blinkMode == LEDS_NO_CHANGE) return;
	if(!led_timeout_count && (blinkMode != LEDS_OFF)) return;
	
	bool isRed = ((blinkMode == LEDS_RED_OFF) || (blinkMode == LEDS_RED_BLINK_FAST) || (blinkMode == LEDS_RED_BLINK_SLOW) || (blinkMode == LEDS_RED_ON_CONSTANT));
	bool isGreen = ((blinkMode == LEDS_GREEN_OFF) || (blinkMode == LEDS_GREEN_BLINK_FAST) || (blinkMode == LEDS_GREEN_BLINK_SLOW) || (blinkMode == LEDS_GREEN_ON_CONSTANT));
	bool isBoth = !isRed && !isGreen;	
	
	if(isRed && resetTimeout)
	{
		lastRedBlinkSetting = LEDS_NUMBER_OF_SETTINGS; // A user action should always reset the red LED behavior
	}
		
	if((isRed && (blinkMode != lastRedBlinkSetting)) || (isGreen && (blinkMode != lastGreenBlinkSetting)) || (isBoth && (blinkMode != lastBothBlinkSetting)))
	{
		TCB1.INTCTRL &= ~TCB_CAPT_bm;   /* Capture or Timeout: disabled */

		switch(blinkMode)
		{
			case LEDS_OFF:
			{
				red_blink_count = 0;
				green_blink_count = 0;
				LED_set_RED_level(OFF);
				LED_set_GREEN_level(OFF);
				red_led_configured = false;
				green_led_configured = false;
			}
			break;
			
			case LEDS_RED_OFF:
			{
				LED_set_RED_level(OFF);
				red_blink_count = 0;
				red_led_configured = false;
				g_enable_manual_transmissions = false; /* Only the red LED can send individual characters */
			}
			break;
			
			case LEDS_GREEN_OFF:
			{
				LED_set_GREEN_level(OFF);
				green_blink_count = 0;
				green_led_configured = false;
			}
			break;
			
			case LEDS_RED_BLINK_FAST:
			{
				red_blink_on_period = BRIEF_ON;
				red_blink_off_period = BRIEF_OFF;
				red_blink_count = red_blink_on_period;	
				red_led_configured = true;			
				timer_red_blink_inhibit = false; /* Enable timer LED control */
			}
			break;
			
			case LEDS_GREEN_BLINK_FAST:
			{
				green_blink_on_period = BRIEF_ON;
				green_blink_off_period = BRIEF_OFF;	
				green_blink_count = green_blink_on_period;			
				green_led_configured = true;			
				timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;
			
			case LEDS_RED_BLINK_SLOW:
			{
				red_blink_on_period = FAST_ON;
				red_blink_off_period = SLOW_OFF;
				red_blink_count = red_blink_on_period;				
				red_led_configured = true;			
				timer_red_blink_inhibit = false; /* Enable timer LED control */
			}
			break;
			
			case LEDS_GREEN_BLINK_SLOW:
			{
				green_blink_on_period = SLOW_ON;
				green_blink_off_period = SLOW_OFF;	
				green_blink_count = green_blink_on_period;			
				green_led_configured = true;			
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
				red_led_configured = true;			
				green_led_configured = true;			
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
				red_led_configured = true;			
				green_led_configured = true;			
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
				red_led_configured = true;			
				green_led_configured = true;			
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
				red_led_configured = true;			
				green_led_configured = true;			
				timer_red_blink_inhibit = timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;
			
			case LEDS_RED_ON_CONSTANT:
			{
				red_blink_on_period = SLOW_ON;
				red_blink_off_period = 0;
				red_blink_count = red_blink_on_period;
				red_led_configured = true;			
				timer_red_blink_inhibit = false; /* Enable timer LED control */
			}
			break;
			
			case LEDS_GREEN_ON_CONSTANT:
			{
				green_blink_on_period = SLOW_ON;
				green_blink_off_period = 0;
				green_blink_count = green_blink_on_period;
				green_led_configured = true;			
				timer_green_blink_inhibit = false; /* Enable timer LED control */
			}
			break;
			
			default:
			{
				
			}
			break;
		}
		
		TCB1.INTCTRL |= TCB_CAPT_bm;   /* Capture or Timeout: enabled */
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
}
