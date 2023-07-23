#include "atmel_start.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/sleep.h>
#include <atomic.h>

#include "serialbus.h"
#include "transmitter.h"
#include "morse.h"
#include "adc.h"
#include "util.h"
#include "binio.h"
#include "eeprommanager.h"
#include "binio.h"
#include "leds.h"
#include "CircularStringBuff.h"
#include "rtc.h"

#include <cpuint.h>
#include <ccp.h>
#include <atomic.h>


/***********************************************************************
 * Local Typedefs
 ************************************************************************/

typedef enum
{
	WD_SW_RESETS,
	WD_HW_RESETS,
	WD_FORCE_RESET,
	WD_DISABLE
} WDReset;

typedef enum
{
	POWER_UP_START,
	AWAKENED_BY_CLOCK,
	AWAKENED_BY_BUTTONPRESS
} Awakened_t;

typedef enum
{
	HARDWARE_OK,
	HARDWARE_NO_RTC = 0x01,
	HARDWARE_NO_SI5351 = 0x02,
//	HARDWARE_NO_WIFI = 0x04
} HardwareError_t;


typedef enum {
	SYNC_Searching_for_slave,
	SYNC_Align_to_Second_Transition,
	SYNC_Waiting_for_CLK_T_reply,
	SYNC_Waiting_for_CLK_S_reply,
	SYNC_Waiting_for_CLK_F_reply,
	SYNC_Waiting_for_CLK_D_reply,
	SYNC_Waiting_for_ID_reply,
	SYNC_Waiting_for_ID_CodeSpeed_reply,
	SYNC_Waiting_for_Pattern_CodeSpeed_reply,
	SYNC_Waiting_for_EVT_reply,
	SYNC_Waiting_for_NoEvent_Freq_reply,
	SYNC_Waiting_for_Freq_Low_reply,
	SYNC_Waiting_for_Freq_Med_reply,
	SYNC_Waiting_for_Freq_Hi_reply,
	SYNC_Waiting_for_Freq_Beacon_reply,
// 	SYNC_Waiting_for_PAT_A_reply,
// 	SYNC_Waiting_for_PAT_B_reply,
// 	SYNC_Waiting_for_PAT_C_reply,
// 	SYNC_Waiting_for_FRE_A_reply,
// 	SYNC_Waiting_for_FRE_B_reply,
// 	SYNC_Waiting_for_FRE_C_reply,
	SYNC_Waiting_for_ACK
} SyncState_t;

#define PROGRAMMING_MESSAGE_TIMEOUT_PERIOD 9000


/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
static char g_tempStr[50] = { '\0' };
static volatile EC g_last_error_code = ERROR_CODE_NO_ERROR;
static volatile SC g_last_status_code = STATUS_CODE_IDLE;

static volatile bool g_powering_off = false;

static volatile uint16_t g_util_tick_countdown = 0;
static volatile bool g_battery_measurements_active = false;
static volatile uint16_t g_maximum_battery = 0;

static volatile bool g_start_event = false;

static volatile int32_t g_on_the_air = 0;
static volatile int g_sendID_seconds_countdown = 0;
static volatile uint16_t g_code_throttle = 50;
static volatile uint16_t g_sleepshutdown_seconds = 120;
static volatile bool g_report_seconds = false;
static volatile int g_hardware_error = (int)HARDWARE_OK;

char g_messages_text[STATION_ID+1][MAX_PATTERN_TEXT_LENGTH + 1];
volatile uint8_t g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
volatile uint8_t g_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
volatile uint8_t g_foxoring_pattern_codespeed = EEPROM_FOXORING_PATTERN_CODESPEED_DEFAULT;
volatile uint16_t g_time_needed_for_ID = 0;
volatile int16_t g_on_air_seconds = EEPROM_ON_AIR_TIME_DEFAULT;                      /* amount of time to spend on the air */
volatile int16_t g_off_air_seconds = EEPROM_OFF_AIR_TIME_DEFAULT;                    /* amount of time to wait before returning to the air */
volatile int16_t g_intra_cycle_delay_time = EEPROM_INTRA_CYCLE_DELAY_TIME_DEFAULT;   /* offset time into a repeating transmit cycle */
volatile int16_t g_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;              /* amount of time between ID/callsign transmissions */
volatile time_t g_event_start_epoch = EEPROM_START_TIME_DEFAULT;
volatile time_t g_event_finish_epoch = EEPROM_FINISH_TIME_DEFAULT;
volatile bool g_event_enabled = EEPROM_EVENT_ENABLED_DEFAULT;                        /* indicates that the conditions for executing the event are set */
volatile bool g_event_commenced = false;
volatile float g_voltage_threshold = EEPROM_BATTERY_THRESHOLD_V;
volatile float g_battery_voltage = 0.;
volatile bool g_seconds_transition = false;
volatile bool g_sending_station_ID = false;											/* Allows a small extension of transmissions to ensure the ID is fully sent */
volatile bool g_muteAfterID = false;												/* Inhibit any transmissions after the ID has been sent */
volatile uint32_t g_event_checksum = 0;
volatile uint8_t g_days_to_run = 1;
volatile uint8_t g_days_run = 0;
extern uint16_t g_clock_calibration;

static volatile bool g_run_event_forever = false;
static volatile bool g_go_to_sleep_now = false;
static volatile bool g_sleeping = false;
static volatile time_t g_time_to_wake_up = 0;
static volatile Awakened_t g_awakenedBy = POWER_UP_START;
static volatile SleepType g_sleepType = SLEEP_FOREVER;

#define NUMBER_OF_POLLED_ADC_CHANNELS 1
static ADC_Active_Channel_t g_adcChannelOrder[NUMBER_OF_POLLED_ADC_CHANNELS] = { ADCExternalBatteryVoltage };
enum ADC_Result_Order { EXTERNAL_BATTERY_VOLTAGE };
static const uint16_t g_adcChannelConversionPeriod_ticks[NUMBER_OF_POLLED_ADC_CHANNELS] = { TIMER2_0_5HZ };
static volatile uint16_t g_adcCountdownCount[NUMBER_OF_POLLED_ADC_CHANNELS] = { 2000 };
//static uint16_t g_ADCFilterThreshold[NUMBER_OF_POLLED_ADC_CHANNELS] = { 500, 500, 500 };
static volatile bool g_adcUpdated[NUMBER_OF_POLLED_ADC_CHANNELS] = { false };
static volatile uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS];

volatile uint16_t g_switch_closed_time = 0;
volatile uint16_t g_handle_counted_presses = 0;
volatile uint16_t g_switch_presses_count = 0;
volatile bool g_long_button_press = false;

static volatile uint16_t g_programming_countdown = 0;
static volatile uint16_t g_programming_msg_throttle = 0;
static volatile uint16_t g_send_clone_success_countdown = 0;
static SyncState_t g_programming_state = SYNC_Searching_for_slave;
bool g_cloningInProgress = false;
Enunciation_t g_enunciator = LED_ONLY;

uint16_t g_Event_Configuration_Check = 0;

leds LEDS = leds();
CircularStringBuff g_text_buff = CircularStringBuff(TEXT_BUFF_SIZE);

EepromManager g_ee_mgr;

bool g_isMaster = false;
uint16_t isMasterCountdownSeconds = 0;
Fox_t g_fox[] = {FOX_1, FOX_1, SPRINT_S1, FOXORING_FOX1, INVALID_FOX}; /* none, classic, sprint, foxoring, blind */

Event_t g_event = EEPROM_EVENT_SETTING_DEFAULT;
Frequency_Hz g_frequency = EEPROM_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_low = EEPROM_FREQUENCY_LOW_DEFAULT;
Frequency_Hz g_frequency_med = EEPROM_FREQUENCY_MED_DEFAULT;
Frequency_Hz g_frequency_hi = EEPROM_FREQUENCY_HI_DEFAULT;
Frequency_Hz g_frequency_beacon = EEPROM_FREQUENCY_BEACON_DEFAULT;

int8_t g_utc_offset;
uint8_t g_unlockCode[UNLOCK_CODE_SIZE + 1];

volatile bool g_enable_manual_transmissions = false;

/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
void handle_1sec_tasks(void);
bool eventEnabled(void);
void handleSerialBusMsgs(void);
void wdt_init(WDReset resetType);
uint16_t throttleValue(uint8_t speed);
EC activateEventUsingCurrentSettings(SC* statusCode);
EC launchEvent(SC* statusCode);
EC hw_init(void);
EC rtc_init(void);
bool antennaIsConnected(void);
void powerDown3V3(void);
void powerUp3V3(void);
char* convertEpochToTimeString(time_t epoch, char* buf, size_t size);
time_t String2Epoch(bool *error, char *datetime);
void reportSettings(void);
uint16_t timeNeededForID(void);
Frequency_Hz getFrequencySetting(void);
char* getCurrentPatternText(void);
int getPatternCodeSpeed(void);
int getFoxCodeSpeed(void);
Fox_t getFoxSetting(void);
void handleSerialCloning(void);
bool eventScheduled(void);
bool eventScheduledForNow(void);
bool eventScheduledForTheFuture(void);
void syncSystemTimeToRTC(void);

/*******************************/
/* Hardcoded event support     */
/*******************************/
void initializeAllEventSettings(bool disableEvent);
void suspendEvent(void);
void stopEventNow(EventActionSource_t activationSource);
void startEventNow(EventActionSource_t activationSource);
bool startEventUsingRTC(void);
void setupForFox(Fox_t fox, EventAction_t action);
time_t validateTimeString(char* str, time_t* epochVar);
bool reportTimeTill(time_t from, time_t until, const char* prefix, const char* failMsg);
ConfigurationState_t clockConfigurationCheck(void);
void reportConfigErrors(void);
/*******************************/
/* End hardcoded event support */
/*******************************/

ISR(RTC_CNT_vect)
{
	uint8_t x = RTC.INTFLAGS;
	
    if (x & RTC_OVF_bm )
    {
        system_tick();

		if(g_on_the_air < 0)
		{
			g_on_the_air++;
		}
		else if(g_on_the_air > 0)
		{
			g_on_the_air--;
		}
		
		g_seconds_transition = true;

		if(g_sleeping)
		{
			if(g_sleepType != SLEEP_FOREVER)
			{
				if(g_sleepType == SLEEP_UNTIL_NEXT_XMSN)
				{
					if((g_on_the_air > -6) || (g_time_to_wake_up <= time(null))) /* Always wake up at least 5 seconds before showtime */
					{
						g_go_to_sleep_now = false;
						g_sleeping = false;
						g_awakenedBy = AWAKENED_BY_CLOCK;
					}
				}
				else
				{
					if(g_time_to_wake_up <= time(null))
					{
						g_go_to_sleep_now = false;
						g_sleeping = false;
						g_awakenedBy = AWAKENED_BY_CLOCK;
					}
				}
			}
		}
		else
		{
			handle_1sec_tasks();
		}
	}
 
    RTC.INTFLAGS = (RTC_OVF_bm | RTC_CMP_bm);
}


/**
1-Second Interrupts:
One-second counter based on RTC.
*/
void handle_1sec_tasks(void)
{
	time_t temp_time = 0;
	
	if(isMasterCountdownSeconds) isMasterCountdownSeconds--;

	if(!g_cloningInProgress)
	{
		if(g_event_commenced && !g_run_event_forever)
		{
			if(g_event_finish_epoch)
			{
				temp_time = time(null);

				if(temp_time >= g_event_finish_epoch)
				{
					g_last_status_code = STATUS_CODE_EVENT_FINISHED;
					g_on_the_air = 0;
					keyTransmitter(OFF);
					g_event_enabled = false;
					g_event_commenced = false;
					g_sleepshutdown_seconds = 120;
					LEDS.init();
				
					if(g_days_run < g_days_to_run)
					{
						g_event_start_epoch += SECONDS_24H;
						g_event_finish_epoch += SECONDS_24H;
						g_sleepType = SLEEP_UNTIL_START_TIME;
						g_go_to_sleep_now = true;
						g_days_run++;
					}
					else
					{
						g_sleepType = SLEEP_FOREVER;
						g_go_to_sleep_now = true;
					}
				}
			}
		}

		if(g_event_enabled && !g_isMaster)
		{
			if(g_event_commenced) /* an event is in progress */
			{
				if(g_sendID_seconds_countdown)
				{
					g_sendID_seconds_countdown--;
				}
			}
			else /* waiting for the start time to arrive */
			{
				if(g_event_start_epoch > MINIMUM_VALID_EPOCH) /* a start time has been set */
				{
					temp_time = time(null);

					if(temp_time >= g_event_start_epoch) /* Time for the event to start */
					{
						powerToTransmitter(ON);
				
						if(g_intra_cycle_delay_time)
						{
							g_last_status_code = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
							g_on_the_air = -g_intra_cycle_delay_time;
							g_sendID_seconds_countdown = g_intra_cycle_delay_time + g_on_air_seconds - g_time_needed_for_ID;
						}
						else
						{
							g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
							g_on_the_air = g_on_air_seconds;
							g_sendID_seconds_countdown = g_on_air_seconds - g_time_needed_for_ID;
							g_code_throttle = throttleValue(g_pattern_codespeed);
							bool repeat = true;
							makeMorse(getCurrentPatternText(), &repeat, NULL);
						}

						g_event_commenced = true;
						LEDS.init();
					}
				}
			}
		}
	}

	/**************************************
	* Delay before sleep
	***************************************/
	if(g_sleepshutdown_seconds) g_sleepshutdown_seconds--;

	if(!g_sleepshutdown_seconds)
	{
 		if(g_sleepType == DO_NOT_SLEEP)
		{
			if(!g_event_enabled && !eventScheduled()) 
			{
				g_sleepType = SLEEP_FOREVER;
			}
		}
		else if(g_isMaster || g_cloningInProgress)
		{
			g_sleepshutdown_seconds = 120;
		}
		else
		{
			LEDS.init();
			g_go_to_sleep_now = true;								
		}
	}
}


/**
Periodic tasks not requiring precise timing. Rate = 300 Hz
*/
ISR(TCB0_INT_vect)
{
	static bool initializeManualTransmissions = true;
	static uint8_t fiftyMS = 6;
	static bool on_air_finished = false;
	static bool transitionPrepped = false;
	
	uint8_t x = TCB0.INTFLAGS;
	
    if(x & TCB_CAPT_bm)
    {
		static bool conversionInProcess = false;
		static int8_t indexConversionInProcess = 0;
		static uint16_t codeInc = 0;
		bool repeat, finished;
		static uint16_t switch_closures_count_period = 0;
		uint8_t holdSwitch;
		static uint8_t buttonReleased = false;
		static uint8_t longPressEnabled = true;
		static bool muteAfterID = false;				/* Inhibit any transmissions immediately after the ID has been sent */
		
		fiftyMS++;
		if(!(fiftyMS % 6))
		{
			holdSwitch = portDdebouncedVals() & (1 << SWITCH);
			debounce();
			
			if(holdSwitch != (portDdebouncedVals() & (1 << SWITCH))) /* Change detected */
			{	
				if(holdSwitch) /* Switch was open, so now it must be closed */
				{
					if(!LEDS.active())
					{
						LEDS.init();
					}
					else
					{
						g_switch_presses_count++;
						buttonReleased = false;
						if(g_switch_presses_count == 1)
						{
							serialbus_init(SB_BAUD, SERIALBUS_USART);
						}
					}
				}
				else /* Switch is now open */
				{
					g_switch_closed_time = 0;
					buttonReleased = true;
					longPressEnabled = true;
					
					if(g_send_clone_success_countdown || g_cloningInProgress) 
					{
						g_send_clone_success_countdown = 0;
						g_cloningInProgress = false;
						g_programming_msg_throttle = 0;
					}
				}
			}
			else if(!holdSwitch) /* Switch closed */
			{
				if(!g_long_button_press && longPressEnabled)
				{
					if(++g_switch_closed_time >= 200)
					{
						g_long_button_press = true;
						g_switch_closed_time = 0;
						g_switch_presses_count = 0;
						longPressEnabled = false;
					}
				}
			}
		
			if(switch_closures_count_period)
			{
				switch_closures_count_period--;
				
				if(!switch_closures_count_period)
				{
					if(g_switch_presses_count && (g_switch_presses_count < 3))
					{
						g_handle_counted_presses = g_switch_presses_count;
					}
					
					g_switch_presses_count = 0;
				}
			}
			else if(g_switch_presses_count == 1 && buttonReleased)
			{
				switch_closures_count_period = 50;
			}
			else if(g_switch_presses_count > 2)
			{
				g_switch_presses_count = 0;
			}
		}
		
		if(g_util_tick_countdown)
		{
			g_util_tick_countdown--;
		}
		
		if(g_programming_countdown > 0) g_programming_countdown--;
		if(g_programming_msg_throttle) g_programming_msg_throttle--;
		if(g_send_clone_success_countdown) g_send_clone_success_countdown--;
							
		static bool key = false;

		if(g_event_enabled && g_event_commenced) /* Handle cycling transmissions */
		{
			initializeManualTransmissions = true;
			
			if((g_on_the_air > 0) || (g_sending_station_ID) || (!g_off_air_seconds))
			{
				on_air_finished = true;
				transitionPrepped = false;
				
				if(!g_sending_station_ID && (!g_off_air_seconds || (g_on_the_air <= g_time_needed_for_ID)) && !g_sendID_seconds_countdown && g_time_needed_for_ID)
				{
					g_last_status_code = STATUS_CODE_SENDING_ID;
					g_code_throttle = throttleValue(g_id_codespeed);
					repeat = false;
					makeMorse(g_messages_text[STATION_ID], &repeat, NULL);  /* Send only once */
					g_sending_station_ID = true;
					g_sendID_seconds_countdown = g_ID_period_seconds;
				}
				
				if(codeInc)
				{
					codeInc--;

					if(!codeInc)
					{
						key = makeMorse(NULL, &repeat, &finished);
						
						if(!repeat && finished) /* ID has completed, so resume pattern */
						{
							g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
							g_code_throttle = throttleValue(g_pattern_codespeed);
							repeat = true;
							makeMorse(getCurrentPatternText(), &repeat, NULL);
							muteAfterID = g_sending_station_ID && g_off_air_seconds;
							g_sending_station_ID = false;
							if(!g_off_air_seconds)
							{
								g_on_the_air = g_on_air_seconds;
							}
						}
						
						codeInc = g_code_throttle;
					}
				}
				else
				{
					codeInc = g_code_throttle;
				}
				
				if(muteAfterID)
				{
					key = OFF;
				}
				
				keyTransmitter(key);
				LEDS.setRed(key);
			}
			else if(g_on_the_air < 0)
			{
				transitionPrepped = false;
			}
			else if(!g_on_the_air)
			{
				if(!transitionPrepped)
				{
					transitionPrepped = true;
					
					if(on_air_finished)
					{
// 						if(g_off_air_seconds)
// 						{
						keyTransmitter(OFF);
				
						if(key)
						{
							key = OFF;
							keyTransmitter(OFF);
						}
						
						g_on_the_air = -g_off_air_seconds;
						on_air_finished = false;							
						/* Enable sleep during off-the-air periods */
						int32_t timeRemaining = 0;
						time_t temp_time = time(null);
						if(temp_time < g_event_finish_epoch)
						{
							timeRemaining = timeDif(g_event_finish_epoch, temp_time);
							g_last_status_code = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
						}

						/* Don't sleep for the last cycle to ensure that the event doesn't end while
						* the transmitter is sleeping - which can cause problems with loading the next event */
						if(timeRemaining > (g_off_air_seconds + g_on_air_seconds + 15))
						{
							if(g_off_air_seconds > 15) /* Don't bother to sleep if the off-air time is short */
							{
								time_t seconds_to_sleep = (time_t)(g_off_air_seconds - 10);
								g_time_to_wake_up = temp_time + seconds_to_sleep;
								g_sleepType = SLEEP_UNTIL_NEXT_XMSN;
								g_go_to_sleep_now = true;
								g_sendID_seconds_countdown = MAX(0, g_ID_period_seconds - (int)seconds_to_sleep);
							}
						}
// 						}
// 						else /* Transmissions are continuous */
// 						{
// 							g_on_the_air = g_on_air_seconds;
// 							g_code_throttle = throttleValue(g_pattern_codespeed);
// 						}
	
						muteAfterID = false;
						g_sending_station_ID = false;
				
						/* Resume normal pattern */
						repeat = true;
						makeMorse(getCurrentPatternText(), &repeat, NULL);    /* Reset pattern to start */
						LEDS.setRed(OFF);
					}
					else /* Off-the-air period just finished, or the event just began while off the air */
					{
						g_on_the_air = g_on_air_seconds;
						on_air_finished = false;
						g_code_throttle = throttleValue(g_pattern_codespeed);
						repeat = true;
						makeMorse(getCurrentPatternText(), &repeat, NULL);
						codeInc = g_code_throttle;
					}
				}
			}
		}
		else if(g_enable_manual_transmissions) /* Handle single-character transmissions */
		{
			static bool charFinished = true;
			static bool idle = true;
			bool sendBuffEmpty = g_text_buff.empty();
			repeat = false;
			
			if(initializeManualTransmissions)
			{
				initializeManualTransmissions = false;
				g_text_buff.reset();
				makeMorse((char*)"\0", &repeat, null); /* reset makeMorse */
				sendBuffEmpty = true;
				charFinished = true;
			}
			
			if(sendBuffEmpty && charFinished)
			{
				if(!idle)
				{
					if(key)
					{
						key = OFF;
						keyTransmitter(OFF);
						LEDS.setRed(OFF);
					}
				
					codeInc = g_code_throttle;
					makeMorse((char*)"\0", &repeat, null); /* reset makeMorse */
					idle = true;
				}
			}
			else 
			{
				idle = false;
				
				if(codeInc)
				{
					codeInc--;

					if(!codeInc)
					{
						key = makeMorse(null, &repeat, &charFinished);

						if(charFinished) /* Completed, send next char */
						{
							if(!g_text_buff.empty())
							{
								static char cc[2]; /* Must be static because makeMorse saves only a pointer to the character array */
								g_code_throttle = throttleValue(getPatternCodeSpeed());
								cc[0] = g_text_buff.get();
								cc[1] = '\0';
								makeMorse(cc, &repeat, null);
								key = makeMorse(null, &repeat, &charFinished);
							}
						}

						if(g_enunciator == LED_AND_RF) keyTransmitter(key);
						LEDS.setRed(key);
						codeInc = g_code_throttle;
					}
				}
				else
				{
					if(g_enunciator == LED_AND_RF) keyTransmitter(key);
					LEDS.setRed(key);
					codeInc = g_code_throttle;
				}
			}
		}

		/**
		 * Handle Periodic ADC Readings
		 * The following algorithm allows multiple ADC channel readings to be performed at different polling intervals. */
  		if(!conversionInProcess)
  		{
 			/* Note: countdowns will pause while a conversion is in process. Conversions are so fast that this should not be an issue though. */
 			indexConversionInProcess = -1;
 
 			for(uint8_t i = 0; i < NUMBER_OF_POLLED_ADC_CHANNELS; i++)
 			{
 				if(g_adcCountdownCount[i])
 				{
 					g_adcCountdownCount[i]--;
 				}
 
 				if(g_adcCountdownCount[i] == 0)
 				{
 					indexConversionInProcess = (int8_t)i;
 				}
 			}

 			if(indexConversionInProcess >= 0)
 			{
 				g_adcCountdownCount[indexConversionInProcess] = g_adcChannelConversionPeriod_ticks[indexConversionInProcess];    /* reset the tick countdown */
 				ADC0_setADCChannel(g_adcChannelOrder[indexConversionInProcess]);
 				ADC0_startConversion();
 				conversionInProcess = true;
 			}
 		}
 		else if(ADC0_conversionDone())   /* wait for conversion to complete */
 		{
 			uint16_t hold = ADC0_read(); //ADC;
 			
 			if((hold > 10) && (hold < 4090))
 			{
 				g_adcUpdated[indexConversionInProcess] = true;
  				g_lastConversionResult[indexConversionInProcess] = hold;
				g_battery_voltage = (0.00705 * (float)g_lastConversionResult[EXTERNAL_BATTERY_VOLTAGE]) + 0.05;
 			}
 
 			conversionInProcess = false;
 		}
    }

    TCB0.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* clear all interrupt flags */
}

/**
Handle switch closure interrupts
*/
ISR(PORTD_PORT_vect)
{
	uint8_t x = VPORTD.INTFLAGS;
	
	if(x & (1 << SWITCH))
	{
		if(g_sleeping)
		{
			g_go_to_sleep_now = false;
			g_sleeping = false;
			g_awakenedBy = AWAKENED_BY_BUTTONPRESS;	
		}
		
		g_sleepshutdown_seconds = MAX(120U, g_sleepshutdown_seconds);
	}
	
	VPORTD.INTFLAGS = 0xFF; /* Clear all flags */
}


void powerDown3V3(void)
{
	PORTA_set_pin_level(RF_OUT_ENABLE, LOW);
	PORTA_set_pin_level(V3V3_PWR_ENABLE, LOW);
	PORTD_set_pin_level(VDIV_ENABLE, LOW);
}

void powerUp3V3(void)
{
	PORTA_set_pin_level(V3V3_PWR_ENABLE, HIGH);	
	PORTA_set_pin_level(RF_OUT_ENABLE, LOW);
	PORTD_set_pin_level(VDIV_ENABLE, HIGH);
}

int main(void)
{
	atmel_start_init();
	LEDS.blink(LEDS_OFF, true);
	powerUp3V3();
	
	g_ee_mgr.initializeEEPROMVars();
	g_ee_mgr.readNonVols();
	g_isMaster = false; /* Never start up as master */
	
	RTC_set_calibration(g_clock_calibration);
					
	/* Check that the RTC is running */
	set_system_time(YEAR_2000_EPOCH);
	time_t now = time(null);
	while((util_delay_ms(2000)) && (now == time(null)));
	
	sb_send_string((char*)PRODUCT_NAME_LONG);
	sprintf(g_tempStr, "\n* SW Ver: %s\n", SW_REVISION);
	sb_send_string(g_tempStr);
	sb_send_string(TEXT_RESET_OCCURRED_TXT);
	
	if(now == time(null))
	{
		g_hardware_error |= (int)HARDWARE_NO_RTC;
		RTC_init_backup();
		sb_send_string(TEXT_RTC_NOT_RESPONDING_TXT);	
	}
	
	g_sleepshutdown_seconds = 120;
	
	if(init_transmitter(getFrequencySetting()) != ERROR_CODE_NO_ERROR)
	{
		if(!txIsInitialized())
		{
			g_hardware_error |= (int)HARDWARE_NO_SI5351;
			sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);	
		}
	}
	
	g_start_event = eventEnabled(); /* Start any event stored in EEPROM */
	
	reportSettings();
	sb_send_NewPrompt();

	while (1) {
		if(g_handle_counted_presses)
		{
			if(g_handle_counted_presses == 1)
			{
				if(!g_isMaster && !g_cloningInProgress)
				{
					if(eventScheduled())
					{
						g_event_enabled = false;
						
						if(!g_run_event_forever)
						{
							setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER); // Immediately start transmissions
						}
						else
						{
 							startEventUsingRTC();
						}
					}
					else
					{
						startEventNow(PROGRAMMATIC);						
					}
				}
			}
			else if (g_handle_counted_presses == 2)
			{
				suspendEvent();
			}
			
			g_handle_counted_presses = 0;
		}
		
		if(g_isMaster)
		{
			handleSerialCloning();
			
			if(g_text_buff.empty())
			{
				if((g_battery_voltage >= 0.1) && (g_battery_voltage <= g_voltage_threshold))
				{
					LEDS.sendCode((char*)"V ");
				}
				else if(g_send_clone_success_countdown)
				{
					LEDS.sendCode((char*)"X ");
				}
				else
				{
					LEDS.sendCode((char*)"M ");
				}
			}
			
			if(!isMasterCountdownSeconds)
			{
				g_isMaster = false;
				g_sleepshutdown_seconds = 240; /* Ensure sleep occurs */
				g_send_clone_success_countdown = 0;
				g_start_event = eventEnabled(); /* Start any event stored in EEPROM */
				LEDS.init();
			}
		}
		else
		{
			handleSerialBusMsgs();
			
			if(g_cloningInProgress && !g_programming_countdown)
			{
				g_cloningInProgress = false;
			}

			if(g_text_buff.empty())
			{
				if((g_battery_voltage >= 0.1) && (g_battery_voltage <= g_voltage_threshold))
				{
					LEDS.sendCode((char*)"V ");
				}
				else if(g_cloningInProgress)
				{
					LEDS.blink(LEDS_RED_ON_CONSTANT, true);
				}
				else if(g_hardware_error & ((int)HARDWARE_NO_RTC | (int)HARDWARE_NO_SI5351 ))
				{
					LEDS.blink(LEDS_RED_BLINK_FAST);
				}
				else if(!g_event_commenced)
				{
					if(g_send_clone_success_countdown)
					{
						LEDS.sendCode((char*)"X ");
					}
					else
					{
						if(eventScheduledForNow()) /* An event should be running now, but isn't = error */
						{
							LEDS.blink(LEDS_RED_BLINK_FAST);
						}
						else if(eventScheduled()) /* An event is scheduled to run in the future = OK */
						{
							LEDS.sendCode((char*)"E  ");
						}
						else /* No event is scheduled to run now, nor in the future = warning */
						{
							LEDS.blink(LEDS_RED_BLINK_FAST);
						}
					}
				}	
			}
		}
		
		if(g_long_button_press)
		{
			g_long_button_press = false;
			g_isMaster = !g_isMaster;
			
			if(g_isMaster)
			{
				g_event_commenced = false;
				isMasterCountdownSeconds = 600; /* Remain Master for 10 minutes */
				g_sleepshutdown_seconds = 720;
			}
			else
			{
				isMasterCountdownSeconds = 0;
				g_sleepshutdown_seconds = 120;
				sb_send_NewPrompt();
				g_event_commenced = false;
				g_start_event = true;
			}
			
			g_cloningInProgress = false;
			g_programming_countdown = 0;
			g_send_clone_success_countdown = 0;
			LEDS.init();
		}
		
		if(g_start_event)
		{
			g_start_event = false;
			
			if(!g_isMaster)
			{
				g_last_error_code = launchEvent((SC*)&g_last_status_code);
				g_sleepshutdown_seconds = 120;
			}
		}
				
		/********************************
		 * Handle sleep
		 ******************************/
		if(g_go_to_sleep_now && !g_cloningInProgress)
		{
			LEDS.blink(LEDS_OFF);
			serialbus_disable();
			shutdown_transmitter();	

			if(g_sleepType == SLEEP_FOREVER)
			{
				g_time_to_wake_up = FOREVER_EPOCH;
			}
			
			system_sleep_config();

			SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_STDBY_gc);		
			g_sleeping = true;
			
			/* Disable BOD? */
			
 			while(g_go_to_sleep_now)
 			{
				set_sleep_mode(SLEEP_MODE_STANDBY);
//					set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				DISABLE_INTERRUPTS();
				sleep_enable();
				ENABLE_INTERRUPTS();
				sleep_cpu();  /* Sleep occurs here */
				sleep_disable();
 			}
 
			CLKCTRL_init();
			/* Re-enable BOD? */
			g_sleeping = false;
			atmel_start_init();
			powerUp3V3();
			RTC_set_calibration(g_clock_calibration);
			while(util_delay_ms(2000));
			init_transmitter();
			
			if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS)
			{	
				LEDS.init();
				g_sleepshutdown_seconds = 120;
				g_handle_counted_presses = 0;
				g_switch_presses_count = 0;
			}
			else
			{
				serialbus_disable();
			}

			g_start_event = true;

 			g_last_status_code = STATUS_CODE_RETURNED_FROM_SLEEP;
		}
	}
}

void __attribute__((optimize("O0"))) handleSerialBusMsgs()
//void handleSerialBusMsgs()
{
	SerialbusRxBuffer* sb_buff;

	while((sb_buff = nextFullSBRxBuffer()))
	{
		bool suppressResponse = false;

		SBMessageID msg_id = sb_buff->id;

		switch(msg_id)
		{
			case SB_MESSAGE_SET_FOX:
			{
				int c1 = (int)(sb_buff->fields[SB_FIELD1][0]);
				int c2 = (int)(sb_buff->fields[SB_FIELD1][1]);
				
				if(c1)
				{
					if(c1 == 'B')
					{
						c1 = BEACON;
					}
					else if(g_event == EVENT_FOXORING)
					{
						if((c1 == '1') && (c2 == '\0'))
						{
							c1 = FOXORING_FOX1;
						}
						else if((c1 == '2') && (c2 == '\0'))
						{
							c1 = FOXORING_FOX2;
						}
						else
						{
							c1 = FOXORING_FOX3;
						}		
					}
					else if(g_event == EVENT_SPRINT)
					{
						if((c1 == 'S') && (c2 == '\0'))
						{
							c1 = SPECTATOR;
						}
						else if(c2 == 'F')
						{
							if(c1 == '1')
							{
								c1 = SPRINT_F1;
							}
							else if(c1 == '2')
							{
								c1 = SPRINT_F2;
							}
							else if(c1 == '3')
							{
								c1 = SPRINT_F3;
							}
							else if(c1 == '4')
							{
								c1 = SPRINT_F4;
							}
							else if(c1 == '5')
							{
								c1 = SPRINT_F5;
							}
						}
						else
						{
							if(c1 == '1')
							{
								c1 = SPRINT_S1;
							}
							else if(c1 == '2')
							{
								c1 = SPRINT_S2;
							}
							else if(c1 == '3')
							{
								c1 = SPRINT_S3;
							}
							else if(c1 == '4')
							{
								c1 = SPRINT_S4;
							}
							else if(c1 == '5')
							{
								c1 = SPRINT_S5;
							}
						}
					}
					else if((g_event == EVENT_CLASSIC) || (g_event == EVENT_BLIND_ARDF))
					{
						if(c1 == '1')
						{
							c1 = FOX_1;
						}
						else if(c1 == '2')
						{
							c1 = FOX_2;
						}
						else if(c1 == '3')
						{
							c1 = FOX_3;
						}
						else if(c1 == '4')
						{
							c1 = FOX_4;
						}
						else if(c1 == '5')
						{
							c1 = FOX_5;
						}
					}
					else
					{
						c1 = INVALID_FOX;
					}

					if((c1 >= BEACON) && (c1 < INVALID_FOX))
					{
 						Fox_t holdFox = (Fox_t)c1;
						 
						switch(g_event)
						{
							case EVENT_CLASSIC:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_classic, (void*)&holdFox);
							}
							break;
							
							case EVENT_SPRINT:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_sprint, (void*)&holdFox);
							}
							break;
							
							case EVENT_FOXORING:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_foxoring, (void*)&holdFox);
							}
							break;
							
							case EVENT_BLIND_ARDF:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_blind, (void*)&holdFox);
							}
							break;
							
							default: /* none */
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_none, (void*)&holdFox);
							}
							break;
						}
						 
 						if(holdFox != getFoxSetting())
 						{
							g_fox[g_event] = holdFox;
							setupForFox(holdFox, START_EVENT_WITH_STARTFINISH_TIMES);
						}
					}
				} /* if(c1) */

				reportSettings();
			}
			break;
			
// 			case SB_MESSAGE_ANT_TUNE:
// 			{
// 				static uint16_t setting;
// 
// 				if(sb_buff->fields[SB_FIELD1][0])
// 				{
// 					setting = (uint16_t)atoi(sb_buff->fields[SB_FIELD1]);
// 					
// 					if(setting <= 1023)
// 					{
// 						DAC0_setVal(setting);
// 						sprintf(g_tempStr, "DAC=%u\n", setting);
// 					}
// 					else
// 					{
// 						sprintf(g_tempStr, "err\n");
// 					}
// 			
// 					sb_send_string(g_tempStr);
// 				}
// 				else
// 				{
// 					sprintf(g_tempStr, "err\n");
// 					sb_send_string(g_tempStr);
// 				}
// 			}
// 			break;
			
			case SB_MESSAGE_SLP:
			{
				if(sb_buff->fields[SB_FIELD1][0])
				{
					if(sb_buff->fields[SB_FIELD1][0] == '0')    
					{
						g_sleepType = DO_NOT_SLEEP;
					}
					else if (sb_buff->fields[SB_FIELD1][0] == '1')
					{
						g_event_enabled = eventEnabled(); // Set sleep type based on current event settings
					}
					else
					{
						g_sleepType = SLEEP_FOREVER;
						g_go_to_sleep_now = true;
					}
				}
				else
				{
					g_go_to_sleep_now = true;
				}
			}
			break;
				
			case SB_MESSAGE_TX_FREQ:
			{
				if(sb_buff->fields[SB_FIELD1][0])
				{
					Frequency_Hz f;
					if(g_cloningInProgress)
					{
						if(!frequencyVal(sb_buff->fields[SB_FIELD2], &f))
						{
							char freqTier = sb_buff->fields[SB_FIELD1][0];
							
							if(freqTier == 'L')
							{
								g_frequency_low = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Low, (void*)&f);
							}
							else if(freqTier == 'M')
							{
								g_frequency_med = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Med, (void*)&f);
							}
							else if(freqTier == 'H')
							{
								g_frequency_hi = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void*)&f);
							}
							else if(freqTier == 'B')
							{
								g_frequency_beacon = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Beacon, (void*)&f);
							}
							else
							{
								g_frequency = f;
								g_ee_mgr.updateEEPROMVar(Frequency, (void*)&f);
							}
							
							g_event_checksum += f;
					
							sb_send_string((char*)"FRE\r");
							g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
						}
					}
					else if(!frequencyVal(sb_buff->fields[SB_FIELD1], &f))
					{
						g_frequency = f;
						
						if(!txSetFrequency(&f, true))
						{
							if(getFoxSetting() == BEACON)
							{
								g_frequency_beacon = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Beacon, (void*)&f);
							}
							else if(g_event == EVENT_FOXORING)
							{
								if(getFoxSetting() == FOXORING_FOX1)
								{
									g_frequency_low = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Low, (void*)&f);
								}
								else if(getFoxSetting() == FOXORING_FOX2)
								{
									g_frequency_med = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Med, (void*)&f);
								}
								else if(getFoxSetting() == FOXORING_FOX3)
								{
									g_frequency_hi = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void*)&f);
								}
							}
							else if(g_event == EVENT_SPRINT)
							{
								if((getFoxSetting() >= SPRINT_S1) && (getFoxSetting() <= SPRINT_S5))
								{
									g_frequency_low = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Low, (void*)&f);
								}
								else if(getFoxSetting() == SPECTATOR)
								{
									g_frequency_med = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Med, (void*)&f);
								}
								else if((getFoxSetting() >= SPRINT_F1) && (getFoxSetting() <= SPRINT_F5))
								{
									g_frequency_hi = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void*)&f);
								}
							}
							else if(g_event == EVENT_CLASSIC)
							{
								g_frequency_low = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Low, (void*)&f);
							}
							else // No event set
							{
								g_frequency = f;
								g_ee_mgr.updateEEPROMVar(Frequency, (void*)&f);
							}
						}
						else
						{
							sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
						}
					}
					else
					{
						sb_send_string((char*)"\n3500 kHz < Freq < 4000 kHz\n");
					}
				}
				
				if(!(g_cloningInProgress)) reportSettings();
			}
			break;
				
			case SB_MESSAGE_PATTERN:
			{
				if(g_cloningInProgress)
				{
					if(sb_buff->fields[SB_FIELD1][0] && sb_buff->fields[SB_FIELD2][0])
					{
						int len = MIN(MAX_PATTERN_TEXT_LENGTH, strlen(sb_buff->fields[SB_FIELD2]));
						
						for(int i=0; i<len; i++)
						{
							g_event_checksum += sb_buff->fields[SB_FIELD2][i];
						}
					
						sb_send_string((char*)"PAT\r");
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
				else
				{
					if(sb_buff->fields[SB_FIELD1][0])
					{
						if(strlen(sb_buff->fields[SB_FIELD1]) <= MAX_PATTERN_TEXT_LENGTH)
						{
							strcpy(g_tempStr, sb_buff->fields[SB_FIELD1]);
						
							if(g_event == EVENT_FOXORING)
							{
								strcpy(g_messages_text[FOXORING_PATTERN_TEXT], g_tempStr);
								g_ee_mgr.updateEEPROMVar(Foxoring_pattern_text, g_messages_text[FOXORING_PATTERN_TEXT]);
							}
							else
							{
								strcpy(g_messages_text[PATTERN_TEXT], g_tempStr);
							}
						}
						else
						{
							sb_send_string((char*)"Illegal pattern\n");
						}
					}
				}
				
				if(!(g_cloningInProgress)) reportSettings();
			}
			break;
			
			case SB_MESSAGE_KEY:
			{
				if(sb_buff->fields[SB_FIELD1][0])
				{
					if(sb_buff->fields[SB_FIELD1][0] == '0')    
					{
 						stopEventNow(PROGRAMMATIC);
						LEDS.setRed(OFF);
 						keyTransmitter(OFF);
					}
					else if(sb_buff->fields[SB_FIELD1][0] == '1')  
					{
 						stopEventNow(PROGRAMMATIC);
						LEDS.setRed(ON);
 						keyTransmitter(ON);
					}
					else
					{
						sb_send_string((char*)"err\n");
					}
				}
				else
				{
					LEDS.setRed(OFF);
					LEDS.init();
 					keyTransmitter(OFF);
					startEventNow(PROGRAMMATIC);
				}
			}
			break;

			case SB_MESSAGE_GO:
			{
				if(sb_buff->fields[SB_FIELD1][0])
				{
					if(sb_buff->fields[SB_FIELD1][0] == '0')       /* Stop the event. Re-sync will occur on next start */
					{
 						stopEventNow(PROGRAMMATIC);
					}
					else if(sb_buff->fields[SB_FIELD1][0] == '1')  /* Start the event, re-syncing to a start time of now - same as a button press */
					{
						LEDS.setRed(OFF);
						g_event_enabled = false; /* ensure that it will run immediately */
 						startEventNow(PROGRAMMATIC);
					}
					else if(sb_buff->fields[SB_FIELD1][0] == '2')  /* Start the event at the programmed start time */
					{
 						g_event_enabled = false;					/* Disable an event currently underway */
 						startEventUsingRTC();
					}
					else if(sb_buff->fields[SB_FIELD1][0] == '3')  /* Start the event immediately with transmissions starting now */
					{
 						setupForFox(INVALID_FOX, START_TRANSMISSIONS_NOW);
					}
					else
					{
						sb_send_string((char*)"err\n");
					}
				}
				else
				{
					sb_send_string((char*)"err\n");
				}
			}
			break;

			case SB_MESSAGE_SET_STATION_ID:
			{
				if(sb_buff->fields[SB_FIELD1][0])
				{
					int len = 0;
					
					if((sb_buff->fields[SB_FIELD1][0] == '\"') && (sb_buff->fields[SB_FIELD1][1] == '\"')) /* "" */
					{
						g_messages_text[STATION_ID][0] = '\0';
					}
					else
					{
						strcpy(g_tempStr, " "); /* Add a Space before ID gets sent */
						strcat(g_tempStr, sb_buff->fields[SB_FIELD1]);

						if(sb_buff->fields[SB_FIELD2][0])
						{
							strcat(g_tempStr, " ");
							strcat(g_tempStr, sb_buff->fields[SB_FIELD2]);
						}
						
						len = MIN(MAX_PATTERN_TEXT_LENGTH, strlen(g_tempStr));
						strncpy((char*)g_messages_text[STATION_ID], g_tempStr, len);
						g_messages_text[STATION_ID][len] = '\0';
					}
					
					if(g_cloningInProgress)
					{				
						for(int i=0; i<len; i++)
						{
							g_event_checksum += g_messages_text[STATION_ID][i];
						}

						sb_send_string((char*)"ID\r");
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}

				g_ee_mgr.updateEEPROMVar(StationID_text, g_messages_text[STATION_ID]);
				
				if(g_messages_text[STATION_ID][0])
				{
					g_time_needed_for_ID = timeNeededForID();
				}
				
				if(!(g_cloningInProgress)) reportSettings();
			}
			break;


			case SB_MESSAGE_CODE_SETTINGS:
			{
				char c = sb_buff->fields[SB_FIELD1][0];
				char x = sb_buff->fields[SB_FIELD2][0];

				if(c)
				{
					if(isdigit(x))
					{
						uint8_t speed = atol(sb_buff->fields[SB_FIELD2]);
						
						if(c == 'I')
						{
 							g_id_codespeed = speed;
							g_ee_mgr.updateEEPROMVar(Id_codespeed, (void*)&g_id_codespeed);

							if(g_messages_text[STATION_ID][0])
							{
								g_time_needed_for_ID = timeNeededForID();
							}
							
							if(g_cloningInProgress)
							{
								g_event_checksum += speed;
								sb_send_string((char*)"SPD I\r");
							}
						}
						else if((c == 'F') || ((g_event == EVENT_FOXORING) && !g_cloningInProgress))
						{
							g_foxoring_pattern_codespeed = speed;
							g_ee_mgr.updateEEPROMVar(Foxoring_Pattern_Code_Speed, (void*)&g_foxoring_pattern_codespeed);
							if(g_event_commenced) g_code_throttle = throttleValue(getPatternCodeSpeed());
							
							if(g_cloningInProgress)
							{
								g_event_checksum += speed;
								sb_send_string((char*)"SPD F\r");
							}
						}
						else if(c == 'P')
						{
							g_pattern_codespeed = speed;
							g_ee_mgr.updateEEPROMVar(Pattern_Code_Speed, (void*)&g_pattern_codespeed);
							
							if(g_cloningInProgress)
							{
								g_event_checksum += speed;
								sb_send_string((char*)"SPD P\r");
							}
						}
						else
						{
							sprintf(g_tempStr, "err\n");
						}						
					}
					else
					{
						sprintf(g_tempStr, "err\n");
					}
				}
				else
				{
					sprintf(g_tempStr, "err\n");
				}

				if(!(g_cloningInProgress)) reportSettings();
			}
			break;

			case SB_MESSAGE_MASTER:
			{
				if(sb_buff->fields[SB_FIELD1][0] == 'P')
				{
					if(!g_send_clone_success_countdown)
					{
						g_cloningInProgress = true;
						suspendEvent();
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
						g_event_checksum = 0;
						sb_send_string((char*)"MAS\r");
					}
				}
				else if((sb_buff->fields[SB_FIELD1][0] == 'Q') && g_cloningInProgress)
				{
					uint32_t sum = atol(sb_buff->fields[SB_FIELD2]);
					g_cloningInProgress = false;
					g_programming_countdown = 0;
					if(sum == g_event_checksum)
					{
						sb_send_string((char*)"MAS ACK\r");
						g_send_clone_success_countdown = 18000;
						setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);   /* Start the event if one is configured */
//						g_start_event = true;
					}
					else
					{
						sb_send_string((char*)"MAS NAK\r");
					}
				}
				else
				{
					if(sb_buff->fields[SB_FIELD1][0])
					{
						if((sb_buff->fields[SB_FIELD1][0] == 'M') || (sb_buff->fields[SB_FIELD1][0] == '1'))
						{
 							g_isMaster = true;
							g_sleepshutdown_seconds = 720;
							isMasterCountdownSeconds = 600; /* Remain Master for 10 minutes */
						}
					}
				}
			}
			break;
			
			case SB_MESSAGE_EVENT:
			{
				if(g_cloningInProgress)
				{
					char c = sb_buff->fields[SB_FIELD1][0];
					sb_send_string((char*)"EVT F\r");
					if(c == 'F')
					{
						g_event = EVENT_FOXORING;
					}
					else if(c == 'C')
					{
						g_event = EVENT_CLASSIC;
					}
					else if(c == 'S')
					{
						g_event = EVENT_SPRINT;
					}
					else if(c == 'B')
					{
						g_event = EVENT_BLIND_ARDF;
					}
					else
					{
						g_event = EVENT_NONE;
					}
					
					sb_send_string((char*)"EVT\r");
					
					g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_event_checksum += c;
				}
				else
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'F')
					{
						g_event = EVENT_FOXORING;
						g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
						init_transmitter(getFrequencySetting());
						setupForFox(getFoxSetting(), START_NOTHING);
					}
					else if(sb_buff->fields[SB_FIELD1][0] == 'C')
					{
						g_event = EVENT_CLASSIC;
						g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
						init_transmitter(getFrequencySetting());
						setupForFox(getFoxSetting(), START_NOTHING);
					}
					else if(sb_buff->fields[SB_FIELD1][0] == 'S')
					{
						g_event = EVENT_SPRINT;
						g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
						init_transmitter(getFrequencySetting());
						setupForFox(getFoxSetting(), START_NOTHING);
					}
					else if(sb_buff->fields[SB_FIELD1][0] == 'B')
					{
						g_event = EVENT_BLIND_ARDF;
						g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
						init_transmitter(getFrequencySetting());
						setupForFox(getFoxSetting(), START_NOTHING);
					}
					else if(sb_buff->fields[SB_FIELD1][0])
					{
						g_event = EVENT_NONE;
						g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
						init_transmitter(getFrequencySetting());
						setupForFox(getFoxSetting(), START_NOTHING);
					}
				}
				
				if(!(g_cloningInProgress)) reportSettings();
			}
			break;

			case SB_MESSAGE_CLOCK:
			{
				char f1 = sb_buff->fields[SB_FIELD1][0];
				
				if(!f1 || f1 == 'T')   /* Current time format "YYMMDDhhmmss" */
				{		 
					if(sb_buff->fields[SB_FIELD2][0])
					{
						strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
  						time_t t;
						  
						if(g_cloningInProgress)
						{
							t = atol(g_tempStr);
						}
						else
						{	
							g_tempStr[12] = '\0';               /* truncate to no more than 12 characters */
							t = validateTimeString(g_tempStr, null);
						}
  
  						if(t)
  						{
							RTC_zero();
 							set_system_time(t);
							 
							if(g_cloningInProgress)
							{
								sprintf(g_tempStr, "CLK T %lu\r", t);
								sb_send_string(g_tempStr);
								g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
								g_event_checksum += t;
							}
							else
							{
  								setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);   /* Start the event if one is configured */
							}
 						}
					}
				}
				else if(f1 == 'S')  /* Event start time */
				{
					strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
					time_t s;
						
					if(g_cloningInProgress)
					{
						s = atol(g_tempStr);
					}
					else
					{
						g_tempStr[12] = '\0';               /* truncate to no more than 12 characters */
						s = validateTimeString(g_tempStr, null);
					}
 
 					if(s || g_cloningInProgress)
 					{
 						g_event_start_epoch = s;
						g_ee_mgr.updateEEPROMVar(Event_start_epoch, (void*)&g_event_start_epoch);

						if(g_cloningInProgress)
						{
							sb_send_string((char*)"CLK S\r");
							g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
							g_event_checksum += s;
						}
						else
						{
							g_days_to_run = 1;
							g_days_run = 0;
							
 							g_event_finish_epoch = MAX(g_event_finish_epoch, (g_event_start_epoch + SECONDS_24H));
							
							g_ee_mgr.updateEEPROMVar(Event_finish_epoch, (void*)&g_event_finish_epoch);
 							setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
							if(eventScheduled())
							{
								startEventUsingRTC();
							}
 						}
					 }
				}
				else if(f1 == 'F')  /* Event finish time */
				{
					strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
					time_t f;
					
					if(g_cloningInProgress)
					{
						f = atol(g_tempStr);
					}
					else
					{
						g_tempStr[12] = '\0';               /* truncate to no more than 12 characters */
						f = validateTimeString(g_tempStr, null);
					}					
 
 					if(f || g_cloningInProgress)
 					{
						g_event_finish_epoch = f;

						g_ee_mgr.updateEEPROMVar(Event_finish_epoch, (void*)&g_event_finish_epoch);
												
						if(g_cloningInProgress)
						{
							sb_send_string((char*)"CLK F\r");
							g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
							g_event_checksum += f;
						}
						else
						{
							g_days_to_run = 1;
							g_days_run = 0;

 							setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
							if(eventScheduled())
							{
								startEventUsingRTC();
							}
						}
 					}
				}
				else if(f1 == 'D') /* Run the event multiple days */
				{
					if(sb_buff->fields[SB_FIELD2][0])
					{
						strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
						uint8_t days;
					
						days = atol(g_tempStr);
					
						if(days > 1)
						{
							g_days_to_run = days;
						}
						else
						{
							g_days_to_run = 1;
						}
						
						g_days_run = 0;
					
						g_ee_mgr.updateEEPROMVar(Days_to_run, (void*)&g_days_to_run);
					
						if(g_cloningInProgress)
						{
							sb_send_string((char*)"CLK D\r");
							g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
							g_event_checksum += g_days_to_run;
						}
						else
						{
							g_event_finish_epoch = MIN(g_event_start_epoch + DAY - HOUR, g_event_finish_epoch);
							eventScheduled();
							g_ee_mgr.updateEEPROMVar(Event_start_epoch, (void*)&g_event_start_epoch);
							g_ee_mgr.updateEEPROMVar(Event_finish_epoch, (void*)&g_event_finish_epoch);
						}	
					}
					else
					{
						sprintf(g_tempStr, "\nDays to run: %d\n", g_days_to_run);
						sb_send_string(g_tempStr);
						suppressResponse = true;
						sb_send_NewPrompt();
					}
				}
				else if(f1 == 'C' && !g_cloningInProgress)  /* Clock calibration */
				{
					strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
					uint16_t cal;
					
					cal = atoi(g_tempStr);
					RTC_set_calibration(cal);
				}

				if(!(g_cloningInProgress) && !suppressResponse) reportSettings();
			}
			break;

			case SB_MESSAGE_VOLTS:
			{
				char txt[6];

				if(sb_buff->fields[SB_FIELD1][0])
				{
					float v = atof(sb_buff->fields[SB_FIELD1]);

					if((v >= 0.1) && (v <= 15.))
					{
 						g_voltage_threshold = v;
						g_ee_mgr.updateEEPROMVar(Voltage_threshold, (void*)&g_voltage_threshold);
					}
					else
					{
						sb_send_string((char*)"\nErr: 0.1 V < thresh < 15.0 V\n");
					}

				}

				dtostrf(g_battery_voltage, 4, 1, txt);
				txt[5] = '\0';
  				sprintf(g_tempStr, "\nBat =%s Volts\n", txt);
 				sb_send_string(g_tempStr);

				dtostrf(g_voltage_threshold, 4, 1, txt);
				txt[5] = '\0';
 				sprintf(g_tempStr, "thresh =%s Volts\n", txt);
 				sb_send_string(g_tempStr);
			}
			break;
			
			case SB_MESSAGE_HELP:
			{
				if(!g_cloningInProgress)
				{
					reportSettings();
					sb_send_string(HELP_TEXT_TXT);
				}
			}
			break;

			default:
			{
				suppressResponse = true;
			}
			break;
		}

		sb_buff->id = SB_MESSAGE_EMPTY;
		if(!(g_cloningInProgress) && !suppressResponse)
		{
			sb_send_NewLine();
			sb_send_NewPrompt();
		}

// 		g_LED_timeout_countdown = LED_TIMEOUT_SECONDS;
// 		g_config_error = NULL_CONFIG;   /* Trigger a new configuration enunciation */
	}
}


/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
bool __attribute__((optimize("O0"))) eventEnabled()
{
	if(g_run_event_forever) 
	{
		g_sleepType = DO_NOT_SLEEP;
		return(true);
	}
	
	g_go_to_sleep_now = false;
	
	if(!eventScheduled()) /* A future event has not been set, and no event is scheduled to run right now */
	{
		g_sleepType = SLEEP_FOREVER;
		g_time_to_wake_up = MAX_TIME;
		g_sleepshutdown_seconds = 120;
		return(false); /* completed events are never enabled */
	}
	
	time_t now = time(null);
	int32_t dif = timeDif(now, g_event_start_epoch);

	g_sleepType = SLEEP_UNTIL_START_TIME;
	g_time_to_wake_up = g_event_start_epoch - 10; /* sleep time needs to be calculated to allow time for power-up (coming out of sleep) prior to the event start */
	
	if(dif >= -30)  /* Don't sleep if the event starts in 30 seconds or less, or has already started */
	{
		g_sleepshutdown_seconds = 120;
		return( true);
	}

	/* If we reach here, we have an event that will not start for at least 30 seconds. */
	return( true);
}


void wdt_init(WDReset resetType)
{
	
}

uint16_t throttleValue(uint8_t speed)
{
	float temp;
	speed = CLAMP(5, (int8_t)speed, 20);
	temp = (3544L / (uint16_t)speed) / 10L; /* tune numerator to achieve "PARIS " sent 8 times in 60 seconds at 8 WPM */
	return( (uint16_t)temp);
}

void syncSystemTimeToRTC(void)
{
	g_seconds_transition = false;  /* Sync to RTC second transition */
	while(!g_seconds_transition);
}

EC __attribute__((optimize("O0"))) launchEvent(SC* statusCode)
{
	EC ec = activateEventUsingCurrentSettings(statusCode);

	if(ec)
	{
		g_last_error_code = ec;
	}
	else
	{
		g_event_enabled = eventEnabled();
	}

	return( ec);
}

EC activateEventUsingCurrentSettings(SC* statusCode)
{
	syncSystemTimeToRTC();
	time_t now = time(null);
	
	/* Make sure everything has been sanely initialized */
	if(!g_run_event_forever)
	{
		if(now < MINIMUM_VALID_EPOCH) /* The RTC has not been set */
		{
			return( ERROR_CODE_EVENT_NOT_CONFIGURED);
		}
		
		if(!g_event_start_epoch)
		{
			return( ERROR_CODE_EVENT_MISSING_START_TIME);
		}

		if(g_event_start_epoch >= g_event_finish_epoch)   /* Finish must be later than start */
		{
			return( ERROR_CODE_EVENT_NOT_CONFIGURED);
		}
	}

	if(!g_on_air_seconds)
	{
		return( ERROR_CODE_EVENT_MISSING_TRANSMIT_DURATION);
	}

	if(g_intra_cycle_delay_time > (g_off_air_seconds + g_on_air_seconds))
	{
		return( ERROR_CODE_EVENT_TIMING_ERROR);
	}

	char *c = getCurrentPatternText();
	if(c[0] == '\0')
	{
		return( ERROR_CODE_EVENT_PATTERN_NOT_SPECIFIED);
	}

	if((g_pattern_codespeed < MIN_CODE_SPEED_WPM) || (g_pattern_codespeed > MAX_CODE_SPEED_WPM))
	{
		return( ERROR_CODE_EVENT_PATTERN_CODE_SPEED_NOT_SPECIFIED);
	}

	if(g_messages_text[STATION_ID][0] != '\0')
	{
		if((!g_id_codespeed || !g_ID_period_seconds))
		{
			return( ERROR_CODE_EVENT_STATION_ID_ERROR);
		}

		g_time_needed_for_ID = timeNeededForID();
	}
	else
	{
		g_time_needed_for_ID = 0;   /* ID will never be sent */
	}

	if(!g_run_event_forever && (g_event_finish_epoch < now))   /* the event has already finished */
	{
		if(statusCode)
		{
			*statusCode = STATUS_CODE_NO_EVENT_TO_RUN;
		}
	}
	else
	{
		if(g_run_event_forever)
		{
			g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
			g_on_the_air = g_on_air_seconds;
			g_sendID_seconds_countdown = g_on_air_seconds - g_time_needed_for_ID;
			LEDS.init();
			g_event_enabled = true;
			g_event_commenced = true;
		}
		else
		{			
			int32_t dif = timeDif(now, g_event_start_epoch); /* returns arg1 - arg2 */

			if(dif >= 0)                                    /* start time is in the past */
			{
				bool turnOnTransmitter = false;
				int cyclePeriod = g_on_air_seconds + g_off_air_seconds;
				int secondsIntoCycle = dif % cyclePeriod;
				int timeTillTransmit = g_intra_cycle_delay_time - secondsIntoCycle;

				if(timeTillTransmit <= 0)                       /* we should have started transmitting already in this cycle */
				{
					if(g_on_air_seconds <= -timeTillTransmit)   /* we should have finished transmitting in this cycle */
					{
						g_on_the_air = -(cyclePeriod + timeTillTransmit);
						if(statusCode)
						{
							*statusCode = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
						}

						if(!g_event_enabled)
						{
							g_sendID_seconds_countdown = (g_on_air_seconds - g_on_the_air) - g_time_needed_for_ID;
						}
					}
					else    /* we should be transmitting right now */
					{
						g_on_the_air = g_on_air_seconds + timeTillTransmit;
						turnOnTransmitter = true;
						if(statusCode)
						{
							*statusCode = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
						}

						if(!g_event_enabled)
						{
							if(g_time_needed_for_ID < g_on_the_air)
							{
								g_sendID_seconds_countdown = g_on_the_air - g_time_needed_for_ID;
							}
						}
					}
				}
				else    /* it is not yet time to transmit in this cycle */
				{
					g_on_the_air = -timeTillTransmit;
					if(statusCode)
					{
						*statusCode = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
					}

					if(!g_event_enabled)
					{
						g_sendID_seconds_countdown = timeTillTransmit + g_on_air_seconds - g_time_needed_for_ID;
					}
				}

				if(!turnOnTransmitter)
				{
					keyTransmitter(OFF);
				}

				g_event_commenced = true;
				LEDS.init();
			}
			else    /* start time is in the future */
			{
				if(statusCode)
				{
					*statusCode = STATUS_CODE_WAITING_FOR_EVENT_START;
				}
				keyTransmitter(OFF);
			}
		}
	}

	return( ERROR_CODE_NO_ERROR);
}

EC hw_init()
{
	return ERROR_CODE_NO_ERROR;
}

EC rtc_init()
{	
	return ERROR_CODE_NO_ERROR;
}

bool antennaIsConnected()
{
	return(true);
}

void initializeAllEventSettings(bool disableEvent)
{
	
}

void suspendEvent()
{
	g_event_enabled = false;    /* get things stopped immediately */
	g_on_the_air = 0;           /*  stop transmitting */
	g_event_commenced = false;  /* get things stopped immediately */
	g_run_event_forever = false;
	g_sleepshutdown_seconds = 120;
// 	g_sleepType = SLEEP_FOREVER;
	keyTransmitter(OFF);
	bool repeat = false;
	makeMorse((char*)"\0", &repeat, null);  /* reset makeMorse */
	LEDS.init();
}

void startEventNow(EventActionSource_t activationSource)
{
	ConfigurationState_t conf = clockConfigurationCheck();

	if(activationSource == POWER_UP)
	{
		if(conf == CONFIGURATION_ERROR)
		{
			setupForFox(INVALID_FOX, START_NOTHING);
		}
		else
		{
			setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
		}
	}
	else if(activationSource == PROGRAMMATIC)
	{
		if(conf == CONFIGURATION_ERROR)                                                                                             /* Start immediately */
		{
			setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER);
		}
		else if((conf == SCHEDULED_EVENT_WILL_NEVER_RUN) || (conf == SCHEDULED_EVENT_DID_NOT_START)) /* Start immediately */
		{
			setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER);
		}
		else if((conf == WAITING_FOR_START))
		{
			setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
		}
		else                                                                                                                        /*if((conf == EVENT_IN_PROGRESS) */
		{
			setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER);                                                                  /* Let the RTC start the event */
		}
	}
	else                                                                                                                            /* PUSHBUTTON */
	{
		if(conf == CONFIGURATION_ERROR)                                                                                             /* No scheduled event */
		{
			setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER);
		}
		else                                                                                                                        /* if(buttonActivated) */
		{
			if(conf == WAITING_FOR_START)
			{
				setupForFox(INVALID_FOX, START_TRANSMISSIONS_NOW);                                                                         /* Start transmitting! */
			}
			else if(conf == SCHEDULED_EVENT_WILL_NEVER_RUN)
			{
				setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);                                                              /* rtc starts the event */
			}
			else                                                                                                                    /* Event should be running now */
			{
				setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);                                                              /* start the running event */
			}
		}
	}
}

void stopEventNow(EventActionSource_t activationSource)
{
	ConfigurationState_t conf = clockConfigurationCheck();

	if(activationSource == PROGRAMMATIC)
	{
		setupForFox(INVALID_FOX, START_NOTHING);
	}
	else    /* if(activationSource == PUSHBUTTON) */
	{
		if(conf == WAITING_FOR_START)
		{
			setupForFox(INVALID_FOX, START_TRANSMISSIONS_NOW);
		}
		if(conf == SCHEDULED_EVENT_WILL_NEVER_RUN)
		{
			setupForFox(INVALID_FOX, START_NOTHING);
		}
		else    /*if(conf == CONFIGURATION_ERROR) */
		{
			setupForFox(INVALID_FOX, START_NOTHING);
		}
	}

// 	if(g_sync_pin_stable == STABLE_LOW)
// 	{
// 		digitalWrite(PIN_LED, OFF); /*  LED Off */
// 	}
}

bool startEventUsingRTC(void)
{
	bool err = false;
	time_t now = time(null);
	ConfigurationState_t state = clockConfigurationCheck();

	if(state != CONFIGURATION_ERROR)
	{
		setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
		reportTimeTill(now, g_event_start_epoch, "\nStarts in: ", "\nIn progress\n");

		if(g_event_start_epoch < now)
		{
			reportTimeTill(now, g_event_finish_epoch, "Time Remaining: ", NULL);
		}
		else
		{
			reportTimeTill(g_event_start_epoch, g_event_finish_epoch, "Lasts: ", NULL);
		}
	}
	else
	{
		err = true;
		reportConfigErrors();
	}
	
	return err;
}



void setupForFox(Fox_t fox, EventAction_t action)
{
	bool delayNotSet = true;
	
	g_run_event_forever = false;
	g_sleepshutdown_seconds = 120;
	
	if(fox == INVALID_FOX)
	{
		fox = getFoxSetting();
	}

	g_event_enabled = false;
	g_event_commenced = false;
	LEDS.setRed(OFF);

	switch(fox)
	{
		case FOX_1:
		{
			delayNotSet = false;
			g_intra_cycle_delay_time = 0;
		}
		case FOX_2:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 60;
			}
		}
		case FOX_3:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 120;
			}
		}
		case FOX_4:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 180;
			}
		}
		case FOX_5:
		{
			/* Set the Morse code pattern and speed */
			if(delayNotSet)
			{
				g_intra_cycle_delay_time = 240;
			}
			
			g_sendID_seconds_countdown = 60;			/* wait 1 minute to send the ID */
			g_on_air_seconds = 60;						/* on period is very long */
			g_off_air_seconds = 240;                    /* off period is very short */
		}
		break;

		case SPRINT_S1:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 0;
			}
		}
		case SPRINT_S2:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 12;
			}
		}
		case SPRINT_S3:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 24;
			}
		}
		case SPRINT_S4:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 36;
			}
		}
		case SPRINT_S5:
		{
			{
				if(delayNotSet)
				{
					g_intra_cycle_delay_time = 48;
				}
			}

			g_sendID_seconds_countdown = 600;			/* wait 10 minutes send the ID */
			g_on_air_seconds = 12;						/* on period is very long */
			g_off_air_seconds = 48;						/* off period is very short */
		}
		break;

		case SPRINT_F1:
		{
			delayNotSet = false;
			g_intra_cycle_delay_time = 0;
		}
		case SPRINT_F2:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 12;
			}
		}
		case SPRINT_F3:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 24;
			}
		}
		case SPRINT_F4:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_intra_cycle_delay_time = 36;
			}
		}
		case SPRINT_F5:
		{
			if(delayNotSet)
			{
				g_intra_cycle_delay_time = 48;
			}

			g_sendID_seconds_countdown = 600;			/* wait 10 minutes send the ID */
			g_on_air_seconds = 12;						/* on period is very long */
			g_off_air_seconds = 48;						/* off period is very short */
		}
		break;

#if SUPPORT_TEMP_AND_VOLTAGE_REPORTING
		case REPORT_BATTERY:
		{
			g_intra_cycle_delay_time = 0;
// 			g_on_air_interval_seconds = 30;
// 			g_cycle_period_seconds = 60;
// 			g_number_of_foxes = 2;
// 			g_pattern_codespeed = SPRINT_SLOW_CODE_SPEED;
// 			g_fox_id_offset = REPORT_BATTERY - 1;
// 			g_id_interval_seconds = 60;
		}
		break;
#endif // SUPPORT_TEMP_AND_VOLTAGE_REPORTING

		case FOXORING_FOX1:
		case FOXORING_FOX2:
		case FOXORING_FOX3:
		{
			init_transmitter(getFrequencySetting());
			g_intra_cycle_delay_time = 0;
			g_sendID_seconds_countdown = 600;			/* wait 10 minutes send the ID */
			g_on_air_seconds = 60;						/* on period is very long */
			g_off_air_seconds = 0;						/* off period is very short */
		}
		break;

		case SPECTATOR:
		case BEACON:
		default:
		{
			g_intra_cycle_delay_time = 0;
			g_sendID_seconds_countdown = 600;			/* wait 10 minutes send the ID */
			g_on_air_seconds = 60;						/* on period is very long */
			g_off_air_seconds = 0;						/* off period is very short */
		}
		break;
	}

	if(action == START_NOTHING)
	{
		g_event_commenced = false; 
		g_event_enabled = false;
		keyTransmitter(OFF);
		LEDS.setRed(OFF);
	}
	else if(action == START_EVENT_NOW_AND_RUN_FOREVER)
	{
		syncSystemTimeToRTC();
		g_run_event_forever = true;
		g_sleepshutdown_seconds = UINT16_MAX;
		launchEvent((SC*)&g_last_status_code);
	}
	else if(action == START_TRANSMISSIONS_NOW)                                  /* Immediately start transmitting, regardless RTC or time slot */
	{
		g_on_the_air = g_on_air_seconds;			/* start out transmitting */
		g_event_commenced = true;                   /* get things running immediately */
		g_event_enabled = true;                     /* get things running immediately */
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		LEDS.init();
	}
	else         /* if(action == START_EVENT_WITH_STARTFINISH_TIMES) */
	{
		launchEvent((SC*)&g_last_status_code);
	}

// 	sendMorseTone(OFF);
// 	g_code_throttle    = 0;                 /* Adjusts Morse code speed */
// 	g_on_the_air       = false;             /* Controls transmitter Morse activity */

// 	g_config_error = NULL_CONFIG;           /* Trigger a new configuration enunciation */
// 	digitalWrite(PIN_LED, OFF);             /*  LED Off - in case it was left on */
// 
// 	digitalWrite(PIN_CW_KEY_LOGIC, OFF);    /* TX key line */
// 	g_sendAMmodulation = false;
// 	g_LED_enunciating = false;
// 	g_config_error = NULL_CONFIG;           /* Trigger a new configuration enunciation */
}

time_t validateTimeString(char* str, time_t* epochVar)
{
	time_t valid = 0;
	int len = strlen(str);
	time_t minimumEpoch = MINIMUM_VALID_EPOCH;
	uint8_t validationType = 0;
	time_t now = time(null);

	if(epochVar == &g_event_start_epoch)
	{
		minimumEpoch = MAX(now, MINIMUM_VALID_EPOCH);
		validationType = 1;
	}
	else if(epochVar == &g_event_finish_epoch)
	{
		minimumEpoch = MAX(g_event_start_epoch, now);
		validationType = 2;
	}
	
	if(len == 10)
	{
		str[10] = '0';
		str[11] = '0';
		str[12] = '\0';
		len = 12;
	}

	if((len == 12) && (only_digits(str)))
	{
		time_t ep = String2Epoch(NULL, str);    /* String format "YYMMDDhhmmss" */

		if(ep > minimumEpoch)
		{
			valid = ep;
		}
		else
		{
			if(validationType == 1)         /* start time validation */
			{
				sb_send_string(TEXT_ERR_START_IN_PAST_TXT);
			}
			else if(validationType == 2)    /* finish time validation */
			{
				if(ep < time(null))
				{
					sb_send_string(TEXT_ERR_FINISH_IN_PAST_TXT);
				}
				else
				{
					sb_send_string(TEXT_ERR_FINISH_BEFORE_START_TXT);
				}
			}
			else    /* current time validation */
			{
				sb_send_string(TEXT_ERR_TIME_IN_PAST_TXT);
			}
		}
	}
	else if(len)
	{
		sb_send_string(TEXT_ERR_INVALID_TIME_TXT);
	}

	return(valid);
}


bool reportTimeTill(time_t from, time_t until, const char* prefix, const char* failMsg)
{
	bool failure = false;

	if(from >= until)   /* Negative time */
	{
		failure = true;
		if(failMsg)
		{
			sb_send_string((char*)failMsg);
		}
	}
	else
	{
		if(prefix)
		{
			sb_send_string((char*)prefix);
		}
		time_t dif = until - from;
		uint16_t years = dif / YEAR;
		time_t hold = dif - (years * YEAR);
		uint16_t days = hold / DAY;
		hold -= (days * DAY);
		uint16_t hours = hold / HOUR;
		hold -= (hours * HOUR);
		uint16_t minutes = hold / MINUTE;
		uint16_t seconds = hold - (minutes * MINUTE);

		g_tempStr[0] = '\0';

		if(years)
		{
			sprintf(g_tempStr, "%d yrs ", years);
			sb_send_string(g_tempStr);
		}

		if(days)
		{
			sprintf(g_tempStr, "%d days ", days);
			sb_send_string(g_tempStr);
		}

		if(hours)
		{
			sprintf(g_tempStr, "%d hrs ", hours);
			sb_send_string(g_tempStr);
		}

		if(minutes)
		{
			sprintf(g_tempStr, "%d min ", minutes);
			sb_send_string(g_tempStr);
		}

		sprintf(g_tempStr, "%d sec", seconds);
		sb_send_string(g_tempStr);

		sb_send_NewLine();
		g_tempStr[0] = '\0';
	}

	return( failure);
}



ConfigurationState_t clockConfigurationCheck(void)
{
	time_t now = time(null);
	
	if((g_event_finish_epoch <= MINIMUM_VALID_EPOCH) || (g_event_start_epoch <= MINIMUM_VALID_EPOCH) || (now <= MINIMUM_VALID_EPOCH))
	{
		return(CONFIGURATION_ERROR);
	}

	if(g_event_finish_epoch <= g_event_start_epoch) /* Event configured to finish before it started */
	{
		return(CONFIGURATION_ERROR);
	}

	if(now > g_event_finish_epoch)  /* The scheduled event is over */
	{
		return(CONFIGURATION_ERROR);
	}

	if(now > g_event_start_epoch)       /* Event should be running */
	{
		if(!g_event_enabled)
		{
			return(SCHEDULED_EVENT_DID_NOT_START);  /* Event scheduled to be running isn't */
		}
		else
		{
			return(EVENT_IN_PROGRESS);              /* Event is running, so clock settings don't matter */
		}
	}
	else if(!g_event_enabled)
	{
		return(SCHEDULED_EVENT_WILL_NEVER_RUN);
	}

	return(WAITING_FOR_START);  /* Future event hasn't started yet */
}

void reportConfigErrors(void)
{
	time_t now = time(null);

	if(g_messages_text[STATION_ID][0] == '\0')
	{
		sb_send_string(TEXT_SET_ID_TXT);
	}

	if(now <= MINIMUM_VALID_EPOCH) /* Current time is invalid */
	{
		sb_send_string(TEXT_SET_TIME_TXT);
	}
	
	if(g_event_finish_epoch <= MINIMUM_VALID_EPOCH)
	{
		sb_send_string(TEXT_SET_FINISH_TXT);
		
		if(g_event_start_epoch < MINIMUM_VALID_EPOCH)
		{
			sb_send_string(TEXT_SET_START_TXT);
		}
	}
	else if(g_event_finish_epoch <= now)      /* Event has already finished */
	{
		if(g_event_start_epoch < now)   /* Event has already started */
		{
			sb_send_string(TEXT_SET_START_TXT);
		}

		sb_send_string(TEXT_SET_FINISH_TXT);
	}
	else if(g_event_start_epoch < now)  /* Event has already started */
	{
		if(g_event_start_epoch < MINIMUM_VALID_EPOCH)     /* Start invalid */
		{
			sb_send_string(TEXT_SET_START_TXT);
		}
		else if(!g_event_enabled)
		{
			sb_send_string((char*)"Start with > GO 2\n");
		}
		else
		{
			sb_send_string((char*)"Event running...\n");
		}
	}
}

void reportSettings(void)
{
	if(g_cloningInProgress) return;
	
	char buf[50];
	
	time_t now = time(null);
	
	sb_send_string(TEXT_CURRENT_SETTINGS_TXT);
	
	sprintf(g_tempStr, "*   Time: %s\n", convertEpochToTimeString(now, buf, 50));
	sb_send_string(g_tempStr);
		
	if(!event2Text(g_tempStr, g_event))
	{
		strcpy(buf, g_tempStr);
		sprintf(g_tempStr, "*   Event: %s\n", buf);
	}
	else
	{
		sprintf(g_tempStr, "*   Event: None Set\n");
	}
	
	sb_send_string(g_tempStr);
		
	Fox_t f = getFoxSetting();
	
	if(!fox2Text(g_tempStr, f))
	{
		strcpy(buf, g_tempStr);
		sprintf(g_tempStr, "*   Fox: %s\n", buf);
	}
	else
	{
		sprintf(g_tempStr, "*   Fox: %u\n", (uint16_t)f);
	}
	
	sb_send_string(g_tempStr);
	
	if(g_messages_text[STATION_ID][0])
	{
		sprintf(g_tempStr, "*   Callsign: %s\n", g_messages_text[STATION_ID]);
		sb_send_string(g_tempStr);
	}
	else
	{
		sb_send_string((char*)"*   Callsign: None\n");
	}
		
	sprintf(g_tempStr, "*   Callsign WPM: %d wpm\n", g_id_codespeed);	
	sb_send_string(g_tempStr);
	
	sprintf(g_tempStr, "*   Xmit Pattern: %s\n", getCurrentPatternText());
	sb_send_string(g_tempStr);
	sprintf(g_tempStr, "*   Xmit Pattern WPM: %u\n", getFoxCodeSpeed());
	sb_send_string(g_tempStr);
	
	Frequency_Hz transmitter_freq = getFrequencySetting();

	if(transmitter_freq)
	{
		if(!frequencyString(buf, transmitter_freq))
		{
			sprintf(g_tempStr, "*   Freq: %s\n", buf);						
		}
		else
		{
			sprintf(g_tempStr, "*   Freq: %lu\n", transmitter_freq);
		}
					
		sb_send_string(g_tempStr);
	}
	else
	{
		sb_send_string((char*)"*   Freq: None set\n");
	}
	
	sprintf(g_tempStr, "*   Cal: %d\n", RTC_get_cal()); /* Read value directly from RTC */
	sb_send_string(g_tempStr);
	
	if(g_days_to_run > 1)
	{
		uint8_t days_remaining = g_days_to_run - g_days_run;
		
		if(days_remaining)
		{
			sprintf(g_tempStr, "\n*   == Runs for %d days ==\n", days_remaining);
			sb_send_string(g_tempStr);
		}
	}
	
	sprintf(g_tempStr, "*   Start:  %s\n", convertEpochToTimeString(g_event_start_epoch, buf, 50));
	sb_send_string(g_tempStr);
	sprintf(g_tempStr, "*   Finish: %s\n", convertEpochToTimeString(g_event_finish_epoch, buf, 50));
	sb_send_string(g_tempStr);

	if(g_event != EVENT_NONE)
	{
		sb_send_string(TEXT_EVENT_SETTINGS_TXT);
		
		if(!frequencyString(buf, g_frequency_low))
		{
			sprintf(g_tempStr, "*   Freq Low: %s\n", buf);
			sb_send_string(g_tempStr);
		}

		if(!frequencyString(buf, g_frequency_med))
		{
			sprintf(g_tempStr, "*   Freq Med: %s\n", buf);
			sb_send_string(g_tempStr);
		}

		if(!frequencyString(buf, g_frequency_hi))
		{
			sprintf(g_tempStr, "*   Freq High: %s\n", buf);
			sb_send_string(g_tempStr);
		}

		if(!frequencyString(buf, g_frequency_beacon))
		{
			sprintf(g_tempStr, "*   Beacon Freq: %s\n", buf);
			sb_send_string(g_tempStr);
		}
	}
	
	ConfigurationState_t cfg = clockConfigurationCheck();
	  	
	if((cfg != WAITING_FOR_START) && (cfg != EVENT_IN_PROGRESS) && (cfg != SCHEDULED_EVENT_WILL_NEVER_RUN))
	{
		sb_send_string((char*)"\n* Needed Actions:\n");
		reportConfigErrors();
	}
	else
	{
		reportTimeTill(now, g_event_start_epoch, "*    Starts in: ", "*    In progress\n");
		reportTimeTill(g_event_start_epoch, g_event_finish_epoch, "*    Lasts: ", NULL);
		if(g_event_start_epoch < now)
		{
			reportTimeTill(now, g_event_finish_epoch, "*    Time Remaining: ", NULL);
		}
		  	
		if(!g_event_enabled)
		{
			sb_send_string((char*)"*   Start with > GO 2\n");
		}
	}
}

uint8_t bcd2dec(uint8_t val)
{
	uint8_t result = 10 * (val >> 4) + (val & 0x0F);
	return( result);
}

uint8_t dec2bcd(uint8_t val)
{
	uint8_t result = val % 10;
	result |= (val / 10) << 4;
	return (result);
}

uint8_t char2bcd(char c[])
{
	uint8_t result = (c[1] - '0') + ((c[0] - '0') << 4);
	return( result);
}

time_t epoch_from_ltm(tm *ltm)
{
	time_t epoch = ltm->tm_sec + ltm->tm_min * 60 + ltm->tm_hour * 3600L + ltm->tm_yday * 86400L +
	(ltm->tm_year - 70) * 31536000L + ((ltm->tm_year - 69) / 4) * 86400L -
	((ltm->tm_year - 1) / 100) * 86400L + ((ltm->tm_year + 299) / 400) * 86400L;

	return(epoch);
}


/**
 *   Converts an epoch (seconds since 1900)  into a string with format "ddd dd-mon-yyyy hh:mm:ss zzz"
 */
#define THIRTY_YEARS 946684800
char* convertEpochToTimeString(time_t epoch, char* buf, size_t size)
 {
   struct tm  ts;
	time_t t = epoch;
	
	if(epoch >= THIRTY_YEARS)
	{
		t = epoch - THIRTY_YEARS;
	}

    // Format time, "ddd dd-mon-yyyy hh:mm:ss zzz"
    ts = *localtime(&t);
    strftime(buf, size, "%a %d-%b-%Y %H:%M:%S", &ts);
   return buf;
 }



/* Returns the UNIX epoch value for the character string passed in datetime. If datetime is null then it returns
the UNIX epoch for the time held in the DS3231 RTC. If error is not null then it holds 1 if an error occurred */
time_t String2Epoch(bool *error, char *datetime)
{
	time_t epoch = 0;
	uint8_t data[7] = { 0, 0, 0, 0, 0, 0, 0 };

	struct tm ltm = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int16_t year = 100;                 /* start at 100 years past 1900 */
	uint8_t month;
	uint8_t date;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	if(datetime)                            /* String format "YYMMDDhhmmss" */
	{
		data[0] = char2bcd(&datetime[10]);  /* seconds in BCD */
		data[1] = char2bcd(&datetime[8]);   /* minutes in BCD */
		data[2] = char2bcd(&datetime[6]);   /* hours in BCD */
		/* data[3] =  not used */
		data[4] = char2bcd(&datetime[4]);   /* day of month in BCD */
		data[5] = char2bcd(&datetime[2]);   /* month in BCD */
		data[6] = char2bcd(&datetime[0]);   /* 2-digit year in BCD */

		hours = bcd2dec(data[2]); /* Must be calculated here */

		year += (int16_t)bcd2dec(data[6]);
		ltm.tm_year = year;                         /* year since 1900 */

		year += 1900;                               /* adjust year to calendar year */

		month = bcd2dec(data[5]);
		ltm.tm_mon = month - 1;                     /* mon 0 to 11 */

		date = bcd2dec(data[4]);
		ltm.tm_mday = date;                         /* month day 1 to 31 */

		ltm.tm_yday = 0;
		for(uint8_t mon = 1; mon < month; mon++)    /* months from 1 to 11 (excludes partial month) */
		{
			ltm.tm_yday += month_length(year, mon);;
		}

		ltm.tm_yday += (ltm.tm_mday - 1);

		seconds = bcd2dec(data[0]);
		minutes = bcd2dec(data[1]);

		ltm.tm_hour = hours;
		ltm.tm_min = minutes;
		ltm.tm_sec = seconds;

		epoch = epoch_from_ltm(&ltm);
	}

	if(error)
	{
		*error = (epoch == 0);
	}

	return(epoch);
}

uint16_t timeNeededForID(void)
{
	return((uint16_t)(((float)timeRequiredToSendStrAtWPM((char*)g_messages_text[STATION_ID], g_id_codespeed)) / 1000.));
}

Fox_t getFoxSetting(void)
{
	return g_fox[g_event];
}

int getFoxCodeSpeed(void)
{
	if(g_fox[g_event] == BEACON)
	{
		return(g_pattern_codespeed);
	}
	else if(g_event == EVENT_FOXORING)
	{
		return(g_foxoring_pattern_codespeed);
	}
	
	return(g_pattern_codespeed);
}

int getPatternCodeSpeed(void)
{
	if(!g_event_commenced)
	{
		return ENUNCIATION_BLINK_WPM;
	}
	
	return(getFoxCodeSpeed());
}

char* getCurrentPatternText(void)
{
	char* c;
	
	switch(g_fox[g_event])
	{
		case FOX_1:
		{
			c = (char*)"MOE";
		}
		break;
		
		case FOX_2:
		{
			c = (char*)"MOI";
		}
		break;
		
		case FOX_3:
		{
			c = (char*)"MOS";
		}
		break;
		
		case FOX_4:
		{
			c = (char*)"MOH";
		}
		break;
		
		case FOX_5:
		{
			c = (char*)"MO5";
		}
		break;
		
		case SPECTATOR:
		{
			c = (char*)"S";
		}
		break;
		
		case SPRINT_S1:
		{
			c = (char*)"ME";
		}
		break;
		
		case SPRINT_S2:
		{
			c = (char*)"MI";
		}
		break;
		
		case SPRINT_S3:
		{
			c = (char*)"MS";
		}
		break;
		
		case SPRINT_S4:
		{
			c = (char*)"MH";
		}
		break;
		
		case SPRINT_S5:
		{
			c = (char*)"M5";
		}
		break;
		
		case SPRINT_F1:
		{
			c = (char*)"OE";
		}
		break;
		
		case SPRINT_F2:
		{
			c = (char*)"OI";
		}
		break;
		
		case SPRINT_F3:
		{
			c = (char*)"OS";
		}
		break;
		
		case SPRINT_F4:
		{
			c = (char*)"OH";
		}
		break;
		
		case SPRINT_F5:
		{
			c = (char*)"O5";
		}
		break;
		
		case FOXORING_FOX1:
		case FOXORING_FOX2:
		case FOXORING_FOX3:
		{
			c = g_messages_text[FOXORING_PATTERN_TEXT];
		}
		break;
		
		case BEACON:
		{
			c = (char*)"MO";
		}
		break;
		
		default:
		{
			c = g_messages_text[PATTERN_TEXT];
		}
		break;
	}
	
	return c;
}

Frequency_Hz getFrequencySetting(void)
{
	Frequency_Hz freq;
	
	switch(g_fox[g_event])
	{
		case BEACON:
		{
			freq = g_frequency_beacon;
		}
		break;
		
		case FOX_1:
		case FOX_2:
		case FOX_3:
		case FOX_4:
		case FOX_5:
		{
			freq = g_frequency_low;
		}
		break;
		
		case SPECTATOR:
		{
			freq = g_frequency_med;
		}
		break;
		
		case SPRINT_S1:
		case SPRINT_S2:
		case SPRINT_S3:
		case SPRINT_S4:
		case SPRINT_S5:
		{
			freq = g_frequency_low;
		}
		break;
		
		case SPRINT_F1:
		case SPRINT_F2:
		case SPRINT_F3:
		case SPRINT_F4:
		case SPRINT_F5:
		{
			freq = g_frequency_hi;
		}
		break;
		
		case FOXORING_FOX1:
		{
			freq = g_frequency_low;
		}
		break;
		
		case FOXORING_FOX2:
		{
			freq = g_frequency_med;
		}
		break;
		
		case FOXORING_FOX3:
		{
			freq = g_frequency_hi;
		}
		break;
		
		default:
		{
			freq = g_frequency;
		}
		break;
	}

	return(freq);
}


void handleSerialCloning(void)
{
	if(g_programming_countdown == 0)
	{
		g_programming_state = SYNC_Searching_for_slave;
		g_cloningInProgress = false;
		g_programming_countdown = -1;
	}
	
	SerialbusRxBuffer* sb_buff = nextFullSBRxBuffer();
	SBMessageID msg_id;
	
	if(sb_buff)
	{
		isMasterCountdownSeconds = 600; /* Extend Master time */
		LEDS.init(); /* Extend or resume LED operation */
	}
	
	if(!g_programming_msg_throttle && !g_cloningInProgress)
	{
		sb_send_master_string((char*)"MAS P\r"); /* Set slave to active cloning state */
		g_programming_msg_throttle = 600;
		g_programming_state = SYNC_Searching_for_slave;
	}
	
	
	switch(g_programming_state)
	{
		case SYNC_Searching_for_slave:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_MASTER) /* Slave responds with MAS message */
				{
					g_cloningInProgress = true;
					g_seconds_transition = false;
					g_programming_state = SYNC_Align_to_Second_Transition;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
// 				else
// 				{
// 					handleSerialBusMsgs();
// 				}
			}			
		}
		break;
		
		
		case SYNC_Align_to_Second_Transition:
		{
			if(g_seconds_transition)
			{
				time_t now = time(null);
				g_event_checksum = now;
				sprintf(g_tempStr, "CLK T %lu\r", now); /* Set slave's RTC */
				sb_send_master_string(g_tempStr);
				g_programming_state = SYNC_Waiting_for_CLK_T_reply;
				g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
			}
		}
		break;
		
		
		case SYNC_Waiting_for_CLK_T_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_CLOCK)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'T')
					{
						g_event_checksum += g_event_start_epoch;
						g_programming_state = SYNC_Waiting_for_CLK_S_reply;
 						sprintf(g_tempStr, "CLK S %lu\r", g_event_start_epoch);
 						sb_send_master_string(g_tempStr);
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
			}
		}
		break;

		case SYNC_Waiting_for_CLK_S_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_CLOCK)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'S')
					{
						g_event_checksum += g_event_finish_epoch;
						g_programming_state = SYNC_Waiting_for_CLK_F_reply;
 						sprintf(g_tempStr, "CLK F %lu\r", g_event_finish_epoch);
 						sb_send_master_string(g_tempStr);
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
			}
		}	
		break;

		case SYNC_Waiting_for_CLK_F_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_CLOCK)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'F')
					{
						g_event_checksum += g_days_to_run;
						g_programming_state = SYNC_Waiting_for_CLK_D_reply;
 						sprintf(g_tempStr, "CLK D %d\r", g_days_to_run);
 						sb_send_master_string(g_tempStr);
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
			}
		}
		break;
				
		case SYNC_Waiting_for_CLK_D_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_CLOCK)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'D')
					{
						g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
						g_programming_state = SYNC_Waiting_for_ID_reply;
						
						char* ptr1=NULL;
						char* ptr2=NULL;
						int ch = ' ';
						
						for(uint8_t i=0; i<strlen(g_messages_text[STATION_ID]); i++)
						{
							g_event_checksum += g_messages_text[STATION_ID][i];
						}
 
						ptr1 = strchr( g_messages_text[STATION_ID], ch );
						
						if(ptr1)
						{
							ptr1++;
							ptr2 = strchr((const char*)ptr1, ch);
							
							if(ptr2)
							{
								*ptr2 = '\0';
								ptr2++;
								
								sprintf(g_tempStr, "ID %s %s\r", ptr1, ptr2);
							}
							else
							{
								sprintf(g_tempStr, "ID %s\r", ptr1);
							}
						}
						else
						{
							strcpy(g_tempStr, "ID \"\"\r");
						}
 
						sb_send_master_string(g_tempStr);
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
			}
		}
		break;

		case SYNC_Waiting_for_ID_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_SET_STATION_ID)
				{
					g_event_checksum += g_id_codespeed;
					g_programming_state = SYNC_Waiting_for_ID_CodeSpeed_reply;
					sprintf(g_tempStr, "SPD I %u\r", g_id_codespeed);
					sb_send_master_string(g_tempStr);
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}	
		break;


		case SYNC_Waiting_for_ID_CodeSpeed_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_CODE_SETTINGS)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'I')
					{
						g_programming_state = SYNC_Waiting_for_Pattern_CodeSpeed_reply;
						sprintf(g_tempStr, "SPD P %u\r", g_pattern_codespeed);
						
						g_event_checksum += g_pattern_codespeed;
						sb_send_master_string(g_tempStr);
						g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
			}
		}
		break;
		
		
		case SYNC_Waiting_for_Pattern_CodeSpeed_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_CODE_SETTINGS)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'P')
					{
						char c = '\0';
					
						if(g_event == EVENT_CLASSIC)
						{
							c = 'C';
						}
						else if(g_event == EVENT_FOXORING)
						{
							c = 'F';
						}
						else if(g_event == EVENT_SPRINT)
						{
							c = 'S';
						}
						else
						{
							c = 'N';
						}
						
						g_event_checksum += c;
						sprintf(g_tempStr, "EVT %c\r", c);
						sb_send_master_string(g_tempStr); /* Set slave's event */
						g_programming_state = SYNC_Waiting_for_EVT_reply;
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					}
				}
			}
		}	
		break;
		
		
		case SYNC_Waiting_for_EVT_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_EVENT) /* Slave responds with EVT message */
				{
					g_event_checksum += g_frequency;
					g_programming_state = SYNC_Waiting_for_NoEvent_Freq_reply;
					sprintf(g_tempStr, "FRE X %lu\r", g_frequency);
					sb_send_master_string(g_tempStr);
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}
		break;
		
		case SYNC_Waiting_for_NoEvent_Freq_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_TX_FREQ) /* Slave responds with EVT message */
				{
					g_event_checksum += g_frequency_low;
					g_programming_state = SYNC_Waiting_for_Freq_Low_reply;
					sprintf(g_tempStr, "FRE L %lu\r", g_frequency_low);
					sb_send_master_string(g_tempStr);
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}
		break;
			
		case SYNC_Waiting_for_Freq_Low_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_TX_FREQ)
				{
					g_event_checksum += g_frequency_med;
					g_programming_state = SYNC_Waiting_for_Freq_Med_reply;
					sprintf(g_tempStr, "FRE M %lu\r", g_frequency_med);
					sb_send_master_string(g_tempStr);
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}	
		break;

		case SYNC_Waiting_for_Freq_Med_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_TX_FREQ)
				{
					g_event_checksum += g_frequency_hi;
					g_programming_state = SYNC_Waiting_for_Freq_Hi_reply;
					sprintf(g_tempStr, "FRE H %lu\r", g_frequency_hi);
					sb_send_master_string(g_tempStr);
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}	
		break;

		case SYNC_Waiting_for_Freq_Hi_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_TX_FREQ)
				{
					g_event_checksum += g_frequency_beacon;
					g_programming_state = SYNC_Waiting_for_Freq_Beacon_reply;
					sprintf(g_tempStr, "FRE B %lu\r", g_frequency_beacon);
					sb_send_master_string(g_tempStr);
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}	
		break;

		case SYNC_Waiting_for_Freq_Beacon_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_TX_FREQ)
				{
					g_programming_state = SYNC_Waiting_for_ACK;
					sprintf(g_tempStr, "MAS Q %lu\r", g_event_checksum);
					sb_send_master_string(g_tempStr);
				}
			}
		}	
		break;


		case SYNC_Waiting_for_ACK:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_MASTER)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'A')
					{
						g_send_clone_success_countdown = 18000;
					}
					else
					{
						isMasterCountdownSeconds = 600; /* Extend Master time */
						g_programming_msg_throttle = 0;
						g_cloningInProgress = false;
					}
					
					g_programming_state = SYNC_Searching_for_slave;
				}
			}
		}
		break;
	}
	
	if(sb_buff) sb_buff->id = SB_MESSAGE_EMPTY;

}

bool eventScheduledForNow(void)
{
	time_t now = time(null);	
	bool result = false;
	
	if((now > MINIMUM_VALID_EPOCH) && (g_event_start_epoch > MINIMUM_VALID_EPOCH))
	{
		result = ((g_event_start_epoch < now) && (g_event_finish_epoch > now));
	}
	
	return(result);
}

bool eventScheduledForTheFuture(void)
{
	time_t now = time(null);	
	bool result = false;
	
	if(now > MINIMUM_VALID_EPOCH)
	{
		result = ((g_event_start_epoch > now) && (g_event_finish_epoch > g_event_start_epoch));
	}
	
	return(result);
}

bool eventScheduled(void)
{
	time_t now = time(null);	
	bool result = false;
	
	if(now > MINIMUM_VALID_EPOCH)
	{
		result = eventScheduledForTheFuture() || eventScheduledForNow();
		
		if(!result)
		{ 
			uint8_t days_remaining = g_days_to_run - g_days_run;
			if(days_remaining > 0)
			{
				if((g_event_start_epoch > MINIMUM_VALID_EPOCH) && (g_event_finish_epoch > g_event_start_epoch))
				{
					uint16_t days = days_remaining;
					time_t s = g_event_start_epoch;
					time_t f = g_event_finish_epoch;
				
					while((s < now) && days--)
					{
						s += SECONDS_24H;
						f += SECONDS_24H;
					}
				
					if(s > now)
					{
						g_event_start_epoch = s;
						g_event_finish_epoch = f;
						result = true;
					}
				}
			}
		}
	}
	
	return(result);
}