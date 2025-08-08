#include "atmel_start.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/sleep.h>
#include <atomic.h>
#include <math.h>

#include "serialbus.h"
#include "transmitter.h"
#include "morse.h"
#include "adc.h"
#include "util.h"
#include "eeprommanager.h"
#include "binio.h"
#include "leds.h"
#include "CircularStringBuff.h"
#include "rtc.h"
#include "rstctrl.h"

#include <cpuint.h>
#include <ccp.h>

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
	AWAKENED_INIT,
	POWER_UP_START,
	AWAKENED_BY_CLOCK,
	AWAKENED_BY_BUTTONPRESS
} Awakened_t;

typedef enum
{
	HARDWARE_OK,
	HARDWARE_NO_RTC = 0x01,
	HARDWARE_NO_SI5351 = 0x02
} HardwareError_t;


typedef enum {
	SYNC_Searching_for_slave,
	SYNC_Waiting_for_FUN_A_reply,
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

#define PROGRAMMING_MESSAGE_TIMEOUT_PERIOD 3000


/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
#define TEMP_STRING_SIZE 50
static char g_tempStr[TEMP_STRING_SIZE+1] = { '\0' };
static volatile EC g_last_error_code = ERROR_CODE_NO_ERROR;
static volatile SC g_last_status_code = STATUS_CODE_IDLE;

volatile bool g_device_enabled = false;

static volatile bool g_powering_off = false;

static volatile bool g_battery_measurements_active = false;
static volatile uint16_t g_maximum_battery = 0;

static volatile bool g_start_event = false;

static volatile int32_t g_on_the_air = 0;
static volatile int g_sendID_seconds_countdown = 0;
static volatile uint16_t g_code_throttle = 50;
static volatile uint16_t g_sleepshutdown_seconds = 300;
static volatile bool g_report_seconds = false;
static volatile int g_hardware_error = (int)HARDWARE_OK;
static volatile bool g_commence_transmissions = false;

char g_messages_text[STATION_ID+1][MAX_PATTERN_TEXT_LENGTH + 2];
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
volatile float g_internal_voltage_low_threshold = EEPROM_INT_BATTERY_LOW_THRESHOLD_V;
volatile float g_internal_bat_voltage = 0.;
volatile bool g_internal_bat_detected = false;
volatile float g_external_voltage = 0.;
volatile float g_processor_temperature = MINIMUM_VALID_TEMP - 1.;
volatile float g_processor_min_temperature = MAXIMUM_VALID_TEMP + 1.;
volatile float g_processor_max_temperature = MINIMUM_VALID_TEMP - 1.;
volatile bool g_restart_conversions = false;
volatile bool g_seconds_transition = false;
volatile bool g_sending_station_ID = false;											/* Allows a small extension of transmissions to ensure the ID is fully sent */
volatile bool g_muteAfterID = false;												/* Inhibit any transmissions after the ID has been sent */
volatile uint32_t g_event_checksum = 0;
volatile uint8_t g_days_to_run = 1;
volatile uint8_t g_days_run = 0;
volatile Function_t g_function = Function_ARDF_TX;
extern uint16_t g_clock_calibration;

static volatile bool g_run_event_forever = false;
static volatile bool g_go_to_sleep_now = false;
static volatile bool g_sleeping = false;
static volatile time_t g_time_to_wake_up = 0;
static volatile Awakened_t g_awakenedBy = POWER_UP_START;
static volatile SleepType g_sleepType = SLEEP_FOREVER;
static volatile time_t g_time_since_wakeup = 0;
static volatile bool g_check_for_long_wakeup_press = true;
static volatile bool g_device_wakeup_complete = false;
static volatile bool g_charge_battery = false;
extern bool g_enable_boost_regulator;
static volatile bool g_turn_on_fan = false;


#define NUMBER_OF_POLLED_ADC_CHANNELS 3
static ADC_Active_Channel_t g_adcChannelOrder[NUMBER_OF_POLLED_ADC_CHANNELS] = { ADCInternalBatteryVoltage, ADCExternalBatteryVoltage, ADCTemperature };
static const uint16_t g_adcChannelConversionPeriod_ticks[NUMBER_OF_POLLED_ADC_CHANNELS] = { TIMER2_0_5HZ, TIMER2_0_5HZ, TIMER2_0_5HZ };
static volatile uint16_t g_adcCountdownCount[NUMBER_OF_POLLED_ADC_CHANNELS] = { 2000, 2000, 4000 };
static volatile bool g_adcUpdated[NUMBER_OF_POLLED_ADC_CHANNELS] = { false, false, false };
static volatile uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS] = { 0, 0, 0 };
static volatile bool g_temperature_shutdown = false;

volatile uint16_t g_switch_closed_time = 0;
volatile uint16_t g_handle_counted_presses = 0;
volatile uint16_t g_switch_presses_count = 0;
volatile bool g_long_button_press = false;
volatile uint16_t g_button_hold_countdown;

#define NUMBER_OF_TEST_FREQUENCIES (4)
uint8_t g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;

static volatile uint16_t g_programming_countdown = 0;
static volatile uint16_t g_programming_msg_throttle = 0;
static volatile uint16_t g_send_clone_success_countdown = 0;
static SyncState_t g_programming_state = SYNC_Searching_for_slave;
bool g_cloningInProgress = false;
Enunciation_t g_enunciator = LED_ONLY;
static volatile uint16_t g_key_down_countdown = 0;
static volatile bool g_reset_after_keydown = false;
static volatile uint32_t g_utility_countdown = 0;

uint16_t g_Event_Configuration_Check = 0;

leds LEDS = leds();
CircularStringBuff g_text_buff = CircularStringBuff(TEXT_BUFF_SIZE);

EepromManager g_ee_mgr;

bool g_isMaster = false;
uint16_t isMasterCountdownSeconds = 0;
Fox_t g_fox[] = {FOX_1, FOX_1, SPRINT_S1, FOXORING_FOX1, INVALID_FOX}; /* none, classic, sprint, foxoring */

Event_t g_event = EEPROM_EVENT_SETTING_DEFAULT;
Frequency_Hz g_frequency = EEPROM_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_low = EEPROM_FREQUENCY_LOW_DEFAULT;
Frequency_Hz g_frequency_med = EEPROM_FREQUENCY_MED_DEFAULT;
Frequency_Hz g_frequency_hi = EEPROM_FREQUENCY_HI_DEFAULT;
Frequency_Hz g_frequency_beacon = EEPROM_FREQUENCY_BEACON_DEFAULT;

int8_t g_utc_offset;
uint8_t g_unlockCode[UNLOCK_CODE_SIZE + 2];

extern volatile uint16_t g_i2c_failure_count;
extern volatile bool g_enable_external_battery_control;

volatile bool g_enable_manual_transmissions = true;

/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
void handle_1sec_tasks(void);
bool eventEnabled(void);
void handleSerialBusMsgs(void);
uint16_t throttleValue(uint8_t speed);
EC activateEventUsingCurrentSettings(SC* statusCode);
EC launchEvent(SC* statusCode);
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
bool noEventWillRun(void);
void configRedLEDforEvent(void);
bool switchIsClosed(void);

/*******************************/
/* Hardcoded event support     */
/*******************************/
void suspendEvent(void);
void startEventNow(void);
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
	
	g_time_since_wakeup++;
	
	if(isMasterCountdownSeconds) isMasterCountdownSeconds--;

	if(!g_cloningInProgress)
	{
		temp_time = time(null);

		if(g_event_commenced && !g_run_event_forever)
		{
			if(g_event_finish_epoch) /* If a finish time has been set */
			{
				if(temp_time >= g_event_finish_epoch)
				{
					g_last_status_code = STATUS_CODE_EVENT_FINISHED;
					g_on_the_air = 0;
					keyTransmitter(OFF);
					g_event_enabled = false;
					g_event_commenced = false;
					LEDS.init();
					g_days_run++;
					g_sleepshutdown_seconds = 3;
					
					if(!g_enable_external_battery_control) setExtBatLoadSwitch(OFF, INITIALIZE_LS);
				
					if(g_days_run < g_days_to_run)
					{
						g_event_start_epoch += SECONDS_24H;
						g_event_finish_epoch += SECONDS_24H;
						g_sleepType = SLEEP_UNTIL_START_TIME;
						g_go_to_sleep_now = true;
					}
					else
					{
						g_sleepType = SLEEP_FOREVER;
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
						}
						
						g_code_throttle = throttleValue(getFoxCodeSpeed());
						bool repeat = true;
						makeMorse(getCurrentPatternText(), &repeat, NULL, CALLER_AUTOMATED_EVENT);
						
						g_commence_transmissions = true;
						LEDS.init();
						
						if(!g_enable_external_battery_control) setExtBatLoadSwitch(ON, INITIALIZE_LS);
					}
				}
			}
		}
	}

	/**************************************
	* Delay before sleep
	***************************************/
	if(g_sleepshutdown_seconds) 
	{
		g_sleepshutdown_seconds--;

		if(!g_sleepshutdown_seconds)
		{
 			if(g_isMaster || g_cloningInProgress)
 			{
	 			g_sleepshutdown_seconds = 300; /* Never sleep while cloning or while master */
 			}
			else if(g_event_commenced && g_event_enabled && ((g_sleepType != SLEEP_UNTIL_NEXT_XMSN) && (g_sleepType != SLEEP_UNTIL_START_TIME)) && (g_fox[g_event] != FREQUENCY_TEST_BEACON))
			{
				g_sleepshutdown_seconds = 300; /* Never sleep during active transmissions */
			}
			else
			{
				if(g_sleepType == DO_NOT_SLEEP)
				{
					if(noEventWillRun()) /* Should never evaluate to true, but checking just in case */
					{
						g_sleepType = SLEEP_FOREVER;
					}
				}
				else if(!g_go_to_sleep_now)
				{
					if(g_sleepType == SLEEP_FOREVER)
					{
						if(eventScheduledForTheFuture()) /* Should never evaluate to true here, but checking just in case */
						{
							g_sleepType = SLEEP_UNTIL_START_TIME;
						}
					}
				
					/* If we reach here, g_sleepType = 	SLEEP_UNTIL_START_TIME, SLEEP_UNTIL_NEXT_XMSN, or SLEEP_FOREVER */
					g_go_to_sleep_now = true;
				}
			}
		}
	}
}


/**
Periodic tasks not requiring precise timing. Rate = 300 Hz
*/
ISR(TCB0_INT_vect)
{
	static uint8_t fiftyMS = 6;
	static bool on_air_finished = false;
	static bool transitionPrepped = false;
	static int idTimeout = 0;
	static bool id_timed_out = false;
	
	uint8_t x = TCB0.INTFLAGS;
	
    if(x & TCB_CAPT_bm)
    {
		static bool conversionInProcess = false;
		static int8_t indexConversionInProcess = 0;
		static uint16_t codeInc = 0;
		bool repeat, finished;
		static uint16_t switch_closures_count_period = 0;
		uint8_t holdSwitch = 0;
		static uint8_t buttonReleased = false;
		static uint8_t longPressEnabled = true;
		static bool muteAfterID = false;				/* Inhibit any transmissions immediately after the ID has been sent */
		
		if(g_key_down_countdown) 
		{
			g_key_down_countdown--;
			
			if(!g_key_down_countdown)
			{
				g_reset_after_keydown = true;
			}
		}
		
		if(g_button_hold_countdown)
		{
			g_button_hold_countdown--;
		}
		
		if(g_utility_countdown)
		{
			g_utility_countdown--;
		}
		
		/* Handle pushbutton press detection */
		
		if(g_device_wakeup_complete)
		{
			fiftyMS++;
			if(!(fiftyMS % 6))
			{
				holdSwitch = portDdebouncedVals() & (1 << SWITCH);
				debounce();
			
				if(holdSwitch != (portDdebouncedVals() & (1 << SWITCH))) /* Change detected */
				{	
					if(holdSwitch) /* Switch was open, so it must have just now closed */
					{
						if(LEDS.active())
						{
							g_switch_presses_count++;
							buttonReleased = false;
						}
						else
						{
							longPressEnabled = false;
						}
					}
					else /* Switch is now open */
					{
						if(!LEDS.active())
						{
							LEDS.init();
							serialbus_init(SB_BAUD, SERIALBUS_USART);
						}
						else
						{
							g_switch_closed_time = 0;
							buttonReleased = true;
						}
					
						longPressEnabled = true;
					}
				}
				else if(!holdSwitch && LEDS.active()) /* Switch closed, LEDs operating */
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
		
				if(switch_closures_count_period) // Time to check if button presses have occurred
				{
					static uint8_t hold_switch_presses_count = 0;
					switch_closures_count_period--;
				
					if(!switch_closures_count_period) // Time's up - examine how many button presses were counted
					{
						if(g_switch_presses_count && (g_switch_presses_count <= MAXIMUM_NUM_OF_KEYPRESSES))
						{
							g_handle_counted_presses = g_switch_presses_count;
						}
					
						g_switch_presses_count = 0;
						hold_switch_presses_count = 0;
					}
					else if(g_switch_presses_count != hold_switch_presses_count) // Press detected - wait a while longer to see if there's another one
					{
						hold_switch_presses_count = g_switch_presses_count;
						switch_closures_count_period = 40;
					}
				}
				else if(g_switch_presses_count == 1 && buttonReleased)
				{
					switch_closures_count_period = 40;
				}
				else if(g_switch_presses_count > MAXIMUM_NUM_OF_KEYPRESSES) // Too many button presses - ignore them entirely
				{
					g_switch_presses_count = 0;
				}
			}
		}
		else
		{
			longPressEnabled = false;
			g_switch_closed_time = 0;
			g_switch_presses_count = 0;
			switch_closures_count_period = 0;
			g_long_button_press = false;
		}
				
		if(g_programming_countdown > 0) g_programming_countdown--;
		if(g_programming_msg_throttle) g_programming_msg_throttle--;
		if(g_send_clone_success_countdown) g_send_clone_success_countdown--;
							
		static bool key = false;
		if(idTimeout)
		{
			idTimeout--;
			
			if(!idTimeout)
			{
				id_timed_out = true;
			}
		}
		
		if(g_event_enabled && g_event_commenced && !g_isMaster) /* Handle cycling transmissions */
		{
			if((g_on_the_air > 0) || (g_sending_station_ID && !id_timed_out) || (!g_off_air_seconds))
			{
				on_air_finished = true;
				transitionPrepped = false;
				
				/* Interrupt transmissions and send the ID under these conditions */
				if(!g_sending_station_ID && (!g_off_air_seconds || (g_on_the_air <= g_time_needed_for_ID)) && !g_sendID_seconds_countdown && g_time_needed_for_ID) /* Time to identify on the air */
				{
					g_last_status_code = STATUS_CODE_SENDING_ID;
					g_code_throttle = throttleValue(g_id_codespeed);
					repeat = false;
					makeMorse(g_messages_text[STATION_ID], &repeat, NULL, CALLER_AUTOMATED_EVENT);  /* Send only once */
					g_sending_station_ID = true;
					idTimeout = g_code_throttle << 2;
					id_timed_out = false;
					g_sendID_seconds_countdown = g_ID_period_seconds;
					codeInc = g_code_throttle;
				}
				
				if(codeInc)
				{
					codeInc--;

					if(!codeInc)
					{
						key = makeMorse(NULL, &repeat, &finished, CALLER_AUTOMATED_EVENT);
						
						if(!repeat && finished) /* ID has completed or timed out, so resume pattern */
						{
							g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
							g_code_throttle = throttleValue(getFoxCodeSpeed());
							repeat = true;
							makeMorse(getCurrentPatternText(), &repeat, NULL, CALLER_AUTOMATED_EVENT);
							muteAfterID = g_sending_station_ID && g_off_air_seconds;
							g_sending_station_ID = false;
							idTimeout = 0;
							id_timed_out = false;
							
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
					id_timed_out = false;
					
					if(on_air_finished)
					{
						on_air_finished = false;							
						keyTransmitter(OFF);
						key = OFF;
						
						g_on_the_air = -g_off_air_seconds;
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
								if(g_sleepshutdown_seconds < 60) 
								{
									time_t seconds_to_sleep = (time_t)(g_off_air_seconds - 10);
									g_time_to_wake_up = temp_time + seconds_to_sleep;
									g_sleepType = SLEEP_UNTIL_NEXT_XMSN;
									g_go_to_sleep_now = true;
									g_sendID_seconds_countdown = MAX(0, g_ID_period_seconds - (int)seconds_to_sleep);
								}
							}
						}
	
						muteAfterID = false;
						g_sending_station_ID = false;
						idTimeout = 0;
						id_timed_out = false;
				
						/* Resume normal pattern */
						g_code_throttle = throttleValue(getFoxCodeSpeed());
						repeat = true;
						makeMorse(getCurrentPatternText(), &repeat, NULL, CALLER_AUTOMATED_EVENT);    /* Reset pattern to start */
						LEDS.setRed(OFF);
					}
					else /* Off-the-air period just finished, or the event just began while off the air */
					{
						g_on_the_air = g_on_air_seconds;
						g_code_throttle = throttleValue(getFoxCodeSpeed());
						repeat = true;
						makeMorse(getCurrentPatternText(), &repeat, NULL, CALLER_AUTOMATED_EVENT);
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
			
			if(lastMorseCaller() != CALLER_MANUAL_TRANSMISSIONS)
			{
				g_text_buff.reset();
				sendBuffEmpty = true;
				charFinished = true;
				makeMorse((char*)"\0", &repeat, null, CALLER_MANUAL_TRANSMISSIONS); 
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
						key = makeMorse(null, &repeat, &charFinished, CALLER_MANUAL_TRANSMISSIONS);

						if(charFinished) /* Completed, send next char */
						{
							if(!g_text_buff.empty())
							{
								static char cc[2]; /* Must be static because makeMorse saves only a pointer to the character array */
								g_code_throttle = throttleValue(getPatternCodeSpeed());
								cc[0] = g_text_buff.get();
								cc[1] = '\0';
								makeMorse(cc, &repeat, null, CALLER_MANUAL_TRANSMISSIONS);
								key = makeMorse(null, &repeat, &charFinished, CALLER_MANUAL_TRANSMISSIONS);
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
  		if(!conversionInProcess || g_restart_conversions)
  		{
 			/* Note: countdowns will pause while a conversion is in process. Conversions are so fast that this should not be an issue though. */
 			indexConversionInProcess = -1;
			g_restart_conversions = false;
 
 			for(uint8_t i = 0; i < NUMBER_OF_POLLED_ADC_CHANNELS; i++)
 			{
 				if(g_adcCountdownCount[i])
 				{
 					g_adcCountdownCount[i]--;
 				}
 
 				if(g_adcCountdownCount[i] == 0)
 				{
 					indexConversionInProcess = (int8_t)i;
					break;
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
			uint16_t hold = ADC0_read(); 		
				
 			if(hold < 4090)
 			{
 				g_adcUpdated[indexConversionInProcess] = true;
  				g_lastConversionResult[indexConversionInProcess] = hold;
				  
				if(g_adcChannelOrder[indexConversionInProcess] == ADCInternalBatteryVoltage)
				{
					float val = (0.00725 * (float)g_lastConversionResult[indexConversionInProcess]) + 0.05;
					float delta = fabs(g_internal_bat_voltage - val);
					
					if(delta > .1)
					{
						g_internal_bat_voltage = val;
					}
					else
					{
						g_internal_bat_voltage = (20. * g_internal_bat_voltage + val) / 21.;
					}
					
					g_internal_bat_detected = (g_internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE);
				}
				else if(g_adcChannelOrder[indexConversionInProcess] == ADCExternalBatteryVoltage)
				{
					if(!g_enable_external_battery_control || getExtBatLSEnable()) // Don't try to read a disconnected external battery
					{
						g_external_voltage = (0.00725 * (float)g_lastConversionResult[indexConversionInProcess]) + 0.05;
					}
				}
				else if(g_adcChannelOrder[indexConversionInProcess] == ADCTemperature)
				{
					float temp = temperatureCfromADC(hold);
					
					if(isValidTemp(temp))
					{
						g_processor_temperature = isValidTemp(g_processor_temperature) ? (g_processor_temperature + temp)/2. : temp;
						if(g_processor_temperature > g_processor_max_temperature) g_processor_max_temperature = g_processor_temperature;
						if(g_processor_temperature < g_processor_min_temperature) g_processor_min_temperature = g_processor_temperature;
						if(g_internal_bat_detected)
						{
							g_temperature_shutdown = g_temperature_shutdown ? !(g_processor_temperature < 55.) : g_processor_temperature > 60.;
						}
						else
						{
							g_temperature_shutdown = g_temperature_shutdown ? !(g_processor_temperature < 80.) : g_processor_temperature > 85.;
						}
						
						if(g_turn_on_fan)
						{
							if(g_processor_temperature <= FAN_TURN_OFF_TEMP)
							{
								g_turn_on_fan = false;
							}
						}
						else
						{
							if(g_processor_temperature > FAN_TURN_ON_TEMP)
							{
								g_turn_on_fan = true;
							}
						}
					}
				}
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
		else if(!sb_enabled())
		{
			serialbus_init(SB_BAUD, SERIALBUS_USART);
		}
		
		g_sleepshutdown_seconds = 300;
	}
	
	VPORTD.INTFLAGS = 0xFF; /* Clear all flags */
}

bool switchIsClosed(void)
{
	debounce();
	debounce();
	debounce();
	debounce();
	return(!(portDdebouncedVals() & (1 << SWITCH)));
}

/* Entry point for firmware; performs hardware initialization and main loop. */
int main(void)
{
	bool buttonHeldClosed = true;
	bool buttonReleasedDuringStartup = false;
	bool internal_bat_error = false;
	bool external_pwr_error = false;
	
	atmel_start_init();
	LEDS.blink(LEDS_OFF, true);
	
	g_ee_mgr.initializeEEPROMVars();
	
	g_ee_mgr.readNonVols();
	g_isMaster = false; /* Never start up as master */

	if(g_frequency == EEPROM_FREQUENCY_DEFAULT)
	{ 
		g_frequency = getFrequencySetting();
		txSetFrequency(&g_frequency, true);
	}
	
	RTC_set_calibration(g_clock_calibration);
					
	LEDS.blink(LEDS_RED_ON_CONSTANT);
	LEDS.blink(LEDS_GREEN_ON_CONSTANT);
	g_button_hold_countdown = 1000;
	
	while(util_delay_ms(2500)); /* Avoid possible race conditions with peripheral devices powering up */
	
	reportSettings();

	/* Check that the RTC is running */
	set_system_time(YEAR_2000_EPOCH);
	time_t now = time(null);
	while((util_delay_ms(4500)) && (now == time(null)));
		
	if(now == time(null))
	{
		LEDS.blink(LEDS_GREEN_OFF); // Signal that the first attempt failed
		LEDS.blink(LEDS_RED_OFF);
		while((util_delay_ms(3000)) && (now == time(null)));
	}

	sb_send_string(TEXT_RESET_OCCURRED_TXT);

	if(!g_device_enabled)
	{
		sb_send_string(TEXT_DEVICE_DISABLED_TXT);
	}

	if(now == time(null))
	{
		g_hardware_error |= (int)HARDWARE_NO_RTC;
		RTC_init_backup();
	}

	if(g_device_enabled)
	{
		int tries = 5;
		powerToTransmitter(ON);

		while(g_device_enabled && tries && (!txIsInitialized()))
		{
			--tries;
 			powerToTransmitter(ON);
		}
	}

	if(!txIsInitialized())
	{
		g_hardware_error |= (int)HARDWARE_NO_SI5351;
	}

	g_run_event_forever = false;
	g_start_event = g_device_enabled && eventEnabled(); /* Start any event stored in EEPROM */
	sb_send_NewPrompt();

	g_sleepshutdown_seconds = 300;
				
	buttonReleasedDuringStartup = false;
	
	/* Disable automatic ADC readings */
	TCB0.INTCTRL = 0;   /* Capture or Timeout: disable interrupts */
	TCB0.CTRLA = 0; /* Disable timer */
	/* The external battery control load switch is initialized to ON, so we don't need to set it here */
	g_internal_bat_voltage = readVoltage(ADCInternalBatteryVoltage);
	g_external_voltage = readVoltage(ADCExternalBatteryVoltage);
	
	g_internal_bat_detected = (g_internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE);

	if(g_internal_bat_detected && (g_internal_bat_voltage < g_internal_voltage_low_threshold) && (g_enable_external_battery_control > 0)) // An internal battery is present & not fully charged & charging from an external battery is enabled
	{
		g_charge_battery = true;
		setExtBatLoadSwitch(ON, INTERNAL_BATTERY_CHARGING);
	}
	g_restart_conversions = true;
	TIMERB_init();
	powerToTransmitter(OFF);

	while(1) 
	{
		if(g_check_for_long_wakeup_press)
		{
			buttonHeldClosed = switchIsClosed();
			
			if(!buttonHeldClosed || buttonReleasedDuringStartup) /* Pushbutton released early; go back to sleep */
			{				
				g_go_to_sleep_now = true;
				g_check_for_long_wakeup_press = false;
				g_handle_counted_presses = 0;
				LEDS.blink(LEDS_OFF);
			}
			else if(!g_button_hold_countdown) /* Pushbutton held down long enough; power up */
			{
				LEDS.init();
				g_long_button_press = false;
				g_check_for_long_wakeup_press = false;

				// If the event is disabled, schedule it to start.
				if(eventScheduled() && !g_event_enabled && !g_start_event)
				{
					g_start_event = true;
				}

				configRedLEDforEvent();
				sb_send_string(TEXT_NOT_SLEEPING_TXT);
				sb_send_NewPrompt();
			}
			else /* Spin your wheels waiting for above condition to test true */
			{
				LEDS.blink(LEDS_RED_ON_CONSTANT);
				LEDS.blink(LEDS_GREEN_ON_CONSTANT);
			}
		}
		else
		{							
			// Set internal battery charging
			if((g_internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE) && (g_external_voltage > EXT_BAT_CHARGE_SUPPORT_THRESH_LOW))
			{
				if(g_internal_bat_voltage < g_internal_voltage_low_threshold) // An adequate external voltage is present and an internal battery is present & not fully charged
				{
					g_charge_battery = true;
				}
				else if(g_internal_bat_voltage > INT_BAT_CHARGE_THRES_HIGH)
				{
					g_charge_battery = false;
				}
			}
			else
			{
				g_charge_battery = false;
			}
			
			if(g_enable_external_battery_control) 
			{
				setExtBatLoadSwitch(g_charge_battery, INTERNAL_BATTERY_CHARGING);
			}
			else
			{
				if(g_turn_on_fan)
				{
					if(!getExtBatLSEnable())
					{
						setExtBatLoadSwitch(ON, INITIALIZE_LS);
					}
				}
				else
				{
					if(getExtBatLSEnable())
					{
						setExtBatLoadSwitch(OFF, INITIALIZE_LS);
					}
				}
			}
			
			/********************************
			 * Handle sleep
			 ******************************/
			if(g_go_to_sleep_now && !g_cloningInProgress)
			{
				if((g_sleepType == SLEEP_FOREVER) || (g_sleepType == SLEEP_POWER_OFF_OVERRIDE))
				{
//					if(eventScheduled() && (g_sleepType != SLEEP_POWER_OFF_OVERRIDE)) 
					if(eventScheduled()) /* Never sleep forever if an event is scheduled to start in the future */
					{
						g_sleepType = SLEEP_UNTIL_START_TIME;
					}
					else
					{
						g_time_to_wake_up = FOREVER_EPOCH;
					
						time_t now = time(null);

						if(now < MINIMUM_VALID_EPOCH)
						{
							sb_send_string(TEXT_POWER_OFF);
							while((util_delay_ms(2000)) && serialbusTxInProgress());
							while(util_delay_ms(200)); // Let serial xmsn finish before power off
							if(!g_charge_battery) PORTA_set_pin_level(POWER_ENABLE, LOW); /* No need to preserve current time, so power off - but if charging, keep power switch on to measure internal battery level */
						}
						else
						{
							sb_send_string(TEXT_SLEEPING_TXT);
							while((util_delay_ms(2000)) && serialbusTxInProgress());
							while(util_delay_ms(200)); // Let serial xmsn finish before sleep
						}
					}
				}
				
				/* If sleeping until start time, make sure everything is set up properly for when that time arrives */
				if((g_sleepType == SLEEP_UNTIL_START_TIME) || (g_sleepType == SLEEP_POWER_OFF_OVERRIDE))
				{
					g_run_event_forever = false;
					g_event_commenced = false;
					g_start_event = false;
					g_commence_transmissions = false;
				}
				
				if(g_sleepType == SLEEP_POWER_OFF_OVERRIDE)
				{
					g_sleepType = SLEEP_FOREVER;
				}

				powerToTransmitter(OFF);
				if(!g_enable_external_battery_control) setExtBatLoadSwitch(OFF, INITIALIZE_LS);

				DISABLE_INTERRUPTS();
				LEDS.deactivate();
				serialbus_disable();
				
				system_sleep_config();

				SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_STDBY_gc);
				g_sleeping = true;
				g_awakenedBy = AWAKENED_INIT;			
				ENABLE_INTERRUPTS();
				
				/* Disable BOD? */
				
				while(g_go_to_sleep_now)
				{
					if(g_sleepType == SLEEP_FOREVER)
					{
						time_t now = time(null);
						static time_t hold_now = 0;
						
						if((now - hold_now) > 5)
						{
							hold_now = now;
							system_charging_config();
							g_internal_bat_voltage = readVoltage(ADCInternalBatteryVoltage); // Throw out first result following sleep
							g_external_voltage = readVoltage(ADCExternalBatteryVoltage);
							system_sleep_config();
							setExtBatLoadSwitch(RE_APPLY_LS_STATE);
						
							if(g_enable_external_battery_control)
							{

								if(g_internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE)
								{
									if(g_internal_bat_voltage < g_internal_voltage_low_threshold) // An external voltage is present and an internal battery is present & not fully charged
									{
										g_charge_battery = true;
									}
									else if(g_internal_bat_voltage >= INT_BAT_CHARGE_THRES_HIGH)
									{
										g_charge_battery = false;

										if(now < MINIMUM_VALID_EPOCH)
										{
											PORTA_set_pin_level(POWER_ENABLE, LOW); /* No need to preserve current time, so power off */
										}
									}
								}							
							}
							else
							{
								g_charge_battery = false;
							
								if(now < MINIMUM_VALID_EPOCH)
								{
									PORTA_set_pin_level(POWER_ENABLE, LOW); /* No need to preserve current time, so power off */
								}
							}
						}
					}
							
					if(g_enable_external_battery_control) setExtBatLoadSwitch(g_charge_battery, INTERNAL_BATTERY_CHARGING);

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
				g_time_since_wakeup = 0;
				atmel_start_init();
				if(g_enable_external_battery_control) setExtBatLoadSwitch(g_charge_battery, INTERNAL_BATTERY_CHARGING);
				
				if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS)
				{
					g_device_wakeup_complete = false; // Set the flag to ignore key presses other than an initial long press
					g_check_for_long_wakeup_press = true; // Set the flag to check for an initial long keypress before waking up the device
					LEDS.init();
					LEDS.blink(LEDS_RED_ON_CONSTANT);
					LEDS.blink(LEDS_GREEN_ON_CONSTANT);
					buttonHeldClosed = true;
				}
				else
				{
					serialbus_disable();
				}

				RTC_set_calibration(g_clock_calibration);
				while(util_delay_ms(2000));
				
				g_sleepshutdown_seconds = 300;
				
				if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS)
				{
					g_button_hold_countdown = 1000;
					g_handle_counted_presses = 0;
					
					if(g_sleepType == SLEEP_UNTIL_START_TIME) /* User woke up the transmitter early, before the start time */
					{
						g_start_event = false;
						g_event_commenced = false;
						g_commence_transmissions = false;
						if(!g_enable_external_battery_control) setExtBatLoadSwitch(ON, INITIALIZE_LS);
					}
				}
				else
				{
					if(g_sleepType != SLEEP_UNTIL_NEXT_XMSN)
					{
						g_start_event = true;
					}
				}

				g_last_status_code = STATUS_CODE_RETURNED_FROM_SLEEP;
			}
			else
			{
				g_device_wakeup_complete = true; // Set the flag to accept user key presses
			}
			
			if(g_handle_counted_presses)
			{				
				if(g_send_clone_success_countdown || g_cloningInProgress)
				{
					g_send_clone_success_countdown = 0;
					g_cloningInProgress = false;
					g_programming_msg_throttle = 0;
					g_handle_counted_presses = 0; /* Throw out any keypresses that occurred while cloning or sending cloning success pattern */
				}

				if(g_isMaster)
				{
					if(g_handle_counted_presses == 5)
					{
						if(g_key_down_countdown)
						{
							g_key_down_countdown = 0;
							LEDS.setRed(OFF);
							keyTransmitter(OFF);
							powerToTransmitter(OFF);
						}

						isMasterCountdownSeconds = 0;
						g_sleepshutdown_seconds = 300;
						sb_send_NewPrompt();
						g_event_commenced = false;
						g_start_event = true;
						g_cloningInProgress = false;
						g_programming_countdown = 0;
						g_send_clone_success_countdown = 0;
						LEDS.blink(LEDS_RED_OFF);
						g_text_buff.reset();					
					}
					else if (g_handle_counted_presses == 7) // Perform software reset
					{
						RSTCTRL_reset();
					}
					else if(g_handle_counted_presses == 9) // keydown 
					{
						if(!txIsInitialized())
						{
							powerToTransmitter(g_device_enabled);
						}
							
						if(g_key_down_countdown)
						{
							g_key_down_countdown = 0;
							LEDS.setRed(OFF);
							keyTransmitter(OFF);
							powerToTransmitter(OFF);
						}
						else
						{
							g_key_down_countdown = 9000;
							powerToTransmitter(g_device_enabled);
							LEDS.init();
							LEDS.setRed(ON);
							keyTransmitter(ON);
						}

						g_sleepshutdown_seconds = 300; // Shut things down after 5 minutes
					}
				}
				else if(g_handle_counted_presses == 1)
				{
					if(!g_cloningInProgress && g_device_enabled)
					{
						if(g_fox[g_event] == FREQUENCY_TEST_BEACON)
						{
							g_sleepshutdown_seconds = 300;
							if(!txIsInitialized()) powerToTransmitter(g_device_enabled);
							g_code_throttle = throttleValue(getFoxCodeSpeed());

							if(++g_frequency_to_test >= NUMBER_OF_TEST_FREQUENCIES) g_frequency_to_test = 0;
						
							if(g_frequency_to_test == 0)
							{
								txSetFrequency(&g_frequency_low, true);
							}
							else if(g_frequency_to_test == 1)
							{
								txSetFrequency(&g_frequency_med, true);
							}
							else if(g_frequency_to_test == 2)
							{
								txSetFrequency(&g_frequency_hi, true);
							}
							else if(g_frequency_to_test == 3)
							{
								txSetFrequency(&g_frequency_beacon, true);
							}
							
							setupForFox(INVALID_FOX, START_TRANSMISSIONS_NOW);		
						}
						else
						{
							if(eventScheduled())
							{
								/* Implement special behavior if an event is scheduled to commence in the future: have a single button press toggle between
								transmitting and slow blinking. But, in either case, the transmitter should eventually go to sleep and awaken at the
								appointed time for the future event */
								if(g_event_enabled && !g_run_event_forever) /* Start transmitting now, but go to sleep after a while */
								{
									g_event_enabled = false;
									LEDS.blink(LEDS_RED_OFF);
									setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER); // Immediately start transmissions
									g_sleepshutdown_seconds = 300; // run for 5 minutes, then sleep will configure the event to start using RTC
									g_last_error_code = launchEvent((SC*)&g_last_status_code);
									g_sleepType = SLEEP_UNTIL_START_TIME;
									g_start_event = false;
									if(!g_enable_external_battery_control) setExtBatLoadSwitch(ON, INITIALIZE_LS);
								}
								else /* Blink to indicate a future event is programmed */
								{
									g_event_enabled = false;
 									startEventUsingRTC();
									if(!g_enable_external_battery_control) setExtBatLoadSwitch(OFF, INITIALIZE_LS);
								}
							}
							else
							{
								/* No future event has been configured, so just transmit forever */
								if(!g_event_commenced) 
								{
									LEDS.blink(LEDS_RED_OFF);
									setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER); // Immediately start transmissions
								}
							}
						}
					}
				}
				else if (g_handle_counted_presses == 3)
				{
					g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;
					
					if(eventScheduled())
					{
						if(eventScheduledForNow())
						{
							if((!g_event_enabled || !g_event_commenced))
							{
								g_last_error_code = launchEvent((SC*)&g_last_status_code);
								g_sleepshutdown_seconds = 300;
							}
						}
						else
						{
							g_last_error_code = launchEvent((SC*)&g_last_status_code);
							g_sleepshutdown_seconds = 300;
						}
					}
					else
					{
						g_sleepType = SLEEP_FOREVER;
						if(g_event_enabled) suspendEvent();					
					}
				}
				else if (g_handle_counted_presses == 5)
				{
					g_isMaster = true;
					g_event_commenced = false;
					isMasterCountdownSeconds = 600; /* Remain Master for 10 minutes */
					g_sleepshutdown_seconds = 720;
									
					g_cloningInProgress = false;
					g_programming_countdown = 0;
					g_send_clone_success_countdown = 0;
					LEDS.init();
					g_text_buff.reset();
				}
				else if(g_handle_counted_presses == 7)
				{
					if(!g_device_enabled)
					{
						g_device_enabled = true;
						g_ee_mgr.updateEEPROMVar(Device_Enabled, (void*)&g_device_enabled);
						time_t now = time(null);
						now++;
						while(now >= time(null));
						RSTCTRL_reset();
						while(1); // Should never reach here
					}
				}
			
				g_handle_counted_presses = 0;
			}
			
					
			if(g_start_event)
			{
				g_start_event = false;
							
				if(!g_isMaster)
				{
					g_last_error_code = launchEvent((SC*)&g_last_status_code);
					g_sleepshutdown_seconds = 300;
					if(!g_enable_external_battery_control) setExtBatLoadSwitch(ON, INITIALIZE_LS);
				}
			}
			
			if(g_commence_transmissions)
			{
				g_commence_transmissions = false;
				powerToTransmitter(g_device_enabled); // Needs to be done outside an ISR
				g_event_enabled = true;
				g_event_commenced = true;
			}

		
			if(g_isMaster)
			{
				handleSerialCloning();
			
				if(g_text_buff.empty())
				{
					if(!g_key_down_countdown)
					{
						if(g_send_clone_success_countdown)
						{
							LEDS.sendCode((char*)"X ");
						}
						else
						{
							LEDS.sendCode((char*)"M ");
						}
					}
				}
				else /* Make sure the text buffer is being emptied */
				{
					g_enable_manual_transmissions = true; /* There is only one consumer of g_text_buff so it is always OK to enable manual transmissions */
				}
			
				if(!isMasterCountdownSeconds)
				{
					g_isMaster = false;
					g_sleepshutdown_seconds = 300; /* Ensure sleep occurs */
					g_send_clone_success_countdown = 0;
					g_start_event = eventEnabled(); /* Start any event stored in EEPROM */
					LEDS.init();
				}
			}
			else
			{
				handleSerialBusMsgs();
			
				if(g_cloningInProgress && !g_programming_countdown) // Cloning timed out
				{
					g_cloningInProgress = false;
				}

				if(g_text_buff.empty())
				{
					if(!g_device_enabled)
					{
						LEDS.blink(LEDS_RED_THEN_GREEN_BLINK_SLOW, true);
					}
					else if((g_time_since_wakeup < 60) && (g_hardware_error & ((int)HARDWARE_NO_RTC | (int)HARDWARE_NO_SI5351)))
					{
						if(g_hardware_error & ((int)HARDWARE_NO_RTC))
						{
							LEDS.sendCode((char*)"5CLK");
						}
					
						if(g_hardware_error & ((int)HARDWARE_NO_SI5351))
						{
							LEDS.sendCode((char*)"5XMT");
						}
					}
					else if(g_cloningInProgress || g_key_down_countdown)
					{
						LEDS.blink(LEDS_RED_ON_CONSTANT, true);
					}
					else if(!g_event_commenced)
					{					
						if(g_send_clone_success_countdown)
						{
							LEDS.sendCode((char*)"X ");
						}
						else
						{
							if(noEventWillRun()) /* No event is running now, nor will one run in the future */
							{
								LEDS.blink(LEDS_RED_BLINK_FAST);
							}
							else if(eventScheduled()) /* An event is scheduled to run in the future = OK */
							{
								LEDS.sendCode((char*)"E  ");
							}
							else /* Should not reach here, but if we do, it is an error */
							{
								LEDS.blink(LEDS_RED_BLINK_FAST);
							}
						}
					}	
				}
				else /* Make sure the text buffer is being emptied */
				{
					g_enable_manual_transmissions = true; /* There is only one consumer of g_text_buff so it is always OK to enable manual transmissions */
				}
			}
			
			if(g_device_enabled)
			{
				internal_bat_error = (g_internal_bat_detected && (g_internal_bat_voltage <= g_internal_voltage_low_threshold));
				external_pwr_error = (g_external_voltage <= EXT_BAT_PRESENT_VOLTAGE);
				
				if(g_charge_battery)
				{
					LEDS.blink(LEDS_GREEN_ON_CONSTANT);
				}
				else if(internal_bat_error)
				{
					LEDS.blink(LEDS_GREEN_BLINK_FAST);
				}
				else if(external_pwr_error)
				{
					LEDS.blink(LEDS_GREEN_BLINK_SLOW);
				}
				else
				{
					LEDS.blink(LEDS_GREEN_ON_CONSTANT);
				}
			}		
		
			if(g_long_button_press) /* Shut things down and go to sleep or power off */
			{
				g_long_button_press = false;
				suspendEvent();
				g_go_to_sleep_now = true;
				g_check_for_long_wakeup_press = false;
				g_handle_counted_presses = 0;
				LEDS.blink(LEDS_OFF);
				g_send_clone_success_countdown = 0;
				g_cloningInProgress = false;
				g_programming_msg_throttle = 0;
//				g_sleepType = SLEEP_POWER_OFF_OVERRIDE; /* Uncomment to allow a long keypress to prevent a future event from running */
			}
		
			if(g_i2c_failure_count)
			{
	// 			static time_t t = 0;
	// 			time_t now = time(null);
	// 			
	// 			if(difftime(now, t) > 600) /* Update no more often than every 10 minutes */
	// 			{
	// 				t = now;
	// 				g_ee_mgr.updateEEPROMVar(I2C_failure_count, (void*)&g_i2c_failure_count);
	// 			}
			}
		
			if(g_key_down_countdown || g_reset_after_keydown)
			{
				if(g_reset_after_keydown)
				{
					g_reset_after_keydown = false;
 					LEDS.setRed(OFF);
					setupForFox(INVALID_FOX, START_NOTHING); // Stop any running event
 					keyTransmitter(OFF);
					
					if(g_event_enabled) suspendEvent();
					g_sleepType = eventScheduled() ? SLEEP_UNTIL_START_TIME : SLEEP_FOREVER;
					g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;
					
					if(eventScheduled())
					{
						g_last_error_code = launchEvent((SC*)&g_last_status_code);
						g_sleepshutdown_seconds = 300;
					}
				}
			}
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
		
		g_sleepshutdown_seconds = MAX(300U, g_sleepshutdown_seconds);
		LEDS.blink(LEDS_NO_CHANGE, true);

		SBMessageID msg_id = sb_buff->id;

		switch(msg_id)
		{
			case SB_MESSAGE_RESET:
			{
				RSTCTRL_reset();
			}
			break;
			
			case SB_MESSAGE_DEBUG:
			{
				char c1 = (sb_buff->fields[SB_FIELD1][0]);
				sprintf(g_tempStr, "\nI2C error count = %d\n", g_i2c_failure_count);
				sb_send_string(g_tempStr);
				
				if(c1 == '0')
				{
					g_i2c_failure_count = 0;
					g_ee_mgr.updateEEPROMVar(I2C_failure_count, (void*)&g_i2c_failure_count);
					sb_send_string((char*)"Count reset\n");
				}
			}
			break;
			
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
						else if((c1 == 'F') || (c1 == 'T'))
						{
							c1 = FREQUENCY_TEST_BEACON;
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
						g_sleepshutdown_seconds = 3;
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
					char freqTier = sb_buff->fields[SB_FIELD1][0];
					
					Frequency_Hz f;
					if(g_cloningInProgress)
					{
						if(!frequencyVal(sb_buff->fields[SB_FIELD2], &f))
						{							
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
					else if(!frequencyVal(sb_buff->fields[SB_FIELD2], &f))
					{
						if((freqTier == '1') || (freqTier == 'L'))
						{
							g_frequency_low = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Low, (void*)&f);
						}
						else if((freqTier == '2') || (freqTier == 'M'))
						{
							g_frequency_med = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Med, (void*)&f);
						}
						else if((freqTier == '3') || (freqTier == 'H'))
						{
							g_frequency_hi = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void*)&f);
						}
						else if(freqTier == 'B')
						{
							g_frequency_beacon = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Beacon, (void*)&f);
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
							strncpy(g_tempStr, sb_buff->fields[SB_FIELD1], MAX_PATTERN_TEXT_LENGTH);
						
							if(g_event == EVENT_FOXORING)
							{
								strncpy(g_messages_text[FOXORING_PATTERN_TEXT], g_tempStr, MAX_PATTERN_TEXT_LENGTH);
								g_ee_mgr.updateEEPROMVar(Foxoring_pattern_text, g_messages_text[FOXORING_PATTERN_TEXT]);
							}
// 							else
// 							{
// 								strncpy(g_messages_text[PATTERN_TEXT], g_tempStr, MAX_PATTERN_TEXT_LENGTH);
// 							}
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
				if(g_device_enabled)
				{
					if(sb_buff->fields[SB_FIELD1][0])
					{
						if(sb_buff->fields[SB_FIELD1][0] == '0')    
						{
							g_key_down_countdown = 0;
							g_reset_after_keydown = true;
						}
						else if(sb_buff->fields[SB_FIELD1][0] == '1')  
						{
 							setupForFox(INVALID_FOX, START_NOTHING); // Stop any running event

							if(!txIsInitialized())
							{
								powerToTransmitter(g_device_enabled);
							}
							
							g_key_down_countdown = 9000;
							LEDS.init();
							LEDS.setRed(ON);
							keyTransmitter(ON);

							g_sleepshutdown_seconds = 300; // Shut things down after 5 minutes
						}
					}
				}
				else
				{
					sb_send_string(TEXT_DEVICE_DISABLED_TXT);
				}
			}
			break;

			case SB_MESSAGE_GO:
			{
				if(g_device_enabled)
				{
					if(sb_buff->fields[SB_FIELD1][0])
					{
						if(sb_buff->fields[SB_FIELD1][0] == '0')       /* Stop the event. Re-sync will occur on next start */
						{
 							setupForFox(INVALID_FOX, START_NOTHING); // Stop any running event
						}
						else if(sb_buff->fields[SB_FIELD1][0] == '1')  /* Start the event, re-syncing to a start time of now - same as a button press */
						{
							LEDS.setRed(OFF);
							g_event_enabled = false; /* ensure that it will run immediately */
 							startEventNow();
						}
						else if(sb_buff->fields[SB_FIELD1][0] == '2')  /* Start the event at the programmed start time */
						{
 							g_event_enabled = false;					/* Disable an event currently underway */
 							startEventUsingRTC();
						}
						else if(sb_buff->fields[SB_FIELD1][0] == '3')  /* Start the event immediately with transmissions starting now */
						{
							powerToTransmitter(g_device_enabled);
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
				else
				{
					sb_send_string(TEXT_DEVICE_DISABLED_TXT);
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
					}
				}

				g_ee_mgr.updateEEPROMVar(StationID_text, g_messages_text[STATION_ID]);
				
				if(g_messages_text[STATION_ID][0])
				{
					g_time_needed_for_ID = timeNeededForID();
				}
				
				if(g_cloningInProgress)
				{				
						sb_send_string((char*)"ID\r");
						g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
				else
				{
					reportSettings();
				}
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
								
					g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_event_checksum += c;
					sb_send_string((char*)"EVT\r");
				}
				else
				{
					bool validArg = false;
					
					if(sb_buff->fields[SB_FIELD1][0] == 'F')
					{
						g_event = EVENT_FOXORING;
						validArg = true;
					}
					else if(sb_buff->fields[SB_FIELD1][0] == 'C')
					{
						g_event = EVENT_CLASSIC;
						validArg = true;
					}
					else if(sb_buff->fields[SB_FIELD1][0] == 'S')
					{
						g_event = EVENT_SPRINT;
						validArg = true;
					}
					else if(sb_buff->fields[SB_FIELD1][0] == 'B')
					{
						g_event = EVENT_BLIND_ARDF;
						validArg = true;
					}
					else if(sb_buff->fields[SB_FIELD1][0])
					{
						g_event = EVENT_NONE;
						validArg = true;
					}
					
					if(validArg)
					{
						g_ee_mgr.updateEEPROMVar(Event_setting, (void*)&g_event);
						powerToTransmitter(g_device_enabled);
						setupForFox(getFoxSetting(), START_EVENT_WITH_STARTFINISH_TIMES);
					}
				}
				
				if(!(g_cloningInProgress)) reportSettings();
			}
			break;
			
 			case SB_MESSAGE_FUNCTION:
 			{
				char fun = sb_buff->fields[SB_FIELD1][0];
				
				if(fun)
				{
// 					if((fun == 'A') || (fun == (uint8_t)Function_ARDF_TX) || g_cloningInProgress)
// 					{
// 						g_function = Function_ARDF_TX;
// 						g_ee_mgr.updateEEPROMVar(Function, (void*)&g_function);
// 					}
// 					else if((fun == 'Q') || (fun == (uint8_t)Function_QRP_TX))
// 					{
// 						g_function = Function_QRP_TX;
// 						g_ee_mgr.updateEEPROMVar(Function, (void*)&g_function);
// 					}
// 					else if((fun == 'S') || (fun == (uint8_t)Function_Signal_Gen))
// 					{
// 						g_function = Function_Signal_Gen;
// 						g_ee_mgr.updateEEPROMVar(Function, (void*)&g_function);
//					}

					g_function = Function_ARDF_TX;
					g_ee_mgr.updateEEPROMVar(Function, (void*)&g_function);
				}
				
							 
				if(g_cloningInProgress)
				{
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_event_checksum += 'A';
					sprintf(g_tempStr, "FUN A\r");
					sb_send_string(g_tempStr);
				}
				else
				{
					reportSettings();
				}
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
								g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
								g_event_checksum += t;
								sprintf(g_tempStr, "CLK T %lu\r", t);
								sb_send_string(g_tempStr);
							}
							else
							{
  								/* Start the event if one is configured */
								if(eventScheduled())
								{
									startEventUsingRTC();
								}
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

			case SB_MESSAGE_BATTERY:
			{
				char txt[6];

				if(sb_buff->fields[SB_FIELD1][0] == 'T')
				{
					float v = atof(sb_buff->fields[SB_FIELD1]);

					if((v >= INT_BAT_CHARGE_THRESH_LOW_MIN) && (v <= INT_BAT_CHARGE_THRESH_LOW_MAX))
					{
						g_internal_voltage_low_threshold = v;
						g_ee_mgr.updateEEPROMVar(Voltage_threshold, (void*)&g_internal_voltage_low_threshold);
					}
					else
					{
						int16_t d1, d2;
						uint16_t f1, f2;
						float_to_parts_signed(INT_BAT_CHARGE_THRESH_LOW_MIN, &d1, &f1);
						float_to_parts_signed(INT_BAT_CHARGE_THRESH_LOW_MAX, &d2, &f2);						
  						sprintf(g_tempStr, "\nErr: %d.%u V < thresh < %d.%u V\n", d1, f1, d2, f2);
						sb_send_string(g_tempStr);
					}
				}
				else if(sb_buff->fields[SB_FIELD1][0] == 'X')
				{
					char c = (char)sb_buff->fields[SB_FIELD2][0];
					bool updateStoredValue = false;
					
					if(c == '1')
					{
						setDisableTransmissions(false);
						g_enable_external_battery_control = true;
						updateStoredValue = true;
					}
					else if(c == '2')
					{
						setDisableTransmissions(true);
						g_enable_external_battery_control = true;
						updateStoredValue = true;
					}
					else if(c != '\0')
					{
						setDisableTransmissions(false);
						g_enable_external_battery_control = false;
						updateStoredValue = true;
					}
					
					if(updateStoredValue)
					{
						setExtBatLoadSwitch(OFF, INITIALIZE_LS);
						g_ee_mgr.updateEEPROMVar(Enable_External_Battery_Control, (void*)&g_enable_external_battery_control);
					}
				}
// 				else if(sb_buff->fields[SB_FIELD1][0] == 'B')
// 				{
// 					bool v = ((char)sb_buff->fields[SB_FIELD2][0] == '1');
// 					g_enable_boost_regulator = v;					
// 					g_ee_mgr.updateEEPROMVar(Enable_Boost_Regulator, (void*)&g_enable_boost_regulator);
// 				}
				
// 				sprintf(g_tempStr, "\nBoost: %sabled\n", g_enable_boost_regulator ? "En":"Dis");
// 				sb_send_string(g_tempStr);

				dtostrf(g_internal_bat_voltage, 5, 1, txt);
				txt[5] = '\0';
  				sprintf(g_tempStr, "\nInt. Bat =%s Volts\n", txt);
 				sb_send_string(g_tempStr);

				dtostrf(g_internal_voltage_low_threshold, 5, 1, txt);
				txt[5] = '\0';
 				sprintf(g_tempStr, "thresh   =%s Volts\n", txt);
 				sb_send_string(g_tempStr);
				 
				dtostrf(g_external_voltage, 5, 1, txt);
				txt[5] = '\0';
				sprintf(g_tempStr, "Ext. Bat =%s Volts\n", txt);
				sb_send_string(g_tempStr);
				
				sprintf(g_tempStr, "Ext. Bat. Ctrl = %s\n", g_enable_external_battery_control ? "ON":"OFF");
				sb_send_string(g_tempStr);
				sprintf(g_tempStr, "Transmitter = %s\n", getDisableTransmissions() ? "Disabled":"Enabled");
				sb_send_string(g_tempStr);
			}
			break;
			
			case SB_MESSAGE_VER:
			{
				if(!g_cloningInProgress)
				{
					sb_send_string((char*)PRODUCT_NAME_LONG);
					sprintf(g_tempStr, "\n* SW Ver: %s\n", SW_REVISION);
					sb_send_string(g_tempStr);
				}
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
			
			case SB_CR_NO_DATA:
			{
				if(!g_cloningInProgress)
				{
					reportSettings();
				}
			}
			break;

			default:
			{
				if(!g_cloningInProgress)
				{
					sb_send_string(HELP_TEXT_TXT);
				}
			}
			break;
		}

		sb_buff->id = SB_MESSAGE_EMPTY;
		if(!(g_cloningInProgress) && !suppressResponse)
		{
			sb_send_NewLine();
			sb_send_NewPrompt();
		}
	}
}


/***********************************************************************
 * Private Functions
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
		g_sleepshutdown_seconds = 300;
		return(false); /* completed events are never enabled */
	}
	
	time_t now = time(null);
	int32_t dif = timeDif(now, g_event_start_epoch);

	g_time_to_wake_up = g_event_start_epoch - 15; /* sleep time needs to be calculated to allow time for power-up (coming out of sleep) prior to the event start */
	
	if(dif >= -30)  /* Don't sleep if the event starts in 30 seconds or less, or has already started */
	{
		powerToTransmitter(g_device_enabled);
		g_sleepType = SLEEP_AFTER_EVENT;
		g_sleepshutdown_seconds = 300;
		return( true);
	}

	g_sleepType = SLEEP_UNTIL_START_TIME;
	/* If we reach here, we have an event that will not start for at least 30 seconds. */
	return( true);
}


uint16_t throttleValue(uint8_t speed)
{
	float temp;
	speed = CLAMP(5, (int8_t)speed, 20);
	temp = (3544L / (uint16_t)speed) / 10L; /* tune numerator to achieve "PARIS " sent 8 times in 60 seconds at 8 WPM */
	return( (uint16_t)temp);
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


static int secondsIntoCycle;
static int timeTillTransmit;
static int cyclePeriod;


EC activateEventUsingCurrentSettings(SC* statusCode)
{
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
	
	g_frequency = getFrequencySetting();
	txSetFrequency(&g_frequency, true);

	if(g_fox[g_event] == FREQUENCY_TEST_BEACON)
	{
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;		
		powerToTransmitter(g_device_enabled);
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		g_on_the_air = g_on_air_seconds;
		LEDS.init();
		g_commence_transmissions = true;
	}
	else if(!g_run_event_forever && (g_event_finish_epoch < now))   /* the event has already finished */
	{
		if(statusCode)
		{
			*statusCode = STATUS_CODE_NO_EVENT_TO_RUN;
		}
	}
	else
	{
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		g_code_throttle = throttleValue(getFoxCodeSpeed());
		bool repeat = true;
		makeMorse(getCurrentPatternText(), &repeat, NULL, CALLER_AUTOMATED_EVENT);
		
		if(g_run_event_forever)
		{
//			powerToTransmitter(g_device_enabled); /* Power will be applied when transmissions start */
			g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
			g_on_the_air = g_on_air_seconds;
			g_sendID_seconds_countdown = g_on_air_seconds - g_time_needed_for_ID;
			LEDS.blink(LEDS_RED_OFF);
//			g_event_enabled = true;
			g_commence_transmissions = true;
		}
		else
		{		
			int32_t dif = timeDif(now, g_event_start_epoch); /* returns arg1 - arg2 */

			if(dif >= 0)                                    /* start time is in the past */
			{
				bool turnOnTransmitter = false;
//				int cyclePeriod;
				cyclePeriod = g_on_air_seconds + g_off_air_seconds;
//				int secondsIntoCycle;
				secondsIntoCycle = dif % cyclePeriod;
//				int timeTillTransmit;
				timeTillTransmit = g_intra_cycle_delay_time - secondsIntoCycle;

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

				powerToTransmitter(turnOnTransmitter);

				g_commence_transmissions = true;
				LEDS.init();
			}
			else    /* start time is in the future */
			{
				if(statusCode)
				{
					*statusCode = STATUS_CODE_WAITING_FOR_EVENT_START;
				}
				keyTransmitter(OFF);
				powerToTransmitter(OFF);
			}
		}
	}

	return( ERROR_CODE_NO_ERROR);
}


void suspendEvent()
{
	g_event_enabled = false;    /* get things stopped immediately */
	g_on_the_air = 0;           /* stop transmitting */
	g_event_commenced = false;  /* get things stopped immediately */
	g_run_event_forever = false;
	g_sleepshutdown_seconds = 300;
	configRedLEDforEvent();
	powerToTransmitter(OFF);
}

void startEventNow()
{
	ConfigurationState_t conf = clockConfigurationCheck();
	
	if(conf != CONFIGURATION_ERROR)
	{
		setupForFox(INVALID_FOX, START_EVENT_NOW_AND_RUN_FOREVER);                                                                  /* Let the RTC start the event */
	}
	
	configRedLEDforEvent();
}


bool startEventUsingRTC(void)
{
	bool err = false;
	ConfigurationState_t conf = clockConfigurationCheck();

	if(conf != CONFIGURATION_ERROR)
	{
		setupForFox(INVALID_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
		if(eventScheduledForTheFuture())
		{
			powerToTransmitter(OFF);
		}
	}
	else
	{
		err = true;
		reportConfigErrors();
	}

	configRedLEDforEvent();
	
	return err;
}


void configRedLEDforEvent(void)
{		
	if(noEventWillRun())
	{
		if(!g_run_event_forever) LEDS.blink(LEDS_RED_BLINK_FAST, true);
	}
	else
	{
		LEDS.blink(LEDS_RED_OFF);
		
		time_t now = time(null);
	
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
}


void setupForFox(Fox_t fox, EventAction_t action)
{
	bool delayNotSet = true;
	
	g_run_event_forever = false;
	g_sleepshutdown_seconds = 300;
	
	if(fox == INVALID_FOX)
	{
		fox = getFoxSetting();
	}

	g_event_enabled = false;
	g_event_commenced = false;

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
			
// 			init_transmitter(getFrequencySetting());
			g_ID_period_seconds = 60;
			g_sendID_seconds_countdown = g_ID_period_seconds;	/* wait 1 minute to send the ID */
			g_on_air_seconds = 60;								/* on period is one minute */
			g_off_air_seconds = 240;							/* off period = 4 minutes */
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
			if(delayNotSet)
			{
				g_intra_cycle_delay_time = 48;
			}

// 			init_transmitter(getFrequencySetting());
			g_ID_period_seconds = 600;
			g_sendID_seconds_countdown = 600;			/* wait 10 minutes send the ID */
			g_on_air_seconds = 12;						/* on period is 12 seconds */
			g_off_air_seconds = 48;						/* off period = 48 seconds */
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

// 			init_transmitter(getFrequencySetting());
			g_ID_period_seconds = 600;
			g_sendID_seconds_countdown = g_ID_period_seconds;	/* wait 10 minutes send the ID */
			g_on_air_seconds = 12;								/* on period is 12 seconds */
			g_off_air_seconds = 48;								/* off period = 48 seconds */
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
// 			init_transmitter(getFrequencySetting());
			g_intra_cycle_delay_time = 0;
			g_ID_period_seconds = 600;
			g_sendID_seconds_countdown = g_ID_period_seconds;	/* wait 10 minutes send the ID */
			g_on_air_seconds = 600;								/* on period is 10 minutes */
			g_off_air_seconds = 0;								/* off period is very short */
		}
		break;
		
		case FREQUENCY_TEST_BEACON:
		{
// 			init_transmitter(getFrequencySetting());
			g_intra_cycle_delay_time = 0;
			g_ID_period_seconds = 600;
			g_sendID_seconds_countdown = g_ID_period_seconds;	/* wait 10 minutes send the ID */
			g_on_air_seconds = 600;								/* on period 10 minutes */
			g_off_air_seconds = 0;								/* off period is very short */
			g_sleepshutdown_seconds = 300;
		}
		break;
		
		case SPECTATOR:
		case BEACON:
		default:
		{
// 			init_transmitter(getFrequencySetting());
			g_intra_cycle_delay_time = 0;
			g_ID_period_seconds = 600;
			g_sendID_seconds_countdown = g_ID_period_seconds;	/* wait 10 minutes send the ID */
			g_on_air_seconds = 600;								/* on period is 10 minutes */
			g_off_air_seconds = 0;								/* off period is very short */
		}
		break;
	}
	
	if(action == START_NOTHING)
	{
		g_event_commenced = false;                   /* do not get things running yet */
		g_event_enabled = false;                     /* do not get things running yet */
//		keyTransmitter(OFF);
		powerToTransmitter(OFF);
	}
	else if(action == START_EVENT_NOW_AND_RUN_FOREVER)
	{
		powerToTransmitter(g_device_enabled);
		makeMorse(getCurrentPatternText(), NULL, NULL, CALLER_AUTOMATED_EVENT);
		g_run_event_forever = true;
		g_sleepshutdown_seconds = 300;
		g_start_event = true;
	}
	else if(action == START_TRANSMISSIONS_NOW)                                  /* Immediately start transmitting, regardless RTC or time slot */
	{
		powerToTransmitter(g_device_enabled);
		makeMorse(getCurrentPatternText(), NULL, NULL, CALLER_AUTOMATED_EVENT);
		g_run_event_forever = true;
		g_on_the_air = g_on_air_seconds;			/* start out transmitting */
		g_commence_transmissions = true;                   /* get things running immediately */
//		g_event_enabled = true;                     /* get things running immediately */
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		LEDS.blink(LEDS_RED_OFF);
	}
	else         /* if(action == START_EVENT_WITH_STARTFINISH_TIMES) */
	{
		g_run_event_forever = false;
		g_event_commenced = false;	/* do not get things running yet */
		g_event_enabled = false;	/* do not get things running yet */
		g_start_event = true;
		LEDS.blink(LEDS_RED_OFF);
	}
}

/*
 * Function: validateTimeString
 * ----------------------------
 * This function validates a given time string and converts it to an epoch value if valid.
 * It ensures that the provided time string represents a valid future time.
 *
 * Parameters:
 *  - str: A pointer to a string representing the time to validate.
 *  - epochVar: A pointer to the time variable to compare against for validation.
 *
 * Return Value:
 *  - Returns the validated time as an epoch value (`time_t`) if valid, otherwise returns 0.
 */
time_t validateTimeString(char* str, time_t* epochVar)
{
    time_t valid = 0;    // Initialize return value to 0 (indicating invalid by default).
    int len = strlen(str);    // Get the length of the provided string.
    time_t minimumEpoch = MINIMUM_VALID_EPOCH;    // Set the initial minimum valid epoch.
    uint8_t validationType = 0;    // Set the initial validation type to 0.
    time_t now = time(NULL);    // Get the current system time.

    // Determine the minimum epoch and validation type based on the `epochVar`.
    if(epochVar == &g_event_start_epoch)
    {
        // For event start, ensure the minimum epoch is either now or the pre-set minimum valid epoch.
        minimumEpoch = MAX(now, MINIMUM_VALID_EPOCH);
        validationType = 1;    // Indicates start time validation.
    }
    else if(epochVar == &g_event_finish_epoch)
    {
        // For event finish, ensure the minimum epoch is the event start time or the current time.
        minimumEpoch = MAX(g_event_start_epoch, now);
        validationType = 2;    // Indicates finish time validation.
    }

    // If the string length is 10, pad it to a length of 12 by adding "00" for seconds.
    if(len == 10)
    {
        str[10] = '0';
        str[11] = '0';
        str[12] = '\0';
        len = 12;    // Update the length to 12.
    }

    // If the string length is 12 and contains only digits, proceed to validation.
    if((len == 12) && (only_digits(str)))
    {
        // Convert the time string to epoch value (`YYMMDDhhmmss` format).
        time_t ep = String2Epoch(NULL, str);

        // Validate if the calculated epoch is greater than the minimum allowed.
        if(ep > minimumEpoch)
        {
            valid = ep;    // Set the valid time to the calculated epoch.
        }
        else
        {
            // Report appropriate error messages based on the validation type.
            if(validationType == 1)    // Start time validation
            {
                sb_send_string(TEXT_ERR_START_IN_PAST_TXT);    // Start time is in the past.
            }
            else if(validationType == 2)    // Finish time validation
            {
                if(ep < time(NULL))
                {
                    sb_send_string(TEXT_ERR_FINISH_IN_PAST_TXT);    // Finish time is in the past.
                }
                else
                {
                    sb_send_string(TEXT_ERR_FINISH_BEFORE_START_TXT);    // Finish time is before start time.
                }
            }
            else    // Current time validation
            {
                sb_send_string(TEXT_ERR_TIME_IN_PAST_TXT);    // Time is in the past.
            }
        }
    }
    else if(len)    // If the length is non-zero and not 12, it's an invalid time string.
    {
        sb_send_string(TEXT_ERR_INVALID_TIME_TXT);    // Report invalid time string error.
    }

    // Return the validated epoch value (or 0 if validation failed).
    return(valid);
}

/*
 * Function: reportTimeTill
 * ------------------------
 * This function calculates the time difference between two time points (`from` and `until`) and reports it.
 * It also prints a failure message if the `from` time is greater than or equal to the `until` time.
 *
 * Parameters:
 *  - from: The starting time (`time_t` format).
 *  - until: The ending time (`time_t` format).
 *  - prefix: A string that is printed before the time difference if the time difference is positive.
 *  - failMsg: A message to print if `from` is greater than or equal to `until`.
 *
 * Return Value:
 *  - A boolean indicating whether the reporting failed (`true` if failed, `false` otherwise).
 */
bool reportTimeTill(time_t from, time_t until, const char* prefix, const char* failMsg)
{
    bool failure = false;

    // Check if the `from` time is greater than or equal to the `until` time.
    if(from >= until)   // Negative time, failure condition
    {
        failure = true; // Mark as failure

        // Print the failure message if provided.
        if(failMsg)
        {
            sb_send_string((char*)failMsg);
        }
    }
    else
    {
        // Print the prefix message if provided.
        if(prefix)
        {
            sb_send_string((char*)prefix);
        }

        // Calculate the time difference.
        time_t dif = until - from;

        // Extract years from the time difference.
        uint16_t years = dif / YEAR;
        time_t hold = dif - (years * YEAR);

        // Extract days from the remaining time.
        uint16_t days = hold / DAY;
        hold -= (days * DAY);

        // Extract hours from the remaining time.
        uint16_t hours = hold / HOUR;
        hold -= (hours * HOUR);

        // Extract minutes from the remaining time.
        uint16_t minutes = hold / MINUTE;

        // Extract seconds from the remaining time.
        uint16_t seconds = hold - (minutes * MINUTE);

        // Initialize the temporary string.
        g_tempStr[0] = '\0';

        // Report years if non-zero.
        if(years)
        {
            sprintf(g_tempStr, "%d yrs ", years);
            sb_send_string(g_tempStr);
        }

        // Report days if non-zero.
        if(days)
        {
            sprintf(g_tempStr, "%d days ", days);
            sb_send_string(g_tempStr);
        }

        // Report hours if non-zero.
        if(hours)
        {
            sprintf(g_tempStr, "%d hrs ", hours);
            sb_send_string(g_tempStr);
        }

        // Report minutes if non-zero.
        if(minutes)
        {
            sprintf(g_tempStr, "%d min ", minutes);
            sb_send_string(g_tempStr);
        }

        // Always report seconds.
        sprintf(g_tempStr, "%d sec", seconds);
        sb_send_string(g_tempStr);

        // Add a newline after reporting the time.
        sb_send_NewLine();

        // Clear the temporary string.
        g_tempStr[0] = '\0';
    }

    // Return whether there was a failure.
    return failure;
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
		else if(eventScheduled() && (!g_event_enabled && !g_start_event))
		{
			sb_send_string((char*)"Start with > GO 2\n");
		}
		else
		{
			sb_send_string((char*)"None: Event running.\n");
		}
	}
}

/*
 * Function: reportSettings
 * ------------------------
 * This function generates a report of the current settings and sends it through
 * the serial interface. It includes information about the software version, hardware errors,
 * current event and fox settings, frequency settings, and other configuration details.
 * Parameters:
 *  None.
 * Return Value:
 *  None.
 */
void reportSettings(void)
{
	// If cloning is currently in progress, do not proceed with reporting settings.
	if(g_cloningInProgress) return;

	// Buffer for storing temporary strings.
	char buf[50];

	// Get the current time.
	time_t now = time(NULL);

	// Send the product name.
	sb_send_string((char*)PRODUCT_NAME_LONG);

	// Report the software version.
	sprintf(g_tempStr, "\n* SW Ver: %s\n", SW_REVISION);
	sb_send_string(g_tempStr);

	// Check for hardware errors and report them.
	if(g_hardware_error & (int)HARDWARE_NO_RTC)
	{
		sb_send_string(TEXT_RTC_NOT_RESPONDING_TXT);  // RTC not responding.
	}
	
	if(g_hardware_error & (int)HARDWARE_NO_SI5351)
	{
		sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);  // Transmitter not responding.
	}
	
	if(g_temperature_shutdown)
	{
		sb_send_string(TEXT_EXCESSIVE_TEMPERATURE);
	}
	
	if(isValidTemp(g_processor_temperature))
	{
		int16_t integer;
		uint16_t fractional;
		
		if(!float_to_parts_signed(g_processor_temperature, &integer, &fractional))
		{
			sprintf(g_tempStr, "\n*   Cur Temp: %d.%dC\n", integer, fractional);
			sb_send_string(g_tempStr);
		}
	}
	
	if(isValidTemp(g_processor_max_temperature))
	{
		int16_t integer;
		uint16_t fractional;
		
		if(!float_to_parts_signed(g_processor_max_temperature, &integer, &fractional))
		{
			sprintf(g_tempStr, "*   Max Temp: %d.%dC\n", integer, fractional);
			sb_send_string(g_tempStr);
		}
	}
	
	if(isValidTemp(g_processor_min_temperature))
	{
		int16_t integer;
		uint16_t fractional;
		
		if(!float_to_parts_signed(g_processor_min_temperature, &integer, &fractional))
		{
			sprintf(g_tempStr, "*   Min Temp: %d.%dC\n", integer, fractional);
			sb_send_string(g_tempStr);
		}
	}

	// Get and report the current functionality.
	if(!function2Text(g_tempStr, g_function))
	{
		strncpy(buf, g_tempStr, TEMP_STRING_SIZE);
		sprintf(g_tempStr, "\n*   Function: %s\n", buf);
		sb_send_string(g_tempStr);
	}

	// Print the current settings header.
	sb_send_string(TEXT_CURRENT_SETTINGS_TXT);

	// Report the current system time.
	sprintf(g_tempStr, "*   Time: %s\n", convertEpochToTimeString(now, buf, TEMP_STRING_SIZE));
	sb_send_string(g_tempStr);

	// Get and report the current event.
	if(!event2Text(g_tempStr, g_event))
	{
		strncpy(buf, g_tempStr, TEMP_STRING_SIZE);
		sprintf(g_tempStr, "*   Event: %s\n", buf);
	}
	else
	{
		sprintf(g_tempStr, "*   Event: None Set\n");
	}
	sb_send_string(g_tempStr);

	// Get and report the current fox setting.
	Fox_t f = getFoxSetting();
	if(!fox2Text(g_tempStr, f))
	{
		strncpy(buf, g_tempStr, TEMP_STRING_SIZE);
		sprintf(g_tempStr, "*   Fox: %s\n", buf);
	}
	else
	{
		sprintf(g_tempStr, "*   Fox: %u\n", (uint16_t)f);
	}
	sb_send_string(g_tempStr);

	// Report the callsign if it is set, otherwise indicate it is not set.
	if(g_messages_text[STATION_ID][0])
	{
		sprintf(g_tempStr, "*   Callsign: %s\n", g_messages_text[STATION_ID]);
		sb_send_string(g_tempStr);
	}
	else
	{
		sb_send_string((char*)"*   Callsign: None\n");
	}

	// Report the speed for the callsign in words per minute.
	sprintf(g_tempStr, "*   Callsign WPM: %d\n", g_id_codespeed);
	sb_send_string(g_tempStr);

	// Report the transmit pattern and its speed.
	sprintf(g_tempStr, "*   Xmit Pattern: %s\n", getCurrentPatternText());
	sb_send_string(g_tempStr);
	sprintf(g_tempStr, "*   Xmit Pattern WPM: %u\n", getFoxCodeSpeed());
	sb_send_string(g_tempStr);

	// Get and report the transmitter frequency.
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

	// Report the RTC calibration value.
	sprintf(g_tempStr, "*   Cal: %d\n", RTC_get_cal());
	sb_send_string(g_tempStr);

	// If the event runs for more than 1 day, report the number of days remaining.
	if(g_days_to_run > 1)
	{
		uint8_t days_remaining = g_days_to_run - g_days_run;
		if(days_remaining)
		{
			sprintf(g_tempStr, "\n*   == Runs for %d days ==\n", days_remaining);
			sb_send_string(g_tempStr);
		}
	}

	// Report the start and finish times of the event.
	sprintf(g_tempStr, "*   Start:  %s\n", convertEpochToTimeString(g_event_start_epoch, buf, TEMP_STRING_SIZE));
	sb_send_string(g_tempStr);
	sprintf(g_tempStr, "*   Finish: %s\n", convertEpochToTimeString(g_event_finish_epoch, buf, TEMP_STRING_SIZE));
	sb_send_string(g_tempStr);

	// If an event is active, report event-specific frequency settings.
	if(g_event != EVENT_NONE)
	{
		sb_send_string(TEXT_EVENT_SETTINGS_TXT);

		if(frequencyString(buf, g_frequency_low))
		{
			sprintf(g_tempStr, "*   [FRE 1] Freq Low: Error\n");
		}
		else
		{
			sprintf(g_tempStr, "*   [FRE 1] Freq Low: %s\n", buf);
			sb_send_string(g_tempStr);
		}
		
		if(g_event != EVENT_CLASSIC)
		{
			if(frequencyString(buf, g_frequency_med))
			{
				sprintf(g_tempStr, "*   [FRE 2] Error\n");
			}
			else
			{
				sprintf(g_tempStr, "*   [FRE 2] Freq Med: %s\n", buf);
				sb_send_string(g_tempStr);
			}

			if(frequencyString(buf, g_frequency_hi))
			{
				sprintf(g_tempStr, "*   [FRE 3] Error\n");
			}
			else
			{
				sprintf(g_tempStr, "*   [FRE 3] Freq High: %s\n", buf);
				sb_send_string(g_tempStr);
			}
		}

		if(frequencyString(buf, g_frequency_beacon))
		{
			sprintf(g_tempStr, "*   [FRE B] Error\n");
		}
		else
		{
			sprintf(g_tempStr, "*   [FRE B] Beacon Freq: %s\n", buf);
			sb_send_string(g_tempStr);
		}
	}

	// Check the clock configuration state and report necessary actions.
	ConfigurationState_t cfg = clockConfigurationCheck();
	if((cfg != WAITING_FOR_START) && (cfg != EVENT_IN_PROGRESS) && (cfg != SCHEDULED_EVENT_WILL_NEVER_RUN))
	{
		sb_send_string((char*)"\n* Needed Actions:\n");
		reportConfigErrors();
	}
	else
	{
		// Report times for event start, duration, and remaining time.
		reportTimeTill(now, g_event_start_epoch, "\n*   Starts in: ", "\n*   In progress\n");
		reportTimeTill(g_event_start_epoch, g_event_finish_epoch, "*   Lasts: ", NULL);
		if(g_event_start_epoch < now)
		{
			reportTimeTill(now, g_event_finish_epoch, "*   Time Remaining: ", NULL);
		}

		// If the event is disabled, provide instructions to start.
		if(eventScheduled() && !g_event_enabled && !g_start_event)
		{
			sb_send_string((char*)"\n* Needed Action:\n");
			sb_send_string((char*)"\n*  Start with > GO 2\n");
		}
	}

	// If the device is disabled, say so and provide instructions to enable.
	if(!g_device_enabled)
	{
		sprintf(g_tempStr, "\n* Device disabled!");
		sb_send_string(g_tempStr);
		sprintf(g_tempStr, "\n* Press button seven (7) times to enable.");
		sb_send_string(g_tempStr);
	}
	else
	{
		// Warn if transmitter disabled by BAT X command
		if(getDisableTransmissions())
		{
			sprintf(g_tempStr, "\n* WARNING: TRANSMIT DISABLED");
			sb_send_string(g_tempStr);
			sprintf(g_tempStr, "\n* Re-enable with > BAT X 1\n");
			sb_send_string(g_tempStr);
		}
	}
}

/*
 * Function: bcd2dec
 * -----------------
 * This function converts a Binary-Coded Decimal (BCD) value to a decimal value.
 * In BCD, each digit is represented by a 4-bit binary value. This function extracts
 * the tens and units portions of the BCD value and calculates the equivalent decimal value.

 * Parameters:
 *  - val: A uint8_t value representing a number in BCD format.

 * Return:
 *  - A uint8_t value representing the equivalent decimal value.

 * Example:
 *  If val = 0x25 (BCD for 25), the function will return 25 in decimal format.
 *
 * Conversion Steps:
 *  - The tens digit is extracted by shifting the upper 4 bits to the right (val >> 4).
 *  - The units digit is extracted by taking the lower 4 bits (val & 0x0F).
 *  - The final decimal result is calculated as: (10 * tens) + units.
 */
uint8_t bcd2dec(uint8_t val)
{
	uint8_t result = 10 * (val >> 4) + (val & 0x0F);
	return( result);
}

/*
 * Converts a value from decimal to Binary Coded Decimal (BCD)
 * @param val - the decimal value to convert (0-99)
 * @return the value converted to BCD
 */
uint8_t dec2bcd(uint8_t val)
{
	uint8_t result = val % 10;
	result |= (val / 10) << 4;
	return (result);
}

/*
 * Converts a character array to BCD format
 * @param c - a character array representing a number
 * @return the value converted to BCD
 */
uint8_t char2bcd(char c[])
{
	uint8_t result = (c[1] - '0') + ((c[0] - '0') << 4);
	return( result);
}

/*
 * Converts a tm struct to an epoch time (seconds since 1970)
 * @param ltm - a pointer to a tm struct with time information
 * @return epoch value representing the given local time
 */
time_t epoch_from_ltm(tm *ltm)
{
	time_t epoch = ltm->tm_sec + ltm->tm_min * 60 + ltm->tm_hour * 3600L + ltm->tm_yday * 86400L +
	(ltm->tm_year - 70) * 31536000L + ((ltm->tm_year - 69) / 4) * 86400L -
	((ltm->tm_year - 1) / 100) * 86400L + ((ltm->tm_year + 299) / 400) * 86400L;

	return(epoch);
}


/*
 * Converts an epoch time (seconds since 1900) to a human-readable string with format "ddd dd-mon-yyyy hh:mm:ss zzz"
 * @param epoch - the epoch time to convert
 * @param buf - a buffer to store the resulting string
 * @param size - size of the buffer
 * @return pointer to the formatted time string
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



/*
 * Converts a datetime string into an epoch value
 * @param error - pointer to an error flag, set to 1 if an error occurs
 * @param datetime - character string in the format "YYMMDDhhmmss"
 * @return epoch value representing the given date and time, or current RTC time if datetime is null
 */
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

/*
 * Function: getCurrentPatternText
 * -------------------------------
 * This function returns a string containing the current Morse pattern based on the value
 * of the global variable g_fox[g_event]. The pattern is determined by a switch statement
 * that checks the specific value of g_fox[g_event] and assigns the corresponding text.

 * Return:
 *  A pointer to a string representing the pattern text, based on the value of g_fox[g_event].
 *  Possible return values include:
 *  - "MOE", "MOI", "MOS", "MOH", "MO5": Specific patterns for FOX_1 to FOX_5.
 *  - "S": Represents a spectator state.
 *  - "ME", "MI", "MS", "MH", "M5": Patterns for Sprint transmitters S1 to S5.
 *  - "OE", "OI", "OS", "OH", "O5": Patterns for Sprint finishers F1 to F5.
 *  - g_messages_text[FOXORING_PATTERN_TEXT]: Pattern for FOXORING states FOXORING_FOX1, FOXORING_FOX2, and FOXORING_FOX3.
 *  - "<": Indicates a frequency test beacon.
 *  - "MO": Represents the Beacon state.
 *  - g_messages_text[PATTERN_TEXT]: Default pattern text for any unhandled cases.

 * Parameters:
 *  None.

 * Return Value:
 *  A pointer to a constant character string indicating the current pattern.
 */
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
		
		case FREQUENCY_TEST_BEACON:
		{
			if(g_frequency_to_test == 0)
			{
				c = (char*)"< E<";
			}
			else if(g_frequency_to_test == 1)
			{
				c = (char*)"< EE<";
			}
			else if(g_frequency_to_test == 2)
			{
				c = (char*)"< EEE<";
			}
			else // if(g_frequency_to_test == 3)
			{
				c = (char*)"< EEEE<";
			}
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

/*
 * Retrieves the appropriate frequency setting based on the current event type.
 * This function selects the frequency for different event categories such as BEACON, FOX, SPECTATOR, and SPRINT.
 * @return Frequency_Hz - the frequency setting for the given event
 */
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
		
		case FREQUENCY_TEST_BEACON:
		{
			freq = g_frequency_low;
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


/*
 * Handles the process of serial cloning between master and slave devices.
 * This function manages the entire communication and synchronization process, including
 * initiating cloning, handling replies, synchronizing event data, and ensuring proper state transitions.
 * It uses various states to handle different stages of communication, waiting for appropriate replies
 * from the slave and adjusting timings and configurations as needed.
 */

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
	
	if(!g_programming_msg_throttle)
	{
		if(!g_cloningInProgress)
		{
			sb_send_master_string((char*)"MAS P\r"); /* Set slave to active cloning state */
			g_programming_msg_throttle = 600;
			g_programming_state = SYNC_Searching_for_slave;
		}
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
					g_event_checksum = 0;
					sprintf(g_tempStr, "FUN A\r"); /* Set slave to radio orienteering function */
					sb_send_master_string(g_tempStr);
					g_programming_state = SYNC_Waiting_for_FUN_A_reply;
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}			
		}
		break;
		
		
		case SYNC_Waiting_for_FUN_A_reply:
		{
			if(sb_buff)
			{
				msg_id = sb_buff->id;
				if(msg_id == SB_MESSAGE_FUNCTION)
				{
					g_event_checksum += 'A';
					g_seconds_transition = false;
					g_programming_state = SYNC_Align_to_Second_Transition;
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
				}
			}
		}
		break;
		
		
		case SYNC_Align_to_Second_Transition:
		{
			if(g_seconds_transition)
			{
				time_t now = time(null);
				g_event_checksum += now;
				sprintf(g_tempStr, "CLK T %lu\r", now); /* Set slave's RTC */
				sb_send_master_string(g_tempStr);
				g_programming_state = SYNC_Waiting_for_CLK_T_reply;
				g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
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
						g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
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
						g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
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
						g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
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
							strncpy(g_tempStr, "ID \"\"\r", TEMP_STRING_SIZE);
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
						sprintf(g_tempStr, "SPD P %u\r", g_pattern_codespeed);
						
						g_event_checksum += g_pattern_codespeed;
						sb_send_master_string(g_tempStr);
						g_programming_state = SYNC_Waiting_for_Pattern_CodeSpeed_reply;
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
						g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
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
					g_programming_msg_throttle = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
					g_programming_countdown = PROGRAMMING_MESSAGE_TIMEOUT_PERIOD;
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
						g_programming_msg_throttle = 0;
						g_cloningInProgress = false;
					}
					
					isMasterCountdownSeconds = 600; /* Extend Master time */
					g_programming_state = SYNC_Searching_for_slave;
				}
			}
		}
		break;
	}
	
	if(sb_buff) sb_buff->id = SB_MESSAGE_EMPTY;

}

/*
 * Checks if any event will run
 * @return true if no event is scheduled or event is disabled or sleep mode is a permanent variety
 */
bool noEventWillRun(void)
{
	bool result;
	
	result = !eventScheduled() || !g_event_enabled || (g_sleepType == SLEEP_FOREVER);
	
	return result;
}

/*
 * Checks if an event is scheduled to run at the current time
 * @return true if an event is scheduled for the current time
 */
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

/*
 * Checks if an event is scheduled to occur in the future
 * @return true if an event is scheduled for a future time
 */
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

/*
 * Determines whether an event is scheduled based on the current time, future event time, and the number of days remaining
 * @return true if an event is scheduled
 */
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

