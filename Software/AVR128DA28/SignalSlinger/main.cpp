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
#include "globals.h"
#include "shared_state.h"
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
	AWAKENED_BY_BUTTONPRESS,
	AWAKENED_BY_SERIAL_PORT
} Awakened_t;

typedef enum
{
	HARDWARE_OK,
	HARDWARE_NO_RTC = 0x01,
	HARDWARE_NO_SI5351 = 0x02
} HardwareError_t;

typedef enum
{
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

typedef struct
{
	time_t event_start_epoch;
	time_t event_finish_epoch;
	uint8_t days_to_run;
} CloneTimingSnapshot_t;

#define PROGRAMMING_MESSAGE_TIMEOUT_PERIOD 3000

/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
#define TEMP_STRING_SIZE 50
static char g_tempStr[TEMP_STRING_SIZE + 1] = {'\0'};
static volatile EC g_last_error_code = ERROR_CODE_NO_ERROR;
static volatile SC g_last_status_code = STATUS_CODE_IDLE;

volatile bool g_device_enabled = false;

static volatile bool g_foreground_start_event = false;
static volatile bool g_foreground_enable_transmitter = false;
static volatile int g_hardware_error = (int)HARDWARE_OK;
static volatile bool g_defer_cloned_event_start = false;

/* Variables related to the "Event Engine" which consists of two ISRs and the foreground working
together to handle event timing. Communication between ISRs and forground is most efficiently
handled using volatile global variables. */
static volatile int32_t g_evteng_on_the_air = 0;
static volatile int g_evteng_sendID_seconds_countdown = 0;
static volatile uint16_t g_evteng_code_throttle = 50;
static volatile uint16_t g_evteng_sleepshutdown_seconds = 300;
volatile uint8_t g_evteng_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
volatile uint8_t g_evteng_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
volatile int16_t g_evteng_on_air_seconds = EEPROM_ON_AIR_TIME_DEFAULT;   /* amount of time to spend on the air */
volatile int16_t g_evteng_off_air_seconds = EEPROM_OFF_AIR_TIME_DEFAULT; /* amount of time to wait before returning to the air */
static inline void atomic_set_system_time(time_t epoch, bool reset_rtc)
{
	ENTER_CRITICAL(main_set_system_time);
	if(reset_rtc)
	{
		RTC_zero();
	}
	set_system_time(epoch);
	EXIT_CRITICAL(main_set_system_time);
}
volatile int16_t g_evteng_intra_cycle_delay_time = EEPROM_INTRA_CYCLE_DELAY_TIME_DEFAULT; /* offset time into a repeating transmit cycle */
volatile int16_t g_evteng_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;            /* amount of time between ID/callsign transmissions */
volatile time_t g_event_start_epoch = EEPROM_START_TIME_DEFAULT;
volatile time_t g_evteng_loaded_start_epoch = 0;
volatile time_t g_event_finish_epoch = EEPROM_FINISH_TIME_DEFAULT;
volatile time_t g_evteng_loaded_finish_epoch = 0;
volatile bool g_evteng_event_enabled = EEPROM_EVENT_ENABLED_DEFAULT; /* indicates that the conditions for executing the event are set */
volatile bool g_evteng_event_commenced = false;
volatile bool g_evteng_sending_station_ID = false; /* Allows a small extension of transmissions to ensure the ID is fully sent */
static volatile bool g_evteng_run_event_until_canceled = false;
volatile bool g_evteng_initialize_event = false;

/* Various other functions handled in ISRs are most efficiently handled (with care) by volatile
globals. */
volatile float g_internal_voltage_low_threshold = EEPROM_INT_BATTERY_LOW_THRESHOLD_V;
volatile float g_internal_bat_voltage = 0.;
volatile bool g_internal_bat_detected = false;
volatile float g_external_voltage = 0.;
volatile float g_processor_temperature = MINIMUM_VALID_TEMP - 1.;
volatile float g_processor_min_temperature = MAXIMUM_VALID_TEMP + 1.;
volatile float g_processor_max_temperature = MINIMUM_VALID_TEMP - 1.;
volatile bool g_restart_conversions = false;
volatile bool g_seconds_transition = false;
volatile bool g_muteAfterID = false; /* Inhibit any transmissions after the ID has been sent */
volatile uint32_t g_event_checksum = 0;
volatile uint8_t g_days_to_run = 1;
volatile uint8_t g_days_run = 0;
volatile Function_t g_function = Function_ARDF_TX;
volatile uint8_t g_foxoring_pattern_codespeed = EEPROM_FOXORING_PATTERN_CODESPEED_DEFAULT;
volatile uint16_t g_time_needed_for_ID = 0;
char g_messages_text[STATION_ID + 1][MAX_PATTERN_TEXT_LENGTH + 2];

static volatile bool g_go_to_sleep_now = false;
static volatile bool g_sleeping = false;
static volatile time_t g_time_to_wake_up = 0;
static volatile Awakened_t g_awakenedBy = POWER_UP_START;
static volatile SleepType g_sleepType = SLEEP_FOREVER;
static volatile uint8_t g_button_wake_prior_sleep_type = SLEEP_FOREVER;
static volatile bool g_button_wake_prior_event_enabled = false;
static volatile bool g_button_wake_prior_event_commenced = false;
static volatile time_t g_seconds_since_wakeup = 0;
static volatile bool g_foreground_check_for_long_wakeup_press = true;
static volatile bool g_device_wakeup_complete = false;
static volatile bool g_foreground_enable_serialbus = false;
static volatile bool g_charge_battery = false;
static volatile bool g_turn_on_fan = false;
static volatile bool g_foreground_report_settings = false;
static volatile uint16_t g_report_settings_countdown = 0;
static bool g_event_launched_by_user_action = false;
static bool g_start_event_after_keydown = false; /* Foreground-only: one-press keydown should launch a synced event afterward */

#define NUMBER_OF_POLLED_ADC_CHANNELS 3
static ADC_Active_Channel_t g_adcChannelOrder[NUMBER_OF_POLLED_ADC_CHANNELS] = {ADCInternalBatteryVoltage, ADCExternalBatteryVoltage, ADCTemperature};
static const uint16_t g_adcChannelConversionPeriod_ticks[NUMBER_OF_POLLED_ADC_CHANNELS] = {TIMER2_0_5HZ, TIMER2_0_5HZ, TIMER2_0_5HZ};
static volatile uint16_t g_adcCountdownCount[NUMBER_OF_POLLED_ADC_CHANNELS] = {2000, 2000, 4000};
static volatile uint16_t g_lastConversionResult[NUMBER_OF_POLLED_ADC_CHANNELS] = {0, 0, 0};
static volatile bool g_thermal_shutdown = false;

volatile uint16_t g_foreground_handle_counted_presses = 0;
volatile uint16_t g_switch_presses_count = 0;
volatile bool g_long_button_press = false;
volatile uint16_t g_button_hold_countdown;

#define NUMBER_OF_TEST_FREQUENCIES (4)
uint8_t g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;

static volatile uint16_t g_programming_countdown = 0;
static volatile uint16_t g_programming_msg_throttle = 0;
static volatile uint16_t g_send_clone_success_countdown = 0;
static SyncState_t g_programming_state = SYNC_Searching_for_slave;
static CloneTimingSnapshot_t g_clone_timing_snapshot = {0, 0, 1};
volatile bool g_cloningInProgress = false;

volatile Enunciation_t g_enunciator = LED_ONLY;
static volatile uint16_t g_key_down_countdown = 0;
static volatile bool g_foreground_reset_after_keydown = false;
static volatile uint16_t g_demo_event_countdown = 0;
static volatile bool g_foreground_reset_after_demo = false;

leds LEDS = leds();
CircularStringBuff g_text_buff(TEXT_BUFF_SIZE);

EepromManager g_ee_mgr;

volatile bool g_isMaster = false;
volatile uint16_t isMasterCountdownSeconds = 0;
Fox_t g_fox[EVENT_NUMBER_OF_EVENTS] = {FOX_1, FOX_1, SPRINT_S1, FOXORING_FOX1, USE_CURRENT_FOX}; /* none, classic, sprint, foxoring */

Event_t g_event = EEPROM_EVENT_SETTING_DEFAULT;
Frequency_Hz g_frequency = EEPROM_FREQUENCY_DEFAULT;
Frequency_Hz g_frequency_low = EEPROM_FREQUENCY_LOW_DEFAULT;
Frequency_Hz g_frequency_med = EEPROM_FREQUENCY_MED_DEFAULT;
Frequency_Hz g_frequency_hi = EEPROM_FREQUENCY_HI_DEFAULT;
Frequency_Hz g_frequency_beacon = EEPROM_FREQUENCY_BEACON_DEFAULT;

int8_t g_utc_offset;
uint8_t g_unlockCode[UNLOCK_CODE_SIZE + 2];

volatile bool g_enable_manual_transmissions = true;
static volatile bool g_meshmode = false;

/***********************************************************************
 * Private Function Prototypes
 *
 * These functions are available only within this file
 ************************************************************************/
void handle_1sec_tasks(void);
bool loadedEventShouldBeEnabled(void);
void handleSerialBusMsgs(void);
uint16_t throttleValue(uint8_t speed);
EC activateEventEngineUsingCurrentSettings(SC *statusCode, time_t startTime, time_t finishTime);
EC launchLoadedEvent(SC *statusCode);
void reportSettings(void);
uint16_t timeNeededForID(void);
Frequency_Hz getFrequencySetting(void);
char *getCurrentPatternText(void);
int getPatternCodeSpeed(void);
int getFoxCodeSpeed(void);
Fox_t getFoxSetting(void);
void handleSerialCloning(void);
bool timeIsSet(void);
bool eventIsScheduledToRun(time_t *start_epoch, time_t *finish_epoch);
bool eventIsScheduledToRun(volatile time_t *start_epoch, volatile time_t *finish_epoch);
bool eventIsScheduledToRunNow(time_t start_epoch, time_t finish_epoch);
bool eventScheduledForTheFuture(time_t start_epoch, time_t finish_epoch);
bool noEventWillRun(void);
bool eventRunning(void);
void restoreStateAfterButtonWakeAuthorization(void);
static inline void clearPendingWakeInterruptFlags(void);
bool shouldPowerTransmitterAfterWake(void);
void configRedLEDforEvent(void);
bool switchIsClosed(void);
bool allClocksSet(Settings_t location);
ConfigurationState_t clockConfigurationCheck(Settings_t location);
bool startEvent(void);

/*******************************/
/* Hardcoded event support     */
/*******************************/
void suspendEvent(void);
void startEventNow(bool configOverride);
void startSyncdEventNow(bool configOverride);
bool startEventUsingRTC(void);
void startTransmissionsNow(bool configOverride);
void setupForFox(Fox_t fox, EventAction_t action);
time_t validateTimeString(char *str, char *errMsg);
time_t validateTimeString(char *str, volatile time_t *epochVar, bool align5min, char *errMsh);
time_t validateTimeString(char *str, volatile time_t *epochVar, bool align5min, char *errMsg, const char *rawInput);
bool reportTimeTill(time_t from, time_t until, const char *prefix, const char *failMsg);
void reportConfigErrors(Settings_t location);
/*******************************/
/* End hardcoded event support */
/*******************************/

static void loadCurrentPatternMorse(bool *repeat, callerID_t caller);
static void loadStationIDMorse(bool *repeat, callerID_t caller);
static const char *completeTimeString_volatile(const char *partialString, volatile time_t *currentEpoch);
static bool parseFinishOffsetToEpoch(const char *offsetString, time_t *finishEpoch, char *errMsg);
static bool cancelManualTransientState(void);
static bool finishTimedEventIfExpired(time_t now);
static void captureCloneTimingSnapshot(void);
static void reinitializeEventEngine(void);
static void loadEventTimingForFox(Fox_t fox);
static void resumeLoadedEventAfterCloneExit(bool deferStartIfAlreadyRunning);
static bool reloadLoadedEventWindowFromSavedSettings(void);
static bool catchUpLoadedMultiDayEventAfterClockSet(void);
static bool advanceLoadedEventWindowAfterCurrentDayCancel(void);
static inline void extendMasterModeTimeout(void);

static bool cancelManualTransientState(void)
{
	bool pending_start_after_keydown = g_start_event_after_keydown;
	bool had_transient_state = pending_start_after_keydown || atomic_read_u16(&g_key_down_countdown) || g_foreground_reset_after_keydown || atomic_read_u16(&g_demo_event_countdown) || g_foreground_reset_after_demo;

	if(had_transient_state)
	{
		atomic_write_u16(&g_key_down_countdown, 0);
		atomic_write_u16(&g_demo_event_countdown, 0);
		g_foreground_reset_after_keydown = false;
		g_foreground_reset_after_demo = false;
		g_start_event_after_keydown = false;
		g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;
	}

	return pending_start_after_keydown;
}

static void captureCloneTimingSnapshot(void)
{
	time_t saved_start_epoch;
	time_t saved_finish_epoch;
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;
	uint8_t total_days = g_days_to_run;
	uint8_t completed_days = g_days_run;
	uint8_t days_remaining = (completed_days < total_days) ? (total_days - completed_days) : 1;

	atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &saved_start_epoch, &saved_finish_epoch);
	atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);

	g_clone_timing_snapshot.event_start_epoch = saved_start_epoch;
	g_clone_timing_snapshot.event_finish_epoch = saved_finish_epoch;
	g_clone_timing_snapshot.days_to_run = total_days ? total_days : 1;

	bool loaded_window_is_active = eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch) || eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch);
	bool loaded_window_differs_from_saved = (loaded_start_epoch != saved_start_epoch) || (loaded_finish_epoch != saved_finish_epoch);

	if(loaded_window_is_active && loaded_window_differs_from_saved)
	{
		g_clone_timing_snapshot.event_start_epoch = loaded_start_epoch;
		g_clone_timing_snapshot.event_finish_epoch = loaded_finish_epoch;
		g_clone_timing_snapshot.days_to_run = days_remaining ? days_remaining : 1;
	}
}

static void reinitializeEventEngine(void)
{
	g_evteng_initialize_event = true;
	util_delay_ms(0);
	while(util_delay_ms(17) && g_evteng_initialize_event)
		; // Wait for event engine to initialize
}

static void loadEventTimingForFox(Fox_t fox)
{
	bool delayNotSet = true;

	if(fox == USE_CURRENT_FOX)
	{
		fox = getFoxSetting();
	}

	switch(fox)
	{
		case FOX_1:
		{
			delayNotSet = false;
			g_evteng_intra_cycle_delay_time = 0;
		}
		case FOX_2:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 60;
			}
		}
		case FOX_3:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 120;
			}
		}
		case FOX_4:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 180;
			}
		}
		case FOX_5:
		{
			if(delayNotSet)
			{
				g_evteng_intra_cycle_delay_time = 240;
			}

			g_evteng_ID_period_seconds = 60;
			g_evteng_sendID_seconds_countdown = g_evteng_ID_period_seconds;
			g_evteng_on_air_seconds = 60;
			g_evteng_off_air_seconds = 240;
		}
		break;

		case SPRINT_S1:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 0;
			}
		}
		case SPRINT_S2:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 12;
			}
		}
		case SPRINT_S3:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 24;
			}
		}
		case SPRINT_S4:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 36;
			}
		}
		case SPRINT_S5:
		{
			if(delayNotSet)
			{
				g_evteng_intra_cycle_delay_time = 48;
			}

			g_evteng_ID_period_seconds = 600;
			g_evteng_sendID_seconds_countdown = 600;
			g_evteng_on_air_seconds = 12;
			g_evteng_off_air_seconds = 48;
		}
		break;

		case SPRINT_F1:
		{
			delayNotSet = false;
			g_evteng_intra_cycle_delay_time = 0;
		}
		case SPRINT_F2:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 12;
			}
		}
		case SPRINT_F3:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 24;
			}
		}
		case SPRINT_F4:
		{
			if(delayNotSet)
			{
				delayNotSet = false;
				g_evteng_intra_cycle_delay_time = 36;
			}
		}
		case SPRINT_F5:
		{
			if(delayNotSet)
			{
				g_evteng_intra_cycle_delay_time = 48;
			}

			g_evteng_ID_period_seconds = 600;
			g_evteng_sendID_seconds_countdown = g_evteng_ID_period_seconds;
			g_evteng_on_air_seconds = 12;
			g_evteng_off_air_seconds = 48;
		}
		break;

#if SUPPORT_TEMP_AND_VOLTAGE_REPORTING
		case REPORT_BATTERY:
		{
			g_evteng_intra_cycle_delay_time = 0;
		}
		break;
#endif // SUPPORT_TEMP_AND_VOLTAGE_REPORTING

		case FOXORING_FOX1:
		case FOXORING_FOX2:
		case FOXORING_FOX3:
		{
			g_evteng_intra_cycle_delay_time = 0;
			g_evteng_ID_period_seconds = 600;
			g_evteng_sendID_seconds_countdown = g_evteng_ID_period_seconds;
			g_evteng_on_air_seconds = 600;
			g_evteng_off_air_seconds = 0;
		}
		break;

		case FREQUENCY_TEST_BEACON:
		{
			g_evteng_intra_cycle_delay_time = 0;
			g_evteng_ID_period_seconds = 600;
			g_evteng_sendID_seconds_countdown = g_evteng_ID_period_seconds;
			g_evteng_on_air_seconds = 600;
			g_evteng_off_air_seconds = 0;
			atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
		}
		break;

		case SPECTATOR:
		case BEACON:
		default:
		{
			g_evteng_intra_cycle_delay_time = 0;
			g_evteng_ID_period_seconds = 600;
			g_evteng_sendID_seconds_countdown = g_evteng_ID_period_seconds;
			g_evteng_on_air_seconds = 600;
			g_evteng_off_air_seconds = 0;
		}
		break;
	}
}

static void resumeLoadedEventAfterCloneExit(bool deferStartIfAlreadyRunning)
{
	bool should_resume_loaded_event = loadedEventShouldBeEnabled();
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;

	atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);

	if(deferStartIfAlreadyRunning && should_resume_loaded_event && eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch))
	{
		g_defer_cloned_event_start = true;
		g_foreground_start_event = false;
	}
	else
	{
		g_defer_cloned_event_start = false;
		g_foreground_start_event = should_resume_loaded_event;
	}
}

static bool reloadLoadedEventWindowFromSavedSettings(void)
{
	time_t saved_start_epoch;
	time_t saved_finish_epoch;
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;

	atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &saved_start_epoch, &saved_finish_epoch);

	if((g_days_to_run > 0) && (g_days_run >= g_days_to_run))
	{
		atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, 0, 0);
		return false;
	}

	loaded_start_epoch = saved_start_epoch;
	loaded_finish_epoch = saved_finish_epoch;

	if((saved_start_epoch <= MINIMUM_VALID_EPOCH) || (saved_finish_epoch <= saved_start_epoch))
	{
		atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, loaded_start_epoch, loaded_finish_epoch);
		return false;
	}

	uint8_t day_index = MIN(g_days_run, (uint8_t)((g_days_to_run > 0) ? (g_days_to_run - 1) : 0));
	loaded_start_epoch += ((time_t)day_index * SECONDS_24H);
	loaded_finish_epoch += ((time_t)day_index * SECONDS_24H);

	if(timeIsSet())
	{
		time_t now = time(null);
		while((loaded_finish_epoch <= now) && ((day_index + 1) < g_days_to_run))
		{
			day_index++;
			loaded_start_epoch += SECONDS_24H;
			loaded_finish_epoch += SECONDS_24H;
		}
	}

	atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, loaded_start_epoch, loaded_finish_epoch);

	return eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch) || eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch);
}

static bool catchUpLoadedMultiDayEventAfterClockSet(void)
{
	time_t saved_start_epoch;
	time_t saved_finish_epoch;
	uint8_t total_days = g_days_to_run ? g_days_to_run : 1;

	atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &saved_start_epoch, &saved_finish_epoch);

	if((saved_start_epoch <= MINIMUM_VALID_EPOCH) || (saved_finish_epoch <= saved_start_epoch))
	{
		return reloadLoadedEventWindowFromSavedSettings();
	}

	if(g_days_run >= total_days)
	{
		g_days_run = total_days;
		atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, 0, 0);
		return false;
	}

	uint8_t day_index = MIN(g_days_run, (uint8_t)(total_days - 1));
	time_t loaded_start_epoch = saved_start_epoch + ((time_t)day_index * SECONDS_24H);
	time_t loaded_finish_epoch = saved_finish_epoch + ((time_t)day_index * SECONDS_24H);
	time_t now = time(null);

	while((loaded_finish_epoch <= now) && ((day_index + 1) < total_days))
	{
		day_index++;
		loaded_start_epoch += SECONDS_24H;
		loaded_finish_epoch += SECONDS_24H;
	}

	if(loaded_finish_epoch <= now)
	{
		g_days_run = total_days;
		atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, 0, 0);
		return false;
	}

	g_days_run = day_index;
	atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, loaded_start_epoch, loaded_finish_epoch);
	return eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch) || eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch);
}

static bool advanceLoadedEventWindowAfterCurrentDayCancel(void)
{
	if((g_days_to_run <= 1) || ((g_days_run + 1) >= g_days_to_run))
	{
		return false;
	}

	g_days_run++;
	return reloadLoadedEventWindowFromSavedSettings();
}

static inline void extendMasterModeTimeout(void)
{
	atomic_write_u16(&isMasterCountdownSeconds, 600); /* Remain Master for 10 minutes */
}

static bool finishTimedEventIfExpired(time_t now)
{
	if(!(g_evteng_event_commenced && !g_evteng_run_event_until_canceled))
	{
		return false;
	}

	time_t loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
	if(!loaded_finish_epoch || (now < loaded_finish_epoch))
	{
		return false;
	}

	g_last_status_code = STATUS_CODE_EVENT_FINISHED;
	g_evteng_on_the_air = 0;
	keyTransmitter(OFF);
	g_evteng_event_enabled = false;
	g_evteng_event_commenced = false;
	LEDS.init();
	g_days_run++;
	atomic_write_u16(&g_evteng_sleepshutdown_seconds, 3);

	if(g_days_run < g_days_to_run)
	{
		time_t loaded_start_epoch;
		time_t loaded_finish_epoch_pair;
		atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch_pair);
		time_t next_start_epoch = loaded_start_epoch + SECONDS_24H;
		time_t next_finish_epoch = loaded_finish_epoch_pair + SECONDS_24H;
		atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, next_start_epoch, next_finish_epoch);
		atomic_write_time(&g_time_to_wake_up, next_start_epoch - 15); /* Wake shortly before the next day's event begins. */
		g_sleepType = SLEEP_UNTIL_START_TIME;
		g_go_to_sleep_now = true;
	}
	else
	{
		g_sleepType = SLEEP_FOREVER;
		atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, 0, 0);
	}

	return true;
}

static void loadCurrentPatternMorse(bool *repeat, callerID_t caller)
{
	static char pattern_snapshot[MAX_PATTERN_TEXT_LENGTH + 2];
	char *pattern = getCurrentPatternText();
	if((pattern == g_messages_text[PATTERN_TEXT]) || (pattern == g_messages_text[FOXORING_PATTERN_TEXT]))
	{
		uint8_t slot = (pattern == g_messages_text[PATTERN_TEXT]) ? PATTERN_TEXT : FOXORING_PATTERN_TEXT;
		messages_text_slot_copy_atomic(slot, pattern_snapshot, sizeof(pattern_snapshot));
	}
	else
	{
		strncpy(pattern_snapshot, pattern, sizeof(pattern_snapshot) - 1);
		pattern_snapshot[sizeof(pattern_snapshot) - 1] = '\0';
	}
	makeMorse(pattern_snapshot, repeat, NULL, caller);
}

static void loadStationIDMorse(bool *repeat, callerID_t caller)
{
	static char station_id_snapshot[MAX_PATTERN_TEXT_LENGTH + 2];
	messages_text_slot_copy_atomic(STATION_ID, station_id_snapshot, sizeof(station_id_snapshot));
	makeMorse(station_id_snapshot, repeat, NULL, caller);
}

static const char *completeTimeString_volatile(const char *partialString, volatile time_t *currentEpoch)
{
	time_t epoch = atomic_read_time(currentEpoch);
	return completeTimeString(partialString, &epoch);
}

static bool parseFinishOffsetToEpoch(const char *offsetString, time_t *finishEpoch, char *errMsg)
{
	if(finishEpoch)
	{
		*finishEpoch = 0;
	}

	const char *error = "* Err: Invalid offset!\n";
	time_t start_epoch = atomic_read_time(&g_event_start_epoch);
	if(start_epoch < MINIMUM_VALID_EPOCH)
	{
		error = "* Err: Start not set!\n";
	}
	else
	{
		do
		{
			if(!offsetString || offsetString[0] != '+')
			{
				break;
			}

			const char *hours_str = offsetString + 1;
			if(!hours_str[0])
			{
				break;
			}

			const char *colon = strchr(hours_str, ':');
			if(colon && strchr(colon + 1, ':'))
			{
				break;
			}

			size_t hour_len = colon ? (size_t)(colon - hours_str) : strlen(hours_str);
			if((hour_len == 0) || (hour_len > 3))
			{
				break;
			}

			uint16_t hours = 0;
			bool valid = true;
			for(size_t i = 0; i < hour_len; i++)
			{
				if(!isdigit((unsigned char)hours_str[i]))
				{
					valid = false;
					break;
				}

				hours = (hours * 10) + (uint16_t)(hours_str[i] - '0');
			}

			if(!valid || (hours > 480))
			{
				break;
			}

			uint8_t minutes = 0;
			if(colon)
			{
				const char *minutes_str = colon + 1;
				size_t minute_len = strlen(minutes_str);
				if((minute_len == 0) || (minute_len > 2))
				{
					break;
				}

				for(size_t i = 0; i < minute_len; i++)
				{
					if(!isdigit((unsigned char)minutes_str[i]))
					{
						valid = false;
						break;
					}

					minutes = (minutes * 10) + (uint8_t)(minutes_str[i] - '0');
				}

				if(!valid || (minutes > 59))
				{
					break;
				}
			}

			if(finishEpoch)
			{
				*finishEpoch = start_epoch + ((time_t)hours * HOUR) + ((time_t)minutes * MINUTE);
			}

			return true;
		} while(false);
	}

	if(errMsg)
	{
		strcpy(errMsg, error);
	}

	return false;
}

/**
1-second interrupt ISR
*/
ISR(RTC_CNT_vect)
{
	uint8_t x = RTC.INTFLAGS;

	if(x & RTC_OVF_bm)
	{
		system_tick();

		if(g_evteng_on_the_air < 0)
		{
			g_evteng_on_the_air++;
		}
		else if(g_evteng_on_the_air > 0)
		{
			g_evteng_on_the_air--;
		}

		g_seconds_transition = true;

		if(g_sleeping)
		{
			finishTimedEventIfExpired(time(null));
			if(g_sleepType != SLEEP_FOREVER)
			{
				time_t time_to_wake_up = atomic_read_time(&g_time_to_wake_up);
				if(g_sleepType == SLEEP_UNTIL_NEXT_XMSN)
				{
					if((g_evteng_on_the_air > -6) || (time_to_wake_up <= time(null))) /* Always wake up at least 5 seconds before showtime */
					{
						g_go_to_sleep_now = false;
						g_sleeping = false;
						g_awakenedBy = AWAKENED_BY_CLOCK;
					}
				}
				else if(g_sleepType == SLEEP_UNTIL_START_TIME)
				{
					if(time_to_wake_up <= time(null))
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

	g_seconds_since_wakeup++;

	if(isMasterCountdownSeconds)
		isMasterCountdownSeconds--;

	if(!g_cloningInProgress)
	{
		temp_time = time(null);
		finishTimedEventIfExpired(temp_time);

		if(g_evteng_event_enabled && !g_isMaster)
		{
			if(g_evteng_event_commenced) /* an event is in progress */
			{
				if(g_evteng_sendID_seconds_countdown)
				{
					g_evteng_sendID_seconds_countdown--;
				}
			}
			else if(g_evteng_run_event_until_canceled)
			{
				if(!g_foreground_enable_transmitter)
				{
					if(g_evteng_intra_cycle_delay_time)
					{
						g_last_status_code = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
						g_evteng_on_the_air = -g_evteng_intra_cycle_delay_time;
						g_evteng_sendID_seconds_countdown = g_evteng_intra_cycle_delay_time + g_evteng_on_air_seconds - g_time_needed_for_ID;
					}
					else
					{
						g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
						g_evteng_on_the_air = g_evteng_on_air_seconds;
						g_evteng_sendID_seconds_countdown = g_evteng_on_air_seconds - g_time_needed_for_ID;
					}

					g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());
					bool repeat = true;
					loadCurrentPatternMorse(&repeat, CALLER_AUTOMATED_EVENT);

					g_foreground_enable_transmitter = true;
					LEDS.init();
				}
			}
			else /* waiting for the start time to arrive */
			{
				time_t loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
				if(loaded_start_epoch > MINIMUM_VALID_EPOCH) /* a start time has been set */
				{
					temp_time = time(null);

					if(temp_time >= loaded_start_epoch) /* Time for the event to start */
					{
						loadEventTimingForFox(USE_CURRENT_FOX);
						g_evteng_event_commenced = true;
						g_evteng_initialize_event = true;
						g_sleepType = SLEEP_AFTER_EVENT;

						if(g_evteng_intra_cycle_delay_time)
						{
							g_last_status_code = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
							g_evteng_on_the_air = -g_evteng_intra_cycle_delay_time;
							g_evteng_sendID_seconds_countdown = g_evteng_intra_cycle_delay_time + g_evteng_on_air_seconds - g_time_needed_for_ID;
						}
						else
						{
							g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
							g_evteng_on_the_air = g_evteng_on_air_seconds;
							g_evteng_sendID_seconds_countdown = g_evteng_on_air_seconds - g_time_needed_for_ID;
						}

						g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());
						bool repeat = true;
						loadCurrentPatternMorse(&repeat, CALLER_AUTOMATED_EVENT);

						g_foreground_enable_transmitter = true;
						LEDS.init();
					}
				}
			}
		}
	}

	/**************************************
	 * Delay before sleep
	 ***************************************/
	if(g_evteng_sleepshutdown_seconds)
	{
		g_evteng_sleepshutdown_seconds--;

		if(!g_evteng_sleepshutdown_seconds)
		{
			if(g_isMaster || g_cloningInProgress)
			{
				atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); /* Never sleep while cloning or while master */
			}
			else if(g_evteng_event_commenced && g_evteng_event_enabled && ((g_sleepType != SLEEP_UNTIL_NEXT_XMSN) && (g_sleepType != SLEEP_UNTIL_START_TIME)) && (getFoxSetting() != FREQUENCY_TEST_BEACON))
			{
				atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); /* Never sleep during active transmissions */
			}
			else
			{
				if(g_sleepType == SLEEP_AFTER_EVENT)
				{
					if(noEventWillRun()) /* Event is done; fall through and request the appropriate sleep state now. */
					{
						g_sleepType = SLEEP_FOREVER;
					}
				}

				if((g_sleepType != SLEEP_AFTER_EVENT) && !g_go_to_sleep_now)
				{
					if(g_sleepType == SLEEP_FOREVER)
					{
						time_t loaded_start_epoch;
						time_t loaded_finish_epoch;
						atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
						if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch)) /* Should never evaluate to true here, but checking just in case */
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
	static uint16_t codeInc = 0;
	static bool muteAfterID = false; /* Inhibit any transmissions immediately after the ID has been sent */
	static bool key = false;

	if(g_evteng_initialize_event)
	{
		g_evteng_initialize_event = false;
		on_air_finished = false;
		transitionPrepped = false;
		idTimeout = 0;
		id_timed_out = false;
		codeInc = 0;
		muteAfterID = false;
		key = OFF;
		g_evteng_sending_station_ID = false;
	}

	uint8_t x = TCB0.INTFLAGS;

	if(x & TCB_CAPT_bm)
	{
		static bool conversionInProcess = false;
		static int8_t indexConversionInProcess = 0;
		bool repeat, finished;
		static uint16_t switch_closures_count_period = 40;
		uint8_t holdSwitch = 0;
		static uint8_t buttonReleased = false;
		static uint8_t longPressEnabled = true;
		static uint16_t switch_closed_time = 0;

		if(g_key_down_countdown)
		{
			g_key_down_countdown--;

			if(!g_key_down_countdown)
			{
				g_foreground_reset_after_keydown = true;
			}
		}

		if(g_button_hold_countdown)
		{
			g_button_hold_countdown--;
		}

		if(g_demo_event_countdown)
		{
			g_demo_event_countdown--;

			if(!g_demo_event_countdown)
			{
				g_foreground_reset_after_demo = true;
			}
		}

		if(g_report_settings_countdown)
		{
			g_report_settings_countdown--;

			if(!g_report_settings_countdown)
			{
				g_foreground_report_settings = true;
			}
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
						g_switch_presses_count++;
						buttonReleased = false;
						switch_closures_count_period = 40;
					}
					else /* Switch is now open */
					{
						switch_closed_time = 0;
						buttonReleased = true;
						longPressEnabled = true;
					}
				}
				else if(!holdSwitch) // && LEDS.active()) /* Switch closed, LEDs operating */
				{
					if(!g_long_button_press && longPressEnabled)
					{
						if(++switch_closed_time >= 200)
						{
							g_long_button_press = true;
							switch_closed_time = 0;
							g_switch_presses_count = 0;
							longPressEnabled = false;
						}
					}
				}

				if(switch_closures_count_period) // Time to check if button presses have occurred
				{
					static uint8_t hold_switch_presses_count = 0;
					switch_closures_count_period--;

					if((g_switch_presses_count != 1) || buttonReleased) // Special case: the first press could be a long press if pushbutton is held long enough
					{
						if(!switch_closures_count_period) // Time's up - examine how many button presses were counted
						{
							if(g_switch_presses_count && (g_switch_presses_count <= MAXIMUM_NUM_OF_KEYPRESSES))
							{
								g_foreground_handle_counted_presses = g_switch_presses_count;
							}

							g_switch_presses_count = 0;
							hold_switch_presses_count = 0;
						}
						else if(g_switch_presses_count != hold_switch_presses_count) // New press detected - wait a while longer to see if there's another one
						{
							hold_switch_presses_count = g_switch_presses_count;
							switch_closures_count_period = 40;
						}
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
			switch_closed_time = 0;
			g_switch_presses_count = 0;
			//			switch_closures_count_period = 0;
			g_long_button_press = false;
		}

		if(g_programming_countdown > 0)
			g_programming_countdown--;
		if(g_programming_msg_throttle)
			g_programming_msg_throttle--;
		if(g_send_clone_success_countdown)
			g_send_clone_success_countdown--;

		if(idTimeout)
		{
			idTimeout--;

			if(!idTimeout)
			{
				id_timed_out = true;
			}
		}

		if(g_evteng_event_enabled && g_evteng_event_commenced && !g_isMaster) /* Handle cycling transmissions */
		{
			if((g_evteng_on_the_air > 0) || (g_evteng_sending_station_ID && !id_timed_out) || (!g_evteng_off_air_seconds))
			{
				on_air_finished = true;

				transitionPrepped = false;

				/* Interrupt transmissions and send the ID under these conditions */
				if(!g_evteng_sending_station_ID && (!g_evteng_off_air_seconds || (g_evteng_on_the_air <= g_time_needed_for_ID)) && !g_evteng_sendID_seconds_countdown && g_time_needed_for_ID) /* Time to identify on the air */
				{
					g_last_status_code = STATUS_CODE_SENDING_ID;
					g_evteng_code_throttle = throttleValue(g_evteng_id_codespeed);
					repeat = false;
					loadStationIDMorse(&repeat, CALLER_AUTOMATED_EVENT); /* Send only once */
					g_evteng_sending_station_ID = true;
					idTimeout = g_evteng_code_throttle << 2;
					id_timed_out = false;
					g_evteng_sendID_seconds_countdown = g_evteng_ID_period_seconds;
					codeInc = g_evteng_code_throttle;
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
							g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());
							repeat = true;
							loadCurrentPatternMorse(&repeat, CALLER_AUTOMATED_EVENT);
							muteAfterID = g_evteng_sending_station_ID && g_evteng_off_air_seconds;
							g_evteng_sending_station_ID = false;
							idTimeout = 0;
							id_timed_out = false;

							if(!g_evteng_off_air_seconds)
							{
								g_evteng_on_the_air = g_evteng_on_air_seconds;
							}
						}

						codeInc = g_evteng_code_throttle;
					}
				}
				else
				{
					codeInc = g_evteng_code_throttle;
				}

				if(muteAfterID)
				{
					key = OFF;
				}

				keyTransmitter(key);
				LEDS.setRed(key);
			}
			else if(g_evteng_on_the_air < 0)
			{
				transitionPrepped = false;
			}
			else if(!g_evteng_on_the_air)
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

						g_evteng_on_the_air = -g_evteng_off_air_seconds;
						/* Enable sleep during off-the-air periods */
						int32_t timeRemaining = SECONDS_24H; // Any  big number will do;
						time_t temp_time = time(null);

						if(timeIsSet())
						{
							time_t loaded_start_epoch;
							time_t loaded_finish_epoch;
							atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
							if((temp_time < loaded_finish_epoch) && (loaded_start_epoch != loaded_finish_epoch))
							{
								timeRemaining = timeDif(loaded_finish_epoch, temp_time);
								g_last_status_code = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
							}
						}

						/* Don't sleep for the last cycle to ensure that the event doesn't end while
						 * the transmitter is sleeping - which can cause problems with loading the next event */
						if(timeRemaining > (g_evteng_off_air_seconds + g_evteng_on_air_seconds + 15))
						{
							//							if(timeIsSet() && (g_evteng_off_air_seconds > 15)) /* Don't bother to sleep if the off-air time is short or we don't know what time it is */
							if(g_evteng_off_air_seconds > 15) /* Don't bother to sleep if the off-air time is short */
							{
								time_t seconds_to_sleep = (time_t)(g_evteng_off_air_seconds - 10); // Wake up 10 seconds before it is time to transmit again
								g_time_to_wake_up = temp_time + seconds_to_sleep;                  // Set the time to wake up
								g_sleepType = SLEEP_UNTIL_NEXT_XMSN;
								g_go_to_sleep_now = true;
								g_evteng_sendID_seconds_countdown = MAX(0, g_evteng_ID_period_seconds - (int)seconds_to_sleep);
							}
						}

						muteAfterID = false;
						g_evteng_sending_station_ID = false;
						idTimeout = 0;
						id_timed_out = false;

						/* Resume normal pattern */
						g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());
						repeat = true;
						loadCurrentPatternMorse(&repeat, CALLER_AUTOMATED_EVENT); /* Reset pattern to start */
						LEDS.setRed(OFF);
					}
					else /* Off-the-air period just finished, or the event just began while off the air */
					{
						g_evteng_on_the_air = g_evteng_on_air_seconds;
						g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());
						repeat = true;
						loadCurrentPatternMorse(&repeat, CALLER_AUTOMATED_EVENT);
						codeInc = g_evteng_code_throttle;
					}
				}
			}
		}
		else if(g_enable_manual_transmissions) /* Handle single-character transmissions */
		{
			static bool charFinished = true;
			static bool idle = true;
			bool sendBuffEmpty = text_buff_empty_atomic();
			repeat = false;

			if(lastMorseCaller() != CALLER_MANUAL_TRANSMISSIONS)
			{
				text_buff_reset_atomic();
				sendBuffEmpty = true;
				charFinished = true;
				makeMorse((char *)"\0", &repeat, null, CALLER_MANUAL_TRANSMISSIONS);
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

					codeInc = g_evteng_code_throttle;
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
							static char cc[2]; /* Must be static because makeMorse saves only a pointer to the character array */
							if(text_buff_try_get_atomic(&cc[0]))
							{
								g_evteng_code_throttle = throttleValue(getPatternCodeSpeed());
								cc[1] = '\0';
								makeMorse(cc, &repeat, null, CALLER_MANUAL_TRANSMISSIONS);
								key = makeMorse(null, &repeat, &charFinished, CALLER_MANUAL_TRANSMISSIONS);
							}
						}

						if(g_enunciator == LED_AND_RF)
							keyTransmitter(key);
						LEDS.setRed(key);
						codeInc = g_evteng_code_throttle;
					}
				}
				else
				{
					if(g_enunciator == LED_AND_RF)
						keyTransmitter(key);
					LEDS.setRed(key);
					codeInc = g_evteng_code_throttle;
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
				g_adcCountdownCount[indexConversionInProcess] = g_adcChannelConversionPeriod_ticks[indexConversionInProcess]; /* reset the tick countdown */
				ADC0_setADCChannel(g_adcChannelOrder[indexConversionInProcess]);
				ADC0_startConversion();
				conversionInProcess = true;
			}
		}
		else if(ADC0_conversionDone()) /* wait for conversion to complete */
		{
			uint16_t hold = ADC0_read();

			if(hold < 4090)
			{
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
						g_processor_temperature = isValidTemp(g_processor_temperature) ? (g_processor_temperature + temp) / 2. : temp;
						if(g_processor_temperature > g_processor_max_temperature)
							g_processor_max_temperature = g_processor_temperature;
						if(g_processor_temperature < g_processor_min_temperature)
							g_processor_min_temperature = g_processor_temperature;

						if(g_internal_bat_detected)
						{
							g_thermal_shutdown = (g_processor_temperature > 60.) ? true : (g_processor_temperature < 50.) ? false
							                                                                                              : g_thermal_shutdown;
						}
						else
						{
							g_thermal_shutdown = (g_processor_temperature > 80.) ? true : (g_processor_temperature < 70.) ? false
							                                                                                              : g_thermal_shutdown;
						}

						g_turn_on_fan = (g_processor_temperature > FAN_TURN_ON_TEMP) ? true : (g_processor_temperature < FAN_TURN_OFF_TEMP) ? false
						                                                                                                                    : g_turn_on_fan;
					}
				}
			}

			conversionInProcess = false;
		}
	}

	TCB0.INTFLAGS = (TCB_CAPT_bm | TCB_OVF_bm); /* clear all interrupt flags */
}

/**
Handle port D pushbutton interrupts
*/
ISR(PORTD_PORT_vect)
{
	uint8_t x = VPORTD.INTFLAGS;

	if(x & (1 << SWITCH))
	{
		if(g_sleeping)
		{
			g_button_wake_prior_sleep_type = (uint8_t)g_sleepType;
			g_button_wake_prior_event_enabled = g_evteng_event_enabled;
			g_button_wake_prior_event_commenced = g_evteng_event_commenced;
			g_go_to_sleep_now = false;
			g_sleeping = false;
			g_awakenedBy = AWAKENED_BY_BUTTONPRESS;
		}
		else if(!sb_enabled())
		{
			g_foreground_enable_serialbus = true;
		}

		atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
	}

	VPORTD.INTFLAGS = 0xFF; /* Clear all flags */
}

/**
Handle port C interrupts (Serial Port during sleep)
*/
ISR(PORTC_PORT_vect)
{
	uint8_t x = VPORTC.INTFLAGS;

	if(x & (1 << SERIAL_RX))
	{
		PORTC_pin_set_isc(SERIAL_RX, PORT_ISC_INTDISABLE_gc); // disable this interrupt

		if(g_sleeping)
		{
			g_go_to_sleep_now = false;
			g_sleeping = false;
			g_awakenedBy = AWAKENED_BY_SERIAL_PORT;
			g_foreground_enable_serialbus = true;
			atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
		}
	}

	VPORTC.INTFLAGS = 0xFF; /* Clear all flags */
}

bool switchIsClosed(void)
{
	/* Intentionally re-prime the debounce history so stale samples from earlier runtime
	 * do not affect this wake-qualification check. */
	debounce();
	debounce();
	debounce();
	debounce();
	return (!(portDdebouncedVals() & (1 << SWITCH)));
}

static inline void clearPendingWakeInterruptFlags(void)
{
	/* Drop any stale edge flags captured while we were still awake so only a
	 * new button press or fresh serial activity can wake the device. */
	VPORTC.INTFLAGS = 0xFF;
	VPORTD.INTFLAGS = 0xFF;
}

/* Entry point for firmware; performs hardware initialization and main loop. */
int main(void)
{
	bool buttonHeldClosed = true;
	bool internal_bat_error = false;
	bool external_pwr_error = false;

	atmel_start_init();
	serialbus_init(SB_BAUD, SERIALBUS_USART);
	LEDS.blink(LEDS_OFF, true);

	g_ee_mgr.initializeEEPROMVars();

	g_ee_mgr.readNonVols();
	reloadLoadedEventWindowFromSavedSettings();
	g_isMaster = false; /* Never start up as master */

	if(g_enable_external_battery_control)
		setExtBatLoadSwitch(ON, INITIALIZE_LS); // Enable external power

	if(g_frequency == EEPROM_FREQUENCY_DEFAULT)
	{
		g_frequency = getFrequencySetting();
		txSetFrequency(&g_frequency, true);
	}

	RTC_set_calibration(g_clock_calibration);

	LEDS.blink(LEDS_RED_ON_CONSTANT);
	LEDS.blink(LEDS_GREEN_ON_CONSTANT);
	atomic_write_u16(&g_button_hold_countdown, 1000);

	while(util_delay_ms(2500))
		; /* Avoid possible race conditions with peripheral devices powering up */

	reportSettings();

	/* Check that the RTC is running */
	atomic_set_system_time(YEAR_2000_EPOCH, false);
	time_t now = time(null);
	while((util_delay_ms(4500)) && (now == time(null)))
		;

	if(now == time(null))
	{
		LEDS.blink(LEDS_GREEN_OFF); // Signal that the first attempt failed
		LEDS.blink(LEDS_RED_OFF);
		while((util_delay_ms(3000)) && (now == time(null)))
			;
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

	int tries = 5;
	powerToTransmitter(ON);

	while(tries && (!txIsInitialized()))
	{
		--tries;
		powerToTransmitter(ON);
	}

	if(!txIsInitialized())
	{
		g_hardware_error |= (int)HARDWARE_NO_SI5351;
	}

	g_evteng_run_event_until_canceled = false;
	g_foreground_start_event = g_device_enabled && loadedEventShouldBeEnabled(); /* Start any event stored in EEPROM */
	sb_send_NewPrompt();

	atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);

	/* Disable automatic ADC readings */
	TCB0.INTCTRL = 0; /* Capture or Timeout: disable interrupts */
	TCB0.CTRLA = 0;   /* Disable timer */
	/* The external battery control load switch is initialized to ON, so we don't need to set it here */
	g_internal_bat_voltage = readVoltage(ADCInternalBatteryVoltage);
	g_external_voltage = readVoltage(ADCExternalBatteryVoltage);
	float internal_bat_voltage_startup = atomic_read_float(&g_internal_bat_voltage);
	float internal_voltage_low_threshold_startup = atomic_read_float(&g_internal_voltage_low_threshold);

	g_internal_bat_detected = (g_internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE);

	if(g_internal_bat_detected && (internal_bat_voltage_startup < internal_voltage_low_threshold_startup) && (g_enable_external_battery_control > 0)) // An internal battery is present & not fully charged & charging from an external battery is enabled
	{
		g_charge_battery = true;
		setExtBatLoadSwitch(ON, INTERNAL_BATTERY_CHARGING);
	}

	g_restart_conversions = true;
	TIMERB_init();
	powerToTransmitter(OFF);

	while(1)
	{
		if(g_foreground_enable_serialbus)
		{
			g_foreground_enable_serialbus = false;
			if(!sb_enabled())
			{
				serialbus_init(SB_BAUD, SERIALBUS_USART);
			}
		}

		if(g_foreground_check_for_long_wakeup_press)
		{
			buttonHeldClosed = switchIsClosed();

			if(!buttonHeldClosed) /* Pushbutton not held; go back to sleep */
			{
				g_go_to_sleep_now = true;
				g_foreground_check_for_long_wakeup_press = false;
				atomic_write_u16(&g_foreground_handle_counted_presses, 0);
				LEDS.blink(LEDS_OFF);
			}
			else if(!atomic_read_u16(&g_button_hold_countdown)) /* Pushbutton held down long enough; power up */
			{
				LEDS.init();
				g_long_button_press = false;
				g_foreground_check_for_long_wakeup_press = false;

				if(g_enable_external_battery_control)
					setExtBatLoadSwitch(ON, INITIALIZE_LS); // Enable external power

				bool restoring_mid_event_button_wake =
				    (g_awakenedBy == AWAKENED_BY_BUTTONPRESS) &&
				    (g_button_wake_prior_sleep_type == SLEEP_UNTIL_NEXT_XMSN) &&
				    g_button_wake_prior_event_commenced;

				if(!g_event_launched_by_user_action && !restoring_mid_event_button_wake) // re-initialize event engine with stored event start and stop if it might be needed
				{
					reloadLoadedEventWindowFromSavedSettings();
				}

				// If the event loaded into the event engine is disabled, set it to start.
				if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch) && !g_evteng_event_enabled && !g_foreground_start_event)
				{
					g_foreground_start_event = true;
				}

				if((g_awakenedBy == AWAKENED_BY_BUTTONPRESS) && !g_foreground_start_event)
				{
					restoreStateAfterButtonWakeAuthorization();
				}

				atomic_write_u16(&g_demo_event_countdown, 0);
				g_foreground_reset_after_demo = false;
				atomic_write_u16(&g_key_down_countdown, 0);
				g_foreground_reset_after_keydown = false;

				configRedLEDforEvent();
				if(!g_meshmode)
				{
					sb_send_NewLine();
					sb_send_string(TEXT_NOT_SLEEPING_TXT);
				}

				if(!g_device_enabled)
				{
					sb_send_string(TEXT_DEVICE_DISABLED_TXT);
				}
#ifdef TEST_MODE_SOFTWARE
				sb_send_string(TEXT_TEST_SOFTWARE_NOTICE_TXT);
#endif

				if(!g_meshmode)
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
			float internal_bat_voltage = atomic_read_float(&g_internal_bat_voltage);
			float external_voltage = atomic_read_float(&g_external_voltage);
			float internal_voltage_low_threshold = atomic_read_float(&g_internal_voltage_low_threshold);
			time_t seconds_since_wakeup = atomic_read_time(&g_seconds_since_wakeup);

			// Set internal battery charging
			if((internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE) && (external_voltage > EXT_BAT_CHARGE_SUPPORT_THRESH_LOW))
			{
				if(internal_bat_voltage < internal_voltage_low_threshold) // An adequate external voltage is present and an internal battery is present & not fully charged
				{
					g_charge_battery = true;
				}
				else if(internal_bat_voltage > INT_BAT_CHARGE_THRES_HIGH)
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
				if(seconds_since_wakeup > 10)
				{
					setExtBatLoadSwitch(g_charge_battery, INTERNAL_BATTERY_CHARGING);
				}
			}

#ifdef HW_TARGET_3_5
			/* Newer hardware has a dedicated fan-control output that is independent of
			 * the auxiliary external-battery switch. */
			if(g_thermal_shutdown) // Extremely high temperature detected
			{
				setCoolingFanLSEnable(ON); // should already be on
				suspendEvent();
				sb_send_string(TEXT_EXCESSIVE_TEMPERATURE);
				atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
				g_thermal_shutdown = false; // It will probably be activated several times before sufficiently cool
			}
			else
			{
				if(g_turn_on_fan)
				{
					setCoolingFanLSEnable(ON);
				}
				else
				{
					setCoolingFanLSEnable(OFF);
				}
			}
#else
			/* Legacy hardware reuses the shared auxiliary switch as fan drive whenever
			 * external-battery control is disabled at runtime. */
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
#endif

			/********************************
			 * Handle sleep
			 ******************************/
			if(g_go_to_sleep_now && !g_cloningInProgress)
			{
				bool enterSleep = true;

				if(g_sleepType == SLEEP_AFTER_EVENT)
				{
					if(g_evteng_event_enabled)
					{
						enterSleep = false;
						g_go_to_sleep_now = false;
						atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); // check again later
					}
				}

				if(enterSleep)
				{
					if((g_sleepType == SLEEP_FOREVER) || (g_sleepType == SLEEP_POWER_OFF_OVERRIDE))
					{
						time_t loaded_start_epoch;
						time_t loaded_finish_epoch;
						atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
						if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch)) /* Never sleep forever if an event is scheduled to start in the future */
						{
							g_sleepType = SLEEP_UNTIL_START_TIME;
						}
						else
						{
							atomic_write_time(&g_time_to_wake_up, FOREVER_EPOCH);

							if(!timeIsSet())
							{
								if(!g_meshmode)
								{
									sb_send_NewLine();
									sb_send_string(TEXT_POWER_OFF);
									while((util_delay_ms(2000)) && serialbusTxInProgress())
										;
									while(util_delay_ms(200))
										; // Let serial xmsn finish before power off
								}

								if(!g_charge_battery)
								{
									PORTA_set_pin_level(POWER_ENABLE, LOW); /* No need to preserve current time, so power off - but if charging, keep power switch on to measure internal battery level */
								}
							}
							else if(!g_meshmode)
							{
								sb_send_NewLine();
								sb_send_string(TEXT_SLEEPING_TXT);
								while((util_delay_ms(2000)) && serialbusTxInProgress())
									;
								while(util_delay_ms(200))
									; // Let serial xmsn finish before sleep
							}
						}
					}

					if(!g_meshmode)
					{
						util_delay_ms(0);

						if(g_sleepType == SLEEP_UNTIL_START_TIME)
						{
							sb_send_NewLine();
							sb_send_string(TEXT_SLEEPING_UNTIL_START_TXT);
						}
						else if(g_sleepType == SLEEP_UNTIL_NEXT_XMSN)
						{
							sb_send_NewLine();
							sb_send_string(TEXT_SLEEPING_UNTIL_NEXT_XMSN);
						}

						while(util_delay_ms(300) && serialbusTxInProgress())
							; // Let serial  finish
						while(util_delay_ms(200))
							;
					}

					/* If sleeping until start time, make sure everything is set up properly for when that time arrives */
					if((g_sleepType == SLEEP_UNTIL_START_TIME) || (g_sleepType == SLEEP_POWER_OFF_OVERRIDE))
					{
						g_evteng_run_event_until_canceled = false;
						g_evteng_event_commenced = false;
						g_foreground_start_event = false;

						time_t loaded_start_epoch;
						time_t loaded_finish_epoch;
						time_t saved_start_epoch;
						time_t saved_finish_epoch;
						atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
						atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &saved_start_epoch, &saved_finish_epoch);
						if(loaded_start_epoch < MINIMUM_VALID_EPOCH) // should never be true
						{
							atomic_write_time(&g_evteng_loaded_start_epoch, saved_start_epoch);
						}

						if(loaded_finish_epoch < MINIMUM_VALID_EPOCH) // should never be true
						{
							atomic_write_time(&g_evteng_loaded_finish_epoch, saved_finish_epoch);
						}
					}

					if(g_sleepType == SLEEP_POWER_OFF_OVERRIDE)
					{
						g_sleepType = SLEEP_FOREVER;
					}

					powerToTransmitter(OFF);
					atomic_write_u16(&g_demo_event_countdown, 0);
					g_foreground_reset_after_demo = false;
					atomic_write_u16(&g_key_down_countdown, 0);
					g_foreground_reset_after_keydown = false;

					DISABLE_INTERRUPTS();
					LEDS.deactivate();
					serialbus_disable();
					system_sleep_config();
					clearPendingWakeInterruptFlags();
					SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_STDBY_gc);
					g_sleeping = true;
					g_awakenedBy = AWAKENED_INIT;
					ENABLE_INTERRUPTS();

					/* Disable BOD? */

					while(g_go_to_sleep_now)
					{
						if((g_sleepType == SLEEP_FOREVER) || (g_sleepType == SLEEP_UNTIL_START_TIME))
						{
							volatile time_t now = time(null);
							static volatile time_t hold_now = 0;

							if(timeDif(now, hold_now) > 90) // Periodically check to see if the internal battery should be charged
							{
								hold_now = now;
								system_charging_config();
								g_internal_bat_voltage = readVoltage(ADCInternalBatteryVoltage); // Throw out first result following sleep
								g_external_voltage = readVoltage(ADCExternalBatteryVoltage);
								float internal_bat_voltage = atomic_read_float(&g_internal_bat_voltage);
								float internal_voltage_low_threshold = atomic_read_float(&g_internal_voltage_low_threshold);
								system_sleep_config();
								setExtBatLoadSwitch(RE_APPLY_LS_STATE); // Undo any charging configuration changes to the LS setting

								if(g_enable_external_battery_control) // Control of an external battery is enabled
								{
									if(internal_bat_voltage > INT_BAT_PRESENT_VOLTAGE) // An internal battery is present
									{
										if(internal_bat_voltage < internal_voltage_low_threshold) // An external voltage is present and an internal battery is present & not fully charged
										{
											g_charge_battery = true;
										}
										else if(internal_bat_voltage >= INT_BAT_CHARGE_THRES_HIGH)
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

						if(g_enable_external_battery_control)
							setExtBatLoadSwitch(g_charge_battery, INTERNAL_BATTERY_CHARGING);

						set_sleep_mode(SLEEP_MODE_STANDBY);
						//					set_sleep_mode(SLEEP_MODE_PWR_DOWN);
						DISABLE_INTERRUPTS();
						clearPendingWakeInterruptFlags();
						sleep_enable();
						ENABLE_INTERRUPTS();
						sleep_cpu(); /* Sleep occurs here */
						sleep_disable();
					}

					CLKCTRL_init();
					/* Re-enable BOD? */
					g_sleeping = false;
					g_seconds_since_wakeup = 0;
					atmel_start_init();
					if(!sb_enabled())
						serialbus_init(SB_BAUD, SERIALBUS_USART);

					if(g_enable_external_battery_control)
						setExtBatLoadSwitch(g_charge_battery, INTERNAL_BATTERY_CHARGING);

					if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS)
					{
						g_device_wakeup_complete = false;                // Set the flag to ignore key presses other than an initial long press
						g_foreground_check_for_long_wakeup_press = true; // Set the flag to check for an initial long keypress before waking up the device
						LEDS.init();
						LEDS.blink(LEDS_RED_ON_CONSTANT);
						LEDS.blink(LEDS_GREEN_ON_CONSTANT);
						buttonHeldClosed = true;
						while(util_delay_ms(2000))
							;
					}

					atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);

					if(g_awakenedBy == AWAKENED_BY_BUTTONPRESS) // A button press woke us up, but need to check that it is held down long enough (~5 secs) for us to consider it
					{
						atomic_write_u16(&g_button_hold_countdown, 1000);
						atomic_write_u16(&g_foreground_handle_counted_presses, 0);

						if((g_button_wake_prior_sleep_type == SLEEP_UNTIL_START_TIME) || (g_button_wake_prior_sleep_type == SLEEP_UNTIL_NEXT_XMSN)) /* User woke up the transmitter early, before transmissions were to start */
						{
							g_foreground_start_event = false;
							if(g_button_wake_prior_sleep_type == SLEEP_UNTIL_START_TIME)
							{
								g_evteng_event_commenced = false;
							}
						}
					}
					else if(g_awakenedBy == AWAKENED_BY_SERIAL_PORT) // Similar to being awakened by a button press, but without the 5-second "hold down the button" timing constraint
					{
						LEDS.init();
						atomic_max_u16(&g_evteng_sleepshutdown_seconds, 300U);
						if(!g_cloningInProgress && !g_meshmode)
						{
							configRedLEDforEvent();
							LEDS.blink(LEDS_GREEN_ON_CONSTANT);
							atomic_write_u16(&g_report_settings_countdown, 100);
						}
					}
					else // Awakened by = AWAKENED_BY_CLOCK (AWAKENED_INIT and POWER_UP_START should not be possibilities here)
					{
						// Sleep type must be either SLEEP_UNTIL_NEXT_XMSN or SLEEP_UNTIL_START_TIME
						// In either case it is time to power up the transmit circuits
						if(g_sleepType == SLEEP_UNTIL_NEXT_XMSN)
						{
							g_sleepType = SLEEP_AFTER_EVENT;
						}
						else
						{
							g_foreground_start_event = true;
						}
					}

					// Restore the transmitter power state that matches the logical state we just woke into.
					bool should_power_tx = shouldPowerTransmitterAfterWake();
					if(powerToTransmitter(should_power_tx) != ERROR_CODE_NO_ERROR)
					{
						if(should_power_tx)
						{
							sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
						}
					}

					g_last_status_code = STATUS_CODE_RETURNED_FROM_SLEEP;
				}
			}
			else
			{
				g_device_wakeup_complete = true; // Set the flag to accept user key presses
			}

			uint16_t counted_presses = atomic_exchange_u16(&g_foreground_handle_counted_presses, 0);
			if(counted_presses)
			{
				bool release_deferred_clone_start = false;
				if(atomic_read_u16(&g_send_clone_success_countdown) || g_cloningInProgress)
				{
					release_deferred_clone_start = atomic_read_u16(&g_send_clone_success_countdown) && g_defer_cloned_event_start;
					atomic_write_u16(&g_send_clone_success_countdown, 0);
					g_cloningInProgress = false;
					atomic_write_u16(&g_programming_msg_throttle, 0);
					resumeLoadedEventAfterCloneExit(false);
					counted_presses = 0; /* Throw out any keypresses that occurred while cloning or sending cloning success pattern */
				}

				if(release_deferred_clone_start)
				{
					g_defer_cloned_event_start = false;
					g_foreground_start_event = true;
				}

				if(g_isMaster)
				{
					if(counted_presses == 5)
					{
						if(atomic_read_u16(&g_key_down_countdown))
						{
							atomic_write_u16(&g_key_down_countdown, 0);
							LEDS.setRed(OFF);
							keyTransmitter(OFF);
							powerToTransmitter(OFF);
						}

						if(atomic_read_u16(&g_demo_event_countdown))
						{
							atomic_write_u16(&g_demo_event_countdown, 0);
							LEDS.setRed(OFF);
							keyTransmitter(OFF);
							powerToTransmitter(OFF);
						}

						atomic_write_u16(&isMasterCountdownSeconds, 0);
						sb_send_NewPrompt();
						g_isMaster = false;
						atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); /* Resume normal slave sleep timeout */
						g_evteng_event_commenced = false;
						g_defer_cloned_event_start = false;
						g_foreground_start_event = loadedEventShouldBeEnabled();
						g_cloningInProgress = false;
						atomic_write_u16(&g_programming_countdown, 0);
						atomic_write_u16(&g_programming_msg_throttle, 0);
						atomic_write_u16(&g_send_clone_success_countdown, 0);
						LEDS.init();
						text_buff_reset_atomic();
					}
					else if(counted_presses == 9) // Perform software reset
					{
						RSTCTRL_reset();
					}
				}
				else // not Master
				{
					if(counted_presses == 1)
					{
						if(!g_cloningInProgress && g_device_enabled)
						{
							if(getFoxSetting() == FREQUENCY_TEST_BEACON)
							{
								atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
								if(!txIsInitialized())
								{
									if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
									{
										sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
									}
								}

								g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());

								if(++g_frequency_to_test >= NUMBER_OF_TEST_FREQUENCIES)
									g_frequency_to_test = 0;

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

								setupForFox(USE_CURRENT_FOX, START_TRANSMISSIONS_NOW);
							}
							else
							{
								uint8_t inc = 0;
								time_t loaded_start_epoch;
								time_t loaded_finish_epoch;

								g_start_event_after_keydown = false;
								atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
								if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
								{
									/* Implement special behavior if an event is scheduled to commence in the future: have a single button press toggle between
									 * transmitting and slow blinking. But, in either case, the transmitter should eventually go to sleep and awaken at the
									 * appointed time for the future event */
									if(atomic_read_u16(&g_key_down_countdown))
									{
										atomic_write_u16(&g_key_down_countdown, 0);
										g_foreground_reset_after_keydown = false;
										inc = 1;
									}

									if(atomic_read_u16(&g_demo_event_countdown))
									{
										atomic_write_u16(&g_demo_event_countdown, 0);
										g_foreground_reset_after_demo = false;
										inc = 2;
									}

									switch(inc)
									{
										case 0: /* 30 second keydown */
										{
											suspendEvent();
#ifdef TEST_MODE_SOFTWARE
											atomic_write_u16(&g_key_down_countdown, 36000); // 120 seconds
#else
											atomic_write_u16(&g_key_down_countdown, 9000); // 30 seconds
#endif
											if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
											{
												sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
											}
											LEDS.init();
											LEDS.setRed(ON);
											keyTransmitter(ON);
											g_evteng_event_enabled = false; // Keydown is not controlled by the Event Engine
											g_evteng_run_event_until_canceled = true;
											g_start_event_after_keydown = true;
										}
										break;

										case 1: /* 30 second demo transmission */
										{
											suspendEvent();

											if(g_evteng_on_air_seconds < 30)
											{
												atomic_write_u16(&g_demo_event_countdown, g_evteng_on_air_seconds * 300); // transmit seconds
											}
											else
											{
												atomic_write_u16(&g_demo_event_countdown, 9000); // 30 seconds
											}

											if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
											{
												sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
											}

											LEDS.init();
											startTransmissionsNow(true);
										}
										break;

										default: //	case 2: Start the event normally
										{
											startEvent();
										}
										break;
									}

									atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); // Sleep after 5 minutes
								}
								else // No event scheduled for the future (one might be in progress, or none is scheduled at all)
								{
									suspendEvent();

									if(atomic_read_u16(&g_demo_event_countdown))
									{
										atomic_write_u16(&g_demo_event_countdown, 0);
										g_foreground_reset_after_demo = false;
										// Restore the currently relevant saved event window.
										reloadLoadedEventWindowFromSavedSettings();
									}

									if(atomic_read_u16(&g_key_down_countdown))
									{
										atomic_write_u16(&g_key_down_countdown, 0); // Cancel countdown
										g_foreground_reset_after_keydown = false;
										g_event_launched_by_user_action = true;
										LEDS.init();
#ifndef TEST_MODE_SOFTWARE
										startSyncdEventNow(true); // Immediately start the event (sync to the clock if possible)
#endif
									}
									else
									{
#ifdef TEST_MODE_SOFTWARE
										atomic_write_u16(&g_key_down_countdown, 36000); // 120 seconds
#else
										atomic_write_u16(&g_key_down_countdown, 9000); // 30 seconds
#endif
										g_start_event_after_keydown = true;
										if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
										{
											sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
										}
										LEDS.init();
										LEDS.setRed(ON);
										keyTransmitter(ON);
										g_evteng_event_enabled = false;
										g_evteng_run_event_until_canceled = true;
									}
								}
							}
						}
					}
					else if(counted_presses == 3)
					{
						g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;

						if(atomic_read_u16(&g_key_down_countdown))
						{
							g_start_event_after_keydown = false;
							atomic_write_u16(&g_key_down_countdown, 0);
							g_foreground_reset_after_keydown = false; // prevent foreground from executing keydown reset
						}

						if(atomic_read_u16(&g_demo_event_countdown))
						{
							atomic_write_u16(&g_demo_event_countdown, 0);
							g_foreground_reset_after_demo = false;
						}

						g_event_launched_by_user_action = false;

						if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch))
						{
							time_t loaded_start_epoch;
							time_t loaded_finish_epoch;
							atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
							if(eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch))
							{
								suspendEvent();
								if(advanceLoadedEventWindowAfterCurrentDayCancel())
								{
									startEventUsingRTC();
								}
								atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
							}
							else // the event is scheduled for the future, so set it up to start (transmissions will stop for now)
							{
								suspendEvent();
								startEventUsingRTC();
								atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
							}
						}
						else
						{
							suspendEvent();
						}
					}
					else if(counted_presses == 5)
					{
						bool had_transient_state = g_start_event_after_keydown || atomic_read_u16(&g_key_down_countdown) || atomic_read_u16(&g_demo_event_countdown) || g_foreground_reset_after_keydown || g_foreground_reset_after_demo;
						if(had_transient_state)
						{
							cancelManualTransientState();
							suspendEvent();
						}

						g_isMaster = true;
						g_evteng_event_commenced = false;
						atomic_write_u16(&isMasterCountdownSeconds, 600);       /* Remain Master for 10 minutes */
						atomic_write_u16(&g_evteng_sleepshutdown_seconds, 720); /* Never sleep while master; slave timeout starts after master ends */

						g_cloningInProgress = false;
						atomic_write_u16(&g_programming_countdown, 0);
						atomic_write_u16(&g_send_clone_success_countdown, 0);
						LEDS.init();
						text_buff_reset_atomic();
					}
					else if(counted_presses == 7)
					{
						if(!g_device_enabled)
						{
							g_device_enabled = true;
							g_ee_mgr.updateEEPROMVar(Device_Enabled, (void *)&g_device_enabled);
							time_t now = time(null);
							now++;
							while(now >= time(null))
								;
							RSTCTRL_reset();
						}
					}
				}
			}

			if(g_foreground_start_event) // This flag instructs the foreground to "launch" the event by first configuring the interrupt settings based on user-set event settings
			{
				g_foreground_start_event = false;

				if(!g_isMaster)
				{
					// Here, we have the foreground loop launch whatever event is already loaded into the Event Engine

					g_evteng_run_event_until_canceled = false;   // Foreground auto-start is only used for scheduled event launches
					setupForFox(USE_CURRENT_FOX, START_NOTHING); // Refresh fox-specific timing before launching a loaded event
					reinitializeEventEngine();

					LEDS.init();
					g_last_error_code = launchLoadedEvent((SC *)&g_last_status_code);

					if(g_last_error_code != ERROR_CODE_NO_ERROR)
					{
						sb_send_string((char *)"* Err: event not launched\n");
					}

					LEDS.init();
					atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
				}
			}

			if(g_foreground_enable_transmitter) // This flag to the foreground allows the transmitter to be turned on using I2C by interrupts
			{
				g_foreground_enable_transmitter = false;
				if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
				{
					sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
				}
			}

			// 			if(eventIsScheduledToRunNow(g_evteng_loaded_start_epoch, g_evteng_loaded_finish_epoch) || g_evteng_run_event_until_canceled)
			// 			{
			// 				if(!get_fet_driver() || !get_V3V3_enable())
			// 				{
			// 					if((g_sleepType != SLEEP_FOREVER) || ((g_evteng_loaded_start_epoch) && (g_evteng_loaded_finish_epoch)))
			// 					{
			// 						sb_send_string((char*)"* Error: power not applied to tx stages\n");
			// 						if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
			// 						{
			// 							sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
			// 						}
			// 					}
			// 				}
			// 			}

			if(g_foreground_report_settings)
			{
				g_foreground_report_settings = false;
				reportSettings();
				sb_send_NewLine();
				sb_send_NewPrompt();
			}

			if(g_isMaster)
			{
				handleSerialCloning();

				if(text_buff_empty_atomic())
				{
					if(!atomic_read_u16(&g_key_down_countdown) && !atomic_read_u16(&g_demo_event_countdown))
					{
						if(atomic_read_u16(&g_send_clone_success_countdown))
						{
							LEDS.sendCode((char *)"X ");
						}
						else
						{
							LEDS.sendCode((char *)"M ");
						}
					}
				}
				else /* Make sure the text buffer is being emptied */
				{
					g_enable_manual_transmissions = true; /* There is only one consumer of g_text_buff so it is always OK to enable manual transmissions */
				}

				if(!atomic_read_u16(&isMasterCountdownSeconds))
				{
					g_isMaster = false;
					atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); /* Ensure sleep occurs */
					atomic_write_u16(&g_send_clone_success_countdown, 0);
					g_foreground_start_event = loadedEventShouldBeEnabled(); /* Start any event stored in EEPROM */
					LEDS.init();
				}
			}
			else
			{
				handleSerialBusMsgs();

				if(g_defer_cloned_event_start && !g_cloningInProgress && !atomic_read_u16(&g_send_clone_success_countdown) && !g_foreground_start_event)
				{
					resumeLoadedEventAfterCloneExit(false);
				}

				if(g_cloningInProgress && !atomic_read_u16(&g_programming_countdown)) // Cloning timed out
				{
					g_cloningInProgress = false;
					resumeLoadedEventAfterCloneExit(false);
				}

				if(text_buff_empty_atomic())
				{
					time_t seconds_since_wakeup = atomic_read_time(&g_seconds_since_wakeup);
					if((seconds_since_wakeup < 60) && (g_hardware_error & ((int)HARDWARE_NO_RTC | (int)HARDWARE_NO_SI5351)))
					{
						if(g_hardware_error & ((int)HARDWARE_NO_RTC))
						{
							LEDS.sendCode((char *)"5CLK");
						}

						if(g_hardware_error & ((int)HARDWARE_NO_SI5351))
						{
							LEDS.sendCode((char *)"5XMT");
						}
					}
					else if(!g_device_enabled)
					{
						LEDS.blink(LEDS_RED_THEN_GREEN_BLINK_SLOW, true);
					}
					else if(g_cloningInProgress || atomic_read_u16(&g_key_down_countdown))
					{
						LEDS.blink(LEDS_RED_ON_CONSTANT, true);
					}
					else if(g_foreground_start_event)
					{
						time_t loaded_start_epoch;
						time_t loaded_finish_epoch;
						atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);

						if(eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch))
						{
							LEDS.blink(LEDS_RED_OFF);
						}
						else if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
						{
							LEDS.blink(LEDS_RED_BLINK_SLOW);
						}
						else
						{
							LEDS.blink(LEDS_RED_BLINK_FAST);
						}
					}
					else if(!g_evteng_event_commenced)
					{
						if(atomic_read_u16(&g_send_clone_success_countdown))
						{
							LEDS.sendCode((char *)"X ");
						}
						else
						{
							time_t loaded_start_epoch;
							time_t loaded_finish_epoch;
							bool active_event_window;
							atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
							active_event_window = eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch) &&
							                      (g_evteng_event_enabled || g_foreground_start_event || (g_sleepType == SLEEP_AFTER_EVENT) || (g_sleepType == SLEEP_UNTIL_NEXT_XMSN));

							if(active_event_window)
							{
								LEDS.blink(LEDS_RED_OFF);
							}
							else if(noEventWillRun()) /* No event is running now, nor will one run in the future */
							{
								LEDS.blink(LEDS_RED_BLINK_FAST);
							}
							else if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch) ||
							        eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch)) /* An event is scheduled to run in the future = OK */
							{
								LEDS.blink(LEDS_RED_BLINK_SLOW);
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
				float internal_bat_voltage = atomic_read_float(&g_internal_bat_voltage);
				float internal_voltage_low_threshold = atomic_read_float(&g_internal_voltage_low_threshold);
				float external_voltage = atomic_read_float(&g_external_voltage);
				bool event_active = (g_evteng_event_enabled && g_evteng_event_commenced && !g_isMaster);
				internal_bat_error = (g_internal_bat_detected && (internal_bat_voltage <= internal_voltage_low_threshold));
				external_pwr_error = (external_voltage <= EXT_BAT_PRESENT_VOLTAGE);

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
					LEDS.blink(event_active ? LEDS_GREEN_OFF : LEDS_GREEN_ON_CONSTANT);
				}
			}

			if(g_long_button_press) /* Shut things down and go to sleep or power off */
			{
				g_long_button_press = false;
				g_foreground_check_for_long_wakeup_press = false;
				atomic_write_u16(&g_foreground_handle_counted_presses, 0);
				LEDS.blink(LEDS_OFF);
				g_isMaster = false;
				atomic_write_u16(&isMasterCountdownSeconds, 0);
				g_defer_cloned_event_start = false;
				atomic_write_u16(&g_send_clone_success_countdown, 0);
				g_cloningInProgress = false;
				atomic_write_u16(&g_programming_countdown, 0);
				atomic_write_u16(&g_programming_msg_throttle, 0);

				{
					time_t loaded_start_epoch;
					time_t loaded_finish_epoch;
					time_t now = time(null);

					finishTimedEventIfExpired(now);
					atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
					if(g_evteng_event_enabled && eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch) && (g_evteng_on_the_air < 0))
					{
						int32_t timeRemaining = SECONDS_24H; // Any big number will do

						if(timeIsSet() && (loaded_start_epoch != loaded_finish_epoch) && (now < loaded_finish_epoch))
						{
							timeRemaining = timeDif(loaded_finish_epoch, now);
						}

						/* Match the event engine's normal off-air sleep guard so we do not wake for an
						 * extra post-finish slot after the user manually puts the unit back to sleep. */
						if((g_evteng_off_air_seconds > 15) && (timeRemaining > (g_evteng_off_air_seconds + g_evteng_on_air_seconds + 15)))
						{
							time_t seconds_to_sleep = MAX((time_t)1, (time_t)(-g_evteng_on_the_air - 10));
							g_evteng_event_enabled = true;
							g_evteng_event_commenced = true;
							atomic_write_time(&g_time_to_wake_up, now + seconds_to_sleep);
							g_evteng_sendID_seconds_countdown = MAX(0, g_evteng_sendID_seconds_countdown - (int)seconds_to_sleep);
							g_sleepType = SLEEP_UNTIL_NEXT_XMSN;
						}
						else if((loaded_start_epoch != loaded_finish_epoch) && (timeRemaining > 0))
						{
							/* Keep tracking the scheduled finish time, but leave transmissions disabled so
							 * a later manual wake cannot resurrect a slot that no longer fully fits before
							 * the event ends. */
							g_evteng_event_enabled = false;
							g_evteng_event_commenced = true;
							atomic_write_time(&g_time_to_wake_up, FOREVER_EPOCH);
							g_sleepType = SLEEP_AFTER_EVENT;
						}
						else
						{
							g_sleepType = SLEEP_AFTER_EVENT;
						}
					}
					else if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
					{
						g_sleepType = SLEEP_UNTIL_START_TIME;
					}
					else if(timeIsSet())
					{
						g_sleepType = SLEEP_FOREVER;
					}
					else
					{
						suspendEvent();
					}
				}

				g_go_to_sleep_now = true;
			}

			if(g_foreground_reset_after_demo)
			{
				atomic_write_u16(&g_demo_event_countdown, 0);
				g_foreground_reset_after_demo = false;

				suspendEvent();

				if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch))
				{
					g_event_launched_by_user_action = false;
					startEventUsingRTC();
				}
				else
				{
					g_sleepType = SLEEP_FOREVER;
					startEventNow(true); // Immediately start the event
				}
			}

			if(g_foreground_reset_after_keydown)
			{
				atomic_write_u16(&g_key_down_countdown, 0);
				g_foreground_reset_after_keydown = false;

				suspendEvent();

				g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;

#ifndef TEST_MODE_SOFTWARE
				if(g_start_event_after_keydown) // indicates that a keypress was used to initiate the keydown when no event was scheduled
				{
					if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch))
					{
						g_event_launched_by_user_action = false;
						startEventUsingRTC();
					}
					else
					{
						g_sleepType = SLEEP_FOREVER;
						startSyncdEventNow(true); // Immediately start the event, synchronized to the clock if possible
					}
				}
#endif

				g_start_event_after_keydown = false;
			}
		}
	}
}

void __attribute__((optimize("O0"))) handleSerialBusMsgs()
// void handleSerialBusMsgs()
{
	SerialbusRxBuffer *sb_buff;

	while((sb_buff = nextFullSBRxBuffer()))
	{
		bool suppressResponse = false;

		atomic_max_u16(&g_evteng_sleepshutdown_seconds, 300U);
		LEDS.blink(LEDS_NO_CHANGE, true);

		SBMessageID msg_id = sb_buff->id;

		switch(msg_id)
		{
			case SB_MODE_MESH:
			{
				char c1 = (sb_buff->fields[SB_FIELD1][0]);

				if(c1 == '1')
				{
					g_meshmode = true;
				}
				else
				{
					g_meshmode = false;
				}
			}
			break;

			case SB_MESSAGE_RESET:
			{
				RSTCTRL_reset();
			}
			break;

			case SB_MESSAGE_TEMPERATURE:
			{
				float processor_temperature = atomic_read_float(&g_processor_temperature);
				if(isValidTemp(processor_temperature))
				{
					int16_t integer;
					uint16_t fractional;

					if(!float_to_parts_signed(processor_temperature, &integer, &fractional))
					{
						if(!g_meshmode)
							sb_send_NewLine();
						sprintf(g_tempStr, "* Temp: %d.%dC\n", integer, fractional);
						sb_send_string(g_tempStr);
					}
				}
				else
				{
					if(!g_meshmode)
						sb_send_NewLine();
					sb_send_string((char *)"* Temp not available\n");
				}
			}
			break;

			case SB_MESSAGE_SET_FOX:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
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
						c1 = USE_CURRENT_FOX;
					}

					if((c1 >= BEACON) && (c1 < USE_CURRENT_FOX))
					{
						Fox_t holdFox = (Fox_t)c1;

						switch(g_event)
						{
							case EVENT_CLASSIC:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_classic, (void *)&holdFox);
							}
							break;

							case EVENT_SPRINT:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_sprint, (void *)&holdFox);
							}
							break;

							case EVENT_FOXORING:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_foxoring, (void *)&holdFox);
							}
							break;

							case EVENT_BLIND_ARDF:
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_blind, (void *)&holdFox);
							}
							break;

							default: /* none */
							{
								g_ee_mgr.updateEEPROMVar(Fox_setting_none, (void *)&holdFox);
							}
							break;
						}

						if(holdFox != getFoxSetting())
						{
							if(!g_cloningInProgress)
							{
								cancelManualTransientState();
							}

							fox_setting_current_slot_write_atomic(holdFox);
							setupForFox(holdFox, START_EVENT_WITH_STARTFINISH_TIMES);

							if(!g_cloningInProgress)
							{
								if(g_evteng_event_enabled) // try to update an event already in progress
								{
									if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch))
									{
										g_event_launched_by_user_action = false;
										startEventUsingRTC();
									}
									else
									{
										g_sleepType = SLEEP_FOREVER;
										startEventNow(true); // Immediately start the event
									}
								}
							}
						}
					}
				} /* if(c1) */

				if(!fox2Text(g_tempStr, getFoxSetting()))
				{
					char buf[TEMP_STRING_SIZE];
					strncpy(buf, g_tempStr, TEMP_STRING_SIZE);
					sprintf(g_tempStr, "* Fox:%s\n", buf);

					if(!g_meshmode)
					{
						sb_send_NewLine();
					}

					sb_send_string(g_tempStr);
				}
			}
			break;

			case SB_MESSAGE_SLP:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				if(sb_buff->fields[SB_FIELD1][0])
				{
					if(sb_buff->fields[SB_FIELD1][0] == '0')
					{
						g_sleepType = SLEEP_AFTER_EVENT;
					}
					else if(sb_buff->fields[SB_FIELD1][0] == '1')
					{
						g_evteng_event_enabled = loadedEventShouldBeEnabled(); // Set sleep type based on current event settings
					}
					else
					{
						//						g_sleepType = SLEEP_FOREVER;
						atomic_write_u16(&g_evteng_sleepshutdown_seconds, 3);
					}
				}
				else
				{
					//					g_sleepType = SLEEP_FOREVER;
					g_go_to_sleep_now = true;
				}
			}
			break;

			case SB_MESSAGE_TX_FREQ:
			{
				uint8_t printFreq = 0;
				atomic_write_u16(&g_report_settings_countdown, 0);
				char freqTier = sb_buff->fields[SB_FIELD1][0];
				char buf[TEMP_STRING_SIZE];

				if(freqTier)
				{
					Frequency_Hz f;
					if(g_cloningInProgress)
					{
						if(!frequencyVal(sb_buff->fields[SB_FIELD2], &f))
						{
							if(freqTier == 'L')
							{
								g_frequency_low = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Low, (void *)&f);
							}
							else if(freqTier == 'M')
							{
								g_frequency_med = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Med, (void *)&f);
							}
							else if(freqTier == 'H')
							{
								g_frequency_hi = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void *)&f);
							}
							else if(freqTier == 'B')
							{
								g_frequency_beacon = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Beacon, (void *)&f);
							}
							else
							{
								g_frequency = f;
								g_ee_mgr.updateEEPROMVar(Frequency, (void *)&f);
							}

							g_event_checksum += f;

							sb_send_string((char *)"FRE\n");
							atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						}
					}
					else if(!frequencyVal(sb_buff->fields[SB_FIELD2], &f)) // set freq based on first argument
					{
						if((freqTier == '1') || (freqTier == 'L'))
						{
							g_frequency_low = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Low, (void *)&f);
							printFreq = 1;
						}
						else if((freqTier == '2') || (freqTier == 'M'))
						{
							g_frequency_med = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Med, (void *)&f);
							printFreq = 2;
						}
						else if((freqTier == '3') || (freqTier == 'H'))
						{
							g_frequency_hi = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void *)&f);
							printFreq = 3;
						}
						else if(freqTier == 'B')
						{
							g_frequency_beacon = f;
							g_ee_mgr.updateEEPROMVar(Frequency_Beacon, (void *)&f);
							printFreq = 4;
						}
					}
					else if(!frequencyVal(sb_buff->fields[SB_FIELD1], &f)) // set freq based on current EVT and FOX
					{
						g_frequency = f;

						if(!txSetFrequency(&f, true))
						{
							if(getFoxSetting() == BEACON)
							{
								g_frequency_beacon = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Beacon, (void *)&f);
								printFreq = 4;
							}
							else if(g_event == EVENT_FOXORING)
							{
								if(getFoxSetting() == FOXORING_FOX1)
								{
									g_frequency_low = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Low, (void *)&f);
									printFreq = 1;
								}
								else if(getFoxSetting() == FOXORING_FOX2)
								{
									g_frequency_med = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Med, (void *)&f);
									printFreq = 2;
								}
								else if(getFoxSetting() == FOXORING_FOX3)
								{
									g_frequency_hi = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void *)&f);
									printFreq = 3;
								}
							}
							else if(g_event == EVENT_SPRINT)
							{
								if((getFoxSetting() >= SPRINT_S1) && (getFoxSetting() <= SPRINT_S5))
								{
									g_frequency_low = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Low, (void *)&f);
									printFreq = 1;
								}
								else if(getFoxSetting() == SPECTATOR)
								{
									g_frequency_med = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Med, (void *)&f);
									printFreq = 2;
								}
								else if((getFoxSetting() >= SPRINT_F1) && (getFoxSetting() <= SPRINT_F5))
								{
									g_frequency_hi = f;
									g_ee_mgr.updateEEPROMVar(Frequency_Hi, (void *)&f);
									printFreq = 3;
								}
							}
							else if(g_event == EVENT_CLASSIC)
							{
								g_frequency_low = f;
								g_ee_mgr.updateEEPROMVar(Frequency_Low, (void *)&f);
								printFreq = 1;
							}
							else // No event set
							{
								g_frequency = f;
								g_ee_mgr.updateEEPROMVar(Frequency, (void *)&f);
								printFreq = 5;
							}
						}
						else
						{
							sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
						}
					}
					else // neither field contains a valid frequency
					{
						if(!sb_buff->fields[SB_FIELD2][0]) // no second argument
						{
							if((freqTier == '1') || (freqTier == 'L'))
							{
								printFreq = 1;
							}
							else if((freqTier == '2') || (freqTier == 'M'))
							{
								printFreq = 2;
							}
							else if((freqTier == '3') || (freqTier == 'H'))
							{
								printFreq = 3;
							}
							else // if(freqTier == 'B')
							{
								printFreq = 4;
							}
						}
						else
						{
							printFreq = 6;
						}
					}
				}
				else
				{
					printFreq = 5;
				}

				if(!g_cloningInProgress)
				{
					if(printFreq && !g_meshmode)
						sb_send_NewLine();

					switch(printFreq)
					{
						case 0:
						{
						}
						break;

						case 1:
						{
							frequencyString(buf, g_frequency_low);
							sprintf(g_tempStr, "* FRE 1=%s\n", buf);
						}
						break;

						case 2:
						{
							frequencyString(buf, g_frequency_med);
							sprintf(g_tempStr, "* FRE 2=%s\n", buf);
						}
						break;

						case 3:
						{
							frequencyString(buf, g_frequency_hi);
							sprintf(g_tempStr, "* FRE 3=%s\n", buf);
						}
						break;

						case 4:
						{
							frequencyString(buf, g_frequency_beacon);
							sprintf(g_tempStr, "* FRE B=%s\n", buf);
						}
						break;

						case 5:
						{
							frequencyString(buf, getFrequencySetting());
							sprintf(g_tempStr, "* FRE=%s\n", buf);
						}
						break;

						default:
						{
							sprintf(g_tempStr, "* 3500 kHz < FRE < 4000 kHz\n");
						}
					}

					sb_send_string(g_tempStr);
				}
			}
			break;

			case SB_MESSAGE_PATTERN:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				if(g_cloningInProgress)
				{
					if(sb_buff->fields[SB_FIELD1][0] && sb_buff->fields[SB_FIELD2][0])
					{
						int len = MIN(MAX_PATTERN_TEXT_LENGTH, strlen(sb_buff->fields[SB_FIELD2]));

						for(int i = 0; i < len; i++)
						{
							g_event_checksum += sb_buff->fields[SB_FIELD2][i];
						}

						sb_send_string((char *)"PAT\n");
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					}
				}
				else
				{
					if(!g_meshmode)
						sb_send_NewLine();

					if(sb_buff->fields[SB_FIELD1][0])
					{
						if(strlen(sb_buff->fields[SB_FIELD1]) <= MAX_PATTERN_TEXT_LENGTH)
						{
							if(g_event == EVENT_FOXORING)
							{
								strncpy(g_tempStr, sb_buff->fields[SB_FIELD1], MAX_PATTERN_TEXT_LENGTH);
								messages_text_slot_write_atomic(FOXORING_PATTERN_TEXT, g_tempStr, strlen(g_tempStr));
								g_ee_mgr.updateEEPROMVar(Foxoring_pattern_text, g_messages_text[FOXORING_PATTERN_TEXT]);

								sb_send_string((char *)"* PAT:");
								sb_send_string(g_tempStr);
								sb_send_NewLine();
							}
							else
							{
								sb_send_string((char *)"* Ignored. Must set EVT F\n");
							}
						}
						else
						{
							sb_send_string((char *)"* Illegal pattern\n");
						}
					}
					else
					{
						strncpy(g_tempStr, g_messages_text[FOXORING_PATTERN_TEXT], MAX_PATTERN_TEXT_LENGTH);
						sb_send_string((char *)"* PAT:");
						sb_send_string(g_tempStr);
						sb_send_NewLine();
					}
				}
			}
			break;

			case SB_MESSAGE_KEY:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);

				if(g_device_enabled)
				{
					if(sb_buff->fields[SB_FIELD1][0])
					{
						if(sb_buff->fields[SB_FIELD1][0] == '0')
						{
							cancelManualTransientState();
							atomic_write_u16(&g_key_down_countdown, 0);
							g_foreground_reset_after_keydown = true;
							if(!g_meshmode)
								sb_send_NewLine();
							sb_send_string((char *)"* KEY UP\n");
						}
						else if(sb_buff->fields[SB_FIELD1][0] == '1')
						{
							cancelManualTransientState();
							setupForFox(USE_CURRENT_FOX, START_NOTHING); // Stop any running event

							if(!txIsInitialized())
							{
								if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
								{
									sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
								}
							}

							atomic_write_u16(&g_key_down_countdown, 9000);
							LEDS.init();
							LEDS.setRed(ON);
							keyTransmitter(ON);
							if(!g_meshmode)
								sb_send_NewLine();
							sb_send_string((char *)"* KEY DOWN\n");

							atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300); // Shut things down after 5 minutes
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
				if(g_cloningInProgress || g_isMaster)
					break;

				atomic_write_u16(&g_report_settings_countdown, 0);

				if(g_device_enabled)
				{
					char arg = sb_buff->fields[SB_FIELD1][0];

					if(arg)
					{
						if((arg == '0') || (arg == '1') || (arg == '2'))
						{
							cancelManualTransientState();
						}

						if(arg == '0') /* Stop an event in progress. Resume countdown to any future event */
						{
							suspendEvent(); // Stop any running event and initialize loaded event engine settings
							setupForFox(USE_CURRENT_FOX, START_NOTHING);
							g_frequency_to_test = NUMBER_OF_TEST_FREQUENCIES;
							g_event_launched_by_user_action = false;
							atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, 0, 0); // Allow the transmitter to sleep forever
						}
						else if(arg == '1') /* Start the event, syncing to the top of the hour */
						{
							g_event_launched_by_user_action = true;
							suspendEvent(); // Stop any running event and initialize loaded event engine settings
							startSyncdEventNow(true);
						}
						else if(arg == '2') /* Start the event at the programmed start time */
						{
							g_event_launched_by_user_action = false;
							suspendEvent();                 // Stop any running event and initialize loaded event engine settings
							g_evteng_event_enabled = false; /* Disable an event currently underway */
							startEventUsingRTC();
						}
						// 						else if(arg == '3')  /* Start the event immediately with transmissions starting now */
						// 						{
						// 							g_event_launched_by_user_action = true;
						// 							suspendEvent(); // Stop any running event and initialize loaded event engine settings
						// 							if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
						// 							{
						// 								sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
						// 							}
						//  							setupForFox(USE_CURRENT_FOX, START_TRANSMISSIONS_NOW);
						// 						}
						else
						{
							sb_send_string((char *)"*err\n");
						}
					}

					char c[12];
					event2Text(c, g_event);
					if(!arg)
						arg = '?';

					if(eventRunning())
					{
						if(g_evteng_run_event_until_canceled)
						{
							sprintf(g_tempStr, "* GO %c:%s; Running Forever", arg, c);
						}
						else
						{
							sprintf(g_tempStr, "* GO %c:%s; %s", arg, c, g_foreground_start_event ? "Starting" : "Running");
						}
					}
					else
					{
						time_t loaded_start_epoch;
						time_t loaded_finish_epoch;
						atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
						if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
						{
							sprintf(g_tempStr, "* GO %c:%s; Scheduled", arg, c);
						}
						else
						{
							sprintf(g_tempStr, "* GO %c:%s; Stopped", arg, c);
						}
					}

					if(!g_meshmode)
					{
						sb_send_NewLine();
					}

					sb_send_string(g_tempStr);
					sb_send_NewLine();
				}
				else
				{
					sb_send_string(TEXT_DEVICE_DISABLED_TXT);
				}
			}
			break;

			case SB_MESSAGE_SET_STATION_ID:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				if(sb_buff->fields[SB_FIELD1][0])
				{
					int len = 0;

					if((sb_buff->fields[SB_FIELD1][0] == '\"') && (sb_buff->fields[SB_FIELD1][1] == '\"')) /* "" */
					{
						messages_text_slot_clear_atomic(STATION_ID);
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
						messages_text_slot_write_atomic(STATION_ID, g_tempStr, len);
					}

					if(g_cloningInProgress)
					{
						for(int i = 0; i < len; i++)
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
					sb_send_string((char *)"ID\n");
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
				}
				else
				{
					sprintf(g_tempStr, "* ID:%s\n", g_messages_text[STATION_ID]);

					if(!g_meshmode)
					{
						sb_send_NewLine();
					}

					sb_send_string(g_tempStr);
				}
			}
			break;

			case SB_MESSAGE_CODE_SETTINGS:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				char c = sb_buff->fields[SB_FIELD1][0];
				char x = sb_buff->fields[SB_FIELD2][0];

				if(c)
				{
					if(isdigit(x))
					{
						uint8_t speed = atol(sb_buff->fields[SB_FIELD2]);

						if(c == 'I')
						{
							g_evteng_id_codespeed = speed;
							g_ee_mgr.updateEEPROMVar(Id_codespeed, (void *)&g_evteng_id_codespeed);

							if(g_messages_text[STATION_ID][0])
							{
								g_time_needed_for_ID = timeNeededForID();
							}

							if(g_cloningInProgress)
							{
								g_event_checksum += speed;
								sb_send_string((char *)"SPD I\n");
							}
						}
						else if((c == 'F') || ((g_event == EVENT_FOXORING) && !g_cloningInProgress))
						{
							g_foxoring_pattern_codespeed = speed;
							g_ee_mgr.updateEEPROMVar(Foxoring_Pattern_Code_Speed, (void *)&g_foxoring_pattern_codespeed);
							if(g_evteng_event_commenced)
								g_evteng_code_throttle = throttleValue(getPatternCodeSpeed());

							if(g_cloningInProgress)
							{
								g_event_checksum += speed;
								sb_send_string((char *)"SPD F\n");
							}
						}
						else if(c == 'P')
						{
							g_evteng_pattern_codespeed = speed;
							g_ee_mgr.updateEEPROMVar(Pattern_Code_Speed, (void *)&g_evteng_pattern_codespeed);

							if(g_cloningInProgress)
							{
								g_event_checksum += speed;
								sb_send_string((char *)"SPD P\n");
							}
						}
						else
						{
							c = '\0';
							sprintf(g_tempStr, "* err\n");
						}
					}

					if(!g_cloningInProgress && c)
					{
						if(!g_meshmode)
						{
							sb_send_NewLine();
						}

						if(c == 'I')
						{
							sprintf(g_tempStr, "* ID SPD:%u WPM\n", g_evteng_id_codespeed);
						}
						else if(c == 'P')
						{
							sprintf(g_tempStr, "* PAT SPD:%u WPM\n", getFoxCodeSpeed());
						}
						else if(c == 'F')
						{
							sprintf(g_tempStr, "* FOX-O SPD:%u WPM\n", g_foxoring_pattern_codespeed);
						}

						sb_send_string(g_tempStr);
					}
				}
				else
				{
					sprintf(g_tempStr, "* err\n");
				}
			}
			break;

			case SB_MESSAGE_MASTER:
			{
				if(!g_meshmode)
				{
					if(sb_buff->fields[SB_FIELD1][0] == 'P') // Target receives clone command from source
					{
						/* Ignore repeated clone probes once a timed event has already been launched so an idle
						 * master cannot suspend and overwrite a target that is already running. */
						if(!atomic_read_u16(&g_send_clone_success_countdown) && !g_evteng_event_commenced)
						{
							g_cloningInProgress = true;
							g_defer_cloned_event_start = false;
							cancelManualTransientState();
							suspendEvent();
							atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
							g_event_checksum = 0;
							sb_send_string((char *)"MAS S\n");
						}
					}
					else if((sb_buff->fields[SB_FIELD1][0] == 'Q') && g_cloningInProgress)
					{
						uint32_t sum = atol(sb_buff->fields[SB_FIELD2]);
						if(sum == atomic_read_u32(&g_event_checksum))
						{
							/* Acknowledge the clone before restarting the event so the target
							 * stays in clone-safe parsing mode until the transfer fully completes. */
							sb_send_string((char *)"MAS ACK\n");
							atomic_write_u16(&g_send_clone_success_countdown, 18000);
							resumeLoadedEventAfterCloneExit(true);
						}
						else
						{
							resumeLoadedEventAfterCloneExit(false);
							sb_send_string((char *)"MAS NAK\n");
						}

						g_cloningInProgress = false;
						atomic_write_u16(&g_programming_countdown, 0);
					}
					else
					{
						if(sb_buff->fields[SB_FIELD1][0])
						{
							if((sb_buff->fields[SB_FIELD1][0] == 'M') || (sb_buff->fields[SB_FIELD1][0] == '1'))
							{
								g_isMaster = true;
								atomic_write_u16(&g_evteng_sleepshutdown_seconds, 720); /* Never sleep while master; slave timeout starts after master ends */
								atomic_write_u16(&isMasterCountdownSeconds, 600);       /* Remain Master for 10 minutes */
							}
						}
					}
				}
			}
			break;

			case SB_MESSAGE_EVENT:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				if(g_cloningInProgress)
				{
					char c = sb_buff->fields[SB_FIELD1][0];

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

					g_ee_mgr.updateEEPROMVar(Event_setting, (void *)&g_event);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					g_event_checksum += c;
					sb_send_string((char *)"EVT\n");
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
						cancelManualTransientState();
						g_ee_mgr.updateEEPROMVar(Event_setting, (void *)&g_event);
						if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
						{
							sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
						}
						reinitializeEventEngine();
						setupForFox(getFoxSetting(), START_EVENT_WITH_STARTFINISH_TIMES);
					}
				}

				if(!g_cloningInProgress)
				{
					char buf[TEMP_STRING_SIZE];
					if(!g_meshmode)
					{
						sb_send_NewLine();
					}

					if(!event2Text(g_tempStr, g_event))
					{
						strncpy(buf, g_tempStr, TEMP_STRING_SIZE);
						sprintf(g_tempStr, "* Event:%s\n", buf);
					}
					else
					{
						sprintf(g_tempStr, "* Event:err\n");
					}

					sb_send_string(g_tempStr);

					time_t now = time(null);
					if(g_event_launched_by_user_action)
					{
						time_t loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
						sb_send_string((char *)"* User launched. \n");
						if(g_evteng_run_event_until_canceled)
						{
							sb_send_string((char *)"* Running forever.\n");
						}
						else if(loaded_finish_epoch > now)
						{
							reportTimeTill(now, loaded_finish_epoch, "* Time remaining: ", NULL);
						}
						else
						{
							sb_send_string((char *)"* Config err 1\n");
						}

						if(g_evteng_on_the_air > 0)
						{
							sb_send_string((char *)"* On the air.\n");
						}
						else
						{
							sprintf(g_tempStr, "* On the air in %d seconds.\n", (int)-g_evteng_on_the_air);
							sb_send_string(g_tempStr);
						}

						if(!g_evteng_event_enabled)
						{
							sb_send_string((char *)"* Event interrupted!\n");
						}
					}
					else if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch))
					{
						time_t loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
						time_t loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
						reportTimeTill(now, loaded_start_epoch, "* Starts in: ", "* In progress\n");
						reportTimeTill(loaded_start_epoch, loaded_finish_epoch, "* Lasts: ", null);
						if(loaded_start_epoch < now)
						{
							reportTimeTill(now, loaded_finish_epoch, "* Time Remaining: ", NULL);
						}

						if(!g_evteng_event_enabled)
						{
							sb_send_string((char *)"* Event interrupted!\n");
						}
					}
					else
					{
						sb_send_string((char *)"* Not scheduled\n");
					}
				}
			}
			break;

			case SB_MESSAGE_FUNCTION:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
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
					g_ee_mgr.updateEEPROMVar(Function, (void *)&g_function);
				}

				if(g_cloningInProgress)
				{
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					g_event_checksum += 'A';
					sprintf(g_tempStr, "FUN A\n");
					sb_send_string(g_tempStr);
				}
				else
				{
					if(!g_meshmode)
						reportSettings();
				}
			}
			break;

			case SB_MESSAGE_CLOCK:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				char f1 = sb_buff->fields[SB_FIELD1][0];

				if(!f1 || f1 == 'T') /* Current time format "YYMMDDhhmmss" */
				{
					if((f1 == 'T') && sb_buff->fields[SB_FIELD2][0])
					{
						strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
						time_t t;

						if(g_cloningInProgress)
						{
							t = atol(g_tempStr);
						}
						else
						{
							g_tempStr[12] = '\0';
							const char *tmp = completeTimeString(g_tempStr, null);
							if(tmp)
								strncpy(g_tempStr, tmp, 13);

							t = validateTimeString(g_tempStr, g_tempStr);

							if(!t)
							{
								sb_send_NewLine();
								sb_send_string(g_tempStr);
							}
						}

						if(t)
						{
							bool time_was_valid = timeIsSet();
							atomic_set_system_time(t, true);

							if(g_cloningInProgress)
							{
								atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
								g_event_checksum += t;
								sprintf(g_tempStr, "CLK T %lu\n", t);
								sb_send_string(g_tempStr);
							}
							else
							{
								bool pending_start_after_keydown = cancelManualTransientState();
								bool had_active_or_pending_event = g_evteng_event_commenced || g_evteng_event_enabled || pending_start_after_keydown;

								if(had_active_or_pending_event)
								{
									/* CLK T should re-sync immediately, not after a stale keydown/demo timeout expires. */
									suspendEvent();
								}

								/* Start the event if one is configured */
								if((!time_was_valid && catchUpLoadedMultiDayEventAfterClockSet()) ||
								   (time_was_valid && eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch)))
								{
									startEventUsingRTC();
								}
								else if(had_active_or_pending_event) // keydown can imply a pending synced event start
								{
									startSyncdEventNow(true);
								}
								else
								{
									suspendEvent();
								}
							}
						}
					}

					if(!g_cloningInProgress)
					{
						char buf[TEMP_STRING_SIZE];
						if(!g_meshmode)
							sb_send_NewLine();

						time_t t = time(null);

						if(t < MINIMUM_VALID_EPOCH)
						{
							sprintf(g_tempStr, "* Time:not set\n");
						}
						else
						{
							sprintf(g_tempStr, "* Time:%s\n", convertEpochToTimeString(t, buf, TEMP_STRING_SIZE));
						}

						sb_send_string(g_tempStr);

						if(!f1)
						{
							char buf[TEMP_STRING_SIZE];
							time_t loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
							time_t loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
							uint8_t days_remaining = (g_days_run < g_days_to_run) ? (uint8_t)(g_days_to_run - g_days_run) : 0;

							if(g_days_to_run > 1)
							{
								eventIsScheduledToRun(&loaded_start_epoch, &loaded_finish_epoch);
							}
							//							if(!g_meshmode) sb_send_NewLine();

							if(loaded_start_epoch < MINIMUM_VALID_EPOCH)
							{
								sprintf(g_tempStr, "* Start:not set\n");
							}
							else
							{
								sprintf(g_tempStr, "* Start:%s\n", convertEpochToTimeString(loaded_start_epoch, buf, TEMP_STRING_SIZE));
							}

							sb_send_string(g_tempStr);

							//							if(!g_meshmode) sb_send_NewLine();

							if(loaded_finish_epoch < MINIMUM_VALID_EPOCH)
							{
								sprintf(g_tempStr, "* Finish:not set\n");
							}
							else
							{
								sprintf(g_tempStr, "* Finish:%s\n", convertEpochToTimeString(loaded_finish_epoch, buf, TEMP_STRING_SIZE));
							}

							sb_send_string(g_tempStr);

							//							if(!g_meshmode) sb_send_NewLine();
							sprintf(g_tempStr, "* Days to run: %d\n", days_remaining ? days_remaining : g_days_to_run);
							sb_send_string(g_tempStr);
						}
					}
				}
				else if(f1 == 'S') /* Event start time */
				{
					if(sb_buff->fields[SB_FIELD2][0])
					{
						strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
						time_t s;
						bool setSequalF = false;
						bool explicitMirrorToFinish = false;

						if(g_cloningInProgress)
						{
							s = atol(g_tempStr);
						}
						else
						{
							char msg[TEMP_STRING_SIZE + 1];
							char rawStartInput[13];
							msg[0] = '\0';
							strncpy(rawStartInput, g_tempStr, sizeof(rawStartInput) - 1);
							rawStartInput[sizeof(rawStartInput) - 1] = '\0';
							g_tempStr[12] = '\0';
							g_tempStr[11] = '0'; // start time seconds are always zero
							g_tempStr[10] = '0';

							if(g_tempStr[0] == '=')
							{
								s = atomic_read_time(&g_event_finish_epoch);
								setSequalF = true;
								explicitMirrorToFinish = true;
							}
							else
							{
								const char *tmp = completeTimeString_volatile(g_tempStr, &g_evteng_loaded_start_epoch);
								if(tmp)
									strncpy(g_tempStr, tmp, 13);
								s = validateTimeString(g_tempStr, &g_event_start_epoch, (g_event == EVENT_CLASSIC), msg, rawStartInput);
							}

							if(msg[0] != '\0')
							{
								sb_send_NewLine();
								sb_send_string(msg);
							}
						}

						if(s || g_cloningInProgress || explicitMirrorToFinish)
						{
							atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_event_start_epoch, s, s);
							g_ee_mgr.updateEEPROMVar(Event_start_epoch, (void *)&g_event_start_epoch);

							if(g_cloningInProgress)
							{
								sb_send_string((char *)"CLK S\n");
								atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
								g_event_checksum += s;
							}
							else
							{
								cancelManualTransientState();
								g_days_to_run = 1;
								g_days_run = 0;

								if(!setSequalF)
								{
									time_t event_start_epoch;
									time_t event_finish_epoch;
									atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &event_start_epoch, &event_finish_epoch);
									time_t new_finish_epoch = MAX(event_finish_epoch, (event_start_epoch + SECONDS_24H));
									atomic_write_time_pair(&g_evteng_loaded_finish_epoch, &g_event_finish_epoch, new_finish_epoch, new_finish_epoch);

									g_ee_mgr.updateEEPROMVar(Event_finish_epoch, (void *)&g_event_finish_epoch);
									startEventUsingRTC();
								}
							}
						}
					}

					if(!g_cloningInProgress)
					{
						char buf[TEMP_STRING_SIZE];
						time_t loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
						if(!g_meshmode)
							sb_send_NewLine();

						if(loaded_start_epoch < MINIMUM_VALID_EPOCH)
						{
							sprintf(g_tempStr, "* Start:not set\n");
						}
						else
						{
							sprintf(g_tempStr, "* Start:%s\n", convertEpochToTimeString(loaded_start_epoch, buf, TEMP_STRING_SIZE));
						}

						sb_send_string(g_tempStr);
					}
				}
				else if(f1 == 'F') /* Event finish time */
				{
					if(sb_buff->fields[SB_FIELD2][0])
					{
						strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
						time_t f;
						bool explicitMirrorToStart = false;

						if(g_cloningInProgress)
						{
							f = atol(g_tempStr);
						}
						else
						{
							g_tempStr[12] = '\0';

							if(g_tempStr[0] == '=')
							{
								f = atomic_read_time(&g_event_start_epoch);
								explicitMirrorToStart = true;
							}
							else if(g_tempStr[0] == '+')
							{
								parseFinishOffsetToEpoch(g_tempStr, &f, g_tempStr);
							}
							else
							{
								g_tempStr[11] = '0'; // finish time seconds are always zero
								g_tempStr[10] = '0';
								const char *tmp = completeTimeString_volatile(g_tempStr, &g_event_finish_epoch);
								if(tmp)
									strncpy(g_tempStr, tmp, 13);
								f = validateTimeString(g_tempStr, &g_event_finish_epoch, false, g_tempStr);
							}

							if(!f)
							{
								sb_send_NewLine();
								sb_send_string(g_tempStr);
							}
						}

						if(f || g_cloningInProgress || explicitMirrorToStart)
						{
							atomic_write_time_pair(&g_event_finish_epoch, &g_evteng_loaded_finish_epoch, f, f);

							g_ee_mgr.updateEEPROMVar(Event_finish_epoch, (void *)&g_event_finish_epoch);

							if(g_cloningInProgress)
							{
								sb_send_string((char *)"CLK F\n");
								atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
								g_event_checksum += f;
							}
							else
							{
								cancelManualTransientState();
								g_days_to_run = 1;
								g_days_run = 0;
								startEventUsingRTC();
							}
						}
					}

					if(!g_cloningInProgress)
					{
						char buf[TEMP_STRING_SIZE];
						time_t event_finish_epoch = atomic_read_time(&g_event_finish_epoch);
						if(!g_meshmode)
							sb_send_NewLine();

						if(event_finish_epoch < MINIMUM_VALID_EPOCH)
						{
							sprintf(g_tempStr, "* Finish:not set\n");
						}
						else
						{
							sprintf(g_tempStr, "* Finish:%s\n", convertEpochToTimeString(event_finish_epoch, buf, TEMP_STRING_SIZE));
						}

						sb_send_string(g_tempStr);
					}
				}
				else if(f1 == 'D') /* Run the event multiple days */
				{
					if(sb_buff->fields[SB_FIELD2][0])
					{
						strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
						uint8_t days;

						if(only_digits((char *)g_tempStr))
						{
							days = atoi(g_tempStr);
						}
						else
						{
							days = g_days_to_run;
						}

						if(days > 1)
						{
							g_days_to_run = days;
						}
						else
						{
							g_days_to_run = 1;
						}

						g_days_run = 0;

						g_ee_mgr.updateEEPROMVar(Days_to_run, (void *)&g_days_to_run);

						if(g_cloningInProgress)
						{
							sb_send_string((char *)"CLK D\n");
							atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
							g_event_checksum += g_days_to_run;
						}
						else
						{
							cancelManualTransientState();
							time_t event_start_epoch;
							time_t event_finish_epoch;
							atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &event_start_epoch, &event_finish_epoch);
							atomic_write_time(&g_event_finish_epoch, MIN(event_start_epoch + DAY - HOUR, event_finish_epoch));

							g_ee_mgr.updateEEPROMVar(Event_start_epoch, (void *)&g_event_start_epoch);
							g_ee_mgr.updateEEPROMVar(Event_finish_epoch, (void *)&g_event_finish_epoch);

							atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &event_start_epoch, &event_finish_epoch);
							eventIsScheduledToRun(&event_start_epoch, &event_finish_epoch);
							atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, event_start_epoch, event_finish_epoch);
						}
					}

					if(!g_cloningInProgress)
					{
						if(!g_meshmode)
							sb_send_NewLine();
						sprintf(g_tempStr, "* Days to run: %d\n", g_days_to_run);
						sb_send_string(g_tempStr);
					}
				}
				else if(f1 == 'C' && !g_cloningInProgress) /* Clock calibration */
				{
					strncpy(g_tempStr, sb_buff->fields[SB_FIELD2], 12);
					uint16_t cal;

					cal = atoi(g_tempStr);
					RTC_set_calibration(cal);
				}
			}
			break;

			case SB_MESSAGE_BATTERY:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				char txt[6];

				if(sb_buff->fields[SB_FIELD1][0] == 'T')
				{
					float v = atof(sb_buff->fields[SB_FIELD1]);

					if((v >= INT_BAT_CHARGE_THRESH_LOW_MIN) && (v <= INT_BAT_CHARGE_THRESH_LOW_MAX))
					{
						g_internal_voltage_low_threshold = v;
						g_ee_mgr.updateEEPROMVar(Voltage_threshold, (void *)&g_internal_voltage_low_threshold);
					}
					else
					{
						int16_t d1, d2;
						uint16_t f1, f2;
						float_to_parts_signed(INT_BAT_CHARGE_THRESH_LOW_MIN, &d1, &f1);
						float_to_parts_signed(INT_BAT_CHARGE_THRESH_LOW_MAX, &d2, &f2);
						sprintf(g_tempStr, "\n* Err: %d.%u V < thresh < %d.%u V\n", d1, f1, d2, f2);
						sb_send_string(g_tempStr);
					}
				}
				else if(sb_buff->fields[SB_FIELD1][0] == 'X')
				{
					char c = (char)sb_buff->fields[SB_FIELD2][0];
					bool updateStoredValue = false;
					bool should_reapply_controlled_power =
					    g_device_enabled &&
					    (g_foreground_enable_transmitter ||
					     get_V3V3_enable() ||
					     get_fet_driver() ||
					     txIsInitialized() ||
					     (g_evteng_event_enabled && (g_evteng_event_commenced || g_evteng_run_event_until_canceled)));

					if(c == '1') // Control the external battery to connect it for transmissions and internal battery charging
					{
						setDisableTransmissions(false);
						g_enable_external_battery_control = true;
						updateStoredValue = true;
					}
					else if(c == '2') // Control the external battery to connect it for charging the internal battery, but disable transmissions
					{
						setDisableTransmissions(true);
						g_enable_external_battery_control = true;
						updateStoredValue = true;
					}
					else if(c != '\0') // On legacy hardware, repurpose the shared auxiliary switch for fan control instead of external-battery control.
					{
						setDisableTransmissions(false);
						g_enable_external_battery_control = false;
						updateStoredValue = true;
					}

					if(updateStoredValue)
					{
						setExtBatLoadSwitch(OFF, INITIALIZE_LS);
						g_ee_mgr.updateEEPROMVar(Enable_External_Battery_Control, (void *)&g_enable_external_battery_control);

						if(g_enable_external_battery_control && should_reapply_controlled_power)
						{
							powerToTransmitter(ON);
						}
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
				g_internal_bat_voltage = readVoltage(ADCInternalBatteryVoltage);
				g_external_voltage = readVoltage(ADCExternalBatteryVoltage);
				float internal_bat_voltage = atomic_read_float(&g_internal_bat_voltage);
				float internal_voltage_low_threshold = atomic_read_float(&g_internal_voltage_low_threshold);
				float external_voltage = atomic_read_float(&g_external_voltage);

				if(!g_meshmode)
					sb_send_NewLine();

				dtostrf(internal_bat_voltage, 5, 1, txt);
				txt[5] = '\0';
				sprintf(g_tempStr, "* Int. Bat =%s Volts\n", txt);
				sb_send_string(g_tempStr);

				dtostrf(internal_voltage_low_threshold, 5, 1, txt);
				txt[5] = '\0';
				sprintf(g_tempStr, "* thresh   =%s Volts\n", txt);
				sb_send_string(g_tempStr);

				dtostrf(external_voltage, 5, 1, txt);
				txt[5] = '\0';
				sprintf(g_tempStr, "* Ext. Bat =%s Volts\n", txt);
				sb_send_string(g_tempStr);

				sprintf(g_tempStr, "* Ext. Bat. Ctrl = %s\n", g_enable_external_battery_control ? "ON" : "OFF");
				sb_send_string(g_tempStr);
				sprintf(g_tempStr, "* Transmitter = %s\n", getDisableTransmissions() ? "Disabled" : "Enabled");
				sb_send_string(g_tempStr);
			}
			break;

			case SB_MESSAGE_VER:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				if(!g_cloningInProgress)
				{
					if(!g_meshmode)
					{
						sb_send_NewLine();
						sb_send_string((char *)"* ");
						sb_send_string((char *)PRODUCT_NAME_LONG);
						sb_send_NewLine();
					}

					// Buffer for storing temporary strings.
					char buf[10];

#ifdef HW_TARGET_3_5
					sprintf(buf, "3.5");
#else
					sprintf(buf, "3.4");
#endif
					sprintf(g_tempStr, "* SW Ver: %s HW Build: %s\n", SW_REVISION, buf);

					sb_send_string(g_tempStr);
				}
			}
			break;

			case SB_MESSAGE_HELP:
			{
				atomic_write_u16(&g_report_settings_countdown, 0);
				if(!g_cloningInProgress && !g_meshmode)
				{
					reportSettings();
					sb_send_string(HELP_TEXT_TXT);
				}
			}
			break;

			case SB_CR_NO_DATA:
			{
				if(!g_cloningInProgress && !g_meshmode)
				{
					atomic_write_u16(&g_report_settings_countdown, 100);
				}
			}
			break;

			default:
			{
				if(!g_cloningInProgress && !atomic_read_u16(&g_report_settings_countdown) && !g_meshmode)
				{
					sb_send_string(HELP_TEXT_TXT);
				}

				atomic_write_u16(&g_report_settings_countdown, 0);
			}
			break;
		}

		sb_buff->id = SB_MESSAGE_EMPTY;
		if(!g_cloningInProgress && !suppressResponse && !atomic_read_u16(&g_report_settings_countdown) && !g_meshmode)
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
bool __attribute__((optimize("O0"))) loadedEventShouldBeEnabled()
{
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;
	time_t time_to_wake_up;

	if(g_evteng_run_event_until_canceled)
	{
		g_sleepType = SLEEP_AFTER_EVENT;
		return (true);
	}

	g_go_to_sleep_now = false;

	if(!eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch)) /* A future event has not been set, and no event is scheduled to run right now */
	{
		g_sleepType = SLEEP_FOREVER;
		atomic_write_time(&g_time_to_wake_up, FOREVER_EPOCH);
		atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
		return (false); /* completed events are never enabled */
	}

	loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
	loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
	time_to_wake_up = atomic_read_time(&g_time_to_wake_up);

	time_t now = time(null);
	int32_t dif = timeDif(now, loaded_start_epoch);

	if(eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch)) // An event should be running right now
	{
		if(g_sleepType == SLEEP_UNTIL_NEXT_XMSN)
		{
			if(time_to_wake_up > now) // Wake-up for the next transmission is scheduled for the future
			{
				return (true); // return without making any changes to sleep type or other settings that could affect waking up for the next transmission
			}
		}
	}

	atomic_write_time(&g_time_to_wake_up, loaded_start_epoch - 15); /* sleep time needs to be calculated to allow time for power-up (coming out of sleep) prior to the event start */

	if(dif >= -30) /* Don't sleep if the event starts in 30 seconds or less, or has already started */
	{
		if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
		{
			sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
		}
		g_sleepType = SLEEP_AFTER_EVENT;
		atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
		return (true);
	}

	g_sleepType = SLEEP_UNTIL_START_TIME;
	/* If we reach here, we have an event that will not start for at least 30 seconds. */
	return (true);
}

uint16_t throttleValue(uint8_t speed)
{
	float temp;
	speed = CLAMP(5, (int8_t)speed, 20);
	temp = (3544L / (uint16_t)speed) / 10L; /* tune numerator to achieve "PARIS " sent 8 times in 60 seconds at 8 WPM */
	return ((uint16_t)temp);
}

EC __attribute__((optimize("O0"))) launchLoadedEvent(SC *statusCode)
{
	time_t loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
	time_t loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
	EC ec = activateEventEngineUsingCurrentSettings(statusCode, loaded_start_epoch, loaded_finish_epoch);

	if(ec)
	{
		g_last_error_code = ec;
	}
	else
	{
		g_evteng_event_enabled = loadedEventShouldBeEnabled();
		configRedLEDforEvent();
	}

	return (ec);
}

EC activateEventEngineUsingCurrentSettings(SC *statusCode, time_t startTime, time_t finishTime)
{
	time_t now = time(null);

	/* Make sure everything has been sanely initialized */
	if(!g_evteng_run_event_until_canceled)
	{
		if(now < MINIMUM_VALID_EPOCH) /* The RTC has not been set */
		{
			return (ERROR_CODE_EVENT_NOT_CONFIGURED);
		}

		if(!startTime)
		{
			return (ERROR_CODE_EVENT_MISSING_START_TIME);
		}

		if(startTime >= finishTime) /* Finish must be later than start */
		{
			return (ERROR_CODE_EVENT_NOT_CONFIGURED);
		}
	}

	if(!g_evteng_on_air_seconds)
	{
		return (ERROR_CODE_EVENT_MISSING_TRANSMIT_DURATION);
	}

	if(g_evteng_intra_cycle_delay_time > (g_evteng_off_air_seconds + g_evteng_on_air_seconds))
	{
		return (ERROR_CODE_EVENT_TIMING_ERROR);
	}

	char *c = getCurrentPatternText();
	if(c[0] == '\0')
	{
		return (ERROR_CODE_EVENT_PATTERN_NOT_SPECIFIED);
	}

	if((g_evteng_pattern_codespeed < MIN_CODE_SPEED_WPM) || (g_evteng_pattern_codespeed > MAX_CODE_SPEED_WPM))
	{
		return (ERROR_CODE_EVENT_PATTERN_CODE_SPEED_NOT_SPECIFIED);
	}

	if(g_messages_text[STATION_ID][0] != '\0')
	{
		if((!g_evteng_id_codespeed || !g_evteng_ID_period_seconds))
		{
			return (ERROR_CODE_EVENT_STATION_ID_ERROR);
		}

		g_time_needed_for_ID = timeNeededForID();
	}
	else
	{
		g_time_needed_for_ID = 0; /* ID will never be sent */
	}

	g_frequency = getFrequencySetting();
	txSetFrequency(&g_frequency, true);

	if(getFoxSetting() == FREQUENCY_TEST_BEACON)
	{
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
		{
			sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
		}
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		g_evteng_on_the_air = g_evteng_on_air_seconds;
		LEDS.init();
	}
	else if(!g_evteng_run_event_until_canceled && (finishTime < now)) /* the event has already finished */
	{
		if(statusCode)
		{
			*statusCode = STATUS_CODE_NO_EVENT_TO_RUN;
		}
	}
	else
	{
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		g_evteng_code_throttle = throttleValue(getFoxCodeSpeed());
		bool repeat = true;
		loadCurrentPatternMorse(&repeat, CALLER_AUTOMATED_EVENT);

		if(g_evteng_run_event_until_canceled)
		{
			g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
			g_evteng_on_the_air = g_evteng_on_air_seconds; // Start transmitting right away, regardless of g_evteng_off_air_seconds
			g_evteng_sendID_seconds_countdown = g_evteng_on_air_seconds - g_time_needed_for_ID;
			LEDS.blink(LEDS_RED_OFF);
			g_evteng_event_enabled = true;
			if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
			{
				sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
			}
		}
		else
		{
			int32_t dif = timeDif(now, startTime); /* returns arg1 - arg2 */

			if(dif >= 0) /* start time is in the past */
			{
				bool turnOnTransmitter = false;
				int cyclePeriod = g_evteng_on_air_seconds + g_evteng_off_air_seconds;
				int secondsIntoCycle = dif % cyclePeriod;
				int timeTillTransmit = g_evteng_intra_cycle_delay_time - secondsIntoCycle;

				g_evteng_event_commenced = true;

				if(timeTillTransmit <= 0) /* we should have started transmitting already in this cycle */
				{
					if(g_evteng_on_air_seconds <= -timeTillTransmit) /* we should have finished transmitting in this cycle */
					{
						g_evteng_on_the_air = -(cyclePeriod + timeTillTransmit);
						if(statusCode)
						{
							*statusCode = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
						}

						if(!g_evteng_event_enabled)
						{
							g_evteng_sendID_seconds_countdown = (g_evteng_on_air_seconds - g_evteng_on_the_air) - g_time_needed_for_ID;
						}
					}
					else /* we should be transmitting right now */
					{
						g_evteng_on_the_air = g_evteng_on_air_seconds + timeTillTransmit;
						turnOnTransmitter = true;
						if(statusCode)
						{
							*statusCode = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
						}

						if(!g_evteng_event_enabled)
						{
							if(g_time_needed_for_ID < g_evteng_on_the_air)
							{
								g_evteng_sendID_seconds_countdown = g_evteng_on_the_air - g_time_needed_for_ID;
							}
						}
					}
				}
				else /* it is not yet time to transmit in this cycle */
				{
					g_evteng_on_the_air = -timeTillTransmit;
					if(statusCode)
					{
						*statusCode = STATUS_CODE_EVENT_STARTED_WAITING_FOR_TIME_SLOT;
					}

					if(!g_evteng_event_enabled)
					{
						g_evteng_sendID_seconds_countdown = timeTillTransmit + g_evteng_on_air_seconds - g_time_needed_for_ID;
					}
				}

				atomic_write_time(&g_time_to_wake_up, now); // Don't sleep during the first cycle
				g_sleepType = SLEEP_AFTER_EVENT;

				atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);

				if(powerToTransmitter(turnOnTransmitter) != ERROR_CODE_NO_ERROR)
				{
					sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
				}

				atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, startTime, finishTime);
				LEDS.init();
			}
			else /* start time is in the future */
			{
				if(statusCode)
				{
					*statusCode = STATUS_CODE_WAITING_FOR_EVENT_START;
				}
				keyTransmitter(OFF);
				powerToTransmitter(OFF);
				atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, startTime, finishTime);
				g_evteng_event_commenced = false;
				atomic_write_time(&g_time_to_wake_up, (time_t)(startTime - 15)); // Wake up before the event starts
				g_sleepType = SLEEP_UNTIL_START_TIME;
				atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
			}
		}
	}

	return (ERROR_CODE_NO_ERROR);
}

void suspendEvent()
{
	keyTransmitter(OFF);
	setupForFox(USE_CURRENT_FOX, START_NOTHING); // Stop any running event
	LEDS.setRed(OFF);
	g_evteng_event_enabled = false;   /* get things stopped immediately */
	g_evteng_on_the_air = 0;          /* stop transmitting */
	g_evteng_event_commenced = false; /* get things stopped immediately */
	g_evteng_run_event_until_canceled = false;
	atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
	reloadLoadedEventWindowFromSavedSettings();
	powerToTransmitter(OFF);
	g_sleepType = SLEEP_FOREVER; /* Prevent the clock from restarting an event that is in progress */
	configRedLEDforEvent();
}

void startTransmissionsNow(bool configOverride)
{
	ConfigurationState_t conf = clockConfigurationCheck(LOADED_SETTINGS);

	if(configOverride || (conf != CONFIGURATION_ERROR))
	{
		reinitializeEventEngine();

		setupForFox(USE_CURRENT_FOX, START_TRANSMISSIONS_NOW); /* Let the RTC start the event */
	}

	configRedLEDforEvent();
}

void startEventNow(bool configOverride)
{
	ConfigurationState_t conf = clockConfigurationCheck(LOADED_SETTINGS);

	if(configOverride || (conf != CONFIGURATION_ERROR))
	{
		reinitializeEventEngine();

		setupForFox(USE_CURRENT_FOX, START_EVENT_NOW_AND_RUN_FOREVER); /* Let the RTC start the event */
	}

	configRedLEDforEvent();
}

void startSyncdEventNow(bool configOverride)
{
	ConfigurationState_t conf = clockConfigurationCheck(LOADED_SETTINGS);

	if(configOverride || (conf != CONFIGURATION_ERROR))
	{
		reinitializeEventEngine();

		setupForFox(USE_CURRENT_FOX, START_EVENT_NOW_AND_RUN_AS_TIMED_EVENT); /* Let the RTC start the event */
	}

	configRedLEDforEvent();
}

bool startEventUsingRTC(void)
{
	bool err = false;
	ConfigurationState_t conf = clockConfigurationCheck(LOADED_SETTINGS);

	if(conf != CONFIGURATION_ERROR)
	{
		reinitializeEventEngine();

		setupForFox(USE_CURRENT_FOX, START_EVENT_WITH_STARTFINISH_TIMES);
		{
			time_t loaded_start_epoch;
			time_t loaded_finish_epoch;
			atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
			if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
			{
				powerToTransmitter(OFF);
			}
			else
			{
				if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
				{
					sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
				}
			}
		}
	}
	else
	{
		err = true;
		reportConfigErrors(LOADED_SETTINGS);
	}

	configRedLEDforEvent();

	return err;
}

void restoreStateAfterButtonWakeAuthorization(void)
{
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;
	time_t now = time(null);
	atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);

	switch((SleepType)g_button_wake_prior_sleep_type)
	{
		case SLEEP_UNTIL_NEXT_XMSN:
		{
			g_evteng_event_enabled = g_button_wake_prior_event_enabled;
			g_evteng_event_commenced = g_button_wake_prior_event_commenced;

			if(finishTimedEventIfExpired(now))
			{
				break;
			}

			if(eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch))
			{
				g_sleepType = SLEEP_AFTER_EVENT;
				g_evteng_event_enabled = true;
				g_evteng_event_commenced = true;
			}
			else if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
			{
				g_sleepType = SLEEP_UNTIL_START_TIME;
				g_evteng_event_enabled = true;
				g_evteng_event_commenced = false;
			}
			else
			{
				g_sleepType = SLEEP_FOREVER;
				g_evteng_event_enabled = false;
				g_evteng_event_commenced = false;
			}
		}
		break;

		case SLEEP_UNTIL_START_TIME:
		{
			if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
			{
				g_sleepType = SLEEP_UNTIL_START_TIME;
				g_evteng_event_commenced = false;
			}
			else if(g_evteng_event_enabled && eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch))
			{
				g_sleepType = SLEEP_AFTER_EVENT;
			}
			else
			{
				g_sleepType = SLEEP_FOREVER;
			}
		}
		break;

		case SLEEP_AFTER_EVENT:
		{
			g_evteng_event_enabled = g_button_wake_prior_event_enabled;
			g_evteng_event_commenced = g_button_wake_prior_event_commenced;

			if(finishTimedEventIfExpired(now))
			{
				break;
			}

			if(eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch))
			{
				g_sleepType = SLEEP_AFTER_EVENT;
				if(g_button_wake_prior_event_enabled)
				{
					g_evteng_event_enabled = true;
					g_evteng_event_commenced = true;
				}
			}
			else if(eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch))
			{
				g_sleepType = SLEEP_UNTIL_START_TIME;
				g_evteng_event_enabled = true;
				g_evteng_event_commenced = false;
			}
			else
			{
				g_sleepType = SLEEP_FOREVER;
				g_evteng_event_enabled = false;
				g_evteng_event_commenced = false;
			}
		}
		break;

		case SLEEP_FOREVER:
		case SLEEP_POWER_OFF_OVERRIDE:
		default:
		{
			g_sleepType = SLEEP_FOREVER;
		}
		break;
	}
}

bool shouldPowerTransmitterAfterWake(void)
{
	if(g_awakenedBy != AWAKENED_BY_BUTTONPRESS)
	{
		return g_device_enabled;
	}

	switch((SleepType)g_button_wake_prior_sleep_type)
	{
		case SLEEP_UNTIL_NEXT_XMSN:
		{
			return (g_device_enabled && g_evteng_event_enabled);
		}

		case SLEEP_AFTER_EVENT:
		{
			return (g_device_enabled && g_evteng_event_enabled && g_evteng_event_commenced);
		}

		case SLEEP_UNTIL_START_TIME:
		case SLEEP_FOREVER:
		case SLEEP_POWER_OFF_OVERRIDE:
		default:
		{
			return false;
		}
	}
}

void configRedLEDforEvent(void)
{
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;
	bool active_event_window;

	atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
	active_event_window = eventIsScheduledToRunNow(loaded_start_epoch, loaded_finish_epoch) &&
	                      (g_evteng_event_enabled || g_foreground_start_event || (g_sleepType == SLEEP_AFTER_EVENT) || (g_sleepType == SLEEP_UNTIL_NEXT_XMSN));

	if(active_event_window)
	{
		LEDS.blink(LEDS_RED_OFF);
	}
	else if(noEventWillRun())
	{
		if(!g_evteng_run_event_until_canceled)
			LEDS.blink(LEDS_RED_BLINK_FAST, true);
	}
	else if(!g_evteng_event_commenced)
	{
		LEDS.blink(LEDS_RED_BLINK_SLOW, true);
	}
	else
	{
		LEDS.blink(LEDS_RED_OFF); // Foreground will handle blinking chores
	}
}

void setupForFox(Fox_t fox, EventAction_t action)
{
	SC sc;

	g_evteng_run_event_until_canceled = false;
	atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);

	g_evteng_event_enabled = false;
	g_evteng_event_commenced = false;
	loadEventTimingForFox(fox);

	g_foreground_enable_transmitter = false;
	g_foreground_start_event = false;

	if(action == START_NOTHING)
	{
		g_evteng_event_commenced = false; /* do not get things running yet */
		g_evteng_event_enabled = false;   /* do not get things running yet */
		powerToTransmitter(OFF);
	}
	else if((action == START_EVENT_NOW_AND_RUN_FOREVER) || (action == START_EVENT_NOW_AND_RUN_AS_TIMED_EVENT)) /* Start the event now, and align event start to the top of the hour */
	{
		if(timeIsSet() && (action != START_EVENT_NOW_AND_RUN_FOREVER)) // First check if time is valid
		{
			time_t now = time(null);
			time_t time_since_midnight = now % SECONDS_24H;
			time_t top_of_last_midnight = timeDif(now, time_since_midnight);
			time_t newFinish = FOREVER_EPOCH; // If start and finish are not set, run forever
			bool forceForever = false;

			if(allClocksSet(SAVED_SETTINGS))
			{
				time_t event_start_epoch;
				time_t event_finish_epoch;
				atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &event_start_epoch, &event_finish_epoch);
				newFinish = now + timeDif(event_finish_epoch, event_start_epoch);
			}
			else
			{
				forceForever = true; // Avoid activating as a forever event
			}

			activateEventEngineUsingCurrentSettings(&sc, top_of_last_midnight, newFinish);

			if(forceForever)
			{
				g_evteng_run_event_until_canceled = true;
				time_t event_start_epoch;
				time_t event_finish_epoch;
				atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &event_start_epoch, &event_finish_epoch);
				if(event_start_epoch == event_finish_epoch)
				{
					atomic_write_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, event_start_epoch, event_start_epoch); // preserve the start = finish flag
				}
			}
		}
		else
		{
			g_evteng_run_event_until_canceled = true;
			g_last_error_code = launchLoadedEvent((SC *)&g_last_status_code);

			if(g_last_error_code != ERROR_CODE_NO_ERROR)
			{
				sb_send_string((char *)"* Err: event not launched\n");
			}

			LEDS.init();
		}

		g_evteng_event_commenced = true;
		g_evteng_event_enabled = true;

		if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
		{
			sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
		}

		loadCurrentPatternMorse(NULL, CALLER_AUTOMATED_EVENT);
		atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);

		g_event_launched_by_user_action = true;
		LEDS.blink(LEDS_RED_OFF);
	}
	else if(action == START_TRANSMISSIONS_NOW) /* Immediately start transmitting, regardless RTC or time slot */
	{
		g_evteng_run_event_until_canceled = true;
		g_evteng_on_the_air = g_evteng_on_air_seconds; /* start out transmitting */
		g_evteng_sendID_seconds_countdown = g_evteng_intra_cycle_delay_time + g_evteng_on_air_seconds - g_time_needed_for_ID;
		g_last_status_code = STATUS_CODE_EVENT_STARTED_NOW_TRANSMITTING;
		LEDS.blink(LEDS_RED_OFF);
		g_event_launched_by_user_action = true;

		loadCurrentPatternMorse(NULL, CALLER_AUTOMATED_EVENT);
		if(powerToTransmitter(g_device_enabled) != ERROR_CODE_NO_ERROR)
		{
			sb_send_string(TEXT_TX_NOT_RESPONDING_TXT);
		}

		g_evteng_event_commenced = true;
		g_evteng_event_enabled = true;

		atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
	}
	else /* if(action == START_EVENT_WITH_STARTFINISH_TIMES) */
	{
		g_event_launched_by_user_action = false;
		g_evteng_event_commenced = false; /* do not get things running yet */
		g_evteng_event_enabled = false;   /* do not get things running yet */
		keyTransmitter(OFF);
		powerToTransmitter(OFF);

		if(allClocksSet(SAVED_SETTINGS))
		{
			reloadLoadedEventWindowFromSavedSettings();
			time_t loaded_start_epoch;
			time_t loaded_finish_epoch;
			atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
			g_evteng_run_event_until_canceled = false;
			EC ec = activateEventEngineUsingCurrentSettings(&sc, loaded_start_epoch, loaded_finish_epoch);
			if(ec == ERROR_CODE_NO_ERROR)
			{
				g_evteng_event_enabled = true;
			}
			LEDS.blink(LEDS_RED_OFF);
		}
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
time_t validateTimeString(char *str, char *errMsg)
{
	return validateTimeString(str, null, false, errMsg, null);
}

time_t validateTimeString(char *str, volatile time_t *epochVar, bool align5min, char *errMsg)
{
	return validateTimeString(str, epochVar, align5min, errMsg, null);
}

time_t validateTimeString(char *str, volatile time_t *epochVar, bool align5min, char *errMsg, const char *rawInput)
{
	time_t valid = 0;                          // Initialize return value to 0 (indicating invalid by default).
	int len = strlen(str);                     // Get the length of the provided string.
	time_t minimumEpoch = MINIMUM_VALID_EPOCH; // Set the initial minimum valid epoch.
	uint8_t validationType = 0;                // Set the initial validation type to 0.
	time_t now = time(NULL);                   // Get the current system time.

	// Determine the minimum epoch and validation type based on the `epochVar`.
	if(epochVar == &g_event_start_epoch)
	{
		// For event start, ensure the minimum epoch is either now or the pre-set minimum valid epoch.
		minimumEpoch = MAX(now, MINIMUM_VALID_EPOCH);
		validationType = 1; // Indicates start time validation.
	}
	else if(epochVar == &g_event_finish_epoch)
	{
		// For event finish, ensure the minimum epoch is the event start time or the current time.
		minimumEpoch = MAX(atomic_read_time(&g_event_start_epoch), now);
		validationType = 2; // Indicates finish time validation.
	}

	// If the string length is 10, pad it to a length of 12 by adding "00" for seconds.
	if(len == 10)
	{
		str[10] = '0';
		str[11] = '0';
		str[12] = '\0';
		len = 12; // Update the length to 12.
	}

	// If the string length is 12 and contains only digits, proceed to validation.
	if((len == 12) && (only_digits(str)))
	{
		// Convert the time string to epoch value (`YYMMDDhhmmss` format).
		time_t ep = String2Epoch(NULL, str);

		if((validationType == 1) && (ep < minimumEpoch) && rawInput && rawInput[0] && only_digits((char *)rawInput))
		{
			// When an event is already running, partial start input may have been expanded against the old start date.
			const char *retry = completeTimeString(rawInput, &now);
			if(retry)
			{
				time_t retryEpoch = String2Epoch(NULL, (char *)retry);
				if(retryEpoch >= minimumEpoch)
				{
					strncpy(str, retry, 13);
					ep = retryEpoch;
				}
			}
		}

		// Validate if the calculated epoch is greater than or equal to the minimum allowed.
		if(ep >= minimumEpoch)
		{
			if(align5min)
			{
				time_t tt = ep % 300; // align to lesser 5-min boundary

				if(tt)
				{
					ep -= tt;
					if(errMsg)
					{
						sprintf(errMsg, TEXT_ERR_ALIGNED_TO_5MIN_TXT);
					}
				}
			}

			valid = ep; // Set the valid time to the calculated epoch.
		}
		else
		{
			// Report appropriate error messages based on the validation type.
			if(validationType == 1) // Start time validation
			{
				if(errMsg)
				{
					sprintf(errMsg, TEXT_ERR_START_IN_PAST_TXT); // Start time is in the past.
				}
			}
			else if(validationType == 2) // Finish time validation
			{
				if(ep < time(NULL))
				{
					if(errMsg)
					{
						sprintf(errMsg, TEXT_ERR_FINISH_IN_PAST_TXT); // Finish time is in the past.
					}
				}
				else
				{
					if(errMsg)
					{
						sprintf(errMsg, TEXT_ERR_FINISH_BEFORE_START_TXT); // Finish time is before start time.
					}
				}
			}
			else // Current time validation
			{
				if(errMsg)
				{
					sprintf(errMsg, TEXT_ERR_TIME_IN_PAST_TXT); // Time is in the past.
				}
			}
		}
	}
	else if(len) // If the length is non-zero and not 12, it's an invalid time string.
	{
		if(errMsg)
		{
			sprintf(errMsg, TEXT_ERR_INVALID_TIME_TXT); // Report invalid time string error.
		}
	}

	// Return the validated epoch value (or 0 if validation failed).
	return (valid);
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
bool reportTimeTill(time_t from, time_t until, const char *prefix, const char *failMsg)
{
	bool failure = false;

	// Check if the `from` time is greater than or equal to the `until` time.
	if(from >= until) // Negative time, failure condition
	{
		failure = true; // Mark as failure

		// Print the failure message if provided.
		if(failMsg)
		{
			sb_send_string((char *)failMsg);
		}
	}
	else
	{
		// Print the prefix message if provided.
		if(prefix)
		{
			sb_send_string((char *)prefix);
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

		// Report seconds if non-zero.
		if(seconds)
		{
			sprintf(g_tempStr, "%d sec", seconds);
			sb_send_string(g_tempStr);
		}

		// Add a newline after reporting the time.
		sb_send_NewLine();

		// Clear the temporary string.
		g_tempStr[0] = '\0';
	}

	// Return whether there was a failure.
	return failure;
}

bool allClocksSet(Settings_t location)
{
	time_t now = time(null);
	time_t start_epoch;
	time_t finish_epoch;

	if(location == SAVED_SETTINGS)
	{
		atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &start_epoch, &finish_epoch);
	}
	else
	{
		atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &start_epoch, &finish_epoch);
	}

	if((finish_epoch <= MINIMUM_VALID_EPOCH) || (start_epoch <= MINIMUM_VALID_EPOCH) || (now <= MINIMUM_VALID_EPOCH))
	{
		return (false);
	}

	if(finish_epoch <= start_epoch) /* Event configured to finish before it started */
	{
		return (false);
	}

	return (true);
}

ConfigurationState_t clockConfigurationCheck(Settings_t location)
{
	time_t start_epoch;
	time_t finish_epoch;

	if(!allClocksSet(location))
	{
		return (CONFIGURATION_ERROR);
	}

	time_t now = time(null);

	if(location == SAVED_SETTINGS)
	{
		atomic_read_time_pair(&g_event_start_epoch, &g_event_finish_epoch, &start_epoch, &finish_epoch);
	}
	else
	{
		atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &start_epoch, &finish_epoch);
	}

	if(now > finish_epoch) /* The scheduled event is over */
	{
		return (CONFIGURATION_ERROR);
	}

	if(location == LOADED_SETTINGS)
	{
		if(now > start_epoch) /* Event should be running */
		{
			if(!g_evteng_event_enabled)
			{
				return (SCHEDULED_EVENT_DID_NOT_START); /* Event scheduled to be running isn't */
			}
			else
			{
				return (EVENT_IN_PROGRESS); /* Event is running, so clock settings don't matter */
			}
		}
		else if(!g_evteng_event_enabled)
		{
			return (SCHEDULED_EVENT_WILL_NEVER_RUN);
		}
	}

	return (WAITING_FOR_START); /* Future event hasn't started yet */
}

void reportConfigErrors(Settings_t location)
{
	time_t now = time(null);
	time_t start_epoch;
	time_t finish_epoch;

	if(g_meshmode)
		return;

	if(g_messages_text[STATION_ID][0] == '\0')
	{
		sb_send_string(TEXT_SET_ID_TXT);
	}

	if(now <= MINIMUM_VALID_EPOCH) /* Current time is invalid */
	{
		sb_send_string(TEXT_SET_TIME_TXT);
	}

	if(location == SAVED_SETTINGS)
	{
		start_epoch = g_event_start_epoch;
		finish_epoch = g_event_finish_epoch;
	}
	else
	{
		start_epoch = g_evteng_loaded_start_epoch;
		finish_epoch = g_evteng_loaded_finish_epoch;
	}

	if(finish_epoch <= MINIMUM_VALID_EPOCH)
	{
		sb_send_string(TEXT_SET_FINISH_TXT);

		if(start_epoch < MINIMUM_VALID_EPOCH)
		{
			sb_send_string(TEXT_SET_START_TXT);
		}
	}
	else if(finish_epoch <= now) /* Event has already finished */
	{
		if(start_epoch < now) /* Event has already started */
		{
			sb_send_string(TEXT_SET_START_TXT);
		}

		sb_send_string(TEXT_SET_FINISH_TXT);
	}
	else if(start_epoch < now) /* Event has already started */
	{
		if(start_epoch < MINIMUM_VALID_EPOCH) /* Start invalid */
		{
			sb_send_string(TEXT_SET_START_TXT);
		}
		else if(eventIsScheduledToRun(&start_epoch, &finish_epoch) && (!g_evteng_event_enabled && !g_foreground_start_event))
		{
			sb_send_string((char *)"Start with > GO 1 or > GO 2\n");
		}
		else
		{
			sb_send_string((char *)"None: Event running.\n");
		}
	}
	else if(start_epoch == finish_epoch)
	{
		sb_send_string(TEXT_SET_START_TXT);
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
	if(g_cloningInProgress)
		return;

	// Buffer for storing temporary strings.
	char buf[TEMP_STRING_SIZE];

	// Get the current time.
	time_t now = time(NULL);
	time_t loaded_start_epoch = atomic_read_time(&g_evteng_loaded_start_epoch);
	time_t loaded_finish_epoch = atomic_read_time(&g_evteng_loaded_finish_epoch);
	time_t event_start_epoch = atomic_read_time(&g_event_start_epoch);
	time_t event_finish_epoch = atomic_read_time(&g_event_finish_epoch);
	float processor_temperature = atomic_read_float(&g_processor_temperature);
	float processor_max_temperature = atomic_read_float(&g_processor_max_temperature);
	float processor_min_temperature = atomic_read_float(&g_processor_min_temperature);
	bool use_shifted_loaded_schedule = false;

	if((g_days_to_run > 1) && (g_days_run > 0))
	{
		time_t effective_start_epoch = loaded_start_epoch;
		time_t effective_finish_epoch = loaded_finish_epoch;
		if(eventIsScheduledToRun(&effective_start_epoch, &effective_finish_epoch) &&
		   ((effective_start_epoch != event_start_epoch) || (effective_finish_epoch != event_finish_epoch)))
		{
			use_shifted_loaded_schedule = true;
			loaded_start_epoch = effective_start_epoch;
			loaded_finish_epoch = effective_finish_epoch;
			event_start_epoch = effective_start_epoch;
			event_finish_epoch = effective_finish_epoch;
		}
	}

	// Send the product name.
	sb_send_string((char *)PRODUCT_NAME_LONG);

	// Report the software version.
#ifdef HW_TARGET_3_5
	sprintf(buf, "3.5");
#else
	sprintf(buf, "3.4");
#endif

	sprintf(g_tempStr, "\n* SW Ver: %s HW Build: %s\n", SW_REVISION, buf);
	sb_send_string(g_tempStr);

	// Check for hardware errors and report them.
	if(g_hardware_error & (int)HARDWARE_NO_RTC)
	{
		sb_send_string(TEXT_RTC_NOT_RESPONDING_TXT); // RTC not responding.
	}

	if(g_hardware_error & (int)HARDWARE_NO_SI5351)
	{
		sb_send_string(TEXT_TX_NOT_RESPONDING_TXT); // Transmitter not responding.
	}

	if(g_thermal_shutdown)
	{
		sb_send_string(TEXT_EXCESSIVE_TEMPERATURE);
	}

	if(isValidTemp(processor_temperature))
	{
		int16_t integer;
		uint16_t fractional;

		if(!float_to_parts_signed(processor_temperature, &integer, &fractional))
		{
			sprintf(g_tempStr, "\n*   Cur Temp: %d.%dC\n", integer, fractional);
			sb_send_string(g_tempStr);
		}
	}

	if(isValidTemp(processor_max_temperature))
	{
		int16_t integer;
		uint16_t fractional;

		if(!float_to_parts_signed(processor_max_temperature, &integer, &fractional))
		{
			sprintf(g_tempStr, "*   Max Temp: %d.%dC\n", integer, fractional);
			sb_send_string(g_tempStr);
		}
	}

	if(isValidTemp(processor_min_temperature))
	{
		int16_t integer;
		uint16_t fractional;

		if(!float_to_parts_signed(processor_min_temperature, &integer, &fractional))
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
		sb_send_string((char *)"*   Callsign: None\n");
	}

	// Report the speed for the callsign in words per minute.
	sprintf(g_tempStr, "*   Callsign WPM: %d\n", g_evteng_id_codespeed);
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
		sb_send_string((char *)"*   Freq: None set\n");
	}

	// 	// Report the RTC calibration value.
	// 	sprintf(g_tempStr, "*   Cal: %d\n", RTC_get_cal());
	// 	sb_send_string(g_tempStr);

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
	// 	if(!g_evteng_loaded_start_epoch) g_evteng_loaded_start_epoch = g_event_start_epoch;
	// 	if(!g_evteng_loaded_finish_epoch) g_evteng_loaded_finish_epoch = g_event_finish_epoch;
	sprintf(g_tempStr, "*   Start:  %s\n", convertEpochToTimeString(event_start_epoch, buf, TEMP_STRING_SIZE));
	sb_send_string(g_tempStr);
	sprintf(g_tempStr, "*   Finish: %s\n", convertEpochToTimeString(event_finish_epoch, buf, TEMP_STRING_SIZE));
	sb_send_string(g_tempStr);
	if(event_finish_epoch == event_start_epoch)
	{
		sprintf(g_tempStr, "*   Event start disabled (Start = Finish)\n");
		sb_send_string(g_tempStr);
	}

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
	ConfigurationState_t cfg = WAITING_FOR_START;
	if(!use_shifted_loaded_schedule)
	{
		cfg = clockConfigurationCheck(SAVED_SETTINGS);
	}

	if(!use_shifted_loaded_schedule && (cfg != WAITING_FOR_START) && (cfg != EVENT_IN_PROGRESS) && (cfg != SCHEDULED_EVENT_WILL_NEVER_RUN))
	{
		sb_send_string((char *)"\n* Needed Actions:\n");
		reportConfigErrors(SAVED_SETTINGS);
	}
	else
	{
		reportTimeTill(now, loaded_start_epoch, "\n*   Starts in: ", "\n*   In progress\n");
		reportTimeTill(loaded_start_epoch, loaded_finish_epoch, "*   Lasts: ", NULL);
		if(loaded_start_epoch < now)
		{
			reportTimeTill(now, loaded_finish_epoch, "*   Time Remaining: ", NULL);
		}

		bool needs_manual_start = use_shifted_loaded_schedule
		                              ? (!eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch) && !g_evteng_event_enabled && !g_foreground_start_event)
		                              : (eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch) && !g_evteng_event_enabled && !g_foreground_start_event);

		if(needs_manual_start)
		{
			sb_send_string((char *)"\n* Needed Action:\n");
			sb_send_string((char *)"\n*  Start with > GO 1, or GO 2\n");
		}
	}

	// If the device is disabled, say so and provide instructions to enable.
	if(!g_device_enabled)
	{
		sb_send_string(TEXT_DEVICE_DISABLED_TXT);
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

#ifdef TEST_MODE_SOFTWARE
	sb_send_string(TEXT_TEST_SOFTWARE_NOTICE_TXT);
#endif
}

uint16_t timeNeededForID(void)
{
	return ((uint16_t)(((float)timeRequiredToSendStrAtWPM((char *)g_messages_text[STATION_ID], g_evteng_id_codespeed)) / 1000.));
}

Fox_t getFoxSetting(void)
{
	return fox_setting_current_atomic();
}

int getFoxCodeSpeed(void)
{
	Fox_t fox;
	Event_t event_snapshot;
	event_and_fox_current_atomic(&event_snapshot, &fox);
	if(fox == BEACON)
	{
		return (g_evteng_pattern_codespeed);
	}
	else if(event_snapshot == EVENT_FOXORING)
	{
		return (g_foxoring_pattern_codespeed);
	}

	return (g_evteng_pattern_codespeed);
}

int getPatternCodeSpeed(void)
{
	if(!g_evteng_event_commenced)
	{
		return ENUNCIATION_BLINK_WPM;
	}

	return (getFoxCodeSpeed());
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
char *getCurrentPatternText(void)
{
	char *c;

	Fox_t fox = getFoxSetting();
	switch(fox)
	{
		case FOX_1:
		{
			c = (char *)"MOE";
		}
		break;

		case FOX_2:
		{
			c = (char *)"MOI";
		}
		break;

		case FOX_3:
		{
			c = (char *)"MOS";
		}
		break;

		case FOX_4:
		{
			c = (char *)"MOH";
		}
		break;

		case FOX_5:
		{
			c = (char *)"MO5";
		}
		break;

		case SPECTATOR:
		{
			c = (char *)"S";
		}
		break;

		case SPRINT_S1:
		{
			c = (char *)"ME";
		}
		break;

		case SPRINT_S2:
		{
			c = (char *)"MI";
		}
		break;

		case SPRINT_S3:
		{
			c = (char *)"MS";
		}
		break;

		case SPRINT_S4:
		{
			c = (char *)"MH";
		}
		break;

		case SPRINT_S5:
		{
			c = (char *)"M5";
		}
		break;

		case SPRINT_F1:
		{
			c = (char *)"OE";
		}
		break;

		case SPRINT_F2:
		{
			c = (char *)"OI";
		}
		break;

		case SPRINT_F3:
		{
			c = (char *)"OS";
		}
		break;

		case SPRINT_F4:
		{
			c = (char *)"OH";
		}
		break;

		case SPRINT_F5:
		{
			c = (char *)"O5";
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
				c = (char *)"< E<";
			}
			else if(g_frequency_to_test == 1)
			{
				c = (char *)"< EE<";
			}
			else if(g_frequency_to_test == 2)
			{
				c = (char *)"< EEE<";
			}
			else // if(g_frequency_to_test == 3)
			{
				c = (char *)"< EEEE<";
			}
		}
		break;

		case BEACON:
		{
			c = (char *)"MO";
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
 * Retrieves the appropriate frequency setting based on the current event type and fox setting.
 * @return Frequency_Hz - the frequency setting for the given event
 */
Frequency_Hz getFrequencySetting(void)
{
	Frequency_Hz freq;

	Fox_t fox = getFoxSetting();
	switch(fox)
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

	return (freq);
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
	if(atomic_read_u16(&g_programming_countdown) == 0)
	{
		g_programming_state = SYNC_Searching_for_slave;
		g_cloningInProgress = false;
		atomic_write_u16(&g_programming_countdown, (uint16_t)-1);
	}

	SerialbusRxBuffer *sb_buff = nextFullSBRxBuffer();
	SBMessageID msg_id;

	if(sb_buff)
	{
		LEDS.init(); /* Extend or resume LED operation */
	}

	if(!atomic_read_u16(&g_programming_msg_throttle))
	{
		if(!g_cloningInProgress)
		{
			sb_send_master_string((char *)"MAS P\n"); /* Set slave to active cloning state */
			atomic_write_u16(&g_programming_msg_throttle, 600);
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
				if((msg_id == SB_MESSAGE_MASTER) && (sb_buff->fields[SB_FIELD1][0] == 'S')) /* Slave responds ready for cloning */
				{
					extendMasterModeTimeout();
					g_cloningInProgress = true;
					captureCloneTimingSnapshot();
					if(g_evteng_event_enabled && g_evteng_event_commenced &&
					   eventIsScheduledToRunNow(g_clone_timing_snapshot.event_start_epoch, g_clone_timing_snapshot.event_finish_epoch))
					{
						/* Pause an actively running timed event on the master while cloning so we can isolate
						 * whether the live event engine state is contributing to clone-time clock skew. */
						suspendEvent();
					}
					g_event_checksum = 0;
					sprintf(g_tempStr, "FUN A\n"); /* Set slave to radio orienteering function */
					sb_send_master_string(g_tempStr);
					g_programming_state = SYNC_Waiting_for_FUN_A_reply;
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += 'A';
					g_seconds_transition = false;
					g_programming_state = SYNC_Align_to_Second_Transition;
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
				sprintf(g_tempStr, "CLK T %lu\n", now); /* Set slave's RTC */
				sb_send_master_string(g_tempStr);
				g_programming_state = SYNC_Waiting_for_CLK_T_reply;
				atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
				atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
						extendMasterModeTimeout();
						time_t event_start_epoch = g_clone_timing_snapshot.event_start_epoch;
						g_event_checksum += event_start_epoch;
						g_programming_state = SYNC_Waiting_for_CLK_S_reply;
						sprintf(g_tempStr, "CLK S %lu\n", event_start_epoch);
						sb_send_master_string(g_tempStr);
						atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
						extendMasterModeTimeout();
						time_t event_finish_epoch = g_clone_timing_snapshot.event_finish_epoch;
						g_event_checksum += event_finish_epoch;
						g_programming_state = SYNC_Waiting_for_CLK_F_reply;
						sprintf(g_tempStr, "CLK F %lu\n", event_finish_epoch);
						sb_send_master_string(g_tempStr);
						atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
						extendMasterModeTimeout();
						g_event_checksum += g_clone_timing_snapshot.days_to_run;
						g_programming_state = SYNC_Waiting_for_CLK_D_reply;
						sprintf(g_tempStr, "CLK D %d\n", g_clone_timing_snapshot.days_to_run);
						sb_send_master_string(g_tempStr);
						atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
						extendMasterModeTimeout();
						atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						g_programming_state = SYNC_Waiting_for_ID_reply;

						char *ptr1 = NULL;
						char *ptr2 = NULL;
						int ch = ' ';

						for(uint8_t i = 0; i < strlen(g_messages_text[STATION_ID]); i++)
						{
							g_event_checksum += g_messages_text[STATION_ID][i];
						}

						ptr1 = strchr(g_messages_text[STATION_ID], ch);

						if(ptr1)
						{
							ptr1++;
							ptr2 = strchr((const char *)ptr1, ch);

							if(ptr2)
							{
								*ptr2 = '\0';
								ptr2++;

								sprintf(g_tempStr, "ID %s %s\n", ptr1, ptr2);
							}
							else
							{
								sprintf(g_tempStr, "ID %s\n", ptr1);
							}
						}
						else
						{
							strncpy(g_tempStr, "ID \"\"\n", TEMP_STRING_SIZE);
						}

						sb_send_master_string(g_tempStr);
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += g_evteng_id_codespeed;
					g_programming_state = SYNC_Waiting_for_ID_CodeSpeed_reply;
					sprintf(g_tempStr, "SPD I %u\n", g_evteng_id_codespeed);
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
						extendMasterModeTimeout();
						sprintf(g_tempStr, "SPD P %u\n", g_evteng_pattern_codespeed);

						g_event_checksum += g_evteng_pattern_codespeed;
						sb_send_master_string(g_tempStr);
						g_programming_state = SYNC_Waiting_for_Pattern_CodeSpeed_reply;
						atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
						extendMasterModeTimeout();
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
						sprintf(g_tempStr, "EVT %c\n", c);
						sb_send_master_string(g_tempStr); /* Set slave's event */
						g_programming_state = SYNC_Waiting_for_EVT_reply;
						atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
						atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += g_frequency;
					g_programming_state = SYNC_Waiting_for_NoEvent_Freq_reply;
					sprintf(g_tempStr, "FRE X %lu\n", g_frequency);
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += g_frequency_low;
					g_programming_state = SYNC_Waiting_for_Freq_Low_reply;
					sprintf(g_tempStr, "FRE L %lu\n", g_frequency_low);
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += g_frequency_med;
					g_programming_state = SYNC_Waiting_for_Freq_Med_reply;
					sprintf(g_tempStr, "FRE M %lu\n", g_frequency_med);
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += g_frequency_hi;
					g_programming_state = SYNC_Waiting_for_Freq_Hi_reply;
					sprintf(g_tempStr, "FRE H %lu\n", g_frequency_hi);
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_event_checksum += g_frequency_beacon;
					g_programming_state = SYNC_Waiting_for_Freq_Beacon_reply;
					sprintf(g_tempStr, "FRE B %lu\n", g_frequency_beacon);
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					g_programming_state = SYNC_Waiting_for_ACK;
					sprintf(g_tempStr, "MAS Q %lu\n", atomic_read_u32(&g_event_checksum));
					sb_send_master_string(g_tempStr);
					atomic_write_u16(&g_programming_msg_throttle, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
					atomic_write_u16(&g_programming_countdown, PROGRAMMING_MESSAGE_TIMEOUT_PERIOD);
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
					extendMasterModeTimeout();
					if(sb_buff->fields[SB_FIELD1][0] == 'A')
					{
						atomic_write_u16(&g_send_clone_success_countdown, 18000);
					}
					else
					{
						atomic_write_u16(&g_programming_msg_throttle, 0);
						g_cloningInProgress = false;
					}

					g_programming_state = SYNC_Searching_for_slave;
				}
			}
		}
		break;
	}

	if(sb_buff)
		sb_buff->id = SB_MESSAGE_EMPTY;
}

/*
 * Checks if any event will run
 * @return true if no event is scheduled or event is disabled or sleep mode is a permanent variety
 */
bool noEventWillRun(void)
{
	time_t loaded_start_epoch;
	time_t loaded_finish_epoch;

	atomic_read_time_pair(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch, &loaded_start_epoch, &loaded_finish_epoch);
	bool event_is_scheduled = eventIsScheduledToRun(&loaded_start_epoch, &loaded_finish_epoch);

	return !event_is_scheduled || (!eventScheduledForTheFuture(loaded_start_epoch, loaded_finish_epoch) && !g_evteng_event_enabled);
}

/*
 * Checks if any event is currently running
 * @return true if an event is currently in progress
 */
bool eventRunning(void)
{
	bool result;

	result = ((g_evteng_event_enabled && (g_evteng_run_event_until_canceled || g_evteng_event_commenced)) && txIsInitialized());

	return result;
}

/*
 * Checks if an event is scheduled to run at the current time
 * @return true if an event is scheduled for the current time
 */
bool eventIsScheduledToRunNow(time_t start_epoch, time_t finish_epoch)
{
	time_t now = time(null);
	bool result = false;

	if(timeIsSet() && (start_epoch > MINIMUM_VALID_EPOCH))
	{
		result = ((start_epoch < now) && (finish_epoch > now));
	}

	return (result);
}

/*
 * Checks if an event is scheduled to occur in the future
 * @return true if an event is scheduled for a future time
 */
bool eventScheduledForTheFuture(time_t start_epoch, time_t finish_epoch)
{
	time_t now = time(null);
	bool result = false;

	if(timeIsSet())
	{
		result = ((start_epoch > now) && (finish_epoch > start_epoch));
	}

	return (result);
}

/*
 * Determines whether an event is scheduled based on the current time, future event time, and the number of days remaining
 * @return true if an event is scheduled
 */
bool eventIsScheduledToRun(volatile time_t *start_epoch, volatile time_t *finish_epoch)
{
	time_t start_local = 0;
	time_t finish_local = 0;
	bool result;

	if(!start_epoch || !finish_epoch)
	{
		return false;
	}

	ENTER_CRITICAL(main_event_sched_read);
	start_local = *start_epoch;
	finish_local = *finish_epoch;
	EXIT_CRITICAL(main_event_sched_read);

	result = eventIsScheduledToRun(&start_local, &finish_local);

	ENTER_CRITICAL(main_event_sched_write);
	*start_epoch = start_local;
	*finish_epoch = finish_local;
	EXIT_CRITICAL(main_event_sched_write);

	return result;
}

bool eventIsScheduledToRun(time_t *start_epoch, time_t *finish_epoch)
{
	bool result = false;

	if(!start_epoch || !finish_epoch || (*start_epoch == *finish_epoch))
	{
		return false;
	}

	time_t now = time(null);

	if(timeIsSet())
	{
		result = eventScheduledForTheFuture(*start_epoch, *finish_epoch) || eventIsScheduledToRunNow(*start_epoch, *finish_epoch);

		if(!result) // If current settings won't run, see if it should run for more days
		{
			if(g_days_to_run > 1)
			{
				if((*start_epoch > MINIMUM_VALID_EPOCH) && (*finish_epoch > *start_epoch))
				{
					time_t s = *start_epoch;
					time_t f = *finish_epoch;
					time_t saved_start_epoch = atomic_read_time(&g_event_start_epoch);
					time_t earliest_remaining_start = saved_start_epoch;
					uint8_t minimum_day_offset = MIN(g_days_run, (uint8_t)(g_days_to_run - 1));

					if(saved_start_epoch > MINIMUM_VALID_EPOCH)
					{
						earliest_remaining_start += ((time_t)minimum_day_offset * SECONDS_24H);
						if(s < earliest_remaining_start)
						{
							time_t delta = earliest_remaining_start - s;
							s += delta;
							f += delta;
						}
					}

					uint8_t day_offset = 0;
					if((saved_start_epoch > MINIMUM_VALID_EPOCH) && (s >= saved_start_epoch))
					{
						time_t delta = s - saved_start_epoch;
						day_offset = MIN((uint8_t)(delta / SECONDS_24H), (uint8_t)(g_days_to_run - 1));
					}

					uint8_t shifts_remaining = ((g_days_to_run - 1) > day_offset) ? ((g_days_to_run - 1) - day_offset) : 0;
					while((f <= now) && shifts_remaining)
					{
						s += SECONDS_24H;
						f += SECONDS_24H;
						shifts_remaining--;
					}

					if(eventScheduledForTheFuture(s, f) || eventIsScheduledToRunNow(s, f))
					{
						*start_epoch = s;
						*finish_epoch = f;
						result = true;
					}
				}
			}
		}
	}

	return (result);
}

bool timeIsSet(void)
{
	time_t now = time(null);
	return (now > MINIMUM_VALID_EPOCH);
}

bool startEvent(void)
{
	bool error = true;

	cancelManualTransientState();
	suspendEvent();
	g_event_launched_by_user_action = false;
	reloadLoadedEventWindowFromSavedSettings();

	if(eventIsScheduledToRun(&g_evteng_loaded_start_epoch, &g_evteng_loaded_finish_epoch))
	{
		error = startEventUsingRTC();
	}
	else
	{
		g_sleepType = SLEEP_FOREVER;
		startEventNow(true); // Immediately start the event
		error = false;
	}

	return error;
}
