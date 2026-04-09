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
 * Cross-translation-unit firmware globals.
 *
 * This header is the single declaration point for state shared across modules.
 * The declarations here must match their definitions exactly, including type
 * qualifiers such as volatile.
 *
 * When a variable is shared with an ISR or a mutable shared object, use the
 * synchronization rules documented here and in shared_state.h rather than
 * reading or writing multi-byte values directly.
 */

#ifndef SIGNALSLINGER_GLOBALS_H_
#define SIGNALSLINGER_GLOBALS_H_

#include "defs.h"
#include "adc.h"
#include "serialbus.h"
#include "leds.h"
#include "CircularStringBuff.h"
#include "eeprommanager.h"

/* adc.cpp */
extern ADC_Init_t g_adc_initialization;

/* i2c.cpp */
extern volatile uint16_t g_i2c0_timeout_ticks;
extern volatile uint16_t g_i2c_failure_count;

/* rtc.cpp */
extern uint16_t g_clock_calibration;

/* serialbus.cpp */
extern volatile uint16_t g_serial_timeout_ticks;
extern volatile USART_Number_t g_serialbus_usart_number;

/* si5351.cpp */
extern bool g_si5351_initialized;

/* transmitter.cpp */
extern volatile Frequency_Hz g_80m_frequency;
extern volatile uint16_t g_80m_power_level_mW;
extern volatile Frequency_Hz g_rtty_offset;
extern volatile bool g_enable_boost_regulator;
extern volatile bool g_enable_external_battery_control;

/* main.cpp
 * Access policy tags:
 *  - ISR->FG: written in ISR, read in foreground (use shared_state atomic reads for multi-byte types)
 *  - FG->ISR: written in foreground, read in ISR (use shared_state atomic writes for multi-byte types)
 *  - Shared object: do not rely on volatile; use module/helper synchronization rules
 *
 * Variables without a tag are currently treated as foreground-owned configuration
 * state and may be read directly unless a nearby comment says otherwise.
 */
extern volatile bool g_device_enabled;

extern volatile uint8_t g_evteng_id_codespeed; /* ISR->FG */
extern volatile uint8_t g_evteng_pattern_codespeed;
extern volatile int16_t g_evteng_on_air_seconds;
extern volatile int16_t g_evteng_off_air_seconds;
extern volatile int16_t g_evteng_intra_cycle_delay_time;
extern volatile int16_t g_evteng_ID_period_seconds;
extern volatile time_t g_event_start_epoch;          /* FG->ISR, use shared_state time helpers */
extern volatile time_t g_evteng_loaded_start_epoch;  /* FG->ISR + ISR->FG, use shared_state time helpers */
extern volatile time_t g_event_finish_epoch;         /* FG->ISR, use shared_state time helpers */
extern volatile time_t g_evteng_loaded_finish_epoch; /* FG->ISR + ISR->FG, use shared_state time helpers */
extern volatile bool g_evteng_event_enabled;
extern volatile bool g_evteng_event_commenced;
extern volatile bool g_evteng_sending_station_ID;
extern volatile bool g_evteng_initialize_event;

extern volatile float g_internal_voltage_low_threshold; /* FG->ISR + ISR->FG, use shared_state float helpers for snapshots */
extern volatile float g_internal_bat_voltage;          /* ISR->FG, use shared_state float helpers for snapshots */
extern volatile bool g_internal_bat_detected;
extern volatile float g_external_voltage;              /* ISR->FG, use shared_state float helpers for snapshots */
extern volatile float g_processor_temperature;         /* ISR->FG, use shared_state float helpers for snapshots */
extern volatile float g_processor_min_temperature;     /* ISR->FG, use shared_state float helpers for snapshots */
extern volatile float g_processor_max_temperature;     /* ISR->FG, use shared_state float helpers for snapshots */
extern volatile bool g_restart_conversions;
extern volatile bool g_seconds_transition;
extern volatile bool g_muteAfterID;
extern volatile uint32_t g_event_checksum;             /* FG->ISR + ISR->FG, use shared_state u32 helpers for snapshots */
extern volatile uint8_t g_days_to_run;
extern volatile uint8_t g_days_run;
extern volatile Function_t g_function;
extern volatile uint8_t g_foxoring_pattern_codespeed;
extern volatile uint16_t g_time_needed_for_ID;

extern char g_messages_text[STATION_ID + 1][MAX_PATTERN_TEXT_LENGTH + 2]; /* Shared object: use shared_state message-slot helpers / Morse snapshot wrappers */

extern volatile uint16_t g_foreground_handle_counted_presses;
extern volatile uint16_t g_switch_presses_count;
extern volatile bool g_long_button_press;
extern volatile uint16_t g_button_hold_countdown;

extern uint8_t g_frequency_to_test; /* FG->ISR + ISR->FG */
extern volatile bool g_cloningInProgress;
extern volatile Enunciation_t g_enunciator;
extern leds LEDS; /* Shared object: methods synchronize internal mutable state */
extern CircularStringBuff g_text_buff; /* Shared object: use shared_state text-buffer helpers */
extern EepromManager g_ee_mgr;

extern volatile bool g_isMaster;
extern volatile uint16_t isMasterCountdownSeconds; /* ISR->FG */
extern Fox_t g_fox[EVENT_NUMBER_OF_EVENTS];

extern Event_t g_event;
extern Frequency_Hz g_frequency;
extern Frequency_Hz g_frequency_low;
extern Frequency_Hz g_frequency_med;
extern Frequency_Hz g_frequency_hi;
extern Frequency_Hz g_frequency_beacon;

extern int8_t g_utc_offset;
extern uint8_t g_unlockCode[UNLOCK_CODE_SIZE + 2];

extern volatile bool g_enable_manual_transmissions; /* FG->ISR + ISR->FG (paired with g_text_buff/LEDS synchronization) */

#endif /* SIGNALSLINGER_GLOBALS_H_ */
