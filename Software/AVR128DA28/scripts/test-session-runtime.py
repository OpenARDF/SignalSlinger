#!/usr/bin/env python3
"""Run production thermal recovery and scheduler helpers against a deterministic clock."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'SignalSlinger/main.cpp').read_text()

def function(signature):
    start = source.index(signature + '\n{')
    body = source.index('{', start)
    depth = 1
    end = body + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    result=source[start:end]
    if signature.startswith('bool activateEventEngineUsingCurrentSettings('):
        result=result.replace('{', '{\n activation_start=startTime;activation_finish=finishTime;', 1)
    return result

stubs = r'''
#include "session_history.h"
#include "thermal_shutdown.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <string>
#include <algorithm>
#define ENTER_CRITICAL(x)
#define EXIT_CRITICAL(x)
#define null nullptr
#define OFF false
#define MIN(a,b) std::min(a,b)
#define MAX(a,b) std::max(a,b)
#define FAN_TURN_ON_TEMP 40
#define FAN_TURN_OFF_TEMP 35
#define MINIMUM_VALID_EPOCH 1609459200
#define SECONDS_24H 86400
static time_t now;
#define time(x) ::now
static bool g_thermal_shutdown_enabled=true, g_thermal_shutdown=false;
static bool g_evteng_event_commenced=false, g_evteng_run_event_until_canceled=false;
static bool g_evteng_event_enabled=false, g_event_launched_by_user_action=false;
static bool g_device_enabled=true, g_event_canceled_by_user=false, g_isMaster=false, g_cloningInProgress=false;
static bool g_go_to_sleep_now=false, g_foreground_enable_transmitter=false, g_foreground_start_event=false;
static bool disabled=false, powered=false, g_foreground_reset_after_keydown=false;
static uint16_t g_key_down_countdown=0, g_evteng_sleepshutdown_seconds=0;
static time_t g_event_start_epoch, g_event_finish_epoch, g_time_to_wake_up;
static volatile time_t g_evteng_loaded_start_epoch, g_evteng_loaded_finish_epoch;
static uint8_t g_days_to_run=3, g_schedule_day_index=0;
static int32_t g_evteng_on_the_air=0;
static float g_processor_temperature=65, g_processor_min_temperature=200, g_processor_max_temperature=-100;
static bool g_turn_on_fan=false;
static int8_t g_thermal_shutdown_threshold=65;
enum SleepType { SLEEP_UNTIL_START_TIME, SLEEP_AFTER_EVENT, SLEEP_UNTIL_NEXT_XMSN, SLEEP_FOREVER, SLEEP_POWER_OFF_OVERRIDE };
static SleepType g_sleepType=SLEEP_FOREVER, g_button_wake_prior_sleep_type=SLEEP_FOREVER;
static bool g_button_wake_prior_event_enabled=false, g_button_wake_prior_event_commenced=false;
enum { POWER_UP_START, AWAKENED_BY_BUTTONPRESS };
static int g_awakenedBy=AWAKENED_BY_BUTTONPRESS;
void restoreStateAfterButtonWakeAuthorization(void);
bool timeIsSet(void);
enum { LEDS_OFF, LEDS_RED_OFF };
static struct {
    bool visible=true;
    void init() { visible=true; } void blink(int) {} void setRed(bool) {}
    bool active() { return visible; } void setWakeAuthorizationBlink(bool) {}
} LEDS;
enum ButtonHoldIntent { BUTTON_HOLD_DEFAULT, BUTTON_HOLD_STOP_TEST, BUTTON_HOLD_SLEEP_SCHEDULE };
static uint8_t g_button_hold_intent=BUTTON_HOLD_DEFAULT,g_long_button_hold_intent=BUTTON_HOLD_DEFAULT;
static bool g_device_wakeup_complete=true,g_sleeping=false,g_consume_current_press_for_led_wake=false,g_pending_led_revival=false;
static uint16_t g_switch_presses_count=0;
static bool input_closed=false;
#define SWITCH 0
#define MAXIMUM_NUM_OF_KEYPRESSES 9
#define BUTTON_HOLD_PREVIEW_SAMPLE_TICKS 50
#define BUTTON_LONG_PRESS_SAMPLE_TICKS 200
static uint8_t sampled_pin=1;
uint8_t portDdebouncedVals() { return sampled_pin; }
void debounce() { sampled_pin=input_closed ? 0 : 1; } // Clean sampled input; physical debounce is a bench check.
void setButtonHoldPreviewIndicator(bool) {}
static bool g_start_event_after_keydown=false, g_foreground_reset_after_demo=false;
static bool g_long_button_press=false, g_foreground_check_for_long_wakeup_press=false, g_defer_cloned_event_start=false;
static uint16_t g_demo_event_countdown=0, g_foreground_handle_counted_presses=0, isMasterCountdownSeconds=0;
static uint16_t g_send_clone_success_countdown=0, g_programming_countdown=0, g_programming_msg_throttle=0;
static int g_frequency_to_test=0;
#define NUMBER_OF_TEST_FREQUENCIES 4
#define FOREVER_EPOCH ((time_t)4294967295UL)
static int g_evteng_off_air_seconds=240, g_evteng_on_air_seconds=60, g_evteng_intra_cycle_delay_time=120;
static int g_evteng_sendID_seconds_countdown=0;
static int g_evteng_pattern_codespeed=8, g_evteng_id_codespeed=10, g_evteng_ID_period_seconds=60, g_evteng_code_throttle=0;
#define MIN_CODE_SPEED_WPM 5
#define MAX_CODE_SPEED_WPM 20
#define STATION_ID 0
#define FREQUENCY_TEST_BEACON 99
static char g_messages_text[1][8]={{0}};
char* getCurrentPatternText() { static char pattern[]="MOS";return pattern; }
bool syncCurrentFrequencySetting(bool) { return false; }
int getFoxSetting() { return 3; }
int getFoxCodeSpeed() { return 8; }
int throttleValue(int) { return 10; }
uint16_t timeNeededForID() { return 5; }
static uint16_t g_time_needed_for_ID=0;
static bool keyed=false;
bool txIsKeyed() { return keyed; }
bool rawSwitchIsClosed() { return input_closed; }
bool get_fet_driver() { return keyed; }
#define LED_RED 2
#define LED_GREEN 4
bool PORTD_get_pin_level(int) { return false; }
void serialbusRxErrors(uint16_t* a,uint16_t* b,uint16_t* c) { *a=*b=*c=0; }
static unsigned rf_power_ons=0;
static time_t activation_start=0, activation_finish=0;
enum EventAction_t { START_NOTHING, START_EVENT_NOW_AND_RUN_FOREVER, START_EVENT_NOW_AND_RUN_AS_TIMED_EVENT, START_TRANSMISSIONS_NOW, START_EVENT_WITH_STARTFINISH_TIMES };
typedef int Fox_t;
#define USE_CURRENT_FOX 3
#define SAVED_SETTINGS 0
bool allClocksSet(int) { return timeIsSet() && g_event_start_epoch>MINIMUM_VALID_EPOCH && g_event_finish_epoch>g_event_start_epoch; }
void loadEventTimingForFox(int) {}
void loadCurrentPatternMorse(void*,int) {}
#define CALLER_AUTOMATED_EVENT 0
#define TEXT_TX_NOT_RESPONDING_TXT "tx error"
static bool savedScheduleCanLimitManualRun(void);
static bool advanceLoadedEventWindowAfterCurrentDayCancel(void);
static bool currentLoadedEventWindowCanceled(void);
static bool manualTestInProgress(void);
static bool handleScheduledButtonHold(void);
static void cancelEventFromButton(void);
void suspendEvent(SessionReason reason=REASON_SETTINGS);
bool startEventUsingRTC(void);
void setupForFox(Fox_t fox, EventAction_t action);
bool activateEventEngineUsingCurrentSettings(time_t start, time_t finish);
static time_t timeDif(time_t a,time_t b) { return a-b; }

static bool evaluateThermalShutdownState(float t, bool prior) { return evaluateThermalShutdownStateForPolicy(t,g_thermal_shutdown_threshold,5,g_thermal_shutdown_enabled,prior); }
bool timeIsSet(void);
bool isValidTemp(float value) { return value > -100 && value < 200; }
bool getDisableTransmissions() { return disabled; }
bool txIsInitialized() { return powered; }
static bool cancelManualTransientState(void);
void keyTransmitter(bool on) { keyed=on; }
bool powerToTransmitter(bool on) { powered=on; if(on) ++rf_power_ons; return true; }
void atomic_write_u16(uint16_t* p, uint16_t v) { *p=v; }
uint16_t atomic_read_u16(volatile uint16_t* p) { return *p; }
int32_t atomic_read_i32(int32_t* p) { return *p; }
void atomic_write_int(int* p,int v) { *p=v; }
void atomic_write_i32(int32_t* p, int32_t v) { *p=v; }
time_t atomic_read_time(volatile time_t* p) { return *p; }
void atomic_write_time(volatile time_t* p, time_t v) { *p=v; }
void atomic_read_time_pair(volatile time_t* a,volatile time_t* b,time_t* c,time_t* d) { *c=*a; *d=*b; }
void atomic_write_time_pair(volatile time_t* a,volatile time_t* b,time_t c,time_t d) { *a=c; *b=d; }
bool eventIsScheduledToRun(time_t*,time_t*);
bool eventIsScheduledToRun(volatile time_t*,volatile time_t*);
void configRedLEDforEvent() {}
bool eventScheduledForTheFuture(time_t a,time_t b);
bool eventIsScheduledToRunNow(time_t a,time_t b);
static bool reloadLoadedEventWindowFromSavedSettings();
static bool finishTimedEventIfExpired(time_t now);
static std::vector<SessionRecord> history;
static std::string report;
void sb_send_string(const char* s) { report+=s; }
void sb_send_master_string(const char* s) { report+=s; }
void appendSessionHistory(SessionRecord r) { history.push_back(r); }
uint8_t sessionHistoryCapacity() { return 6; }
uint8_t readSessionHistory(SessionRecord* out,uint8_t capacity) {
    unsigned count=std::min<unsigned>(capacity,history.size());
    std::copy(history.end()-count,history.end(),out);return count;
}
static unsigned launches=0;
bool launchLoadedEvent() {
    ++launches;
    if(g_evteng_run_event_until_canceled) g_evteng_on_the_air=g_evteng_on_air_seconds;
    if(!g_evteng_run_event_until_canceled && now >= g_evteng_loaded_finish_epoch) return false;
    g_evteng_event_enabled=true;g_evteng_event_commenced=true;powered=true;return true;
}
#include "session_runtime.h"
#define SERIAL_LATENCY_SCOPE(id) ((void)0)
#include "ui_bench.h"
bool startEventUsingRTC(void) {
    setupForFox(USE_CURRENT_FOX,START_EVENT_WITH_STARTFINISH_TIMES);
    return false;
}
'''
sleep_begin=source.index('int32_t timeRemaining = SECONDS_24H; // Any  big number will do;')
sleep_end=source.index('\n\t\t\t\t\t\tmuteAfterID = false;', sleep_begin)
sleep_policy='void endSlotSleepPolicy() {\n'+source[sleep_begin:sleep_end]+'\n}\n'
input_begin=source.index('\t\t/* Handle pushbutton press detection */')
input_end=source.index('\n\t\tif(g_programming_countdown > 0)', input_begin)
input_policy='''void sampleButton() {
    static unsigned fiftyMS=0;
    uint8_t holdSwitch=0;
    static bool buttonReleased=false, longPressEnabled=true, consumeHeldPreviewPress=false, wakeAuthSwitchClosed=true;
    static uint16_t switch_closed_time=0, switch_closures_count_period=40;
'''+source[input_begin:input_end]+'\n}\n'
tests = r'''

void seedManualWake(bool exhausted=true) {
    g_session={};history.clear();report.clear();g_session_pending_read=0;g_session_pending_count=0;
    g_thermal_start_pending=false;g_thermal_shutdown=false;g_temperature_fresh_seconds=10;
    g_device_enabled=true;disabled=false;g_event_canceled_by_user=false;
    g_evteng_event_enabled=false;g_evteng_event_commenced=false;
    g_event_launched_by_user_action=false;g_evteng_run_event_until_canceled=false;
    g_foreground_start_event=false;g_foreground_reset_after_keydown=false;g_key_down_countdown=0;
    g_start_event_after_keydown=false;g_foreground_reset_after_demo=false;g_demo_event_countdown=0;
    g_isMaster=false;g_cloningInProgress=false;g_button_action_count=0;g_long_button_hold_intent=BUTTON_HOLD_DEFAULT;
    g_event_start_epoch=exhausted ? 1788860400 : 1789551600;
    g_event_finish_epoch=g_event_start_epoch+9*3600;g_days_to_run=3;g_schedule_day_index=0;
    g_evteng_loaded_start_epoch=g_event_start_epoch;g_evteng_loaded_finish_epoch=g_event_finish_epoch;
    now=1789547061;resyncLoadedEventWindowAfterClockSet();
    assert(g_schedule_day_index==(exhausted ? 3 : 0));
    // State produced by the manual start recorded at 08:36:48 in Joseph's log.
    now=1789547808;g_evteng_loaded_start_epoch=now-now%86400;g_evteng_loaded_finish_epoch=now+9*3600;
    g_event_launched_by_user_action=true;g_evteng_event_enabled=true;g_evteng_event_commenced=true;
    noteSessionStarted();flushSessionHistory();
    g_sleepType=SLEEP_UNTIL_NEXT_XMSN;g_button_wake_prior_sleep_type=g_sleepType;
    g_button_wake_prior_event_enabled=true;g_button_wake_prior_event_commenced=true;
    now=1789547940;g_evteng_on_the_air=-180;g_time_to_wake_up=now+170;
    g_go_to_sleep_now=false;g_awakenedBy=AWAKENED_BY_BUTTONPRESS;
}
void testButtonWake() {
    for(bool exhausted : {false,true}) {
        for(SleepType prior : {SLEEP_UNTIL_NEXT_XMSN,SLEEP_AFTER_EVENT}) {
            seedManualWake(exhausted);g_sleepType=prior;g_button_wake_prior_sleep_type=prior;
            auto session=g_session.record;auto writes=history.size();auto starts=launches;
            auto wake_at=g_time_to_wake_up;
            // Repeated authorized wakes do not shift the finish, slot countdown,
            // calendar progress, or session; there is no relaunch or EEPROM write.
            for(int repeat=0;repeat<3;++repeat) {
                restoreEventAfterWakeAuthorization();
                assert(g_evteng_loaded_start_epoch==1789516800 && g_evteng_loaded_finish_epoch==1789580208);
                assert(g_evteng_event_enabled && g_evteng_event_commenced && g_sleepType==SLEEP_AFTER_EVENT);
                assert(g_evteng_on_the_air==-180 && g_time_to_wake_up==wake_at);
                assert(g_schedule_day_index==(exhausted ? 3 : 0) && !g_foreground_start_event);
                assert(g_session.record.action==session.action && g_session.record.finish==session.finish);
            }
            reportSessionHistory();
            assert(history.size()==writes && launches==starts);
            assert(report.find("* Runtime: v=1 start=1789516800 finish=1789580208 onair=-180 sleep=1 enabled=1 commenced=1 manual=1 forever=0")!=std::string::npos);
            assert(report.find("* Wake before: v=1 start=1789516800 finish=1789580208")!=std::string::npos);
            assert(report.find("* Wake after: v=1 start=1789516800 finish=1789580208")!=std::string::npos);
        }
    }
    // Indefinite manual runs need neither unequal dates nor a valid clock.
    for(bool clock_set : {false,true}) {
        seedManualWake();g_evteng_run_event_until_canceled=true;
        now=clock_set ? now : 1000;
        g_event_finish_epoch=g_event_start_epoch=clock_set ? 1789551600 : 0;
        g_evteng_loaded_start_epoch=g_evteng_loaded_finish_epoch=g_event_start_epoch;
        auto finish=g_evteng_loaded_finish_epoch;
        restoreEventAfterWakeAuthorization();
        assert(g_evteng_event_enabled && g_evteng_event_commenced && g_evteng_run_event_until_canceled);
        assert(g_evteng_loaded_finish_epoch==finish && g_evteng_on_the_air==-180);
        assert(g_button_wake_at==(clock_set ? (uint32_t)now : 0));
    }
    // A stop during the authorization hold must override the captured pre-wake flags.
    for(int blocked=0;blocked<7;++blocked) {
        seedManualWake();
        switch(blocked) {
            case 0: g_event_canceled_by_user=true;noteSessionStopped(REASON_USER);break;
            case 1: g_device_enabled=false;noteSessionStopped(REASON_DEVICE_DISABLED);break;
            case 2: disabled=true;noteSessionStopped(REASON_DEVICE_DISABLED);break;
            case 3: g_thermal_shutdown=true;handleThermalSession();break;
            case 4: g_temperature_fresh_seconds=0;handleThermalSession();break;
            case 5: g_evteng_event_enabled=false;g_evteng_event_commenced=false;noteSessionStopped(REASON_USER);break;
            case 6: g_button_wake_prior_event_enabled=false;g_evteng_event_enabled=false;noteSessionStopped(REASON_USER);break;
        }
        auto action=g_session.record.action;
        restoreEventAfterWakeAuthorization();
        assert(!g_evteng_event_enabled && !g_foreground_start_event);
        assert(g_session.record.action==action && !g_session.active());
    }
    seedManualWake();now=1789580207;restoreEventAfterWakeAuthorization();assert(g_evteng_event_enabled);
    now=1789580208;restoreEventAfterWakeAuthorization();flushSessionHistory();
    assert(!g_evteng_event_enabled && !g_session.active() && g_session.record.reason==REASON_FINISH);
    auto records=history.size();restoreEventAfterWakeAuthorization();flushSessionHistory();assert(history.size()==records);
    // Scheduled-event controls retain the existing calendar and wake behavior.
    seedManualWake(false);g_event_launched_by_user_action=false;
    g_evteng_loaded_start_epoch=now-60;g_evteng_loaded_finish_epoch=now+3600;
    auto start=g_evteng_loaded_start_epoch,finish=g_evteng_loaded_finish_epoch;
    restoreEventAfterWakeAuthorization();
    assert(g_evteng_event_enabled && g_evteng_event_commenced && g_evteng_loaded_start_epoch==start && g_evteng_loaded_finish_epoch==finish);
    seedManualWake(false);g_event_launched_by_user_action=false;g_event_canceled_by_user=true;
    restoreEventAfterWakeAuthorization();assert(!g_evteng_event_enabled && !g_foreground_start_event);
    seedManualWake(false);g_event_launched_by_user_action=false;g_button_wake_prior_event_commenced=false;
    g_button_wake_prior_sleep_type=SLEEP_UNTIL_START_TIME;g_evteng_event_commenced=false;
    restoreEventAfterWakeAuthorization();
    assert(g_sleepType==SLEEP_UNTIL_START_TIME && g_evteng_loaded_start_epoch==g_event_start_epoch && !g_evteng_event_commenced);
    seedManualWake();g_event_launched_by_user_action=false;g_button_wake_prior_event_commenced=false;
    g_button_wake_prior_sleep_type=SLEEP_FOREVER;g_evteng_event_enabled=false;g_evteng_event_commenced=false;
    restoreEventAfterWakeAuthorization();assert(!g_foreground_start_event && !g_evteng_event_enabled);
    seedManualWake(false);g_event_launched_by_user_action=false;g_evteng_event_enabled=false;g_evteng_event_commenced=false;
    g_awakenedBy=POWER_UP_START;auto count=g_button_wake_count;
    restoreEventAfterWakeAuthorization();
    assert(g_foreground_start_event && g_evteng_loaded_start_epoch==g_event_start_epoch && g_button_wake_count==count);
    // Diagnostic history is bounded and reporting is passive, even at long uptimes.
    seedManualWake();g_button_wake_count=UINT32_MAX;restoreEventAfterWakeAuthorization();
    assert(g_button_wake_count==UINT32_MAX);
    g_button_wake_count=0;report.clear();reportWakeDiagnostics();
    assert(report.find("count=0")!=std::string::npos && report.find("* Wake before:")==std::string::npos);
    puts("Button wake: manual/indefinite preservation, repeated wakes, cancellation, thermal stop, exact finish, scheduled controls and passive diagnostics passed.");
}


void seedScheduledButton(int day=0, bool future=false, int phase=30) {
    seedManualWake(false);
    g_event_start_epoch=1788860400;g_event_finish_epoch=g_event_start_epoch+9*3600;
    g_schedule_day_index=day;g_days_to_run=3;
    now=g_event_start_epoch+day*SECONDS_24H+(future ? -600 : phase);
    g_event_launched_by_user_action=false;g_evteng_run_event_until_canceled=false;
    g_session={};history.clear();report.clear();g_session_pending_read=g_session_pending_count=0;
    assert(reloadLoadedEventWindowFromSavedSettings());
    startEventUsingRTC();noteSessionStarted();flushSessionHistory();
    keyed=false;rf_power_ons=0;g_button_action_count=0;g_long_button_press=false;LEDS.visible=true;
}
void markTemporary(int kind) {
    // A keydown suspends the engine; a demo temporarily runs it immediately.
    suspendEvent();
    if(kind==0) { g_key_down_countdown=100;g_evteng_run_event_until_canceled=true; }
    if(kind==1) { g_demo_event_countdown=100;g_evteng_run_event_until_canceled=true;g_event_launched_by_user_action=true;g_evteng_event_enabled=true;g_evteng_event_commenced=true; }
    if(kind==2) g_foreground_reset_after_keydown=true;
    if(kind==3) g_foreground_reset_after_demo=true;
    if(kind==4) g_start_event_after_keydown=true;
    powered=true;keyed=true;
}

void buttonTicks(bool closed,int ticks) { input_closed=closed;for(int i=0;i<ticks;++i) sampleButton(); }
void releaseButton() { buttonTicks(false,600);g_foreground_handle_counted_presses=0;g_long_button_press=false; }
void testButtonInput() {
    seedScheduledButton();releaseButton();
    // An authorized wake hold cannot become an awake long hold without release.
    g_device_wakeup_complete=false;buttonTicks(true,1200);
    g_device_wakeup_complete=true;buttonTicks(true,3000);
    assert(!g_long_button_press && !g_foreground_handle_counted_presses);
    releaseButton();buttonTicks(true,1500);
    assert(g_long_button_press);handleLongButtonPress();
    assert(g_schedule_day_index==1);
    buttonTicks(true,3000);assert(!g_long_button_press && !g_foreground_handle_counted_presses);
    releaseButton();
    // The first hold after LED timeout also only revives the display.
    seedScheduledButton();LEDS.visible=false;buttonTicks(true,1500);
    assert(g_pending_led_revival && !g_long_button_press);
    LEDS.init();buttonTicks(true,1500);assert(!g_long_button_press);
    releaseButton();
    // Expiration while the button is down must not change stop-test into cancel-day.
    seedScheduledButton();markTemporary(0);buttonTicks(true,120);
    assert(g_button_hold_intent==BUTTON_HOLD_STOP_TEST);
    cancelManualTransientState();startEventUsingRTC();
    buttonTicks(true,1500);assert(g_long_button_press && g_long_button_hold_intent==BUTTON_HOLD_STOP_TEST);
    handleLongButtonPress();assert(g_schedule_day_index==0 && g_button_action==BUTTON_STOP_TEST && g_evteng_event_enabled);
    releaseButton();
    // Similarly, a hold begun before start cannot cancel the day when start passes.
    seedScheduledButton(0,true);buttonTicks(true,120);
    assert(g_button_hold_intent==BUTTON_HOLD_SLEEP_SCHEDULE);
    now=g_event_start_epoch+1;startEventUsingRTC();buttonTicks(true,1500);
    assert(g_long_button_press);handleLongButtonPress();
    assert(g_schedule_day_index==0 && g_button_action==BUTTON_SLEEP_SCHEDULE && g_evteng_event_enabled);
    releaseButton();
    // Three separate short presses still reach the existing cancellation command.
    seedScheduledButton();
    for(int i=0;i<3;++i) { buttonTicks(true,30);buttonTicks(false,30); }
    buttonTicks(false,600);assert(g_foreground_handle_counted_presses==3 && !g_long_button_press);
    puts("Button input: wake/LED holds require release, transient expiry and scheduled-start boundaries preserve press intent, triple press passed.");
}
void testButtonEventPolicy() {
    // Ordinary scheduled cancellation is independent of the on-air phase.
    for(int day : {0,1,2}) for(int phase : {30,130,240}) for(bool hold : {false,true}) {
        seedScheduledButton(day,false,phase);
        if(hold) handleLongButtonPress();else cancelEventFromButton();
        assert(!keyed && !manualTestInProgress());
        assert(g_button_action==BUTTON_CANCEL_DAY && g_button_action_day_before==day);
        if(day<2) {
            assert(g_schedule_day_index==day+1 && !g_event_canceled_by_user);
            assert(g_evteng_loaded_start_epoch==g_event_start_epoch+(day+1)*86400);
            assert(g_sleepType==SLEEP_UNTIL_START_TIME && g_time_to_wake_up==g_evteng_loaded_start_epoch-15);
        } else { assert(!g_evteng_event_enabled && g_event_canceled_by_user); }
        if(hold) assert(g_go_to_sleep_now && !g_long_button_press);
        // A later user wake must not resurrect the canceled day.
        g_button_wake_prior_sleep_type=g_sleepType;
        g_button_wake_prior_event_enabled=g_evteng_event_enabled;
        g_button_wake_prior_event_commenced=g_evteng_event_commenced;
        restoreEventAfterWakeAuthorization();
        if(day==2) assert(!g_evteng_event_enabled && !g_foreground_start_event);
        else assert(g_evteng_loaded_start_epoch==g_event_start_epoch+(day+1)*86400);
    }
    // A hold during a carrier/demo (including pending expiry) preserves today.
    for(bool future : {false,true}) for(int phase : {30,130,240}) for(int kind=0;kind<5;++kind) {
        seedScheduledButton(1,future,phase);auto start=g_evteng_loaded_start_epoch, finish=g_evteng_loaded_finish_epoch;
        auto prior_countdown=g_evteng_on_the_air;
        markTemporary(kind);handleLongButtonPress();
        if(!future) {
            assert(prior_countdown==(phase==30 ? -90 : (phase==130 ? 50 : -180)));
            assert(g_evteng_on_the_air==prior_countdown);
        }
        assert(!manualTestInProgress() && !keyed && !powered && g_go_to_sleep_now);
        assert(g_schedule_day_index==1 && g_evteng_loaded_start_epoch==start && g_evteng_loaded_finish_epoch==finish);
        assert(!g_event_canceled_by_user && !g_event_launched_by_user_action && !g_evteng_run_event_until_canceled);
        assert(g_evteng_event_enabled && g_button_action==BUTTON_STOP_TEST);
        assert(g_sleepType==(future ? SLEEP_UNTIL_START_TIME : SLEEP_UNTIL_NEXT_XMSN));
        auto countdown=g_evteng_on_the_air;
        g_button_wake_prior_sleep_type=g_sleepType;g_button_wake_prior_event_enabled=true;
        g_button_wake_prior_event_commenced=g_evteng_event_commenced;
        restoreEventAfterWakeAuthorization();
        assert(g_evteng_loaded_start_epoch==start && g_evteng_loaded_finish_epoch==finish && g_schedule_day_index==1);
        if(!future) assert(g_evteng_on_the_air==countdown && g_evteng_event_enabled);
    }
    // Three presses still cancel TODAY during temporary tests; future tests re-arm.
    for(bool future : {false,true}) for(int kind=0;kind<5;++kind) {
        seedScheduledButton(0,future);markTemporary(kind);cancelEventFromButton();
        assert(!manualTestInProgress() && !keyed && !g_event_launched_by_user_action);
        assert(g_schedule_day_index==(future ? 0 : 1));
        assert(g_sleepType==SLEEP_UNTIL_START_TIME);
    }
    // A plain hold before a future event just sleeps, with no day advancement.
    seedScheduledButton(1,true);auto next_start=g_evteng_loaded_start_epoch;
    handleLongButtonPress();
    assert(g_go_to_sleep_now && g_sleepType==SLEEP_UNTIL_START_TIME && g_schedule_day_index==1);
    assert(g_evteng_loaded_start_epoch==next_start);
    // Never resurrect a canceled day, or bypass a thermal/sensor block, by stopping a demo.
    seedScheduledButton();markTemporary(1);g_event_canceled_by_user=true;handleLongButtonPress();
    assert(!g_evteng_event_enabled && !powered && g_schedule_day_index==0);
    for(bool hot : {false,true}) {
        seedScheduledButton();markTemporary(1);g_thermal_shutdown=hot;g_temperature_fresh_seconds=hot ? 10 : 0;
        handleLongButtonPress();assert(!g_evteng_event_enabled && !powered && !keyed && g_schedule_day_index==0);
    }
    // Stopping a test with no schedule cannot launch an indefinite run by timeout.
    seedManualWake();markTemporary(0);handleLongButtonPress();
    assert(!manualTestInProgress() && !g_evteng_event_enabled && !g_evteng_event_commenced && !g_evteng_run_event_until_canceled);
    assert(g_go_to_sleep_now && g_sleepType==SLEEP_FOREVER && !powered);
    // Exhausted/missing/equal/invalid settings cannot impose their duration on manual starts.
    for(int setting=0;setting<6;++setting) {
        seedManualWake();g_evteng_event_enabled=false;g_evteng_event_commenced=false;
        g_event_launched_by_user_action=false;g_evteng_run_event_until_canceled=false;
        if(setting==1) g_event_finish_epoch=g_event_start_epoch;
        if(setting==2) g_event_finish_epoch=g_event_start_epoch=0;
        if(setting==3) { g_schedule_day_index=0;g_days_to_run=1; }
        if(setting==4) { g_schedule_day_index=0;g_event_finish_epoch=g_event_start_epoch-1; }
        if(setting==5) now=1000;
        auto start=g_event_start_epoch, finish=g_event_finish_epoch;auto day=g_schedule_day_index;
        assert(!savedScheduleCanLimitManualRun());
        setupForFox(USE_CURRENT_FOX,START_EVENT_NOW_AND_RUN_AS_TIMED_EVENT);
        assert(g_evteng_event_enabled && g_evteng_run_event_until_canceled && g_event_launched_by_user_action);
        if(timeIsSet()) { assert(activation_start==now-now%86400 && activation_finish==FOREVER_EPOCH); }
        else assert(g_evteng_on_the_air==60); // Relative manual start, no calendar alignment.
        assert(g_event_start_epoch==start && g_event_finish_epoch==finish && g_schedule_day_index==day);
        g_go_to_sleep_now=false;g_evteng_on_the_air=-240;endSlotSleepPolicy();
        assert(g_go_to_sleep_now && g_sleepType==SLEEP_UNTIL_NEXT_XMSN && g_time_to_wake_up==now+230);
        g_button_wake_prior_sleep_type=g_sleepType;g_button_wake_prior_event_enabled=true;g_button_wake_prior_event_commenced=true;
        for(int wake=0;wake<3;++wake) {
            restoreEventAfterWakeAuthorization();
            assert(g_evteng_event_enabled && g_evteng_run_event_until_canceled && g_evteng_on_the_air==-240);
        }
        now+=10*3600;assert(!finishTimedEventIfExpired(now));
        cancelEventFromButton();assert(!g_evteng_event_enabled && !g_evteng_run_event_until_canceled);
    }
    // Relevant saved windows keep their existing duration semantics for explicit immediate starts.
    for(bool future : {false,true}) {
        seedScheduledButton(1,future);auto day=g_schedule_day_index;
        assert(savedScheduleCanLimitManualRun());
        setupForFox(USE_CURRENT_FOX,START_EVENT_NOW_AND_RUN_AS_TIMED_EVENT);
        assert(!g_evteng_run_event_until_canceled && g_evteng_loaded_finish_epoch==now+9*3600);
        assert(g_schedule_day_index==day);
    }
    // A temporary stop at the finish boundary follows normal calendar expiry,
    // without skipping the following day as an explicit day cancellation would.
    seedScheduledButton();markTemporary(0);now=g_event_finish_epoch;handleLongButtonPress();
    assert(g_schedule_day_index==1 && g_evteng_loaded_start_epoch==g_event_start_epoch+86400);
    assert(g_button_action==BUTTON_STOP_TEST && !keyed && !powered);
    auto writes=history.size();report.clear();reportWakeDiagnostics();
    assert(report.find("* Button action: v=1")!=std::string::npos && report.find("action=1 day_before=0 day_after=1")!=std::string::npos);
    assert(history.size()==writes); // Bounded diagnostics do not write EEPROM.
    puts("Button policy: active/future/last-day holds, temporary-test priority, cancellation, clockless/expired manual starts and diagnostics passed.");
}

void testUiBenchHooks() {
    uint16_t n=0;
    for(const char *bad : {"", "-1", "+2", "1x", "751", "999999999999999999999"})
        assert(!parseUiBenchNumber(bad,1,750,&n));
    assert(parseUiBenchNumber("750",1,750,&n) && n==750);
    clearUiButtonInjection();uint8_t old=1, current=0;
    applyUiButtonSample(&old,&current);assert(old==1 && current==0);
    seedScheduledButton();releaseButton();g_go_to_sleep_now=false;
    report.clear();assert(handleUiBenchCommand('B',"10"));
    assert(g_ui_button_injecting && g_ui_button_samples==10);
    handleUiBenchCommand('B',"20");assert(g_ui_button_samples==10); // Busy cannot overwrite.
    for(int i=0;i<10;++i) { old=current=1;applyUiButtonSample(&old,&current);assert(current==0); }
    current=1;applyUiButtonSample(&old,&current);
    assert(!g_ui_button_injecting && current==1 && old==0);
    handleUiBenchCommand('B',"250");current=1;applyUiButtonSample(&old,&current);
    assert(current==0);clearUiButtonInjection();
    assert(g_ui_button_injecting && !g_ui_button_samples); // Sleep interrupted the hold.
    current=1;applyUiButtonSample(&old,&current);
    assert(old==0 && current==1 && !g_ui_button_injecting); // Release still reaches detector.
    g_sleeping=true;handleUiBenchCommand('B',"10");assert(!g_ui_button_injecting);
    g_sleeping=false;input_closed=true;handleUiBenchCommand('B',"10");assert(!g_ui_button_injecting);
    input_closed=false;
    handleUiBenchCommand('T',"1");
    for(int i=0;i<40;++i) { ++now;captureUiBenchTick(); }
    assert(g_ui_trace_count==32 && g_ui_trace_next==8);
    report.clear();handleUiBenchCommand('T',"");assert(!g_ui_trace_interval);
    assert(g_ui_trace[(g_ui_trace_next+31)%32].at==now);
    auto count=g_ui_trace_count;captureUiBenchTick();assert(g_ui_trace_count==count);
    assert(report.find("* UI T end")!=std::string::npos);
    puts("UI bench: inactive passthrough, bounded input/release, busy/sleep/physical guards, parser limits, trace wrap/freeze passed.");
}

int main() {
    // Joseph's GO 1 run finished 57 seconds beyond the saved schedule.
    // Reporting must describe its outcome without mutating any runtime state.
    g_session={};g_session.begin(1789542600,1789516800,1789543257,false);
    g_session.stop(REASON_FINISH);g_evteng_event_enabled=false;
    const auto before=g_session.record;
    assert(reportManualEventOutcome());
    assert(report=="* Event completed.\n");
    assert(g_session.record.action==before.action && g_session.record.finish==before.finish);
    assert(!g_evteng_event_enabled && !powered && launches==0);
    report.clear();g_session={};g_session.begin(1,2,3,false);
    g_evteng_event_enabled=true;
    assert(!reportManualEventOutcome() && report.empty());
    g_evteng_event_enabled=false;g_session.stop(REASON_USER);
    assert(reportManualEventOutcome() && report=="* Event interrupted!\n");
    report.clear();g_session={};g_session.begin(1,2,100,true);
    g_session.pause(REASON_THERMAL);
    assert(!reportManualEventOutcome() && report.empty());
    g_session.resume(10,true,true);g_session.stop(REASON_FINISH);
    assert(reportManualEventOutcome() && report=="* Event finished with interruptions.\n");
    report.clear();g_session={};
    g_processor_temperature=30;
    updateTemperatureState(65); // The filter must not hide a hot raw reading.
    assert(transmitterThermallyBlocked());
    updateTemperatureState(61);assert(transmitterThermallyBlocked());
    updateTemperatureState(60);assert(!transmitterThermallyBlocked());
    updateTemperatureState(-1000);assert(transmitterThermallyBlocked());
    updateTemperatureState(30);assert(!transmitterThermallyBlocked());
    g_event_start_epoch=1788526800;g_event_finish_epoch=g_event_start_epoch+9*3600;
    now=g_event_start_epoch+100;g_temperature_fresh_seconds=10;
    assert(reloadLoadedEventWindowFromSavedSettings());
    assert(launchLoadedEvent());noteSessionStarted();flushSessionHistory();
    assert(history.back().action==SESSION_STARTED);
    unsigned initial=launches;
    g_thermal_shutdown=true;handleThermalSession();flushSessionHistory();
    assert(!powered && !g_evteng_event_enabled && g_evteng_event_commenced);
    assert(history.back().action==SESSION_PAUSED && history.back().reason==REASON_THERMAL);
    for(int i=0;i<10;++i) handleThermalSession();
    assert(launches==initial && g_session_pending_count==0 && g_schedule_day_index==0);
    g_thermal_shutdown=false;g_temperature_fresh_seconds=0;handleThermalSession();
    assert(launches==initial); // Cool but stale is unsafe.
    g_temperature_fresh_seconds=10;now+=300;handleThermalSession();flushSessionHistory();
    assert(powered && launches==initial+1 && history.back().action==SESSION_RESUMED);
    now=g_event_finish_epoch;assert(finishTimedEventIfExpired(now));flushSessionHistory();
    assert(history.back().action==SESSION_FINISHED_INTERRUPTED);
    assert(g_schedule_day_index==1 && g_evteng_loaded_start_epoch==g_event_start_epoch+86400);
    assert(!finishTimedEventIfExpired(now) && g_schedule_day_index==1);
    // Second day stays hot until finish; the third day is still scheduled.
    now=g_event_start_epoch+86400;assert(launchLoadedEvent());noteSessionStarted();
    g_thermal_shutdown=true;handleThermalSession();
    now=g_event_finish_epoch+86400;assert(finishTimedEventIfExpired(now));flushSessionHistory();
    assert(history.back().action==SESSION_INTERRUPTED && history.back().reason==REASON_THERMAL);
    assert(g_schedule_day_index==2 && g_evteng_loaded_start_epoch==g_event_start_epoch+2*86400);
    now=g_event_start_epoch+2*86400;g_thermal_shutdown=false;
    assert(launchLoadedEvent());noteSessionStarted();
    now=g_event_finish_epoch+2*86400;assert(finishTimedEventIfExpired(now));flushSessionHistory();
    assert(history.back().action==SESSION_COMPLETED && scheduledDaysRemaining()==0);
    reportSessionHistory();assert(report.find("action=4 reason=1 remaining=0")!=std::string::npos);
    // A hot pre-start window that expires must advance without inventing completion.
    g_session={};g_schedule_day_index=0;now=g_event_start_epoch-10;reloadLoadedEventWindowFromSavedSettings();
    g_thermal_start_pending=true;g_thermal_shutdown=true;g_evteng_event_commenced=false;g_evteng_event_enabled=false;
    now=g_event_finish_epoch;handleThermalSession();
    assert(g_schedule_day_index==1 && !g_thermal_start_pending);
    assert(g_evteng_loaded_start_epoch==g_event_start_epoch+86400);
    // User cancellation and charge-only mode must prevent a cooling resume.
    now=g_event_start_epoch+86400;g_thermal_shutdown=false;launchLoadedEvent();noteSessionStarted();
    g_thermal_shutdown=true;handleThermalSession();g_thermal_shutdown=false;
    g_event_canceled_by_user=true;initial=launches;handleThermalSession();assert(launches==initial);
    g_event_canceled_by_user=false;disabled=true;handleThermalSession();assert(launches==initial);
    disabled=false;noteSessionStopped(REASON_USER);flushSessionHistory();
    // Reset recovery preserves uncertainty about the physical stop time.
    g_session={};g_session.begin(g_event_start_epoch,now,now+100,true);queueSessionRecord();flushSessionHistory();
    g_session={};restoreSessionHistory();flushSessionHistory();
    assert(history.back().action==SESSION_INTERRUPTED && history.back().reason==REASON_RESET);
    assert(history.back().timestamp==0 && !(history.back().flags&SESSION_TIME_VALID));
    // A KEY test cannot leave RF on, or restart itself, after overheating.
    g_session={};g_evteng_event_enabled=false;g_evteng_event_commenced=false;
    g_key_down_countdown=100;powered=true;g_thermal_shutdown=false;noteSessionStarted();
    g_thermal_shutdown=true;handleThermalSession();flushSessionHistory();
    assert(!powered && !g_key_down_countdown && !g_session.active());
    assert(history.back().action==SESSION_INTERRUPTED && history.back().reason==REASON_THERMAL);
    testButtonWake();
    testButtonEventPolicy();
    testButtonInput();
    testUiBenchHooks();
}
'''
with tempfile.TemporaryDirectory(prefix='signalslinger-runtime-') as temp:
    cpp=Path(temp)/'runtime.cpp'
    cpp.write_text(stubs + ''.join(function(signature) for signature in [
        'bool timeIsSet(void)',
        'bool activateEventEngineUsingCurrentSettings(time_t startTime, time_t finishTime)',
        'static bool cancelManualTransientState(void)',
        'static bool manualTestInProgress(void)',
        'static uint8_t buttonHoldIntent(void)',
        'static void cancelEventFromButton(void)',
        'static bool handleScheduledButtonHold(void)',
        'static void handleLongButtonPress(void)',
        'static bool savedScheduleCanLimitManualRun(void)',
        'static bool advanceLoadedEventWindowAfterCurrentDayCancel(void)',
        'void suspendEvent(SessionReason reason)',
        'void setupForFox(Fox_t fox, EventAction_t action)',
        'bool eventIsScheduledToRunNow(time_t start_epoch, time_t finish_epoch)',
        'bool eventScheduledForTheFuture(time_t start_epoch, time_t finish_epoch)',
        'bool eventIsScheduledToRun(time_t *start_epoch, time_t *finish_epoch)',
        'bool eventIsScheduledToRun(volatile time_t *start_epoch, volatile time_t *finish_epoch)',
        'static bool currentLoadedEventWindowCanceled(void)',
        'static bool resyncLoadedEventWindowAfterClockSet(void)',
        'void restoreStateAfterButtonWakeAuthorization(void)',
        'static void restoreEventAfterWakeAuthorization(void)',
    ]) + function('static bool reloadLoadedEventWindowFromSavedSettings(void)') + function('static bool finishTimedEventIfExpired(time_t now)') + function('static void updateTemperatureState(float temperature)') + sleep_policy + input_policy + tests)
    binary=Path(temp)/'runtime-test'
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'SignalSlinger/include'),str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('Session runtime: three days, cooling, expired start, cancellation, reset and manual RF stop passed.')
