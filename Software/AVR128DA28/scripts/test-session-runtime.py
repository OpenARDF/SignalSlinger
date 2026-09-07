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
    return source[start:end]

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
static time_t g_event_start_epoch, g_event_finish_epoch, g_evteng_loaded_start_epoch, g_evteng_loaded_finish_epoch, g_time_to_wake_up;
static uint8_t g_days_to_run=3, g_schedule_day_index=0;
static int32_t g_evteng_on_the_air=0;
static float g_processor_temperature=65, g_processor_min_temperature=200, g_processor_max_temperature=-100;
static bool g_turn_on_fan=false;
static int8_t g_thermal_shutdown_threshold=65;
static enum { SLEEP_FOREVER, SLEEP_UNTIL_START_TIME } g_sleepType;
static struct { void init() {} } LEDS;
static bool evaluateThermalShutdownState(float t, bool prior) { return evaluateThermalShutdownStateForPolicy(t,g_thermal_shutdown_threshold,5,g_thermal_shutdown_enabled,prior); }
bool timeIsSet() { return now > MINIMUM_VALID_EPOCH; }
bool isValidTemp(float value) { return value > -100 && value < 200; }
bool getDisableTransmissions() { return disabled; }
bool txIsInitialized() { return powered; }
void cancelManualTransientState() { g_key_down_countdown=0; }
void keyTransmitter(bool) {}
void powerToTransmitter(bool on) { powered=on; }
void atomic_write_u16(uint16_t* p, uint16_t v) { *p=v; }
void atomic_write_i32(int32_t* p, int32_t v) { *p=v; }
time_t atomic_read_time(time_t* p) { return *p; }
void atomic_write_time(time_t* p, time_t v) { *p=v; }
void atomic_read_time_pair(time_t* a,time_t* b,time_t* c,time_t* d) { *c=*a; *d=*b; }
void atomic_write_time_pair(time_t* a,time_t* b,time_t c,time_t d) { *a=c; *b=d; }
bool eventScheduledForTheFuture(time_t a,time_t b) { return a > now && b > a; }
bool eventIsScheduledToRunNow(time_t a,time_t b) { return now >= a && now < b; }
static bool reloadLoadedEventWindowFromSavedSettings();
static bool finishTimedEventIfExpired(time_t now);
static std::vector<SessionRecord> history;
static std::string report;
void sb_send_string(const char* s) { report+=s; }
void appendSessionHistory(SessionRecord r) { history.push_back(r); }
uint8_t sessionHistoryCapacity() { return 6; }
uint8_t readSessionHistory(SessionRecord* out,uint8_t capacity) {
    unsigned count=std::min<unsigned>(capacity,history.size());
    std::copy(history.end()-count,history.end(),out);return count;
}
static unsigned launches=0;
bool launchLoadedEvent() {
    ++launches;
    if(now >= g_evteng_loaded_finish_epoch) return false;
    g_evteng_event_enabled=true;g_evteng_event_commenced=true;powered=true;return true;
}
#include "session_runtime.h"
'''
tests = r'''
int main() {
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
}
'''
with tempfile.TemporaryDirectory(prefix='signalslinger-runtime-') as temp:
    cpp=Path(temp)/'runtime.cpp'
    cpp.write_text(stubs + function('static bool reloadLoadedEventWindowFromSavedSettings(void)') + function('static bool finishTimedEventIfExpired(time_t now)') + function('static void updateTemperatureState(float temperature)') + tests)
    binary=Path(temp)/'runtime-test'
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'SignalSlinger/include'),str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('Session runtime: three days, cooling, expired start, cancellation, reset and manual RF stop passed.')
