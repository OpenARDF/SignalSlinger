/* Included by main.cpp after its globals and forward declarations. Hardware
 * effects stay in the foreground; the RTC only captures bounded RAM records. */
#ifndef SIGNALSLINGER_SESSION_RUNTIME_H
#define SIGNALSLINGER_SESSION_RUNTIME_H

static SessionLifecycle g_session;
static SessionRecord g_session_pending[8];
static volatile uint8_t g_session_pending_read = 0, g_session_pending_count = 0;
static bool g_session_history_gap = false;
static volatile uint8_t g_temperature_fresh_seconds = 0;
static volatile float g_last_temperature_sample = -1000;
static bool g_thermal_start_pending = false;

bool transmitterThermallyBlocked(void)
{
    return g_thermal_shutdown_enabled && (g_thermal_shutdown || !g_temperature_fresh_seconds);
}

/* Caller holds a critical section when invoked from foreground. */
static void queueSessionRecord(bool time_known = true)
{
    SessionRecord record = g_session.record;
    time_t now = time(null);
    record.timestamp = time_known && timeIsSet() ? (uint32_t)now : 0;
    record.flags &= ~SESSION_TIME_VALID;
    if(record.timestamp) record.flags |= SESSION_TIME_VALID;
    float safety_temperature = g_last_temperature_sample > g_processor_temperature ? g_last_temperature_sample : g_processor_temperature;
    record.temperature = g_temperature_fresh_seconds && isValidTemp(safety_temperature) ? (int16_t)(safety_temperature * 10) : INT16_MIN;
    record.threshold = (uint8_t)g_thermal_shutdown_threshold;
    if(g_session_pending_count == 8) {
        g_session_pending_read = (g_session_pending_read + 1) % 8;
        --g_session_pending_count; g_session_history_gap = true;
    }
    if(g_session_history_gap) record.flags |= SESSION_HISTORY_GAP;
    g_session_pending[(g_session_pending_read + g_session_pending_count) % 8] = record;
    ++g_session_pending_count;
    g_session.record.timestamp = record.timestamp;
    g_session.record.flags = record.flags;
}

static void flushSessionHistory(void)
{
    for(;;) {
        SessionRecord record;
        bool have_record;
        ENTER_CRITICAL(session_pop);
        have_record = g_session_pending_count != 0;
        if(have_record) {
            record = g_session_pending[g_session_pending_read];
            g_session_pending_read = (g_session_pending_read + 1) % 8;
            --g_session_pending_count;
        }
        EXIT_CRITICAL(session_pop);
        if(!have_record) break;
        appendSessionHistory(record);
    }
}

static void restoreSessionHistory(void)
{
    SessionRecord latest;
    if(readSessionHistory(&latest, 1)) {
        g_session.record = latest;
        if(g_session.active()) {
            g_session.stop(REASON_RESET);
            queueSessionRecord(false); /* A reset cannot reveal the exact power-loss time. */
        }
    }
}

static void noteSessionStarted(void)
{
    ENTER_CRITICAL(session_begin);
    time_t start = g_evteng_loaded_start_epoch, finish = g_evteng_loaded_finish_epoch;
    bool scheduled = !g_evteng_run_event_until_canceled && !g_event_launched_by_user_action;
    if(g_key_down_countdown && !g_session.active() && !transmitterThermallyBlocked() && !getDisableTransmissions()) {
        if(g_session.begin(0, time(null), 0, false)) queueSessionRecord();
    }
    if((g_evteng_event_commenced || g_evteng_run_event_until_canceled) && g_evteng_event_enabled &&
       !transmitterThermallyBlocked() && !getDisableTransmissions() && !g_session.paused() && !g_key_down_countdown && !g_foreground_reset_after_keydown) {
        if(g_session.begin(g_event_start_epoch, start, finish, scheduled)) queueSessionRecord();
    }
    EXIT_CRITICAL(session_begin);
}

static void noteSessionStopped(SessionReason reason)
{
    ENTER_CRITICAL(session_stop);
    if(g_session.stop(reason)) queueSessionRecord();
    EXIT_CRITICAL(session_stop);
}

static bool sessionCooling(void)
{
    ENTER_CRITICAL(session_cooling);
    bool cooling = g_session.paused();
    EXIT_CRITICAL(session_cooling);
    return cooling;
}

/* A pause preserves commenced state so the RTC can still close this window at
 * its original finish. It never increments the calendar index on a hot sample. */
static void handleThermalSession(void)
{
    if(transmitterThermallyBlocked()) {
        if(g_evteng_event_enabled || g_evteng_event_commenced || g_evteng_run_event_until_canceled || g_key_down_countdown || txIsInitialized()) {
            ENTER_CRITICAL(session_trip);
            bool scheduled = !g_evteng_run_event_until_canceled && !g_event_launched_by_user_action;
            if(scheduled && !g_evteng_event_commenced) g_thermal_start_pending = true;
            if(!g_session.active() && g_evteng_event_commenced && g_evteng_event_enabled)
                g_session.begin(g_event_start_epoch, g_evteng_loaded_start_epoch, g_evteng_loaded_finish_epoch, scheduled);
            if(!g_session.active() && g_key_down_countdown) g_session.begin(0, time(null), 0, false);
            if(g_session.pause(g_thermal_shutdown ? REASON_THERMAL : REASON_SENSOR)) queueSessionRecord();
            g_evteng_event_enabled = false;
            g_foreground_enable_transmitter = false;
            g_foreground_start_event = false;
            g_evteng_on_the_air = 0;
            EXIT_CRITICAL(session_trip);
            keyTransmitter(OFF); powerToTransmitter(OFF);
        }
    }

    if(sessionCooling()) {
        /* Keep ADC and normal fan policy alive while waiting for cooling. */
        g_go_to_sleep_now = false;
        atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
        time_t now = time(null);
        SessionRecord record;
        ENTER_CRITICAL(session_resume_check);
        record = g_session.record;
        bool authorized = g_device_enabled && !getDisableTransmissions() && !g_event_canceled_by_user && !g_isMaster && !g_cloningInProgress &&
            record.start == (uint32_t)g_evteng_loaded_start_epoch && record.finish == (uint32_t)g_evteng_loaded_finish_epoch;
        bool resume = g_session.canResume(now, !transmitterThermallyBlocked(), authorized);
        EXIT_CRITICAL(session_resume_check);
        if(resume && launchLoadedEvent()) {
            ENTER_CRITICAL(session_resumed);
            if(g_session.resume(time(null), !transmitterThermallyBlocked(), authorized)) queueSessionRecord();
            EXIT_CRITICAL(session_resumed);
        }
        if(!(record.flags & SESSION_SCHEDULED)) {
            /* Manual/key/demo modes require another explicit start. */
            noteSessionStopped((SessionReason)record.reason);
            cancelManualTransientState();
            g_evteng_event_commenced = false;
            g_evteng_run_event_until_canceled = false;
            g_sleepType = SLEEP_FOREVER;
        }
    }

    if(g_thermal_start_pending && !sessionCooling()) {
        g_go_to_sleep_now = false;
        atomic_write_u16(&g_evteng_sleepshutdown_seconds, 300);
        if(!g_device_enabled || g_event_canceled_by_user) g_thermal_start_pending = false;
        else if(timeIsSet() && atomic_read_time(&g_evteng_loaded_finish_epoch) <= time(null)) {
            /* A blocked start missed its window; preserve the next calendar day. */
            g_thermal_start_pending = false;
            if(reloadLoadedEventWindowFromSavedSettings()) {
                atomic_write_time(&g_time_to_wake_up, atomic_read_time(&g_evteng_loaded_start_epoch) - 15);
                g_sleepType = SLEEP_UNTIL_START_TIME;
                g_go_to_sleep_now = true;
            }
        }
        else if(!transmitterThermallyBlocked() && !g_isMaster && !g_cloningInProgress) {
            g_thermal_start_pending = false;
            g_foreground_start_event = true;
        }
    }
}

static uint8_t scheduledDaysRemaining(void)
{
    if(!timeIsSet()) return g_days_to_run;
    uint8_t day = sessionDayIndex(atomic_read_time(&g_event_start_epoch), atomic_read_time(&g_event_finish_epoch), g_days_to_run, time(null));
    if(day < g_schedule_day_index) day = g_schedule_day_index;
    return day < g_days_to_run ? g_days_to_run - day : 0;
}

static void reportSessionHistory(void)
{
    flushSessionHistory();
    SessionRecord current;
    ENTER_CRITICAL(session_report);
    current = g_session.record;
    EXIT_CRITICAL(session_report);
    uint8_t state = SESSION_NONE;
    if(sessionActionActive(current.action)) state = current.action;
    else if(g_event_start_epoch > MINIMUM_VALID_EPOCH && g_event_finish_epoch > g_event_start_epoch && g_days_to_run && !scheduledDaysRemaining()) {
        uint64_t end = (uint64_t)atomic_read_time(&g_event_finish_epoch) + (uint64_t)(g_days_to_run ? g_days_to_run - 1 : 0) * 86400;
        state = current.schedule == (uint32_t)g_event_start_epoch && current.finish == end ? current.action : SESSION_EXPIRED;
    }
    else if(current.schedule == (uint32_t)g_event_start_epoch && current.action == SESSION_INTERRUPTED && current.reason == REASON_USER)
        state = SESSION_INTERRUPTED;
    char line[180];
    snprintf(line, sizeof(line), "* Session state: v=1 action=%u reason=%u remaining=%u blocked=%u\n",
             state, current.reason, scheduledDaysRemaining(), transmitterThermallyBlocked() ? 1 : 0);
    sb_send_string(line);
    SessionRecord history[16];
    uint8_t count = readSessionHistory(history, 16);
    snprintf(line, sizeof(line), "* Session history: v=1 count=%u capacity=%u\n", count, sessionHistoryCapacity());
    sb_send_string(line);
    for(uint8_t i = 0; i < count; ++i) {
        const SessionRecord& r = history[i];
        snprintf(line, sizeof(line), "* Session record: v=1 seq=%lu base=%lu start=%lu finish=%lu at=%lu action=%u reason=%u flags=%u temp=%d limit=%u\n",
                 (unsigned long)r.sequence, (unsigned long)r.schedule, (unsigned long)r.start, (unsigned long)r.finish,
                 (unsigned long)r.timestamp, r.action, r.reason, r.flags, r.temperature, r.threshold);
        sb_send_string(line);
    }
}

#endif
