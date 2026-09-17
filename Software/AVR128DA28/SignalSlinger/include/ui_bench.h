#ifndef SIGNALSLINGER_UI_BENCH_H
#define SIGNALSLINGER_UI_BENCH_H

/* Opt-in, bounded RAM instrumentation. No EEPROM, forced RF, synthetic clock,
 * or wake-authority bypass. UI B joins the existing debounced input stream. */
static volatile uint16_t g_ui_button_samples = 0;
static volatile bool g_ui_button_injecting = false;
static uint8_t g_ui_button_previous = (1 << SWITCH);

static void clearUiButtonInjection(void)
{
    g_ui_button_samples = 0;
    // A hold can enter standby before its synthetic release is sampled. Keep
    // that release pending for the next awake sample, so the production detector
    // can re-arm long presses exactly as it does after a real switch opens.
    g_ui_button_injecting = g_ui_button_injecting && (g_ui_button_previous == 0);
    if(!g_ui_button_injecting) g_ui_button_previous = (1 << SWITCH);
}

static void applyUiButtonSample(uint8_t *previous, uint8_t *current)
{
    if(!g_ui_button_injecting) return;
    *previous = g_ui_button_previous;
    if(g_ui_button_samples) {
        *current = 0;
        --g_ui_button_samples;
    } else {
        // Deliver a release sample before returning to the real switch stream.
        g_ui_button_injecting = false;
    }
    g_ui_button_previous = *current;
}

struct UiBenchSample {
    uint32_t at;
    int32_t onair;
    uint16_t flags, key_ticks, demo_ticks;
    uint8_t sleep, day;
};
static const uint8_t UI_TRACE_CAPACITY = 32;
static UiBenchSample g_ui_trace[UI_TRACE_CAPACITY];
static volatile uint8_t g_ui_trace_interval = 0, g_ui_trace_due = 0;
static volatile uint8_t g_ui_trace_next = 0, g_ui_trace_count = 0;

static UiBenchSample uiBenchSample(void)
{
    UiBenchSample s;
    ENTER_CRITICAL(ui_bench_snapshot);
    s.at = (uint32_t)time(null);
    s.onair = g_evteng_on_the_air;
    s.key_ticks = g_key_down_countdown;
    s.demo_ticks = g_demo_event_countdown;
    s.sleep = g_sleepType;
    s.day = g_schedule_day_index;
    s.flags = (g_sleeping ? 1U : 0U) | (g_evteng_event_enabled ? 2U : 0U) |
        (g_evteng_event_commenced ? 4U : 0U) | (g_event_launched_by_user_action ? 8U : 0U) |
        (g_evteng_run_event_until_canceled ? 16U : 0U) | (txIsKeyed() ? 32U : 0U) |
        (txIsInitialized() ? 64U : 0U) | (PORTD_get_pin_level(LED_RED) ? 128U : 0U) |
        (PORTD_get_pin_level(LED_GREEN) ? 256U : 0U) | (g_thermal_shutdown ? 512U : 0U) |
        (g_event_canceled_by_user ? 1024U : 0U) | (g_device_wakeup_complete ? 2048U : 0U);
    EXIT_CRITICAL(ui_bench_snapshot);
    return s;
}

static void captureUiBenchTick(void)
{
    if(!g_ui_trace_interval) return;
    if(g_ui_trace_due && --g_ui_trace_due) return;
    g_ui_trace_due = g_ui_trace_interval;
    g_ui_trace[g_ui_trace_next] = uiBenchSample();
    g_ui_trace_next = (g_ui_trace_next + 1) % UI_TRACE_CAPACITY;
    if(g_ui_trace_count < UI_TRACE_CAPACITY) ++g_ui_trace_count;
}

static void reportUiBenchSample(const char *label, const UiBenchSample &s)
{
    char line[128];
    snprintf(line, sizeof(line), "* %s v=1 at=%lu onair=%ld flags=%u sleep=%u day=%u key=%u demo=%u\n",
        label, (unsigned long)s.at, (long)s.onair, s.flags, s.sleep, s.day, s.key_ticks, s.demo_ticks);
    sb_send_master_string(line);
}

static bool parseUiBenchNumber(const char *arg, long minimum, long maximum, uint16_t *value)
{
    if(!arg || !arg[0]) return false;
    // Deliberately accept only decimal digits; reject signs, tails and overflow.
    uint32_t n = 0;
    for(const char *p = arg; *p; ++p) {
        if(*p < '0' || *p > '9') return false;
        n = n * 10 + (unsigned)(*p - '0');
        if(n > (uint32_t)maximum) return false;
    }
    if(n < (uint32_t)minimum) return false;
    *value = (uint16_t)n;
    return true;
}

static bool handleUiBenchCommand(char command, const char *arg)
{
#ifdef SIGNALSLINGER_LATENCY_DIAGNOSTICS
    if(command == 'L') { serialLatencyControl(arg); return true; }
#endif
    if(command == 'D') {
        sb_send_master_string((char *)"* UI bench=1 samples_max=750 trace_capacity=32\n");
        reportUiBenchSample("UI state", uiBenchSample());
        char line[128];
        snprintf(line, sizeof(line), "* UI pins raw=%u led_active=%u fet=%u inject=%u trace=%u fresh=%u\n",
            rawSwitchIsClosed() ? 1 : 0, LEDS.active() ? 1 : 0, get_fet_driver() ? 1 : 0,
            atomic_read_u16(&g_ui_button_samples), g_ui_trace_interval, g_temperature_fresh_seconds);
        sb_send_master_string(line);
        uint16_t overrun, framing, parity;
        serialbusRxErrors(&overrun, &framing, &parity);
        snprintf(line, sizeof(line), "* UI serial overrun=%u framing=%u parity=%u\n", overrun, framing, parity);
        sb_send_master_string(line);
        reportWakeDiagnostics();
        return true;
    }
    if(command == 'B') {
        uint16_t samples;
        if(!parseUiBenchNumber(arg, 1, 750, &samples) || g_ui_button_injecting ||
           !g_device_wakeup_complete || g_sleeping || g_go_to_sleep_now || rawSwitchIsClosed() ||
           g_isMaster || g_cloningInProgress || g_long_button_press ||
           atomic_read_u16(&g_foreground_handle_counted_presses)) {
            sb_send_master_string((char *)"* Err: UI B 1-750 requires awake idle button\n");
        } else {
            ENTER_CRITICAL(ui_bench_press);
            g_ui_button_previous = (1 << SWITCH);
            g_ui_button_samples = samples;
            g_ui_button_injecting = true;
            EXIT_CRITICAL(ui_bench_press);
            sb_send_master_string((char *)"* UI B queued\n");
        }
        return true;
    }
    if(command == 'T') {
        if(arg && arg[0]) {
            uint16_t interval;
            if(!parseUiBenchNumber(arg, 0, 30, &interval)) {
                sb_send_master_string((char *)"* Err: UI T 0-30\n");
                return true;
            }
            ENTER_CRITICAL(ui_bench_trace);
            g_ui_trace_interval = 0;
            if(interval) { g_ui_trace_count = g_ui_trace_next = 0; g_ui_trace_due = 1; }
            g_ui_trace_interval = (uint8_t)interval;
            EXIT_CRITICAL(ui_bench_trace);
            sb_send_master_string((char *)"* UI T set\n");
        } else {
            // Freeze before reporting so wraparound cannot mix generations.
            g_ui_trace_interval = 0;
            uint8_t count = g_ui_trace_count;
            uint8_t first = (g_ui_trace_next + UI_TRACE_CAPACITY - count) % UI_TRACE_CAPACITY;
            for(uint8_t i = 0; i < count; ++i)
                reportUiBenchSample("UI trace", g_ui_trace[(first + i) % UI_TRACE_CAPACITY]);
            sb_send_master_string((char *)"* UI T end\n");
        }
        return true;
    }
    return false;
}
#endif
