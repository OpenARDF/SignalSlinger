// Included once by driver_isr.cpp. RTC is observed, never reconfigured.
#ifdef SIGNALSLINGER_LATENCY_DIAGNOSTICS
#include "serial_latency.h"
#include <stdio.h>
#include <atomic.h>
volatile bool g_serial_latency_enabled=false;
struct LatencyMetric { uint16_t calls, maximum, irq_maximum, over_2ms, busy_entry, gap; };
struct LatencyFault { uint16_t tick, mask, elapsed; uint8_t scope, cpu, status, nvm; };
static LatencyMetric latency_metrics[LAT_COUNT];
static LatencyFault latency_faults[16];
static uint16_t latency_mask=0;
static uint8_t latency_fault_count=0;
static bool latency_fault_latched=false;
static uint16_t latency_tick_previous=0;
static bool latency_tick_valid=false;
static uint8_t latencyRxStatus() {
    return g_serialbus_usart_number==USART_0 ? USART0.RXDATAH : USART1.RXDATAH;
}
static void latencyFault(uint8_t scope, uint16_t elapsed, uint8_t status) {
    // RXDATAH retains old error bits when no unread byte is present.
    // Only attribute a pending hardware overflow when RXCIF is also set.
    if((status & (USART_BUFOVF_bm | USART_RXCIF_bm)) != (USART_BUFOVF_bm | USART_RXCIF_bm) || latency_fault_latched) return;
    latency_fault_latched=true;
    if(latency_fault_count>=16) return;
    LatencyFault &f=latency_faults[latency_fault_count++];
    f.tick=RTC.CNT; f.mask=latency_mask; f.elapsed=elapsed; f.scope=scope;
    f.cpu=CPUINT.STATUS; f.status=status; f.nvm=NVMCTRL.STATUS;
}
SerialLatencyToken serialLatencyBegin(uint8_t id) {
    SerialLatencyToken t;
    ENTER_CRITICAL(latency_begin);
    t.context=CPUINT.STATUS; t.nvm=NVMCTRL.STATUS; t.prior_mask=latency_mask; t.start=RTC.CNT;
    if(id==LAT_TCB2) {
        if(latency_tick_valid) {
            uint16_t gap=t.start>=latency_tick_previous ? t.start-latency_tick_previous : (uint32_t)t.start+RTC.PER+1-latency_tick_previous;
            if(gap>latency_metrics[id].gap) latency_metrics[id].gap=gap;
        }
        latency_tick_previous=t.start; latency_tick_valid=true;
    }
    latency_mask |= (1U<<id);
    EXIT_CRITICAL(latency_begin);
    return t;
}
void serialLatencyEnd(uint8_t id, SerialLatencyToken t) {
    ENTER_CRITICAL(latency_end);
    uint16_t now=RTC.CNT;
    uint16_t elapsed=now>=t.start ? now-t.start : (uint32_t)now+RTC.PER+1-t.start;
    LatencyMetric &m=latency_metrics[id];
    if(m.calls!=UINT16_MAX) ++m.calls;
    if((t.nvm & 2) && m.busy_entry!=UINT16_MAX) ++m.busy_entry;
    if(elapsed>m.maximum) m.maximum=elapsed;
    if(t.context && elapsed>m.irq_maximum) m.irq_maximum=elapsed;
    if(elapsed>=66 && m.over_2ms!=UINT16_MAX) ++m.over_2ms;
    latencyFault(id,elapsed,latencyRxStatus());
    latency_mask=t.prior_mask;
    EXIT_CRITICAL(latency_end);
}
void serialLatencyRx(uint8_t status) {
    if(!g_serial_latency_enabled) return;
    ENTER_CRITICAL(latency_rx);
    latencyFault(LAT_RX,0,status);
    latency_fault_latched=false; // This byte's status was consumed by RXDATAL.
    EXIT_CRITICAL(latency_rx);
}
void serialLatencyControl(const char *arg) {
    if(arg && arg[0]) {
        if((arg[0]!='0' && arg[0]!='1') || arg[1]) {
            sb_send_master_string((char*)"* Err: UI L [0|1]\n"); return;
        }
        ENTER_CRITICAL(latency_control);
        g_serial_latency_enabled=false;
        if(arg[0]=='1') {
            memset(latency_metrics,0,sizeof(latency_metrics));
            latency_mask=0; latency_fault_count=0; latency_fault_latched=false; latency_tick_valid=false;
            g_serial_latency_enabled=true;
        }
        EXIT_CRITICAL(latency_control);
        sb_send_master_string((char*)"* UI L set\n"); return;
    }
    // Freeze before printing. No unsolicited output from any interrupt.
    g_serial_latency_enabled=false;
    char line[128];
    snprintf(line,sizeof(line),"* LAT v=1 hz=32768 period=%u capacity=16 faults=%u\n",RTC.PER,latency_fault_count);
    sb_send_master_string(line);
    for(uint8_t i=0;i<LAT_COUNT;++i) {
        LatencyMetric &m=latency_metrics[i];
        snprintf(line,sizeof(line),"* LAT metric id=%u calls=%u max=%u irq_max=%u over2ms=%u busy=%u gap=%u\n",i,m.calls,m.maximum,m.irq_maximum,m.over_2ms,m.busy_entry,m.gap);
        sb_send_master_string(line);
    }
    for(uint8_t i=0;i<latency_fault_count;++i) {
        LatencyFault &f=latency_faults[i];
        snprintf(line,sizeof(line),"* LAT fault tick=%u scope=%u mask=%u elapsed=%u cpu=%u status=%u nvm=%u\n",f.tick,f.scope,f.mask,f.elapsed,f.cpu,f.status,f.nvm);
        sb_send_master_string(line);
    }
    sb_send_master_string((char*)"* LAT end\n");
}
#endif
