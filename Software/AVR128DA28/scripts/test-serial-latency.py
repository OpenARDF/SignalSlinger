#!/usr/bin/env python3
"""Host-check actual opt-in latency accounting with deterministic RTC/register values."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parent.parent
stubs=r'''#include <stdint.h>
#include <cstring>
#include <string>
#include <cassert>
#define SIGNALSLINGER_LATENCY_DIAGNOSTICS
#define ENTER_CRITICAL(x)
#define EXIT_CRITICAL(x)
#define USART_BUFOVF_bm 64
#define USART_RXCIF_bm 128
#define USART_0 0
int g_serialbus_usart_number=0;
struct { uint16_t CNT=0,PER=32767; } RTC;
struct { uint8_t RXDATAH=0; } USART0,USART1;
struct { uint8_t STATUS=0; } CPUINT,NVMCTRL;
std::string output;
void sb_send_master_string(char *s) { output+=s; }
#include "serial_latency.h"
#include "serial_latency_impl.h"
'''
tests=r'''
int main() {
    { SerialLatencyScope s(LAT_KEY); RTC.CNT=100; }
    assert(latency_metrics[LAT_KEY].calls==0);
    serialLatencyControl("1"); CPUINT.STATUS=1; RTC.CNT=32760;
    auto t=serialLatencyBegin(LAT_TCB0);RTC.CNT=10;serialLatencyEnd(LAT_TCB0,t);
    assert(latency_metrics[LAT_TCB0].maximum==18 && latency_metrics[LAT_TCB0].irq_maximum==18);
    serialLatencyControl("1"); RTC.CNT=100;
    auto outer=serialLatencyBegin(LAT_TCB0);auto key=serialLatencyBegin(LAT_KEY);auto inner=serialLatencyBegin(LAT_I2C_READ);
    RTC.CNT=200;USART0.RXDATAH=192;serialLatencyEnd(LAT_I2C_READ,inner);
    assert(latency_fault_count==1 && latency_faults[0].scope==LAT_I2C_READ);
    assert(latency_faults[0].mask==((1<<LAT_TCB0)|(1<<LAT_KEY)|(1<<LAT_I2C_READ)));
    serialLatencyEnd(LAT_KEY,key);serialLatencyEnd(LAT_TCB0,outer);
    assert(latency_fault_count==1 && latency_mask==0);
    assert(latency_metrics[LAT_TCB0].over_2ms==1);
    serialLatencyRx(192);assert(latency_fault_count==1);
    for(int i=0;i<30;++i) serialLatencyRx(192);
    assert(latency_fault_count==16);
    NVMCTRL.STATUS=2;t=serialLatencyBegin(LAT_EE_READ);RTC.CNT+=320;serialLatencyEnd(LAT_EE_READ,t);
    assert(latency_metrics[LAT_EE_READ].busy_entry==1);
    USART0.RXDATAH=0;latency_metrics[LAT_KEY].calls=UINT16_MAX;
    t=serialLatencyBegin(LAT_KEY);serialLatencyEnd(LAT_KEY,t);
    assert(latency_metrics[LAT_KEY].calls==UINT16_MAX);
    serialLatencyControl("");assert(!g_serial_latency_enabled && output.find("* LAT end")!=std::string::npos);
    serialLatencyControl("1");assert(latency_fault_count==0 && latency_metrics[LAT_TCB0].calls==0);
    USART0.RXDATAH=64;t=serialLatencyBegin(LAT_EE_READ);serialLatencyEnd(LAT_EE_READ,t);
    assert(latency_fault_count==0); // Stale error bits without RXCIF are not a new fault.
    USART0.RXDATAH=0;
    CPUINT.STATUS=2;RTC.CNT=100;t=serialLatencyBegin(LAT_TCB2);serialLatencyEnd(LAT_TCB2,t);
    RTC.CNT=430;t=serialLatencyBegin(LAT_TCB2);serialLatencyEnd(LAT_TCB2,t);
    assert(latency_metrics[LAT_TCB2].gap==330);
    serialLatencyControl("0");serialLatencyControl("2");assert(!g_serial_latency_enabled);
    assert(output.find("* Err:")!=std::string::npos);
    puts("Serial latency: disabled path, RTC wrap, nested attribution, fault bounds, saturation and freeze/reset passed.");
}
'''
with tempfile.TemporaryDirectory(prefix="ss-latency-host-") as tmp:
    tmp=Path(tmp);(tmp/"atomic.h").write_text("")
    (tmp/"test.cpp").write_text(stubs+tests)
    subprocess.run(["c++","-std=c++17","-Wall","-Wextra","-Werror","-I",str(tmp),"-I",str(root/"SignalSlinger/include"),str(tmp/"test.cpp"),"-o",str(tmp/"test")],check=True)
    subprocess.run([str(tmp/"test")],check=True)
