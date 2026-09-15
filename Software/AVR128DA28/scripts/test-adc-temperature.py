#!/usr/bin/env python3
"""Exercise the production ADC driver against a timing-aware register model."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
source = (root / 'SignalSlinger/src/adc.cpp').read_text()
source = re.sub(r'^#include.*$', '', source, flags=re.MULTILINE)
stubs = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "adc.h"
#define F_CPU 24000000UL
#define MINIMUM_VALID_TEMP (-20.)
#define MAXIMUM_VALID_TEMP 85.
#define ISR(name) void name()
constexpr uint8_t ADC_MUXPOS_AIN0_gc=0, ADC_MUXPOS_AIN1_gc=1;
constexpr uint8_t ADC_MUXPOS_AIN4_gc=4, ADC_MUXPOS_AIN5_gc=5, ADC_MUXPOS_TEMPSENSE_gc=0x42;
constexpr uint8_t ADC_PRESC_DIV64_gc=10;
#define ADC_INITDLY_DLY16_gc 0x20
constexpr uint8_t ADC_ENABLE_bm=1, ADC_RESSEL_12BIT_gc=0, ADC_FREERUN_bm=2;
constexpr uint8_t ADC_STCONV_bm=1, ADC_RESRDY_bm=1;
constexpr uint8_t PORT_ISC_gm=7, PORT_ISC_INPUT_DISABLE_gc=4, PORT_PULLUPEN_bm=8;
constexpr uint8_t VREF_REFSEL_2V048_gc=1;
struct { uint8_t PIN0CTRL=0; } PORTD;
struct { uint8_t ADC0REF=0; } VREF;
struct { uint16_t TEMPSENSE0=4096, TEMPSENSE1=2000; } SIGROW;
static bool firstConversion=true;
static uint8_t latchedMux=0;
static unsigned conversions=0;
static bool stalled=false, pending=false;
static unsigned completionPolls=0, pollsLeft=0;
static uint16_t pendingResult=0;
static void advanceConversion();
struct Flags {
    uint8_t value=0;
    operator uint8_t() { advanceConversion(); return value; }
    void operator=(uint8_t clearMask) { value &= ~clearMask; }
};
struct ControlA {
    uint8_t value=0;
    operator uint8_t() const { return value; }
    void operator=(uint8_t next);
    void operator|=(uint8_t next) { *this=uint8_t(value|next); }
};
struct Result { uint16_t value=0; operator uint16_t(); };
struct Command { void operator=(uint8_t); };
struct {
    ControlA CTRLA;
    uint8_t CTRLC=0, CTRLD=0, SAMPCTRL=0, MUXPOS=0, INTCTRL=0;
    Flags INTFLAGS;
    Command COMMAND;
    Result RES;
} ADC0;
void ControlA::operator=(uint8_t next) {
    if(!(value & ADC_ENABLE_bm) && (next & ADC_ENABLE_bm)) {
        firstConversion=true;
        latchedMux=ADC0.MUXPOS;
    }
    if(!(next & ADC_ENABLE_bm)) pending=false;
    value=next;
}
Result::operator uint16_t() { ADC0.INTFLAGS.value=0; return value; }
static void advanceConversion() {
    if(pending && !stalled && (pollsLeft==0 || --pollsLeft==0)) {
        ADC0.RES.value=pendingResult;
        ADC0.INTFLAGS.value=ADC_RESRDY_bm;
        pending=false;
    }
}
void Command::operator=(uint8_t command) {
    assert(command==ADC_STCONV_bm);
    assert(ADC0.CTRLA & ADC_ENABLE_bm);
    assert(ADC0.CTRLC==ADC_PRESC_DIV64_gc);
    assert(VREF.ADC0REF==VREF_REFSEL_2V048_gc);
    ++conversions;
    const double clockUs=64.0e6/F_CPU;
    const unsigned delays[]={0,16,32,64,128,256};
    const unsigned delayIndex=ADC0.CTRLD>>5;
    assert(delayIndex<6);
    const bool settled=!firstConversion || delays[delayIndex]*clockUs>=25.0;
    // Silicon behavior: INITDLY can retain the mux selected at ADC enable.
    const uint8_t mux=(firstConversion && delayIndex) ? latchedMux : ADC0.MUXPOS;
    if(mux==ADC_MUXPOS_TEMPSENSE_gc) {
        // An undersettled sample reads low, so calibration reports false heat.
        const bool acquired=ADC0.SAMPCTRL*clockUs>=28.0;
        pendingResult=(settled && acquired) ? 1702 : 1680;
    } else {
        assert(ADC0.SAMPCTRL==0); // Keep voltage reads short.
        const uint16_t raw=mux==ADC_MUXPOS_AIN0_gc ? 580 :
            mux==ADC_MUXPOS_AIN1_gc ? 1600 : mux==ADC_MUXPOS_AIN4_gc ? 1650 : 1000;
        pendingResult=settled ? raw : 540;
    }
    firstConversion=false;
    pollsLeft=completionPolls;
    pending=true;
}
'''
tests = r'''
static void expectRoomTemperature(float value) { assert(std::fabs(value-24.85f)<0.01f); }
int main() {
    // Repeated foreground TMP requests shut down and restart the ADC each time.
    for(unsigned i=0;i<20;++i) {
        ADC0_SYSTEM_shutdown();
        expectRoomTemperature(readTemperature());
    }
    // Periodic channel switches and status reads must agree with cold starts.
    const ADC_Active_Channel_t voltages[]={ADCInternalBatteryVoltage, ADCExternalBatteryVoltage,
        ADC12VRegulatedVoltage, ADCTXAdjustableVoltage};
    for(auto channel:voltages) {
        for(unsigned i=0;i<5;++i) {
            const float expected=channel==ADCInternalBatteryVoltage ? 4.255f :
                channel==ADCExternalBatteryVoltage ? 11.65f :
                channel==ADC12VRegulatedVoltage ? 12.0125f : 7.3f;
            ADC0_SYSTEM_shutdown();
            assert(std::fabs(readVoltage(channel)-expected)<0.001f);
            ADC0_setADCChannel(ADCTemperature);
            ADC0_startConversion();
            assert(ADC0_conversionDone());
            expectRoomTemperature(temperatureCfromADC(ADC0_read()));
            assert(!ADC0_conversionDone());
        }
    }
    // Battery status also starts with a freshly enabled reference.
    ADC0_SYSTEM_shutdown();
    assert(std::fabs(readVoltage(ADCInternalBatteryVoltage)-4.255f)<0.001f);
    ADC0_SYSTEM_shutdown();
    expectRoomTemperature(readTemperature());
    // No added conversions or background activity; shutdown still disables ADC.
    assert(conversions==62);
    ADC0_setADCChannel(ADCShutdown);
    ADC0_startConversion();
    assert(conversions==62);
    assert(!(ADC0.CTRLA & ADC_ENABLE_bm));
    assert(g_adc_initialization==ADC_NOT_INITIALIZED);
    // Every input must work when a board helper enables the ADC first.
    const ADC_Active_Channel_t channels[]={ADCInternalBatteryVoltage, ADCExternalBatteryVoltage,
        ADC12VRegulatedVoltage, ADCTXAdjustableVoltage, ADCTemperature};
    const uint16_t raw[]={580,1600,1650,1000,1702};
    for(auto previous:channels) {
        for(unsigned i=0;i<5;++i) {
            ADC0_setADCChannel(previous);
            ADC0_startConversion();
            assert(ADC0_conversionDone());
            ADC0_read();
            ADC0_SYSTEM_shutdown();
            ADC0_SYSTEM_init(SINGLE_CONVERSION);
            // Exercise the same public operations as the periodic ADC scheduler.
            ADC0_setADCChannel(channels[i]);
            unsigned before=conversions;
            ADC0_startConversion();
            assert(ADC0_conversionDone());
            assert(ADC0_read()==raw[i]);
            assert(conversions==before+1); // No dummy conversions.
            // And the standby/foreground blocking path, starting pre-enabled.
            ADC0_SYSTEM_init(SINGLE_CONVERSION);
            if(channels[i]==ADCTemperature) expectRoomTemperature(readTemperature());
            else assert(std::fabs(readVoltage(channels[i])-(raw[i]*0.00725f+0.05f))<0.001f);
        }
    }
    // A stale ready flag must not satisfy a newly started asynchronous request.
    completionPolls=3;
    ADC0.RES.value=3000;
    ADC0.INTFLAGS.value=ADC_RESRDY_bm;
    ADC0_setADCChannel(ADCInternalBatteryVoltage);
    ADC0_startConversion();
    assert(!ADC0_conversionDone());
    assert(!ADC0_conversionDone());
    assert(ADC0_conversionDone());
    assert(ADC0_read()==580);
    // Last allowed completion succeeds; the next poll is a timeout.
    completionPolls=10000;
    expectRoomTemperature(readTemperature());
    completionPolls=10001;
    assert(readVoltage(ADCInternalBatteryVoltage)==0);
    assert(!(ADC0.CTRLA & ADC_ENABLE_bm) && !pending);
    assert(!isValidTemp(readTemperature()));
    assert(!(ADC0.CTRLA & ADC_ENABLE_bm) && !pending);
    // A never-completing conversion rejects stale data on both blocking paths.
    stalled=true;
    ADC0.RES.value=1702;
    ADC0.INTFLAGS.value=ADC_RESRDY_bm;
    assert(readVoltage(ADCInternalBatteryVoltage)==0);
    assert(!isValidTemp(readTemperature()));
    assert(!(ADC0.CTRLA & ADC_ENABLE_bm) && !pending);
    // Recovery reinitializes and samples the requested channel correctly.
    stalled=false; completionPolls=3;
    assert(std::fabs(readVoltage(ADCInternalBatteryVoltage)-4.255f)<0.001f);
    expectRoomTemperature(readTemperature());
    unsigned beforeInvalid=conversions;
    assert(readVoltage(ADCShutdown)==0);
    assert(readVoltage(static_cast<ADC_Active_Channel_t>(99))==0);
    assert(conversions==beforeInvalid && !(ADC0.CTRLA & ADC_ENABLE_bm));
    // Existing free-running setup can still transition to a single-shot input.
    completionPolls=0;
    ADC0_setADCChannel(ADCExternalBatteryVoltage);
    ADC0_SYSTEM_init(FREE_RUNNING);
    assert(ADC0.CTRLA & ADC_FREERUN_bm);
    ADC0_setADCChannel(ADCInternalBatteryVoltage);
    assert(!(ADC0.CTRLA & ADC_FREERUN_bm));
    ADC0_startConversion();
    assert(ADC0_conversionDone() && ADC0_read()==580);
    ADC0_SYSTEM_shutdown();
    assert(!(ADC0.CTRLA & ADC_ENABLE_bm) && !pending);
    puts("PASS ADC: all channels, pre-enabled startup, stale flags, timeout boundaries, recovery, shutdown");
}
'''
with tempfile.TemporaryDirectory(prefix='signalslinger-adc-') as temp:
    cpp = Path(temp) / 'test.cpp'
    exe = Path(temp) / 'test'
    cpp.write_text(stubs + '\n' + source + '\n' + tests)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'SignalSlinger/include'), str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
