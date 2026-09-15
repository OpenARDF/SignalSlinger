#!/usr/bin/env python3
"""Exercise the production ADC service and switch arbitration with a 300 Hz clock."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
main = (root / 'SignalSlinger/main.cpp').read_text()
binio = (root / 'SignalSlinger/src/binio.cpp').read_text()
definitions = (root / 'SignalSlinger/include/binio.h').read_text()


def function(source, signature):
    start = source.index(signature)
    body = source.index('{', start)
    depth, end = 1, body + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


enum = definitions[definitions.index('enum hardwareResourceClients'):]
enum = enum[:enum.index('};') + 2]
adc_start = main.index('\t\tbool priorProbe =')
adc_end = main.index('\n\t}\n\n\tTCB0.INTFLAGS', adc_start)
adc_service = main[adc_start:adc_end]
status_start = main.index('ENTER_CRITICAL(battery_status_read);')
status_end = main.index('EXIT_CRITICAL(battery_status_read);', status_start) + len('EXIT_CRITICAL(battery_status_read);')
status_read = main[status_start:status_end]
stubs = r'''
#include "external_battery_feedback.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#define OFF false
#define ENTER_CRITICAL(name)
#define EXIT_CRITICAL(name)
#define NUMBER_OF_POLLED_ADC_CHANNELS 3
#define ADC_CONVERSION_TIMEOUT_TICKS 10
#define INT_BAT_PRESENT_VOLTAGE 0.5f
enum ADC_Active_Channel_t { ADCInternalBatteryVoltage, ADCExternalBatteryVoltage, ADCTemperature };
static bool g_device_wakeup_complete=true, g_sleeping=false, g_go_to_sleep_now=false;
static bool g_enable_external_battery_control=true, g_restart_conversions=false;
static bool g_internal_bat_detected=false;
static float g_external_voltage=0.1f, g_internal_bat_voltage=4.2f;
static int g_temperature_fresh_seconds=10;
static ADC_Active_Channel_t g_adcChannelOrder[] = { ADCInternalBatteryVoltage, ADCExternalBatteryVoltage, ADCTemperature };
static uint16_t g_adcCountdownCount[] = {2000,2000,4000};
static uint16_t g_lastConversionResult[3] = {};
static ExternalBatteryFeedback g_external_battery_feedback;
static struct { bool visible=true; bool active() { return visible; } } LEDS;
static unsigned nowTick=0, powerOnTick=0, conversions[3]={};
static bool switchOn=false, batteryConnected=true, adcStuck=false;
static ADC_Active_Channel_t channel=ADCInternalBatteryVoltage;
static uint16_t converted=0;
static std::vector<unsigned> externalSampleTicks;
void setExtBatLSEnable(bool on) {
    if(on && !switchOn) powerOnTick=nowTick;
    switchOn=on;
}
bool getExtBatLSEnable() { return switchOn; }
uint16_t adcConversionPeriodTicks(uint8_t) { return 2400; }
float temperatureCfromADC(uint16_t) { return 25; }
void updateTemperatureState(float) { g_temperature_fresh_seconds=10; }
void ADC0_setADCChannel(ADC_Active_Channel_t c) { channel=c; }
void ADC0_startConversion() {
    ++conversions[channel];
    converted=channel == ADCInternalBatteryVoltage ? 580 : 1000;
    if(channel == ADCExternalBatteryVoltage) {
        externalSampleTicks.push_back(nowTick);
        if(g_enable_external_battery_control) {
            assert(switchOn); // No conversions of intentionally disconnected power.
            assert(nowTick-powerOnTick >= 3); // At least 10 ms before sampling.
        }
        converted=batteryConnected ? 1800 : 0;
    }
}
bool ADC0_conversionDone() { return !adcStuck; }
uint16_t ADC0_read() { return converted; }
void ADC0_SYSTEM_shutdown() { converted=0; }
float readVoltage(ADC_Active_Channel_t c) {
    ADC0_SYSTEM_shutdown(); // The shared blocking driver aborts any displaced conversion.
    ADC0_setADCChannel(c);ADC0_startConversion();
    return converted*0.00725f+0.05f;
}
'''
production = '\n'.join([
    enum,
    'static volatile bool chargeLScallerStates[NUMBER_OF_LS_CONTROLLERS] = {};',
    function(binio, 'static void updateLoadSwitchCallerState('),
    function(binio, 'static bool anyLoadSwitchCallerEnabled('),
    function(binio, 'static bool getArbitratedLoadSwitchState('),
    function(binio, 'bool setExtBatLoadSwitch(bool onoff,'),
    function(binio, 'bool externalBatteryPowerRequested(void)'),
    function(main, 'static void cancelExternalBatteryFeedback(void)'),
    'void adcTick() { static bool conversionInProcess=false; static int8_t indexConversionInProcess=0; static uint16_t adcConversionWaitTicks=0;\n'
    + adc_service + '\n}',
    'float batteryStatusRead() { ' + status_read + '\nreturn external_voltage; }',
])
tests = r'''
void advance(unsigned ticks) { while(ticks--) { ++nowTick; adcTick(); } }
void restart() {
    cancelExternalBatteryFeedback();
    setExtBatLoadSwitch(OFF, INITIALIZE_LS);
    g_restart_conversions=true;
    g_device_wakeup_complete=true;g_sleeping=false;g_go_to_sleep_now=false;
    g_enable_external_battery_control=true;LEDS.visible=true;
    batteryConnected=true;adcStuck=false;
    for(int i=0;i<3;++i) { conversions[i]=0;g_adcCountdownCount[i]=100; }
    externalSampleTicks.clear();g_external_voltage=0.1f;
}
int main() {
    restart();advance(1);assert(switchOn);
    advance(2);assert(conversions[1]==0 && g_external_voltage<6);
    advance(1);assert(conversions[1]==1 && switchOn);
    advance(1);assert(!switchOn && g_external_voltage>12);
    // A healthy cached battery remains healthy throughout the disconnected interval.
    advance(2999);assert(conversions[1]==1 && !switchOn && g_external_voltage>12);
    advance(5);assert(conversions[1]==2 && !switchOn);
    assert(externalSampleTicks[1]-externalSampleTicks[0]>=3000);

    // Both charging and transmission can acquire power during a measurement.
    for(auto client : {TRANSMITTER, INTERNAL_BATTERY_CHARGING}) {
        restart();advance(1);setExtBatLoadSwitch(true,client);
        advance(5);assert(switchOn && externalBatteryPowerRequested());
        assert(!chargeLScallerStates[BATTERY_MEASUREMENT]);
        unsigned count=conversions[1];advance(305);assert(conversions[1]==count+1);
        LEDS.visible=false;advance(1);count=conversions[1];
        advance(2300);assert(conversions[1]==count); // No 1 Hz polling in the dark.
        advance(110);assert(conversions[1]==count+1 && switchOn);
        setExtBatLoadSwitch(false,client);advance(10);assert(!switchOn);
        count=conversions[1];advance(3001);assert(conversions[1]==count);
    }

    // A real power-on requests a prompt reading even with dark LEDs.
    restart();LEDS.visible=false;advance(1000);assert(conversions[1]==0);
    setExtBatLoadSwitch(true,TRANSMITTER);unsigned edge=nowTick;
    advance(5);assert(conversions[1]==1 && g_external_voltage>12);
    assert(externalSampleTicks.back()-edge<=5);

    // Expiring LEDs cancel a temporary probe, without re-lighting them.
    restart();advance(1);LEDS.visible=false;advance(1);assert(!switchOn);
    advance(3001);assert(conversions[1]==0 && !LEDS.visible);
    LEDS.visible=true;advance(5);assert(conversions[1]==1 && g_external_voltage>12);

    // Battery removal is detected at the next probe, not by reading an off jack.
    batteryConnected=false;advance(3010);assert(g_external_voltage<6 && !switchOn);

    // Sleep entry releases only the probe. No awake probes run during sleep.
    restart();advance(1);g_go_to_sleep_now=true;advance(1);assert(!switchOn);
    g_sleeping=true;unsigned count=conversions[1];advance(10000);
    assert(conversions[1]==count && !switchOn);
    setExtBatLoadSwitch(true,INTERNAL_BATTERY_CHARGING);
    cancelExternalBatteryFeedback();assert(switchOn); // Existing sleep charging survives.
    setExtBatLoadSwitch(false,INTERNAL_BATTERY_CHARGING);
    g_sleeping=false;g_go_to_sleep_now=false;advance(5);
    assert(conversions[1]==count+1);

    // A lost conversion or a busy ADC cannot leave the measurement output on.
    restart();adcStuck=true;advance(100);assert(!switchOn);
    adcStuck=false;advance(3100);assert(g_external_voltage>12 && !switchOn);

    // A busy higher-priority channel also cannot hold probe power indefinitely.
    restart();g_adcCountdownCount[0]=0;adcStuck=true;advance(31);
    assert(!switchOn && conversions[1]==0);
    adcStuck=false;advance(3100);assert(g_external_voltage>12 && !switchOn);

    // Reapplying the same BAT X mode cancels and restarts an in-flight probe safely.
    restart();advance(2);cancelExternalBatteryFeedback();g_restart_conversions=true;
    setExtBatLoadSwitch(OFF,INITIALIZE_LS);advance(5);
    assert(g_external_voltage>12 && !switchOn);

    // Disabling battery control cancels probes and never pulses the legacy fan output.
    restart();advance(1);g_enable_external_battery_control=false;
    setExtBatLoadSwitch(OFF,INITIALIZE_LS);advance(10);assert(!switchOn);
    assert(g_external_voltage>12);
    setExtBatLoadSwitch(true,INITIALIZE_LS);advance(600);assert(switchOn);
    setExtBatLoadSwitch(false,INITIALIZE_LS);advance(600);assert(!switchOn);

    // Faster voltage feedback does not starve internal-battery or temperature sampling.
    restart();setExtBatLoadSwitch(true,TRANSMITTER);advance(7500);
    assert(conversions[0]>=4 && conversions[2]>=4 && conversions[1]>=24);
    // Foreground ADC use discards an interrupted conversion and retries the pending probe.
    restart();advance(4);assert(batteryStatusRead()<6);advance(2);
    assert(!switchOn && g_external_voltage>12);
    count=conversions[1];float cached=g_external_voltage;
    assert(batteryStatusRead()==cached && !switchOn);
    assert(conversions[1]==count && g_external_voltage==cached);
    // Internal/thermal work displaced by a foreground read is promptly retried.
    restart();LEDS.visible=false;g_adcCountdownCount[2]=0;advance(1);
    assert(conversions[2]==1);batteryStatusRead();advance(2);
    assert(conversions[2]==2);
    puts("PASS battery feedback: settling, cadence, cached voltage, arbitration, timeout, sleep, fan mode, ADC sharing");
}
'''

with tempfile.TemporaryDirectory(prefix='signalslinger-battery-test-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(stubs + production + tests)
    executable = Path(temp) / 'test'
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'SignalSlinger/include'), str(cpp),
                    '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
