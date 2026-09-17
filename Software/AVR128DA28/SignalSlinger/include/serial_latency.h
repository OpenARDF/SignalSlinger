#ifndef SIGNALSLINGER_SERIAL_LATENCY_H
#define SIGNALSLINGER_SERIAL_LATENCY_H
// Diagnostic builds only: all scope code disappears from normal firmware.
#ifdef SIGNALSLINGER_LATENCY_DIAGNOSTICS
#include <stdint.h>
enum SerialLatencyId { LAT_RTC, LAT_TCB0, LAT_TCB1, LAT_TCB2, LAT_KEY,
    LAT_I2C_WRITE, LAT_I2C_READ, LAT_EE_BYTE, LAT_EE_WORD, LAT_EE_DWORD,
    LAT_EE_FLOAT, LAT_RX, LAT_EE_READ, LAT_HISTORY, LAT_COUNT };
extern volatile bool g_serial_latency_enabled;
struct SerialLatencyToken { uint16_t start, prior_mask; uint8_t context, nvm; };
SerialLatencyToken serialLatencyBegin(uint8_t id);
void serialLatencyEnd(uint8_t id, SerialLatencyToken token);
void serialLatencyRx(uint8_t status);
void serialLatencyControl(const char *arg);
class SerialLatencyScope {
    uint8_t id; bool enabled; SerialLatencyToken token;
public:
    explicit SerialLatencyScope(uint8_t value):id(value),enabled(g_serial_latency_enabled) {
        if(enabled) token=serialLatencyBegin(id);
    }
    ~SerialLatencyScope() { if(enabled) serialLatencyEnd(id,token); }
};
#define SERIAL_LATENCY_SCOPE(id) SerialLatencyScope serial_latency_scope(id)
#else
#define SERIAL_LATENCY_SCOPE(id) ((void)0)
#endif
#endif
