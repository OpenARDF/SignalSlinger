#!/usr/bin/env python3
"""Exercise the actual line parser with UART loss/corruption at every byte."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parent.parent
header=(root/'SignalSlinger/include/serialbus.h').read_text()
source=(root/'SignalSlinger/driver_isr.cpp').read_text()
types=header[header.index('#define SERIALBUS_USART'):header.index('#define WAITING_FOR_UPDATE')]
helpers=source[source.index('void serial_Rx(uint8_t rx_char);'):source.index('ISR(USART0_RXC_vect)')]
start=source.index('\nvoid serial_Rx(uint8_t rx_char)\n{')
end=source.index('\n/**',start)
parser=source[start:end]
stubs=r'''
#include <stdint.h>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <string>
#define SERIAL_LATENCY_SCOPE(id) ((void)0)
#define MAX_UINT16 UINT16_MAX
#define MIN(a,b) ((a)<(b)?(a):(b))
#define ENTER_CRITICAL(x)
#define EXIT_CRITICAL(x)
#define USART_BUFOVF_bm 0x40
#define USART_FERR_bm 4
#define USART_PERR_bm 2
'''+types+r'''
static SerialbusRxBuffer buffers[4];
SerialbusRxBuffer* nextEmptySBRxBuffer() {
    for(auto &b:buffers) if(b.id==SB_MESSAGE_EMPTY) return &b;
    return nullptr;
}
bool sb_echo_char_isr(uint8_t) { return true; }
'''
tests=r'''
void reset() { memset(buffers,0,sizeof(buffers));serialbus_set_rx_accepting_input(true); }
void feed(const std::string &s) { for(char c:s) serialRxChecked(c,0); }
int main() {
    for(const std::string command : {"GO 1\r","CLK T 260917120030\r","UI P 3\r","UI B 250\r"}) {
        for(unsigned pos=0;pos<command.size();++pos) for(uint8_t error : {0x40,4,2}) {
            reset();
            for(unsigned i=0;i<command.size();++i) serialRxChecked(command[i],i==pos?error:0);
            assert(buffers[0].id==SB_RX_CORRUPT);
            for(unsigned i=1;i<4;++i) assert(buffers[i].id==SB_MESSAGE_EMPTY);
            buffers[0].id=SB_MESSAGE_EMPTY;feed("GO 0\r");
            assert(buffers[0].id==SB_MESSAGE_GO && std::string(buffers[0].fields[0])=="0");
        }
        // Real overrun loses bytes: delete one and flag the next buffered byte.
        for(unsigned pos=0;pos+1<command.size();++pos) {
            reset();
            for(unsigned i=0;i<command.size();++i) if(i!=pos) serialRxChecked(command[i],i==pos+1?0x40:0);
            assert(buffers[0].id==SB_RX_CORRUPT);
        }
    }
    reset();feed("ui p 3\r");assert(buffers[0].id==SB_MESSAGE_UI_DIAGNOSTICS);
    assert(std::string(buffers[0].fields[0])=="P" && std::string(buffers[0].fields[1])=="3");
    reset();feed("GO 1\b0\n");assert(buffers[0].id==SB_MESSAGE_GO && std::string(buffers[0].fields[0])=="0");
    reset();feed("* comment\rGO 2\r");assert(buffers[0].id==SB_MESSAGE_GO);
    reset();feed("GO 1\rGO 2\rGO 0\rUI S\r");
    serialRxChecked('X',0x40);feed("GO 1\r"); // Full queue must not corrupt completed commands.
    assert(buffers[0].id==SB_MESSAGE_GO && std::string(buffers[0].fields[0])=="1");
    assert(buffers[3].id==SB_MESSAGE_UI_DIAGNOSTICS);
    reset();g_rx_overrun=g_rx_framing=g_rx_parity=UINT16_MAX;serialRxChecked('\r',0x46);
    assert(g_rx_overrun==UINT16_MAX && g_rx_framing==UINT16_MAX && g_rx_parity==UINT16_MAX);
    puts("Serial RX: real parser rejects damaged/truncated mutations at every byte, recovers on next line, preserves clean input/full queue, saturates diagnostics.");
}
'''
with tempfile.TemporaryDirectory(prefix='signalslinger-rx-') as tmp:
    cpp=Path(tmp)/'test.cpp';binary=Path(tmp)/'test'
    cpp.write_text(stubs+helpers+parser+tests)
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
