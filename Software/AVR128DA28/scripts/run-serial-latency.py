#!/usr/bin/env python3
"""Stress real UART traffic; optionally measure timing with a diagnostic image."""
import argparse
import importlib.util
import json
from pathlib import Path
import signal
import sys
import time
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location("hardware_bench",Path(__file__).with_name("run-hardware-bench.py"))
bench=importlib.util.module_from_spec(spec);spec.loader.exec_module(bench)
NAMES=["RTC","TCB0","TCB1","TCB2","key","I2C_write","I2C_read","EEPROM_byte","EEPROM_word","EEPROM_dword","EEPROM_float","RX_parser","EEPROM_read","history_append"]
class LatencyBench(bench.Bench):
    def run_case(self,name,fn):
        super().run_case("serial stress with zero-error checks" if self.args.require_clean else "completed serial latency capture; see recorded overruns",fn)
    def restore(self):
        if self.original and not self.args.normal_image:
            self.args.byte_gap_ms=10
            self.wake();self.cmd("UI L 0")
        super().restore()
    def capture(self,label,seconds=8,prefix=""):
        print("CAPTURE "+label,flush=True);self.record("latency_case",label)
        if not self.args.normal_image:self.cmd("UI L 1")
        # Saturate the receive wire with comments, avoiding a command/reply queue
        # flood. Only the explicitly requested prefix can change device state.
        line=b"*"+b"X"*118+b"\r"
        payload=prefix.encode()+line*int(seconds*960/len(line))
        self.record("burst_tx",payload.decode());self.s.write(payload);self.s.flush()
        self.read(seconds+1)
        self.s.write(b"\r");self.read(2,True)
        report="" if self.args.normal_image else self.cmd("UI L",timeout=15)
        state=self.state()
        metrics=[bench.fields(l) for l in report.splitlines() if "* LAT metric " in l]
        faults=[bench.fields(l) for l in report.splitlines() if "* LAT fault " in l]
        if not self.args.normal_image:assert len(metrics)==len(NAMES) and "* LAT end" in report,report
        row={"label":label,"bytes":len(payload),"metrics":[dict(m,name=NAMES[m['id']],max_ms=m['max']*1000/32768,irq_max_ms=m['irq_max']*1000/32768) for m in metrics],"faults":faults,"state":state,"raw":report}
        self.captures.append(row)
        (self.out/"latency.json").write_text(json.dumps(self.captures,indent=2))
        print("RESULT "+label+" faults="+str(len(faults))+" maxima="+str([(NAMES[m['id']],round(m['max']*1000/32768,3)) for m in metrics if m['max']>=33]),flush=True)
        if self.args.require_clean:
            assert 'overrun=0 framing=0 parity=0' in state['raw'],state
            assert not faults, faults
            if metrics:
                timer=next(m for m in metrics if m['id']==3)
                assert timer['calls']>100 and timer['gap']*1000/32768<4, timer
        return row
    def serial_stress(self):
        (self.out/"latency-runner.py").write_text(Path(__file__).read_text())
        self.captures=[]
        if not self.args.normal_image:
            self.cmd("UI L 1");assert "* LAT v=1" in self.cmd("UI L")
        if self.args.focus_demo:
            for i in range(self.args.repeats):
                self.schedule(-600)
                carrier=self.capture("carrier entry trial "+str(i+1),seconds=5,prefix="UI P 1\r")
                assert carrier["state"]["key"]>0,carrier
                demo=self.capture("demo entry trial "+str(i+1),seconds=5,prefix="UI P 1\r")
                assert demo["state"]["demo"]>0 and not demo["state"]["key"],demo
                self.press(3)
            self.schedule(-600)
            self.capture("scheduled finish write",seconds=5,prefix="CLK F 260917130100\r")
            assert "Finish:Thu 17-Sep-2026 13:01:00" in self.cmd("CLK")
            return self.captures
        self.cmd("GO 0");self.capture("idle")
        self.schedule(-600);self.capture("future carrier entry",prefix="UI P 1\r")
        self.capture("future demo entry",prefix="UI P 1\r")
        self.capture("Morse demo running",seconds=12)
        self.press(3)
        self.schedule(130);self.capture("scheduled Morse running",seconds=12)
        self.schedule(-5);self.capture("scheduled start boundary",seconds=12)
        self.schedule(30);self.capture("active carrier entry and expiry",seconds=35,prefix="UI P 1\r")
        return self.captures
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port",required=True);p.add_argument("--uid",required=True);p.add_argument("--output",required=True);p.add_argument("--restore-from")
    p.add_argument("--hardware",choices=["3.4","3.5"],default="3.5",help="Expected hardware build")
    p.add_argument("--require-clean",action="store_true",help="Fail on any UART error or a diagnostic timer service gap of 4 ms or more")
    p.add_argument("--normal-image",action="store_true",help="Check receiver errors and behavior without optional timing instrumentation")
    p.add_argument("--focus-demo",action="store_true");p.add_argument("--repeats",type=int,default=3)
    a=p.parse_args()
    if not 1<=a.repeats<=10:p.error("repeats must be 1-10")
    a.byte_gap_ms=10;a.extended=False;a.case="serial replies during an active event";a.start_at="";a.stop_before=""
    def stop(signum,frame):raise SystemExit("Interrupted; restoring device")
    signal.signal(signal.SIGTERM,stop)
    LatencyBench(a).run()
