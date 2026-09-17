#!/usr/bin/env python3
"""Exercise an identified Classic Fox 3 on the real MCU; retain raw evidence.

Requires pyserial. Changes schedule/RTC, restores the saved schedule and local
wall clock in finally, and leaves RF stopped. Does not claim measured RF/LEDs.
"""
import argparse
import datetime as dt
import json
import re
import signal
import sys
import time
from pathlib import Path
import serial

BASE = dt.datetime(2026, 9, 17, 12)
def epoch(t): return int((t-dt.datetime(1970,1,1)).total_seconds())
def stamp(t): return t.strftime('%y%m%d%H%M%S')
def fields(line): return {k:int(v) for k,v in re.findall(r'(\w+)=(-?\d+)',line)}

class Bench:
    def __init__(self, args):
        if not 0<=args.byte_gap_ms<=100:raise ValueError('byte-gap-ms must be 0-100')
        self.args=args;self.pacing_requested=args.byte_gap_ms;self.out=Path(args.output)
        if self.out.exists() and any(self.out.iterdir()):
            raise ValueError('Use a new output directory to preserve earlier evidence')
        self.out.mkdir(parents=True,exist_ok=True)
        (self.out/'runner.py').write_text(Path(__file__).read_text())
        (self.out/'run-config.json').write_text(json.dumps(dict(vars(args),python=sys.version,pyserial=serial.__version__),indent=2))
        self.log=(self.out/'serial.jsonl').open('a',buffering=1)
        self.results=[];self.s=None;self.original=None
    def record(self, kind, text):
        self.log.write(json.dumps({'host':dt.datetime.now().isoformat(),'kind':kind,'text':text})+'\n')
    def read(self, seconds, prompt=False):
        end=time.monotonic()+seconds;data=b'';last=time.monotonic()
        while time.monotonic()<end:
            b=self.s.read(4096)
            if b:data+=b;last=time.monotonic()
            if prompt and data.endswith(b'> ') and time.monotonic()-last>.18:break
        text=data.decode(errors='replace')
        if text:self.record('rx',text)
        return text
    def cmd(self,c,timeout=12,allow_error=False):
        self.record('tx',c)
        payload=(c+'\r').encode()
        if self.args.byte_gap_ms==0:self.s.write(payload)
        else:
            for byte in payload:
                self.s.write(bytes([byte]));time.sleep(self.args.byte_gap_ms/1000)
        r=self.read(timeout,True)
        if not r.rstrip().endswith('>'):
            raise TimeoutError(f'No complete reply to {c!r}: {r!r}')
        if not allow_error and re.search(r'\*\s*(?:Err:|err\b)',r):raise AssertionError(f'{c}: {r}')
        return r
    def wake(self):
        # A UART wake consumes input, and a clockless reboot can power down after
        # printing its banner. Synchronize with a verified read-only identity
        # reply before any state-changing command; never replay mutations.
        for attempt in range(3):
            self.s.write(b'\r');received=self.read(5,True)+self.read(.4)
            if 'Power off. Press and hold pushbutton' in received:
                # The first pulse may precede startup's deliberate power-off.
                # Send another wake pulse before attempting a complete command.
                self.record('wake_probe_retry','Startup entered sleep after the wake pulse')
                continue
            try:
                reply=self.cmd('INF')
                # Startup banners also contain the UID. Only a full INF reply
                # proves the command was received after UART wake stabilization.
                if ('* INF product=SignalSlinger update=UPD' in reply and
                    '* INF sw=' in reply and '* INF bl=' in reply and
                    '* INF uid='+self.args.uid in reply):return
                self.record('wake_probe_retry','No complete INF command response')
            except (AssertionError,TimeoutError) as error:
                self.record('wake_probe_retry',repr(error))
        raise TimeoutError('Unable to synchronize an awake device with the expected UID')
    def state(self):
        r=self.cmd('UI D');lines=r.splitlines()
        states=[fields(l) for l in lines if '* UI state ' in l]
        runtimes=[fields(l) for l in lines if '* Runtime:' in l]
        if not states or not runtimes:
            self.wake();r=self.cmd('UI D');lines=r.splitlines()
            states=[fields(l) for l in lines if '* UI state ' in l]
            runtimes=[fields(l) for l in lines if '* Runtime:' in l]
        assert states and runtimes,r
        return dict(states[-1],runtime=runtimes[-1],raw=r)
    def quiet(self, seconds):self.read(seconds)
    def schedule(self, phase=30, day=0, days=3, duration=3600):
        self.wake();self.cmd('GO 0')
        self.cmd('CLK T '+stamp(BASE-dt.timedelta(hours=1)))
        self.cmd('CLK S '+stamp(BASE));self.cmd('CLK F '+stamp(BASE+dt.timedelta(seconds=duration)))
        self.cmd('CLK D '+str(days))
        self.cmd('CLK T '+stamp(BASE+dt.timedelta(days=day,seconds=phase)))
        self.cmd('GO 2');s=self.state()
        expected_day=max(0,min(days,(s['at']-epoch(BASE)-duration)//86400+1))
        assert s['runtime']['day']==expected_day,s
        return s
    def press(self,n=1):self.cmd('UI P '+str(n));self.quiet(.25)
    def hold(self):
        assert 'queued' in self.cmd('UI B 250')
        self.quiet(7);self.wake()
        return self.state()
    def trace(self):
        return [fields(l) for l in self.cmd('UI T',timeout=15).splitlines() if '* UI trace ' in l]
    def check_phase(self,s):
        phase=(s['at']-epoch(BASE))%300
        expected=phase-120 if phase<120 else (180-phase if phase<180 else phase-420)
        assert abs(s['onair']-expected)<=2,(expected,s)
    def run_case(self,name,fn):
        print('RUN '+name,flush=True);self.record('case',name)
        try:
            details=fn();self.results.append({'case':name,'status':'PASS','details':details});print('PASS '+name,flush=True)
        except BaseException as e:
            self.results.append({'case':name,'status':'FAIL','error':repr(e)});raise
        finally:self.save()
    def save(self):
        (self.out/'results.json').write_text(json.dumps({'results':self.results,'original':self.original,'requested_byte_gap_ms':self.pacing_requested,'evidence':'MCU state and GPIO readback; no external RF/optical/current measurements'},indent=2))
    def restore(self):
        if not self.original:return
        self.args.byte_gap_ms=max(10,self.args.byte_gap_ms) # Cleanup uses conservative pacing even after a stress failure.
        self.wake();self.cmd('UI T 0');self.cmd('GO 0')
        self.cmd('CLK T '+stamp(dt.datetime.strptime(self.original['S'],'%y%m%d%H%M%S')-dt.timedelta(days=1)))
        for key in ['S','F','D']:self.cmd('CLK '+key+' '+self.original[key])
        self.cmd('CLK T '+stamp(dt.datetime.now()))
        self.cmd('GO 0');s=self.state()
        assert not s['runtime']['enabled'] and not(s['flags']&32),s
        clk=self.cmd('CLK');self.cmd('TMP');self.cmd('BAT')
        assert 'Days to run: '+self.original['D'] in clk
        for key,label in [('S','Start'),('F','Finish')]:
            expected=dt.datetime.strptime(self.original[key],'%y%m%d%H%M%S').strftime('%a %d-%b-%Y %H:%M:%S')
            assert label+':'+expected in clk,(label,clk)
        self.results.append({'case':'restore','status':'PASS','details':'Original schedule and local clock restored; RF stopped; trace disabled.'});self.save()
        self.record('restore','Saved schedule restored, current local clock set, RF stopped; trace disabled.')
    def run(self):
        self.s=serial.Serial(self.args.port,9600,timeout=.05,exclusive=True)
        try:
            if sys.platform=='darwin':
                import fcntl,termios
                fcntl.ioctl(self.s.fileno(),termios.TIOCEXCL)
            self.wake();identity=self.cmd('INF');assert 'uid='+self.args.uid in identity,identity
            assert 'hw=3.5' in identity,identity
            initial=self.state()
            assert 'bench=1' in initial['raw']
            assert not initial['runtime']['enabled'] and not initial['key'] and not initial['demo'], 'Dedicate an idle, stopped device to the bench before running'
            assert 'Event:Classic' in self.cmd('EVT');assert 'Fox:Classic Fox 3' in self.cmd('FOX')
            saved=Path(self.args.restore_from).read_text() if self.args.restore_from else self.cmd('CLK')
            values={}
            for key,label in [('S','Start'),('F','Finish')]:
                value=re.search(r'\* '+label+r':\s*([^\r\n]+)',saved).group(1)
                values[key]=stamp(dt.datetime.strptime(value,'%a %d-%b-%Y %H:%M:%S'))
            values['D']=re.search(r'Days to run: (\d+)',saved).group(1);values['days']=int(values['D'])
            self.original=values;self.save()
            cases=[('hook limits and idle readback',self.limits),('timed short press and carrier cancellation',self.short_press),
                ('future carrier-demo-schedule sequence',self.future_sequence),('future triple press clears temporary tests',self.future_triples),
                ('active triple press advances one day',self.active_triple),('active off-air long hold cancels today',lambda:self.active_hold(30)),
                ('active on-air long hold cancels today',lambda:self.active_hold(130)),('last-day hold stays canceled after serial wake',lambda:self.active_hold(30,2)),
                ('future long hold preserves schedule',self.future_hold),('preview hold release is ignored',self.preview_hold),('timed triple press advances today',self.timed_triple),('scheduled start during hold preserves today',self.start_during_hold),('active carrier long hold preserves phase',self.carrier_hold),
                ('future demo long hold preserves schedule',self.demo_hold),('30-second carrier expiry restores phase',self.carrier_timeout),
                ('demo expiry restores future schedule',self.demo_timeout),('carrier expires while held without canceling today',self.expiry_during_hold),
                ('expired schedule manual run is indefinite',self.expired_manual),('clockless manual run and cancellation',self.clockless)]
            if self.args.extended:cases += [('natural off-air sleep and RTC wake',self.sleep_cycle),('timed finish advances normally',self.finish),('manual run outlives expired duration',self.manual_duration)]
            cases += [('disabled equal-time schedule manual run',self.equal_manual),('serial replies during an active event',self.serial_stress)]
            selected=not self.args.start_at
            matched=0
            for name,fn in cases:
                if self.args.stop_before and self.args.stop_before in name:break
                if self.args.start_at and self.args.start_at in name:selected=True
                if selected and (not self.args.case or self.args.case in name):
                    matched+=1;self.run_case(name,fn)
            if not matched:raise ValueError("No hardware test matches the requested filter")
        finally:
            try:self.restore()
            except BaseException as e:
                self.results.append({'case':'restore','status':'FAIL','error':repr(e)});self.save();raise
            finally:
                if self.s:self.s.close()
                self.save();self.log.close()
    def equal_manual(self):
        self.schedule(-600);self.cmd('CLK S =');self.press();self.press();s=self.state()
        assert s['runtime']['enabled'] and s['runtime']['forever'],s
        self.press(3);stopped=self.state();assert not stopped['runtime']['enabled'],stopped
        return {'running':s,'stopped':stopped}
    def serial_stress(self):
        self.schedule(110);out=[]
        for i in range(40):
            s=self.state();out.append(s)
            assert 'overrun=0 framing=0 parity=0' in s['raw'],s
        return {'snapshots':out}
    def limits(self):
        self.cmd('GO 0')
        for c in ['UI B 0','UI B 751','UI B -1','UI B 1X','UI T 31','UI T -1']:
            assert 'Err:' in self.cmd(c,allow_error=True),c
        s=self.state();assert not s['key'] and not s['demo'] and not s['runtime']['enabled'];return s
    def short_press(self):
        self.schedule(-600);assert 'queued' in self.cmd('UI B 10');self.quiet(2)
        s=self.state();assert s['key']>0 and s['flags']&32,s
        self.press(3);s=self.state();assert not s['key'] and s['runtime']['day']==0;return s
    def future_sequence(self):
        self.schedule(-600);self.press();s=self.state();assert s['key']>0,s
        self.press();s=self.state();assert s['demo']>0 and not s['key'],s
        self.press();s=self.state();assert not s['demo'] and not s['key'] and s['runtime']['start']==epoch(BASE),s
        return s
    def future_triples(self):
        out=[]
        for demo in [False,True]:
            self.schedule(-600);self.press()
            if demo:self.press()
            self.press(3);s=self.state();assert not s['demo'] and not s['key'] and s['runtime']['day']==0,s;out.append(s)
        return out
    def active_triple(self):
        self.schedule();self.press();self.press(3);s=self.state()
        assert s['runtime']['day']==1 and s['runtime']['start']==epoch(BASE)+86400,s;return s
    def active_hold(self,phase,day=0):
        self.schedule(phase,day);self.cmd('UI T 1');s=self.hold();trace=self.trace()
        assert any(t['flags']&1 for t in trace),trace
        if day==2:assert not s['runtime']['enabled'] and s['flags']&1024,s
        else:assert s['runtime']['day']==day+1 and s['runtime']['start']==epoch(BASE)+(day+1)*86400,s
        return {'state':s,'trace':trace}
    def future_hold(self):
        self.schedule(-600);self.cmd('UI T 1');s=self.hold();trace=self.trace()
        assert s['runtime']['day']==0 and s['runtime']['start']==epoch(BASE),s
        assert any(t['flags']&1 for t in trace),trace;return {'state':s,'trace':trace}
    def preview_hold(self):
        self.schedule();assert 'queued' in self.cmd('UI B 100');self.quiet(3)
        s=self.state();assert s['runtime']['day']==0 and not s['key'] and not s['demo'],s
        return s
    def timed_triple(self):
        self.schedule()
        for i in range(3):
            assert 'queued' in self.cmd('UI B 10');self.quiet(.15)
        self.quiet(2);s=self.state();assert s['runtime']['day']==1,s;return s
    def start_during_hold(self):
        self.schedule(-4);s=self.hold()
        assert s['runtime']['day']==0 and s['runtime']['enabled'] and 'action=5 ' in s['raw'],s
        self.check_phase(s);return s
    def manual_duration(self):
        self.schedule(phase=86400+170,days=1,duration=60);self.press();self.press();before=self.state()
        self.cmd('UI T 10');self.quiet(265);self.wake();after=self.state();trace=self.trace()
        assert after['at']-before['at']>60 and after['runtime']['enabled'] and after['runtime']['forever'],after
        first_sleep=next(i for i,t in enumerate(trace) if t['flags']&1)
        assert all(t['flags']&16 for t in trace),trace
        assert any(t['onair']>0 and not(t['flags']&1) for t in trace[first_sleep+1:]),trace
        self.check_phase(after);self.press(3);return {'before':before,'after':after,'trace':trace}
    def carrier_hold(self):
        self.schedule();self.press();s=self.hold();assert s['runtime']['day']==0 and not s['key'],s
        self.check_phase(s);return s
    def demo_hold(self):
        self.schedule(-600);self.press();self.press();s=self.hold()
        assert s['runtime']['day']==0 and not s['demo'] and s['runtime']['start']==epoch(BASE),s;return s
    def carrier_timeout(self):
        self.schedule();self.press();self.quiet(32);s=self.state()
        assert not s['key'] and s['runtime']['day']==0,s;self.check_phase(s);return s
    def demo_timeout(self):
        self.schedule(-600);self.press();self.press();self.quiet(32);s=self.state()
        assert not s['demo'] and s['runtime']['day']==0 and s['runtime']['start']==epoch(BASE),s;return s
    def expiry_during_hold(self):
        self.schedule();self.press();self.quiet(27);s=self.hold()
        assert s['runtime']['day']==0 and not s['key'] and 'action=1 ' in s['raw'],s
        self.check_phase(s);return s
    def expired_manual(self):
        self.schedule(phase=4*86400);self.press();self.press();s=self.state()
        assert s['runtime']['manual'] and s['runtime']['forever'] and s['runtime']['enabled'],s
        self.check_phase(s);self.press(3);stopped=self.state();assert not stopped['runtime']['enabled'],stopped
        return {'running':s,'stopped':stopped}
    def clockless(self):
        self.cmd('GO 0');self.s.write(b'RST\r');self.quiet(5);self.wake()
        assert 'Time:not set' in self.cmd('CLK')
        self.press();self.press();s=self.state();assert s['runtime']['forever'] and s['runtime']['enabled'] and s['onair']>0,s
        self.press(3);stopped=self.state();assert not stopped['runtime']['enabled'],stopped;return {'running':s,'stopped':stopped}
    def sleep_cycle(self):
        self.schedule(170);self.cmd('UI T 10');self.quiet(265);self.wake();s=self.state();trace=self.trace()
        assert any(t['flags']&1 for t in trace),trace
        first_sleep=next(i for i,t in enumerate(trace) if t['flags']&1)
        assert any(t['onair']>0 and not(t['flags']&1) for t in trace[first_sleep+1:]),trace
        assert all(b['at']-a['at']==10 for a,b in zip(trace,trace[1:])),trace
        assert s['runtime']['day']==0,s;self.check_phase(s);return {'state':s,'trace':trace}
    def finish(self):
        self.schedule(phase=50,duration=60);self.cmd('UI T 1');self.quiet(15);self.wake();s=self.state();trace=self.trace()
        assert s['runtime']['day']==1 and s['runtime']['start']==epoch(BASE)+86400,s;return {'state':s,'trace':trace}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--uid',required=True)
    p.add_argument('--byte-gap-ms',type=float,default=10,help='0 enables unpaced serial stress')
    p.add_argument('--output',required=True);p.add_argument('--restore-from')
    p.add_argument('--extended',action='store_true');p.add_argument('--case',default='');p.add_argument('--start-at',default='');p.add_argument('--stop-before',default='')
    def terminate(signum,frame):
        raise SystemExit('Interrupted; restoring device state')
    signal.signal(signal.SIGTERM,terminate)
    Bench(p.parse_args()).run()
