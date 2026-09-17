#!/usr/bin/env python3
"""Check the actual bench wake method against startup/wake reply sequences."""
import ast
from pathlib import Path
from types import SimpleNamespace

root = Path(__file__).resolve().parent.parent
source = (root / 'scripts/run-hardware-bench.py').read_text()
tree = ast.parse(source)
bench = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Bench')
wake = next(n for n in bench.body if isinstance(n, ast.FunctionDef) and n.name == 'wake')
namespace = {}
exec(compile(ast.Module(body=[wake], type_ignores=[]), '<production wake>', 'exec'), namespace)
uid = 'test-device'
identity = '* INF product=SignalSlinger update=UPD\n* INF sw=2.0.5\n* INF bl=BL0.13\n* INF uid=' + uid
banner = 'SignalSlinger startup\n* INF uid=' + uid
class Fixture:
    def __init__(self, reads, replies):
        self.args = SimpleNamespace(uid=uid)
        self.reads = iter(reads); self.replies = iter(replies)
        self.writes = []; self.commands = []; self.events = []
        self.s = SimpleNamespace(write=self.writes.append)
    def read(self, *args): return next(self.reads)
    def record(self, *args): self.events.append(args)
    def cmd(self, command):
        self.commands.append(command)
        reply = next(self.replies)
        if isinstance(reply, Exception): raise reply
        return reply
    wake = namespace['wake']

# A wake pulse sent before startup powers down must be repeated before INF.
f = Fixture(['* Power off. Press and hold pushbutton for power on', '', banner, ''], [identity])
f.wake(); assert f.writes == [b'\r', b'\r'] and f.commands == ['INF']
# A startup banner carries the correct UID but cannot authorize later commands.
f = Fixture([''] * 4, [banner, identity])
f.wake(); assert len(f.writes) == 2 and f.commands == ['INF', 'INF']
# Read-only probes may recover from a wake-damaged line, recording the retry.
f = Fixture([''] * 4, [AssertionError('serial receive error'), identity])
f.wake(); assert len(f.events) == 1 and f.commands == ['INF', 'INF']
# Persistent failure is bounded; no setting-changing commands are ever retried.
f = Fixture([''] * 6, [banner] * 3)
try:
    f.wake()
    raise AssertionError('Unverified wake incorrectly accepted')
except TimeoutError:
    pass
assert len(f.writes) == 3 and f.commands == ['INF'] * 3
print('Hardware bench wake: startup power-off, banner rejection, damaged probe and bounded failure passed.')
