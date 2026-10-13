#!/usr/bin/env python3
"""Capture a pre-update baseline or verify the persistent fault log on hardware.

The verify phase changes only the fault log and resets the MCU. It compares the
unit's identity, settings, and session records with a separately captured
baseline so EEPROM migration regressions cannot hide behind a successful boot.
"""

import argparse
import datetime as dt
import json
import re
import sys
import time
from pathlib import Path

import serial


READ_COMMANDS = ("ID", "EVT", "FOX", "PAT", "SPD I", "SPD F", "CLK", "FRE", "FRE 1", "FRE 2", "FRE 3", "FRE B", "BAT", "TMP", "FRE C")
STABLE_PREFIXES = {
    "ID": ("* ID:",),
    "EVT": ("* Event:", "* Session record:"),
    "FOX": ("* Fox:",),
    "PAT": ("* PAT:",),
    "SPD I": ("* ID SPD:",),
    "SPD F": ("* FOX-O SPD:",),
    "CLK": ("* Start:", "* Finish:", "* Days to run:"),
    "FRE": ("* FRE=",),
    "FRE 1": ("* FRE 1=",),
    "FRE 2": ("* FRE 2=",),
    "FRE 3": ("* FRE 3=",),
    "FRE B": ("* FRE B=",),
    "BAT": ("* thresh", "* Ext. Bat. Ctrl", "* Transmitter"),
    "TMP": ("* Thermal shutdown:", "* Thermal shutdown threshold:"),
    "FRE C": ("* FRE C=",),
}


class FaultBench:
    def __init__(self, args):
        self.args = args
        self.output = Path(args.output)
        if self.output.exists() and any(self.output.iterdir()):
            raise ValueError("Use a new output directory to preserve earlier evidence")
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "runner.py").write_text(Path(__file__).read_text())
        (self.output / "run-config.json").write_text(
            json.dumps(dict(vars(args), python=sys.version, pyserial=serial.__version__), indent=2)
        )
        self.log = (self.output / "serial.jsonl").open("a", buffering=1)
        self.serial = None

    def record(self, kind, text):
        self.log.write(json.dumps({"host": dt.datetime.now().astimezone().isoformat(), "kind": kind, "text": text}) + "\n")

    def read(self, seconds, prompt=False):
        end = time.monotonic() + seconds
        data = b""
        last = time.monotonic()
        while time.monotonic() < end:
            chunk = self.serial.read(4096)
            if chunk:
                data += chunk
                last = time.monotonic()
            if prompt and data.endswith(b"> ") and time.monotonic() - last > 0.18:
                break
        text = data.decode(errors="replace")
        if text:
            self.record("rx", text)
        return text

    def command(self, command, timeout=6, require_prompt=True):
        self.record("tx", command)
        self.serial.write((command + "\r").encode())
        reply = self.read(timeout, prompt=require_prompt)
        if require_prompt and not reply.rstrip().endswith(">"):
            raise TimeoutError(f"No complete reply to {command!r}: {reply!r}")
        if re.search(r"\*\s*(?:Err:|err\b)", reply):
            raise AssertionError(f"{command}: {reply}")
        return reply

    def open(self):
        self.serial = serial.Serial(self.args.port, 9600, timeout=0.05, exclusive=True)
        if sys.platform == "darwin":
            import fcntl
            import termios

            fcntl.ioctl(self.serial.fileno(), termios.TIOCEXCL)

    def close(self):
        if self.serial:
            self.serial.close()
            self.serial = None

    def wake(self):
        for _ in range(4):
            self.serial.write(b"\r")
            self.read(1.5, prompt=True)
            try:
                identity = self.command("INF")
                if f"* INF uid={self.args.uid}" in identity:
                    return identity
            except (AssertionError, TimeoutError):
                pass
        raise TimeoutError("Unable to synchronize with the expected SignalSlinger UID")

    @staticmethod
    def parse_identity(reply):
        sw = re.search(r"\* INF sw=([^ ]+) hw=([^ ]+)", reply)
        uid = re.search(r"\* INF uid=([0-9A-Fa-f]+)", reply)
        if not sw or not uid:
            raise AssertionError(f"Incomplete INF response: {reply}")
        return {"version": sw.group(1), "hardware": sw.group(2), "uid": uid.group(1)}

    def snapshot(self, identity_reply):
        replies = {command: self.command(command) for command in READ_COMMANDS}
        stable = {}
        for command, prefixes in STABLE_PREFIXES.items():
            normalized_lines = []
            for raw_line in replies[command].splitlines():
                marker = raw_line.find("* ")
                if marker >= 0:
                    normalized_lines.append(raw_line[marker:].strip())
            stable[command] = [
                line for line in normalized_lines if line.startswith(prefixes)
            ]
            if not stable[command]:
                raise AssertionError(f"No stable settings lines found for {command}: {replies[command]}")
        max_ever = re.search(r"\* Max Ever:\s*([-0-9.]+)C", replies["TMP"])
        if not max_ever:
            raise AssertionError(f"No maximum temperature in TMP response: {replies['TMP']}")
        return {
            "identity": self.parse_identity(identity_reply),
            "stable": stable,
            "maximumTemperatureC": float(max_ever.group(1)),
        }

    @staticmethod
    def parse_faults(reply):
        header = re.search(r"\* HWF v=1 count=(\d+) capacity=(\d+)", reply)
        if not header:
            raise AssertionError(f"No HWF header: {reply}")
        records = [
            {"code": int(code), "at": int(detected_at), "flags": int(flags)}
            for code, detected_at, flags in re.findall(r"\* HWF v=1 code=(\d+) at=(\d+) flags=(\d+)", reply)
        ]
        if len(records) != int(header.group(1)):
            raise AssertionError(f"HWF count does not match records: {reply}")
        return {"capacity": int(header.group(2)), "records": records}

    def require_expected_fault(self, faults, require_time=False):
        matches = [record for record in faults["records"] if record["code"] == self.args.expected_code]
        if len(matches) != 1:
            raise AssertionError(f"Expected one hardware fault code {self.args.expected_code}: {faults}")
        record = matches[0]
        if record["flags"] & 1:
            if record["at"] <= 0:
                raise AssertionError(f"Time-valid fault has no detection time: {record}")
        elif record["at"] != 0:
            raise AssertionError(f"Time-invalid fault should not claim a detection time: {record}")
        if require_time and not (record["flags"] & 1):
            raise AssertionError(f"Fault re-recorded after clock synchronization has no detection time: {record}")
        return record

    def capture_baseline(self):
        self.open()
        try:
            baseline = self.snapshot(self.wake())
            if baseline["identity"]["uid"].lower() != self.args.uid.lower():
                raise AssertionError("Connected UID changed while capturing baseline")
            (self.output / "baseline.json").write_text(json.dumps(baseline, indent=2))
            (self.output / "results.json").write_text(json.dumps({"status": "PASS", "phase": "baseline", "baseline": baseline}, indent=2))
            print(f"PASS baseline: {baseline['identity']}")
        finally:
            self.close()
            self.log.close()

    def verify(self):
        baseline = json.loads(Path(self.args.baseline).read_text())
        self.open()
        try:
            identity_reply = self.wake()
            before = self.snapshot(identity_reply)
            if before["identity"]["version"] != self.args.expected_version:
                raise AssertionError(f"Expected firmware {self.args.expected_version}, got {before['identity']['version']}")
            if before["identity"]["hardware"] != baseline["identity"]["hardware"] or before["identity"]["uid"].lower() != baseline["identity"]["uid"].lower():
                raise AssertionError("Firmware update changed device identity or hardware build")
            if before["stable"] != baseline["stable"]:
                raise AssertionError("EEPROM-backed settings or session history changed during migration")
            if before["maximumTemperatureC"] < baseline["maximumTemperatureC"]:
                raise AssertionError("Maximum-ever temperature regressed during EEPROM migration")

            migration = {"status": "PASS", "baseline": baseline, "afterUpdate": before}
            (self.output / "migration-results.json").write_text(json.dumps(migration, indent=2))

            initial_faults = self.parse_faults(self.command("HWF"))
            self.require_expected_fault(initial_faults)
            self.command(f"CLK T {dt.datetime.now().strftime('%y%m%d%H%M%S')}")
            cleared_faults = self.parse_faults(self.command("HWF C"))
            cleared_record = self.require_expected_fault(cleared_faults, require_time=True)

            self.record("tx", "RST")
            self.serial.write(b"RST\r")
            self.read(1.0)
            self.close()
            time.sleep(2.0)
            self.open()
            after = self.snapshot(self.wake())
            persisted_faults = self.parse_faults(self.command("HWF"))
            persisted_record = self.require_expected_fault(persisted_faults, require_time=True)
            if persisted_record != cleared_record:
                raise AssertionError("Fault record did not survive reset unchanged")
            if after["stable"] != baseline["stable"]:
                raise AssertionError("Settings or session history changed after reset verification")

            results = {
                "status": "PASS",
                "phase": "verify",
                "baseline": baseline,
                "before": before,
                "after": after,
                "initialFaults": initial_faults,
                "clearedFaults": cleared_faults,
                "persistedFaults": persisted_faults,
            }
            (self.output / "results.json").write_text(json.dumps(results, indent=2))
            print(f"PASS hardware fault bench: code {self.args.expected_code} detected, re-recorded after clear, and persisted across reset")
        finally:
            self.close()
            self.log.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--uid", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--mode", choices=("baseline", "verify"), required=True)
    parser.add_argument("--baseline")
    parser.add_argument("--expected-version")
    parser.add_argument("--expected-code", type=int, default=2)
    args = parser.parse_args()
    if args.mode == "verify" and (not args.baseline or not args.expected_version):
        parser.error("verify mode requires --baseline and --expected-version")
    return args


if __name__ == "__main__":
    arguments = parse_args()
    bench = FaultBench(arguments)
    if arguments.mode == "baseline":
        bench.capture_baseline()
    else:
        bench.verify()
