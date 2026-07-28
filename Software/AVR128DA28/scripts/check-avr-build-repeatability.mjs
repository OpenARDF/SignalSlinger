#!/usr/bin/env node

import { readFileSync } from "node:fs";
import { join } from "node:path";
import { spawnSync } from "node:child_process";
import { workspaceRoot } from "./avr-build-support.mjs";

const label = "SignalSlinger AVR repeatability check";
const rawArguments = process.argv.slice(2);
const bootloader = rawArguments.includes("--bootloader");
const buildArguments = rawArguments.filter((argument) => argument !== "--bootloader");

function fail(message) {
  throw new Error(`${label}: ${message}`);
}

function optionValue(name) {
  const index = buildArguments.indexOf(name);
  return index < 0 ? null : buildArguments[index + 1];
}

function applicationOutputRoot() {
  const hardware = optionValue("--hardware");
  const start = Number(optionValue("--application-start") || 0);
  const relocationSuffix = start ? "-relocated" : "";
  const hardwareSuffix = hardware
    ? `-hw-${hardware.replace(".", "-")}`
    : "-active-hw";
  return join(
    workspaceRoot,
    "tmp",
    `avr-release${relocationSuffix}${hardwareSuffix}`,
  );
}

const script = bootloader
  ? join(workspaceRoot, "scripts", "build-avr-bootloader.mjs")
  : join(workspaceRoot, "scripts", "build-avr-release.mjs");
const outputRoot = bootloader
  ? join(workspaceRoot, "tmp", "avr-bootloader-release")
  : applicationOutputRoot();
const evidencePath = join(outputRoot, "build-evidence.json");

function build() {
  const result = spawnSync(
    process.execPath,
    [script, ...buildArguments],
    {
      cwd: workspaceRoot,
      encoding: "utf8",
      maxBuffer: 64 * 1024 * 1024,
    },
  );
  if (result.stdout) {
    process.stdout.write(result.stdout);
  }
  if (result.stderr) {
    process.stderr.write(result.stderr);
  }
  if (result.error) {
    fail(`build could not run: ${result.error.message}`);
  }
  if (result.status !== 0) {
    fail(`build exited with status ${result.status}`);
  }
  return readFileSync(evidencePath, "utf8");
}

try {
  process.stdout.write(`${label}: pass 1 of 2\n`);
  const first = build();
  process.stdout.write(`${label}: pass 2 of 2\n`);
  const second = build();
  if (first !== second) {
    fail(`build evidence differs between passes: ${evidencePath}`);
  }
  const evidence = JSON.parse(second);
  if (evidence.warningCount !== 0) {
    fail(`repeatable build still reports ${evidence.warningCount} warning(s)`);
  }
  process.stdout.write(
    `${label}: PASS (${evidence.artifacts.length} byte-identical artifacts, zero warnings)\n`,
  );
} catch (error) {
  process.stderr.write(`${error.message}\n`);
  process.exitCode = 2;
}
