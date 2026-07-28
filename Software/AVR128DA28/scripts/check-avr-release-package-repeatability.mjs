#!/usr/bin/env node

import { createHash } from "node:crypto";
import { spawnSync } from "node:child_process";
import { readdirSync, readFileSync, statSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const workspaceRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");

function fail(message) {
  throw new Error(`SignalSlinger release package repeatability: ${message}`);
}

function run(script, args = []) {
  const result = spawnSync(process.execPath, [join(workspaceRoot, "scripts", script), ...args], {
    cwd: workspaceRoot,
    stdio: "inherit",
  });
  if (result.status !== 0) {
    fail(`${script} failed with exit code ${result.status}`);
  }
}

function sha256(filePath) {
  return createHash("sha256").update(readFileSync(filePath)).digest("hex");
}

function snapshot(directory) {
  return readdirSync(directory)
    .filter((name) => statSync(join(directory, name)).isFile())
    .sort()
    .map((name) => ({
      name,
      bytes: statSync(join(directory, name)).size,
      sha256: sha256(join(directory, name)),
    }));
}

function main() {
  run("build-avr-bootloader.mjs");
  for (const hardware of ["3.4", "3.5"]) {
    run("build-avr-release.mjs", [
      "--hardware",
      hardware,
      "--application-start",
      "0x2000",
    ]);
  }

  const first = new Map();
  for (const hardware of ["3.4", "3.5"]) {
    run("package-avr-release.mjs", ["--hardware", hardware]);
    const directory = join(
      workspaceRoot,
      "release-packages",
      `SignalSlinger-${
        readFileSync(join(workspaceRoot, "SignalSlinger", "defs.h"), "utf8")
          .match(/^\s*#define\s+SW_REVISION\s+"([^"]+)"/mu)?.[1]
      }-HW-${hardware}`,
    );
    first.set(hardware, snapshot(directory));
  }

  for (const hardware of ["3.4", "3.5"]) {
    run("package-avr-release.mjs", ["--hardware", hardware]);
    const firstSnapshot = first.get(hardware);
    const packageName = firstSnapshot.find((entry) => entry.name.endsWith(".zip"))?.name;
    const directory = join(
      workspaceRoot,
      "release-packages",
      `SignalSlinger-${
        readFileSync(join(workspaceRoot, "SignalSlinger", "defs.h"), "utf8")
          .match(/^\s*#define\s+SW_REVISION\s+"([^"]+)"/mu)?.[1]
      }-HW-${hardware}`,
    );
    const secondSnapshot = snapshot(directory);
    if (JSON.stringify(firstSnapshot) !== JSON.stringify(secondSnapshot)) {
      fail(`HW-${hardware} package artifacts changed between identical runs`);
    }
    const packageEntry = secondSnapshot.find((entry) => entry.name === packageName);
    process.stdout.write(
      `PASS HW-${hardware}: ${secondSnapshot.length} byte-identical package files; `
        + `${packageEntry.sha256}  ${packageEntry.name}\n`,
    );
  }
}

try {
  main();
} catch (error) {
  process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
  process.exitCode = 2;
}
