#!/usr/bin/env node

import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const firmwareRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(firmwareRoot, "../..");
const definitions = readFileSync(resolve(firmwareRoot, "SignalSlinger/defs.h"), "utf8");
const readme = readFileSync(resolve(repositoryRoot, "README.md"), "utf8");
const version = definitions.match(/^\s*#define\s+SW_REVISION\s+"([^"]+)"\s*$/mu)?.[1];

if (!version || !/^\d+\.\d+\.\d+$/u.test(version)) {
  throw new Error("SignalSlinger/defs.h must contain a semantic SW_REVISION");
}
if (/^\s*#define\s+TEST_MODE_SOFTWARE\b/mu.test(definitions)) {
  throw new Error("TEST_MODE_SOFTWARE is enabled; this source is not releasable");
}

for (const hardware of ["3.4", "3.5"]) {
  const expected = `SignalSlinger-Update-v${version}-HW-${hardware}.hex`;
  if (!readme.includes(expected)) {
    throw new Error(`Repository README does not reference ${expected}`);
  }
}

process.stdout.write(
  `PASS release version consistency: SignalSlinger ${version}, HW-3.4 and HW-3.5\n`,
);
