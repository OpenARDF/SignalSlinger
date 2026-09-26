#!/usr/bin/env node

import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const firmwareRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(firmwareRoot, "../..");
const definitions = readFileSync(resolve(firmwareRoot, "SignalSlinger/defs.h"), "utf8");
const readme = readFileSync(resolve(repositoryRoot, "README.md"), "utf8");
const version = definitions.match(/^\s*#define\s+SW_REVISION\s+"([^"]+)"\s*$/mu)?.[1];
const versionParts = version?.match(/^(\d+\.\d+\.\d+)([a-z]+)?$/u);

if (!versionParts) {
  throw new Error("SignalSlinger/defs.h must contain a release or letter-suffixed test SW_REVISION");
}
if (/^\s*#define\s+TEST_MODE_SOFTWARE\b/mu.test(definitions)) {
  throw new Error("TEST_MODE_SOFTWARE is enabled; this source is not releasable");
}

// Test deployments retain the current release's README assets. Numeric-only
// enforcement remains in the release/package gates, where it belongs.
const baseVersion = versionParts[1];
for (const hardware of ["3.4", "3.5"]) {
  const expected = `SignalSlinger-Update-v${baseVersion}-HW-${hardware}.hex`;
  if (!readme.includes(expected)) {
    throw new Error(`Repository README does not reference ${expected}`);
  }
}

process.stdout.write(
  `PASS version consistency: SignalSlinger ${version}, base release ${baseVersion}, HW-3.4 and HW-3.5\n`,
);
