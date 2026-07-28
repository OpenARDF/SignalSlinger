#!/usr/bin/env node

import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import process from "node:process";
import { repositoryRoot } from "./release-support.mjs";

const scriptDirectory = dirname(fileURLToPath(import.meta.url));

function usage() {
  console.error(
    "Usage: node ./scripts/check-github-release-notes.mjs "
      + "--checklist <checklist.json>",
  );
}

function parseArgs(argv) {
  const options = {};

  for (let index = 2; index < argv.length; index += 1) {
    if (argv[index] === "--checklist") {
      options.checklist = argv[index + 1];
      index += 1;
    } else if (argv[index] === "--help" || argv[index] === "-h") {
      options.help = true;
    } else {
      throw new Error(`Unknown argument: ${argv[index]}`);
    }
  }
  return options;
}

function run(command, args) {
  const result = spawnSync(command, args, {
    encoding: "utf8",
    stdio: ["ignore", "pipe", "pipe"],
  });
  if (result.error) {
    throw result.error;
  }
  if (result.status !== 0) {
    throw new Error(
      result.stderr.trim()
        || result.stdout.trim()
        || `${command} exited with status ${result.status}`,
    );
  }
  return result.stdout;
}

function normalizedBody(value) {
  return value.replace(/\r\n/gu, "\n").trim();
}

try {
  const options = parseArgs(process.argv);
  if (options.help) {
    usage();
    process.exit(0);
  }
  if (!options.checklist) {
    usage();
    process.exit(2);
  }

  run(process.execPath, [
    resolve(scriptDirectory, "check-release-notes.mjs"),
    "--checklist",
    options.checklist,
  ]);

  const checklist = JSON.parse(readFileSync(options.checklist, "utf8"));
  const localBody = readFileSync(
    resolve(repositoryRoot, checklist.releaseNotesFile),
    "utf8",
  );
  const release = JSON.parse(
    run("gh", [
      "release",
      "view",
      checklist.release,
      "--repo",
      checklist.repository,
      "--json",
      "body,url",
    ]),
  );

  if (normalizedBody(release.body ?? "") !== normalizedBody(localBody)) {
    throw new Error(
      `GitHub release ${checklist.release} body does not match `
        + checklist.releaseNotesFile,
    );
  }

  console.log(`PASS published GitHub release notes: ${release.url}`);
  console.log(`Remote body matches ${checklist.releaseNotesFile}`);
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
}
