#!/usr/bin/env node

import { existsSync, readFileSync } from "node:fs";
import { resolve } from "node:path";
import process from "node:process";
import {
  expectedUpdateAsset,
  latestReleaseVersion,
  nextPatch,
  parseVersion,
  readFirmwareVersion,
  readLocalReleaseTags,
  readRemoteHeads,
  readRemoteReleaseTags,
  repositoryRoot,
  runGit,
  tagAlignmentFailures,
} from "./release-support.mjs";

function parseArgs(argv) {
  const options = {};

  for (let index = 2; index < argv.length; index += 1) {
    if (argv[index] === "--candidate-version") {
      options.candidateVersion = argv[index + 1];
      index += 1;
    } else if (argv[index] === "--allow-dirty") {
      options.allowDirty = true;
    } else {
      throw new Error(`Unknown argument: ${argv[index]}`);
    }
  }
  return options;
}

function fail(message) {
  throw new Error(`Release state is not ready: ${message}`);
}

try {
  const options = parseArgs(process.argv);
  const candidateVersion = options.candidateVersion ?? readFirmwareVersion();
  parseVersion(candidateVersion, "candidate version");

  const branch = runGit(["branch", "--show-current"]).stdout;
  if (!["Development2", "main"].includes(branch)) {
    fail(`expected Development2 or main, got '${branch || "detached HEAD"}'`);
  }
  const remoteHeads = readRemoteHeads();
  const localHead = runGit(["rev-parse", "HEAD"]).stdout;
  if (localHead !== remoteHeads.get(branch)) {
    fail(
      `${branch} HEAD ${localHead.slice(0, 12)} does not match origin `
        + `${remoteHeads.get(branch).slice(0, 12)}`,
    );
  }
  const relevantStatus = runGit([
    "status",
    "--porcelain",
    "--",
    "Software/AVR128DA28",
    "README.md",
  ]).stdout;
  if (relevantStatus && !options.allowDirty) {
    fail(`release-relevant files are not clean:\n${relevantStatus}`);
  }

  const remoteTags = readRemoteReleaseTags();
  const localTags = readLocalReleaseTags();
  const alignmentFailures = tagAlignmentFailures(remoteTags, localTags);
  if (alignmentFailures.length > 0) {
    fail(`local release tags disagree with origin:\n- ${alignmentFailures.join("\n- ")}`);
  }

  const latest = latestReleaseVersion(remoteTags);
  const expected = nextPatch(latest);
  if (candidateVersion !== expected) {
    fail(
      `candidate ${candidateVersion} is not the next patch after origin v${latest.text}; `
        + `expected ${expected}`,
    );
  }
  if (remoteTags.has(candidateVersion) || localTags.has(candidateVersion)) {
    fail(`candidate tag v${candidateVersion} already exists`);
  }

  const firmwareVersion = readFirmwareVersion();
  if (firmwareVersion !== candidateVersion) {
    fail(`SW_REVISION is ${firmwareVersion}, expected ${candidateVersion}`);
  }

  const readme = readFileSync(resolve(repositoryRoot, "README.md"), "utf8");
  for (const hardware of ["3.4", "3.5"]) {
    const asset = expectedUpdateAsset(candidateVersion, hardware);
    if (!readme.includes(asset)) {
      fail(`README.md does not reference ${asset}`);
    }
  }

  const checklistPath = resolve(
    repositoryRoot,
    "release-evidence",
    `release-checklist-v${candidateVersion}.json`,
  );
  if (existsSync(checklistPath)) {
    const checklist = JSON.parse(readFileSync(checklistPath, "utf8"));
    const expectedMetadata = {
      release: `v${candidateVersion}`,
      previousRelease: `v${latest.text}`,
      repository: "OpenARDF/SignalSlinger",
      firmwareVersion: candidateVersion,
    };
    for (const [name, value] of Object.entries(expectedMetadata)) {
      if (checklist[name] !== value) {
        fail(`${name} is '${checklist[name]}', expected '${value}'`);
      }
    }
  } else if (branch === "Development2") {
    fail(`missing candidate checklist ${checklistPath}`);
  }

  console.log(`PASS release state: ${branch} candidate v${candidateVersion}`);
  console.log(`Previous origin release: v${latest.text}`);
  console.log("Local semantic release tags match origin; candidate tag is available.");
  if (options.allowDirty && relevantStatus) {
    console.log("Dirty release sources allowed only for version preparation.");
  } else {
    console.log(`${branch} is clean and matches the live origin head.`);
  }
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
}
