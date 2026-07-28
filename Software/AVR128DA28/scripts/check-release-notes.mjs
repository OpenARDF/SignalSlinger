#!/usr/bin/env node

import { existsSync, readFileSync } from "node:fs";
import { resolve } from "node:path";
import process from "node:process";
import {
  expectedUpdateAsset,
  readFirmwareVersion,
  repositoryRoot,
} from "./release-support.mjs";

function usage() {
  console.error(
    "Usage: node ./scripts/check-release-notes.mjs "
      + "[--checklist <checklist.json>]",
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

function sectionBody(lines, heading, failures) {
  const matches = lines
    .map((line, index) => ({ line, index }))
    .filter(({ line }) => line === `## ${heading}`);

  if (matches.length !== 1) {
    failures.push(
      `${heading}: expected exactly one '## ${heading}' heading, found ${matches.length}`,
    );
    return "";
  }

  const start = matches[0].index + 1;
  const nextHeadingOffset = lines
    .slice(start)
    .findIndex((line) => line.startsWith("## "));
  const end = nextHeadingOffset === -1 ? lines.length : start + nextHeadingOffset;
  return lines.slice(start, end).join("\n").trim();
}

function meaningfulBullets(body) {
  return body
    .split("\n")
    .filter((line) => /^-\s+\S/u.test(line))
    .map((line) => line.replace(/^-\s+/u, "").replace(/[`*_]/gu, "").trim())
    .filter((line) => line.length >= 20);
}

try {
  const options = parseArgs(process.argv);
  if (options.help) {
    usage();
    process.exit(0);
  }
  const checklistPath = options.checklist ?? resolve(
    repositoryRoot,
    "release-evidence",
    `release-checklist-v${readFirmwareVersion()}.json`,
  );
  const checklist = JSON.parse(readFileSync(checklistPath, "utf8"));
  const failures = [];
  const version = checklist.firmwareVersion;
  const previousRelease = checklist.previousRelease;
  const releaseNotesFile = checklist.releaseNotesFile;

  if (!/^\d+\.\d+\.\d+$/u.test(version ?? "")) {
    failures.push("checklist firmwareVersion must use MAJOR.MINOR.PATCH");
  }
  if (!/^v\d+\.\d+\.\d+$/u.test(previousRelease ?? "")) {
    failures.push("checklist previousRelease must use vMAJOR.MINOR.PATCH");
  }

  const expectedRelativePath =
    `Software/AVR128DA28/release-notes/v${version}.md`;
  if (releaseNotesFile !== expectedRelativePath) {
    failures.push(`releaseNotesFile must be ${expectedRelativePath}`);
  }

  const notesPath = resolve(repositoryRoot, releaseNotesFile ?? "");
  if (!existsSync(notesPath)) {
    failures.push(`release notes file does not exist: ${notesPath}`);
  }

  if (failures.length === 0) {
    const notes = readFileSync(notesPath, "utf8").replace(/\r\n/gu, "\n");
    const lines = notes.split("\n");
    const title = `# SignalSlinger ${version}`;
    const titleCount = lines.filter((line) => line === title).length;
    if (titleCount !== 1) {
      failures.push(`expected exactly one '${title}' title, found ${titleCount}`);
    }
    if (/\b(?:TODO|TBD|FIXME|PLACEHOLDER)\b/iu.test(notes)) {
      failures.push("release notes contain an unfinished placeholder");
    }

    const userVisible = sectionBody(lines, "User-visible changes", failures);
    const reliability = sectionBody(lines, "Stability and reliability", failures);
    const releaseFiles = sectionBody(lines, "Release files", failures);
    const fullChangelog = sectionBody(lines, "Full changelog", failures);

    if (meaningfulBullets(userVisible).length === 0) {
      failures.push(
        "User-visible changes must contain at least one substantive bullet",
      );
    }
    if (meaningfulBullets(reliability).length === 0) {
      failures.push(
        "Stability and reliability must contain at least one substantive bullet",
      );
    }

    const expectedAssets = [
      expectedUpdateAsset(version, "3.4"),
      `SignalSlinger-v${version}-HW-3.4-Release-Files.zip`,
      expectedUpdateAsset(version, "3.5"),
      `SignalSlinger-v${version}-HW-3.5-Release-Files.zip`,
    ];
    for (const asset of expectedAssets) {
      if (!releaseFiles.includes(asset)) {
        failures.push(`Release files must name ${asset}`);
      }
    }

    const compareUrl =
      `https://github.com/OpenARDF/SignalSlinger/compare/`
      + `${previousRelease}...v${version}`;
    if (!fullChangelog.includes(compareUrl)) {
      failures.push(`Full changelog must link to ${compareUrl}`);
    }
  }

  if (failures.length > 0) {
    console.error("Release notes are not publication-ready:");
    for (const failure of failures) {
      console.error(`- ${failure}`);
    }
    process.exit(1);
  }

  console.log(`PASS release notes: v${version}`);
  console.log(`File: ${releaseNotesFile}`);
  console.log(
    "Required user-visible and stability/reliability sections are substantive.",
  );
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(2);
}
