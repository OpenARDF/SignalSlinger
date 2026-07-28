#!/usr/bin/env node

import {
  existsSync,
  mkdirSync,
  readFileSync,
  writeFileSync,
} from "node:fs";
import { resolve } from "node:path";
import process from "node:process";
import {
  expectedUpdateAsset,
  firmwareRoot,
  latestReleaseVersion,
  nextPatch,
  parseVersion,
  readFirmwareVersion,
  readLocalReleaseTags,
  readRemoteReleaseTags,
  repositoryRoot,
  runGit,
  tagAlignmentFailures,
} from "./release-support.mjs";

function parseArgs(argv) {
  const options = {};

  for (let index = 2; index < argv.length; index += 1) {
    if (argv[index] === "--version") {
      options.version = argv[index + 1];
      index += 1;
    } else if (argv[index] === "--channel") {
      options.channel = argv[index + 1];
      index += 1;
    } else {
      throw new Error(`Unknown argument: ${argv[index]}`);
    }
  }

  if (!options.version) {
    throw new Error("--version is required");
  }
  if (!["stable", "prerelease"].includes(options.channel)) {
    throw new Error("--channel must be stable or prerelease");
  }
  return options;
}

function replaceExactlyOnce(text, pattern, replacement, label) {
  const matches = text.match(pattern) ?? [];
  if (matches.length !== 1) {
    throw new Error(`${label}: expected exactly one match, found ${matches.length}`);
  }
  return text.replace(pattern, replacement);
}

function releaseNotesTemplate(version, previousVersion) {
  return `# SignalSlinger ${version}

SignalSlinger ${version} is the next firmware release after v${previousVersion}.

## User-visible changes

- TODO: Describe the most important behavior that a SignalSlinger user can observe.

## Stability and reliability

- TODO: Describe the most important stability, recovery, verification, or release-safety improvement.

## Release files

- \`SignalSlinger-Update-v${version}-HW-3.4.hex\`
- \`SignalSlinger-v${version}-HW-3.4-Release-Files.zip\`
- \`SignalSlinger-Update-v${version}-HW-3.5.hex\`
- \`SignalSlinger-v${version}-HW-3.5-Release-Files.zip\`

## Full changelog

[Compare v${previousVersion}...v${version}](https://github.com/OpenARDF/SignalSlinger/compare/v${previousVersion}...v${version})
`;
}

try {
  const options = parseArgs(process.argv);
  parseVersion(options.version, "release version");
  const branch = runGit(["branch", "--show-current"]).stdout;
  if (branch !== "Development2") {
    throw new Error(
      `Prepare release versions on Development2, not '${branch || "detached HEAD"}'`,
    );
  }

  const remoteTags = readRemoteReleaseTags();
  const localTags = readLocalReleaseTags();
  const alignmentFailures = tagAlignmentFailures(remoteTags, localTags);
  if (alignmentFailures.length > 0) {
    throw new Error(
      `Reconcile local release tags with origin first:\n- ${alignmentFailures.join("\n- ")}`,
    );
  }

  const latest = latestReleaseVersion(remoteTags);
  const expected = nextPatch(latest);
  if (options.version !== expected) {
    throw new Error(
      `Expected the next patch after origin v${latest.text}: ${expected}; `
        + `got ${options.version}`,
    );
  }
  if (remoteTags.has(options.version) || localTags.has(options.version)) {
    throw new Error(`Release tag v${options.version} already exists`);
  }

  const currentVersion = readFirmwareVersion();
  if (![latest.text, options.version].includes(currentVersion)) {
    throw new Error(
      `SW_REVISION is ${currentVersion}; expected current release ${latest.text} `
        + `or prepared release ${options.version}`,
    );
  }

  const definitionsPath = resolve(firmwareRoot, "SignalSlinger/defs.h");
  const definitions = readFileSync(definitionsPath, "utf8");
  const updatedDefinitions = replaceExactlyOnce(
    definitions,
    /^\s*#define\s+SW_REVISION\s+"\d+\.\d+\.\d+"\s*$/gmu,
    `#define SW_REVISION "${options.version}"`,
    "SignalSlinger/defs.h SW_REVISION",
  );

  const readmePath = resolve(repositoryRoot, "README.md");
  let updatedReadme = readFileSync(readmePath, "utf8");
  for (const hardware of ["3.4", "3.5"]) {
    const escapedHardware = hardware.replace(".", "\\.");
    updatedReadme = replaceExactlyOnce(
      updatedReadme,
      new RegExp(
        `SignalSlinger-Update-v\\d+\\.\\d+\\.\\d+-HW-${escapedHardware}\\.hex`,
        "gu",
      ),
      expectedUpdateAsset(options.version, hardware),
      `README.md HW-${hardware} update asset`,
    );
  }

  const checklistPath = resolve(
    repositoryRoot,
    "release-evidence",
    `release-checklist-v${options.version}.json`,
  );
  const releaseNotesRelativePath =
    `Software/AVR128DA28/release-notes/v${options.version}.md`;
  const releaseNotesPath = resolve(repositoryRoot, releaseNotesRelativePath);
  const checklistTemplate = JSON.parse(
    readFileSync(resolve(firmwareRoot, "release-checklist-template.json"), "utf8"),
  );
  let checklist;
  if (existsSync(checklistPath)) {
    checklist = JSON.parse(readFileSync(checklistPath, "utf8"));
    if (
      checklist.release !== `v${options.version}`
      || checklist.previousRelease !== `v${latest.text}`
      || checklist.firmwareVersion !== options.version
      || checklist.channel !== options.channel
    ) {
      throw new Error(`${checklistPath} exists with conflicting metadata`);
    }
    const existingItems = new Map(
      (checklist.items ?? []).map((item) => [item.id, item]),
    );
    checklist.items = checklistTemplate.items.map(
      (templateItem) => existingItems.get(templateItem.id) ?? templateItem,
    );
  } else {
    checklist = checklistTemplate;
    checklist.release = `v${options.version}`;
    checklist.previousRelease = `v${latest.text}`;
    checklist.channel = options.channel;
    checklist.sourceCommit = "";
    checklist.firmwareVersion = options.version;
    mkdirSync(resolve(repositoryRoot, "release-evidence"), { recursive: true });
  }
  checklist.releaseNotesFile = releaseNotesRelativePath;

  writeFileSync(definitionsPath, updatedDefinitions, "utf8");
  writeFileSync(readmePath, updatedReadme, "utf8");
  if (!existsSync(releaseNotesPath)) {
    mkdirSync(resolve(firmwareRoot, "release-notes"), { recursive: true });
    writeFileSync(
      releaseNotesPath,
      releaseNotesTemplate(options.version, latest.text),
      "utf8",
    );
  }

  const machineEvidence = new Map([
    [
      "release-version-incremented",
      `Origin latest release v${latest.text}; v${options.version} was unused and is the exact next patch.`,
    ],
    [
      "release-tags-reconciled",
      "All local semantic release tag object IDs matched origin; no remote tag was changed.",
    ],
    [
      "firmware-version-readme",
      `SW_REVISION and both README update assets use ${options.version}.`,
    ],
    [
      "release-channel",
      `${options.channel} release selected${options.channel === "stable" ? " for main" : ""}.`,
    ],
  ]);
  for (const item of checklist.items) {
    const evidence = machineEvidence.get(item.id);
    if (evidence && item.status === "pending") {
      item.status = "done";
      item.evidence = evidence;
    }
  }
  writeFileSync(checklistPath, `${JSON.stringify(checklist, null, 2)}\n`, "utf8");

  console.log(`Prepared SignalSlinger v${options.version} (${options.channel}).`);
  console.log(`Previous origin release: v${latest.text}`);
  console.log(`Checklist: ${checklistPath}`);
  console.log(`Release notes: ${releaseNotesPath}`);
  console.log("Bootloader version was intentionally left unchanged.");
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
}
