#!/usr/bin/env node

import fs from "node:fs";
import process from "node:process";

const candidateItems = [
  "branch-announced",
  "working-tree-reviewed",
  "release-channel",
  "firmware-version-readme",
  "source-commit-frozen",
  "full-repository-check",
  "deterministic-avr-builds",
  "artifact-manifest",
  "package-validation",
  "hardware-checklist",
  "live-version-report",
  "rollback-package",
  "release-notes",
  "development2-commit-policy",
];

const releaseItems = [
  ...candidateItems,
  "integration-approved",
  "release-branch-verified",
  "release-approved",
];

const finalItems = [
  ...releaseItems,
  "annotated-tag",
  "github-release-created",
  "hardware-assets-uploaded",
  "remote-release-verified",
];

const requiredByPhase = {
  template: finalItems,
  candidate: candidateItems,
  release: releaseItems,
  final: finalItems,
};

function usage() {
  console.error(
    "Usage: node ./scripts/check-release-checklist.mjs --file <checklist.json> "
      + "--phase <template|candidate|release|final>",
  );
}

function parseArgs(argv) {
  const args = {};

  for (let index = 2; index < argv.length; index += 1) {
    const arg = argv[index];

    if (arg === "--file") {
      args.file = argv[index + 1];
      index += 1;
    } else if (arg === "--phase") {
      args.phase = argv[index + 1];
      index += 1;
    } else if (arg === "--help" || arg === "-h") {
      args.help = true;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }

  return args;
}

function isNonEmptyString(value) {
  return typeof value === "string" && value.trim().length > 0;
}

function itemIsComplete(item) {
  if (item.status === "done") {
    return isNonEmptyString(item.evidence);
  }

  if (item.status === "skipped") {
    return isNonEmptyString(item.skipReason) && isNonEmptyString(item.skipRequestedBy);
  }

  return false;
}

function validateMetadata(checklist, templateMode, failures) {
  if (checklist.repository !== "OpenARDF/SignalSlinger") {
    failures.push("repository: expected OpenARDF/SignalSlinger");
  }
  if (!["stable", "prerelease"].includes(checklist.channel)) {
    failures.push("channel: expected stable or prerelease");
  }
  if (
    !Array.isArray(checklist.hardwareTargets)
    || JSON.stringify(checklist.hardwareTargets) !== JSON.stringify(["HW-3.4", "HW-3.5"])
  ) {
    failures.push("hardwareTargets: expected exactly HW-3.4 and HW-3.5");
  }
  for (const name of ["release", "firmwareVersion"]) {
    if (!isNonEmptyString(checklist[name])) {
      failures.push(`${name}: missing release metadata`);
    }
  }
  if (!templateMode) {
    if (!/^v\d+\.\d+\.\d+$/.test(checklist.release)) {
      failures.push("release: expected vMAJOR.MINOR.PATCH");
    }
    if (!/^\d+\.\d+\.\d+$/.test(checklist.firmwareVersion)) {
      failures.push("firmwareVersion: expected MAJOR.MINOR.PATCH");
    }
    if (checklist.release !== `v${checklist.firmwareVersion}`) {
      failures.push("release and firmwareVersion do not match");
    }
    if (!/^[0-9a-f]{40}$/.test(checklist.sourceCommit)) {
      failures.push("sourceCommit: expected the full lowercase Git commit ID");
    }
  }
}

try {
  const args = parseArgs(process.argv);

  if (args.help) {
    usage();
    process.exit(0);
  }

  if (!args.file || !args.phase) {
    usage();
    process.exit(2);
  }

  const requiredIds = requiredByPhase[args.phase];

  if (!requiredIds) {
    throw new Error(`Unknown phase '${args.phase}'. Expected one of: ${Object.keys(requiredByPhase).join(", ")}`);
  }

  const checklist = JSON.parse(fs.readFileSync(args.file, "utf8"));

  if (!Array.isArray(checklist.items)) {
    throw new Error("Checklist must contain an 'items' array.");
  }

  const templateMode = args.phase === "template";
  const failures = [];
  validateMetadata(checklist, templateMode, failures);
  const itemsById = new Map();

  for (const item of checklist.items) {
    if (!isNonEmptyString(item.id)) {
      failures.push("item: missing id");
      continue;
    }
    if (itemsById.has(item.id)) {
      failures.push(`${item.id}: duplicate checklist item`);
    }
    if (!isNonEmptyString(item.description)) {
      failures.push(`${item.id}: missing description`);
    }
    itemsById.set(item.id, item);
  }

  for (const id of requiredIds) {
    const item = itemsById.get(id);

    if (!item) {
      failures.push(`${id}: missing from checklist`);
      continue;
    }

    if (templateMode) {
      if (item.status !== "pending" || item.evidence !== "") {
        failures.push(`${id}: template items must start pending with empty evidence`);
      }
    } else if (!itemIsComplete(item)) {
      failures.push(
        `${id}: mark status as 'done' with evidence, or 'skipped' with skipReason and skipRequestedBy`,
      );
    }
  }

  if (failures.length > 0) {
    console.error(`Release checklist is invalid for phase '${args.phase}':`);
    for (const failure of failures) {
      console.error(`- ${failure}`);
    }
    process.exit(1);
  }

  console.log(`PASS release checklist phase '${args.phase}' (${requiredIds.length} items checked)`);
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(2);
}
