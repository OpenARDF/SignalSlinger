#!/usr/bin/env node

import fs from "node:fs";
import process from "node:process";

const requiredByPhase = {
  "pre-release": [
    "branch-announced",
    "working-tree-reviewed",
    "firmware-version-readme",
    "release-channel",
    "release-build",
    "prepare-release",
    "release-assets",
    "readme-review",
    "development2-commit-policy",
    "gh-environment",
    "release-notes",
  ],
  "pre-tag": [
    "branch-announced",
    "working-tree-reviewed",
    "firmware-version-readme",
    "release-channel",
    "release-build",
    "prepare-release",
    "release-assets",
    "readme-review",
    "development2-commit-policy",
    "gh-environment",
    "release-notes",
  ],
  final: [
    "branch-announced",
    "working-tree-reviewed",
    "firmware-version-readme",
    "release-channel",
    "release-build",
    "prepare-release",
    "release-assets",
    "readme-review",
    "development2-commit-policy",
    "gh-environment",
    "release-notes",
    "github-release-created",
    "hardware-assets-uploaded",
    "remote-release-verified",
  ],
};

function usage() {
  console.error(
    "Usage: node ./scripts/check-release-checklist.mjs --file <checklist.json> --phase <pre-release|pre-tag|final>",
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

  const itemsById = new Map(checklist.items.map((item) => [item.id, item]));
  const failures = [];

  for (const id of requiredIds) {
    const item = itemsById.get(id);

    if (!item) {
      failures.push(`${id}: missing from checklist`);
      continue;
    }

    if (!itemIsComplete(item)) {
      failures.push(
        `${id}: mark status as 'done' with evidence, or 'skipped' with skipReason and skipRequestedBy`,
      );
    }
  }

  if (failures.length > 0) {
    console.error(`Release checklist is incomplete for phase '${args.phase}':`);
    for (const failure of failures) {
      console.error(`- ${failure}`);
    }
    process.exit(1);
  }

  console.log(`Release checklist passed for phase '${args.phase}' (${requiredIds.length} items checked).`);
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(2);
}
