#!/usr/bin/env node

import { existsSync, readFileSync } from "node:fs";
import { resolve } from "node:path";
import { spawnSync } from "node:child_process";
import process from "node:process";
import {
  parseVersion,
  readFirmwareVersion,
  readRemoteHeads,
  repositoryRoot,
  runGit,
} from "./release-support.mjs";

function parseArgs(argv) {
  const options = {};

  for (let index = 2; index < argv.length; index += 1) {
    if (argv[index] === "--version") {
      options.version = argv[index + 1];
      index += 1;
    } else if (argv[index] === "--path") {
      options.path = argv[index + 1];
      index += 1;
    } else {
      throw new Error(`Unknown argument: ${argv[index]}`);
    }
  }
  if (!options.version) {
    throw new Error("--version is required");
  }
  parseVersion(options.version, "release version");
  return options;
}

try {
  const options = parseArgs(process.argv);
  const firmwareVersion = readFirmwareVersion();
  if (firmwareVersion !== options.version) {
    throw new Error(`SW_REVISION is ${firmwareVersion}, expected ${options.version}`);
  }

  const branch = runGit(["branch", "--show-current"]).stdout;
  if (branch !== "Development2") {
    throw new Error(`Run this from Development2, not '${branch || "detached HEAD"}'`);
  }

  const developmentHead = runGit(["rev-parse", "Development2"]).stdout;
  const remoteHeads = readRemoteHeads();
  const remoteDevelopmentHead = remoteHeads.get("Development2");
  if (developmentHead !== remoteDevelopmentHead) {
    throw new Error("Development2 must be committed and pushed before integration");
  }
  if (runGit(["status", "--porcelain", "--", "Software/AVR128DA28", "README.md"]).stdout) {
    throw new Error("Release-relevant Development2 files must be clean before integration");
  }

  const checklist = JSON.parse(readFileSync(
    resolve(
      repositoryRoot,
      "release-evidence",
      `release-checklist-v${options.version}.json`,
    ),
    "utf8",
  ));
  if (checklist.sourceCommit !== developmentHead) {
    throw new Error(
      `Checklist sourceCommit must be the full Development2 commit ${developmentHead}`,
    );
  }

  const worktreePath = resolve(
    options.path ?? `/private/tmp/SignalSlinger-release-v${options.version}`,
  );
  if (existsSync(worktreePath)) {
    throw new Error(`Release worktree already exists: ${worktreePath}`);
  }

  const mainHead = runGit(["rev-parse", "main"]).stdout;
  const remoteMainHead = remoteHeads.get("main");
  if (mainHead !== remoteMainHead) {
    throw new Error("Local main must match origin/main before integration");
  }

  runGit(["worktree", "add", worktreePath, "main"]);
  const mergeScript = resolve(
    worktreePath,
    "Software/AVR128DA28/merge-development2-into-main.ps1",
  );
  const merge = runGit(
    ["-C", worktreePath, "status", "--short"],
    { cwd: repositoryRoot },
  );
  if (merge.stdout) {
    throw new Error(`New release worktree is unexpectedly dirty:\n${merge.stdout}`);
  }

  const result = spawnSync(
    "pwsh",
    ["-NoLogo", "-NoProfile", "-File", mergeScript],
    {
      cwd: worktreePath,
      encoding: "utf8",
      stdio: "inherit",
    },
  );
  if (result.error) {
    throw result.error;
  }
  if (result.status !== 0) {
    throw new Error(`Release integration helper failed with exit code ${result.status}`);
  }

  console.log(`Release integration worktree: ${worktreePath}`);
  console.log("The renormalized Development2-to-main merge is staged but not committed.");
  console.log("Review it, run the release preflight there, and commit only after approval.");
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
}
