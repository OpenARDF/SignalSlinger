import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

export const firmwareRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
export const repositoryRoot = resolve(firmwareRoot, "../..");

export function runGit(args, { allowFailure = false, cwd = repositoryRoot } = {}) {
  const result = spawnSync("git", args, {
    cwd,
    encoding: "utf8",
    stdio: ["ignore", "pipe", "pipe"],
  });

  if (result.error) {
    throw result.error;
  }
  if (result.status !== 0 && !allowFailure) {
    const detail = result.stderr.trim() || result.stdout.trim();
    throw new Error(`git ${args.join(" ")} failed${detail ? `: ${detail}` : ""}`);
  }

  return {
    status: result.status,
    stdout: result.stdout.trim(),
    stderr: result.stderr.trim(),
  };
}

export function parseVersion(value, label = "version") {
  const match = /^(\d+)\.(\d+)\.(\d+)$/u.exec(value);
  if (!match) {
    throw new Error(`${label} must use MAJOR.MINOR.PATCH; got '${value}'`);
  }

  return {
    text: value,
    major: Number(match[1]),
    minor: Number(match[2]),
    patch: Number(match[3]),
  };
}

export function compareVersions(left, right) {
  return left.major - right.major
    || left.minor - right.minor
    || left.patch - right.patch;
}

export function nextPatch(version) {
  return `${version.major}.${version.minor}.${version.patch + 1}`;
}

export function readFirmwareVersion() {
  const definitions = readFileSync(
    resolve(firmwareRoot, "SignalSlinger/defs.h"),
    "utf8",
  );
  const version = definitions.match(
    /^\s*#define\s+SW_REVISION\s+"([^"]+)"\s*$/mu,
  )?.[1];

  if (!version) {
    throw new Error("Unable to read SW_REVISION from SignalSlinger/defs.h");
  }
  parseVersion(version, "SW_REVISION");
  return version;
}

export function readRemoteReleaseTags() {
  const output = runGit(["ls-remote", "--tags", "origin"]).stdout;
  const tags = new Map();

  for (const line of output.split(/\r?\n/u)) {
    if (!line) {
      continue;
    }
    const [objectId, reference] = line.split(/\s+/u);
    const match = /^refs\/tags\/v(\d+\.\d+\.\d+)$/u.exec(reference);
    if (match) {
      tags.set(match[1], objectId);
    }
  }

  if (tags.size === 0) {
    throw new Error("No semantic release tags were found on origin");
  }
  return tags;
}

export function readRemoteHeads() {
  const output = runGit([
    "ls-remote",
    "--heads",
    "origin",
    "main",
    "Development2",
  ]).stdout;
  const heads = new Map();

  for (const line of output.split(/\r?\n/u)) {
    if (!line) {
      continue;
    }
    const [objectId, reference] = line.split(/\s+/u);
    const match = /^refs\/heads\/(.+)$/u.exec(reference);
    if (match) {
      heads.set(match[1], objectId);
    }
  }
  for (const branch of ["main", "Development2"]) {
    if (!heads.has(branch)) {
      throw new Error(`Origin branch '${branch}' was not found`);
    }
  }
  return heads;
}

export function readLocalReleaseTags() {
  const output = runGit([
    "for-each-ref",
    "--format=%(refname:short)\t%(objectname)",
    "refs/tags",
  ]).stdout;
  const tags = new Map();

  for (const line of output.split(/\r?\n/u)) {
    if (!line) {
      continue;
    }
    const [name, objectId] = line.split("\t");
    const match = /^v(\d+\.\d+\.\d+)$/u.exec(name);
    if (match) {
      tags.set(match[1], objectId);
    }
  }
  return tags;
}

export function latestReleaseVersion(tags) {
  return [...tags.keys()]
    .map((value) => parseVersion(value))
    .sort(compareVersions)
    .at(-1);
}

export function tagAlignmentFailures(remoteTags, localTags) {
  const failures = [];

  for (const version of remoteTags.keys()) {
    if (!localTags.has(version)) {
      failures.push(`origin v${version} is missing locally`);
    }
  }
  for (const [version, localObject] of localTags) {
    const remoteObject = remoteTags.get(version);
    if (!remoteObject) {
      failures.push(`local v${version} is absent from origin`);
    } else if (localObject !== remoteObject) {
      failures.push(
        `v${version}: local ${localObject.slice(0, 12)} != origin ${remoteObject.slice(0, 12)}`,
      );
    }
  }
  return failures;
}

export function expectedUpdateAsset(version, hardware) {
  return `SignalSlinger-Update-v${version}-HW-${hardware}.hex`;
}
