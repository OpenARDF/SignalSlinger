#!/usr/bin/env node

import { createHash } from "node:crypto";
import { spawnSync } from "node:child_process";
import {
  existsSync,
  readdirSync,
  readFileSync,
  statSync,
} from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import {
  mergeHexImages,
  parseIntelHex,
  summarizeHex,
} from "./intel-hex.mjs";

const workspaceRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");

function fail(message) {
  throw new Error(`SignalSlinger release validation: ${message}`);
}

function parseArguments(argv) {
  let packageDirectory = null;
  for (let index = 0; index < argv.length; index += 1) {
    if (argv[index] === "--package-dir") {
      packageDirectory = resolve(argv[index + 1]);
      index += 1;
    } else {
      fail(`unknown argument: ${argv[index]}`);
    }
  }
  if (!packageDirectory) {
    fail("--package-dir is required");
  }
  if (!existsSync(packageDirectory) || !statSync(packageDirectory).isDirectory()) {
    fail(`package directory does not exist: ${packageDirectory}`);
  }
  return packageDirectory;
}

function oneMatch(directory, pattern, description) {
  const matches = readdirSync(directory).filter((name) => pattern.test(name));
  if (matches.length !== 1) {
    fail(`expected exactly one ${description}; found ${matches.length}`);
  }
  return matches[0];
}

function requiredString(object, name) {
  if (typeof object?.[name] !== "string" || object[name].trim() === "") {
    fail(`release info field '${name}' is missing or empty`);
  }
  return object[name];
}

function requiredInteger(object, name) {
  if (!Number.isSafeInteger(object?.[name])) {
    fail(`release info field '${name}' is not an integer`);
  }
  return object[name];
}

function hexAddress(value, name) {
  if (typeof value !== "string" || !/^0x[0-9A-Fa-f]+$/u.test(value)) {
    fail(`release info field '${name}' is not a hexadecimal address`);
  }
  return Number.parseInt(value.slice(2), 16);
}

function sha256(filePath) {
  return createHash("sha256").update(readFileSync(filePath)).digest("hex").toUpperCase();
}

function assertSafeFileName(fileName) {
  if (
    typeof fileName !== "string"
    || fileName === ""
    || fileName !== fileName.split(/[\\/]/u).at(-1)
    || fileName === "."
    || fileName === ".."
  ) {
    fail(`unsafe package file name: ${fileName}`);
  }
}

function readChecksums(filePath) {
  const checksums = new Map();
  for (const line of readFileSync(filePath, "utf8").split(/\r?\n/u)) {
    if (!line) {
      continue;
    }
    const match = line.match(/^([0-9A-Fa-f]{64})\s{2}(.+)$/u);
    if (!match) {
      fail(`invalid checksum line: ${line}`);
    }
    assertSafeFileName(match[2]);
    if (checksums.has(match[2])) {
      fail(`duplicate checksum entry: ${match[2]}`);
    }
    checksums.set(match[2], match[1].toUpperCase());
  }
  return checksums;
}

function assertMemoryEquals(actual, expected, description) {
  if (actual.size !== expected.size) {
    fail(`${description} byte count differs: ${actual.size} versus ${expected.size}`);
  }
  for (const [address, value] of expected) {
    if (actual.get(address) !== value) {
      fail(`${description} differs at 0x${address.toString(16)}`);
    }
  }
}

function zipEntries(zipPath) {
  const result = spawnSync("unzip", ["-Z1", zipPath], {
    encoding: "utf8",
    stdio: "pipe",
  });
  if (result.status !== 0) {
    fail(`unable to inspect ZIP: ${result.stderr.trim()}`);
  }
  return result.stdout.trim().split(/\r?\n/u).filter(Boolean);
}

function main() {
  const packageDirectory = parseArguments(process.argv.slice(2));
  const releaseInfoName = oneMatch(
    packageDirectory,
    /^SignalSlinger-Release-Info-.*\.json$/u,
    "release-info JSON",
  );
  const checksumsName = oneMatch(
    packageDirectory,
    /^SignalSlinger-Checksums-.*\.txt$/u,
    "checksum file",
  );
  const zipName = oneMatch(
    packageDirectory,
    /^SignalSlinger-.*-Release-Files\.zip$/u,
    "release ZIP",
  );
  const releaseInfo = JSON.parse(
    readFileSync(join(packageDirectory, releaseInfoName), "utf8"),
  );

  if (releaseInfo.format !== "signalslinger-release-info-v1") {
    fail(`unsupported release-info format: ${releaseInfo.format}`);
  }
  if (releaseInfo.product !== "SignalSlinger") {
    fail(`unexpected product: ${releaseInfo.product}`);
  }
  const version = requiredString(releaseInfo, "version");
  if (!/^\d+\.\d+\.\d+$/u.test(version) || Number(version.split(".")[0]) < 2) {
    fail(`invalid bootloader-capable release version: ${version}`);
  }
  const board = requiredString(releaseInfo, "board");
  if (!["HW-3.4", "HW-3.5"].includes(board)) {
    fail(`unsupported board: ${board}`);
  }
  if (!/^[0-9a-f]{40}$/u.test(requiredString(releaseInfo, "gitCommit"))) {
    fail("gitCommit is not a full lowercase Git commit ID");
  }
  if (typeof releaseInfo.sourceTreeDirty !== "boolean") {
    fail("sourceTreeDirty must be a boolean");
  }
  if (releaseInfo.buildProfile?.status !== "reference-version-match") {
    fail("release package was not built with the pinned reference profile");
  }

  const serial = releaseInfo.serialSlinger;
  const workshop = releaseInfo.workshopSetup;
  const appStart = hexAddress(requiredString(serial, "appStartAddress"), "appStartAddress");
  const pageBytes = requiredInteger(serial, "pageBytes");
  const flashBytes = requiredInteger(serial, "flashBytes");
  const bootSectionPages = requiredInteger(workshop, "bootSectionPages");
  if (
    appStart !== 0x2000
    || pageBytes !== 512
    || flashBytes !== 131072
    || bootSectionPages * pageBytes !== appStart
  ) {
    fail("release geometry does not match the qualified AVR128DA28 boot chain");
  }
  if (
    requiredInteger(serial, "updateBaud") !== 115200
    || requiredInteger(serial, "protocolVersion") < 1
    || requiredString(serial, "appInfoCommand") !== "INF"
    || requiredString(serial, "appUpdateCommand") !== "UPD"
    || requiredString(serial, "bootloaderEntryCommand") !== "U"
    || hexAddress(requiredString(workshop, "fuseBootSize"), "fuseBootSize")
      !== bootSectionPages
    || hexAddress(requiredString(workshop, "fuseCodeSize"), "fuseCodeSize") !== 0
  ) {
    fail("release protocol or fuse metadata does not match the qualified profile");
  }

  const checksums = readChecksums(join(packageDirectory, checksumsName));
  for (const [fileName, expectedHash] of checksums) {
    const filePath = join(packageDirectory, fileName);
    if (!existsSync(filePath) || !statSync(filePath).isFile()) {
      fail(`checksum-listed file is missing: ${fileName}`);
    }
    const actualHash = sha256(filePath);
    if (actualHash !== expectedHash) {
      fail(`checksum mismatch for ${fileName}: ${actualHash} versus ${expectedHash}`);
    }
  }

  if (!Array.isArray(releaseInfo.files) || releaseInfo.files.length < 6) {
    fail("release info does not contain the expected file manifest");
  }
  for (const entry of releaseInfo.files) {
    const fileName = requiredString(entry, "fileName");
    assertSafeFileName(fileName);
    const filePath = join(packageDirectory, fileName);
    if (!existsSync(filePath)) {
      fail(`manifest file is missing: ${fileName}`);
    }
    if (requiredInteger(entry, "sizeBytes") !== statSync(filePath).size) {
      fail(`manifest size mismatch for ${fileName}`);
    }
    if (requiredString(entry, "sha256").toUpperCase() !== sha256(filePath)) {
      fail(`manifest hash mismatch for ${fileName}`);
    }
    if (!checksums.has(fileName)) {
      fail(`manifest file is absent from checksums: ${fileName}`);
    }
  }

  const expectedArchiveEntries = new Set([
    ...releaseInfo.files.map((entry) => entry.fileName),
    releaseInfoName,
    checksumsName,
    oneMatch(
      packageDirectory,
      /^README-SignalSlinger-.*\.txt$/u,
      "package README",
    ),
  ]);
  const actualArchiveEntries = zipEntries(join(packageDirectory, zipName));
  for (const entry of actualArchiveEntries) {
    assertSafeFileName(entry);
  }
  if (
    actualArchiveEntries.length !== expectedArchiveEntries.size
    || actualArchiveEntries.some((entry) => !expectedArchiveEntries.has(entry))
  ) {
    fail("release ZIP contents do not exactly match the package manifest");
  }

  const updateName = requiredString(releaseInfo.update, "fileName");
  const firstInstallName = requiredString(releaseInfo.firstInstall, "fileName");
  const bootloaderName = requiredString(workshop, "setupHelperFileName");
  const updateImage = parseIntelHex(join(packageDirectory, updateName));
  const firstInstallImage = parseIntelHex(join(packageDirectory, firstInstallName));
  const bootloaderImage = parseIntelHex(join(packageDirectory, bootloaderName));
  const updateSummary = summarizeHex(updateImage);
  const firstInstallSummary = summarizeHex(firstInstallImage);
  const bootloaderSummary = summarizeHex(bootloaderImage);

  if (updateSummary.first !== appStart || updateSummary.last >= flashBytes) {
    fail("update image is outside the application flash range");
  }
  if (bootloaderSummary.first !== 0 || bootloaderSummary.last >= appStart) {
    fail("bootloader helper is outside the boot section");
  }
  if (firstInstallSummary.first !== 0 || firstInstallSummary.last >= flashBytes) {
    fail("first-install image is outside AVR flash");
  }
  assertMemoryEquals(
    firstInstallImage,
    mergeHexImages(bootloaderImage, updateImage),
    "first-install image",
  );
  if (requiredInteger(releaseInfo.update, "bytesInImage") !== updateSummary.count) {
    fail("update byte count does not match release info");
  }
  if (
    requiredInteger(releaseInfo.firstInstall, "bytesInImage")
    !== firstInstallSummary.count
  ) {
    fail("first-install byte count does not match release info");
  }

  process.stdout.write(
    `PASS release package: SignalSlinger ${version} ${board}\n`
      + `Update 0x${updateSummary.first.toString(16)}..0x${updateSummary.last.toString(16)} `
      + `(${updateSummary.count} bytes)\n`
      + `First install ${firstInstallSummary.count} bytes; `
      + `ZIP ${sha256(join(packageDirectory, zipName))}\n`,
  );
}

try {
  main();
} catch (error) {
  process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
  process.exitCode = 2;
}
