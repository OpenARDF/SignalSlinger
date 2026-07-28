#!/usr/bin/env node

import { createHash } from "node:crypto";
import { spawnSync } from "node:child_process";
import {
  copyFileSync,
  existsSync,
  mkdirSync,
  readFileSync,
  rmSync,
  statSync,
  utimesSync,
  writeFileSync,
} from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import {
  encodeIntelHex,
  mergeHexImages,
  parseIntelHex,
  summarizeHex,
} from "./intel-hex.mjs";

const workspaceRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(workspaceRoot, "../..");
const supportedProgrammers = [
  "atmelice",
  "pickit4",
  "snap",
  "powerdebugger",
  "edbg",
  "medbg",
  "nedbg",
];

function fail(message) {
  throw new Error(`SignalSlinger release package: ${message}`);
}

function parseArguments(argv) {
  const options = { hardware: null, skipValidate: false };
  for (let index = 0; index < argv.length; index += 1) {
    if (argv[index] === "--hardware") {
      options.hardware = argv[index + 1];
      index += 1;
    } else if (argv[index] === "--skip-validate") {
      options.skipValidate = true;
    } else {
      fail(`unknown argument: ${argv[index]}`);
    }
  }
  if (!["3.4", "3.5"].includes(options.hardware)) {
    fail("--hardware must be 3.4 or 3.5");
  }
  return options;
}

function readDefineString(text, name) {
  const match = text.match(new RegExp(`^\\s*#define\\s+${name}\\s+"([^"]+)"\\s*$`, "mu"));
  if (!match) {
    fail(`could not read string define ${name}`);
  }
  return match[1];
}

function readDefineInteger(text, name) {
  const match = text.match(new RegExp(`^\\s*#define\\s+${name}\\s+([^\\s]+)\\s*$`, "mu"));
  if (!match) {
    fail(`could not read numeric define ${name}`);
  }
  const normalized = match[1].replace(/[uUlL]+$/u, "");
  const value = Number(normalized);
  if (!Number.isSafeInteger(value)) {
    fail(`could not parse numeric define ${name}: ${match[1]}`);
  }
  return value;
}

function sha256(filePath) {
  return createHash("sha256").update(readFileSync(filePath)).digest("hex").toUpperCase();
}

function run(command, args, options = {}) {
  const result = spawnSync(command, args, {
    cwd: options.cwd || workspaceRoot,
    encoding: "utf8",
    stdio: options.capture ? "pipe" : "inherit",
  });
  if (result.status !== 0) {
    const detail = options.capture ? result.stderr.trim() : "";
    fail(`${command} failed${detail ? `: ${detail}` : ""}`);
  }
  return options.capture ? result.stdout.trim() : "";
}

function git(...args) {
  return run("git", ["-C", repositoryRoot, ...args], { capture: true });
}

function requireReferenceEvidence(filePath, expected) {
  if (!existsSync(filePath)) {
    fail(`missing build evidence: ${filePath}`);
  }
  const evidence = JSON.parse(readFileSync(filePath, "utf8"));
  if (evidence.status !== "reference-version-match") {
    fail(`${filePath} is not a reference-version build`);
  }
  if (evidence.warningCount !== 0 || evidence.warnings?.length !== 0) {
    fail(`${filePath} contains compiler warnings`);
  }
  for (const [name, value] of Object.entries(expected)) {
    if (evidence[name] !== value) {
      fail(`${filePath} has ${name}=${evidence[name]}, expected ${value}`);
    }
  }
  return evidence;
}

function packageFile(sourcePath, destinationPath, kind, purpose) {
  if (resolve(sourcePath) !== resolve(destinationPath)) {
    copyFileSync(sourcePath, destinationPath);
  }
  return {
    fileName: destinationPath.split("/").at(-1),
    kind,
    purpose,
    sizeBytes: statSync(destinationPath).size,
    sha256: sha256(destinationPath),
  };
}

function workshopSetupScript({
  board,
  version,
  bootloaderFile,
  applicationFile,
}) {
  return `[CmdletBinding()]
param(
    [string]$Port = 'COM6',
    [ValidateSet('Auto', 'Atprogram', 'Pymcuprog')]
    [string]$Backend = 'Auto',
    [switch]$CheckPrereqs,
    [switch]$CheckProgrammer,
    [switch]$ProgramFuses,
    [switch]$ConfirmFuseWrite,
    [switch]$SkipSerialValidation,
    [switch]$DryRun,
    [string]$AtprogramPath = 'C:\\Program Files (x86)\\Atmel\\Studio\\7.0\\atbackend\\atprogram.exe',
    [string]$PymcuprogCommand = 'pymcuprog',
    [string]$Tool = 'Auto',
    [string]$ToolSerial = '',
    [string]$SupportedTools = '${supportedProgrammers.join(",")}',
    [string]$Clock = '100kHz'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$scriptRoot = $PSScriptRoot
$provisionScript = Join-Path $scriptRoot 'provision-bootloader.ps1'
$bootloaderHex = Join-Path $scriptRoot '${bootloaderFile}'
$applicationHex = Join-Path $scriptRoot '${applicationFile}'

foreach($required in @($provisionScript, $bootloaderHex, $applicationHex))
{
    if(-not (Test-Path -LiteralPath $required))
    {
        Write-Host ("SS_SETUP_ERROR code=missing_package_file detail={0}" -f (($required -replace '\\s+', '_') -replace '[^A-Za-z0-9._:\\\\-]', '_'))
        throw "Required setup file not found: $required"
    }
}

Write-Host 'SignalSlinger update-support setup'
Write-Host 'Board: ${board}'
Write-Host 'Firmware: ${version}'
Write-Host ''

if(-not $CheckPrereqs -and -not ($ProgramFuses -and $ConfirmFuseWrite))
{
    Write-Warning 'This script will not write fuses unless both -ProgramFuses and -ConfirmFuseWrite are provided.'
    Write-Host 'To check this computer without touching the SignalSlinger, run with -CheckPrereqs.'
    Write-Host 'To prepare a connected SignalSlinger, run again with -ProgramFuses -ConfirmFuseWrite.'
    Write-Host ''
}

$provisionArgs = @{
    Port = $Port
    Backend = $Backend
    BootloaderHexPath = $bootloaderHex
    ApplicationHexPath = $applicationHex
    SkipBuild = $true
    AtprogramPath = $AtprogramPath
    PymcuprogCommand = $PymcuprogCommand
    Tool = $Tool
    ToolSerial = $ToolSerial
    SupportedTools = $SupportedTools
    Clock = $Clock
}

foreach($switchName in @('CheckPrereqs', 'CheckProgrammer', 'ProgramFuses', 'ConfirmFuseWrite', 'SkipSerialValidation', 'DryRun'))
{
    if(Get-Variable -Name $switchName -ValueOnly)
    {
        $provisionArgs[$switchName] = $true
    }
}

try
{
    & $provisionScript @provisionArgs
    exit 0
}
catch
{
    Write-Host ("Setup failed: {0}" -f $_.Exception.Message)
    exit 1
}
`;
}

function main() {
  const options = parseArguments(process.argv.slice(2));
  const hardwareSlug = options.hardware.replace(".", "-");
  const board = `HW-${options.hardware}`;
  const bootRoot = join(workspaceRoot, "tmp", "avr-bootloader-release");
  const appRoot = join(workspaceRoot, "tmp", `avr-release-relocated-hw-${hardwareSlug}`);
  const bootHexPath = join(bootRoot, "SignalSlingerBootloader.hex");
  const appHexPath = join(appRoot, "SignalSlinger.hex");

  const bootEvidence = requireReferenceEvidence(
    join(bootRoot, "build-evidence.json"),
    { applicationStart: 0x2000 },
  );
  const appEvidence = requireReferenceEvidence(
    join(appRoot, "build-evidence.json"),
    { hardwareTarget: options.hardware, applicationStart: 0x2000 },
  );
  for (const filePath of [bootHexPath, appHexPath]) {
    if (!existsSync(filePath)) {
      fail(`missing build artifact: ${filePath}`);
    }
  }

  const definitions = readFileSync(join(workspaceRoot, "SignalSlinger", "defs.h"), "utf8");
  const bootConfig = readFileSync(
    join(workspaceRoot, "bootloader", "include", "bootloader_config.h"),
    "utf8",
  );
  const version = readDefineString(definitions, "SW_REVISION");
  if (!/^\d+\.\d+\.\d+$/u.test(version) || Number(version.split(".")[0]) < 2) {
    fail(`SW_REVISION must be a bootloader-capable semantic version; got ${version}`);
  }
  const product = readDefineString(definitions, "PRODUCT_NAME_SHORT");
  const bootloaderVersion = readDefineString(
    bootConfig,
    "SIGNALSLINGER_BOOTLOADER_VERSION",
  );
  const protocolVersion = readDefineInteger(
    bootConfig,
    "SIGNALSLINGER_BOOT_PROTOCOL_VERSION",
  );
  const bootSectionPages = readDefineInteger(
    bootConfig,
    "SIGNALSLINGER_BOOT_SECTION_PAGES",
  );
  const pageBytes = readDefineInteger(bootConfig, "SIGNALSLINGER_FLASH_PAGE_BYTES");
  const flashBytes = readDefineInteger(bootConfig, "SIGNALSLINGER_FLASH_BYTES");
  const updateBaud = readDefineInteger(bootConfig, "SIGNALSLINGER_BOOT_USART_BAUD");
  const appStart = bootSectionPages * pageBytes;
  if (appStart !== 0x2000) {
    fail(`bootloader configuration resolves application start to 0x${appStart.toString(16)}`);
  }

  const bootImage = parseIntelHex(bootHexPath);
  const appImage = parseIntelHex(appHexPath);
  const bootSummary = summarizeHex(bootImage);
  const appSummary = summarizeHex(appImage);
  if (bootSummary.first !== 0 || bootSummary.last >= appStart) {
    fail("bootloader image is outside the reserved boot section");
  }
  if (appSummary.first !== appStart || appSummary.last >= flashBytes) {
    fail("application image is outside the allowed application section");
  }
  const firstInstallImage = mergeHexImages(bootImage, appImage);
  const firstInstallSummary = summarizeHex(firstInstallImage);

  const outputRoot = join(
    workspaceRoot,
    "release-packages",
    `SignalSlinger-${version}-${board}`,
  );
  rmSync(outputRoot, { recursive: true, force: true });
  mkdirSync(outputRoot, { recursive: true });

  const friendlyVersion = `v${version}`;
  const names = {
    update: `SignalSlinger-Update-${friendlyVersion}-${board}.hex`,
    firstInstall: `SignalSlinger-First-Install-${friendlyVersion}-${board}.hex`,
    bootloader:
      `SignalSlinger-Setup-Helper-${friendlyVersion}-${board}-${bootloaderVersion}.hex`,
    releaseInfo: `SignalSlinger-Release-Info-${friendlyVersion}-${board}.json`,
    checksums: `SignalSlinger-Checksums-${friendlyVersion}-${board}.txt`,
    readme: `README-SignalSlinger-${friendlyVersion}-${board}.txt`,
    zip: `SignalSlinger-${friendlyVersion}-${board}-Release-Files.zip`,
    setupLauncher: `Prepare-SignalSlinger-Updates-${friendlyVersion}-${board}.ps1`,
  };
  const output = Object.fromEntries(
    Object.entries(names).map(([key, value]) => [key, join(outputRoot, value)]),
  );

  writeFileSync(output.firstInstall, encodeIntelHex(firstInstallImage), "ascii");
  const files = [
    packageFile(
      appHexPath,
      output.update,
      "update",
      "For SerialSlinger to update a SignalSlinger that already supports software updates.",
    ),
    packageFile(
      output.firstInstall,
      output.firstInstall,
      "first-install",
      "For workshop setup of a new board using a programmer.",
    ),
    packageFile(
      bootHexPath,
      output.bootloader,
      "setup-helper",
      "Used by workshop setup tools when installing update support.",
    ),
  ];

  writeFileSync(
    output.setupLauncher,
    workshopSetupScript({
      board,
      version: friendlyVersion,
      bootloaderFile: names.bootloader,
      applicationFile: names.update,
    }),
    "utf8",
  );
  files.push(packageFile(
    output.setupLauncher,
    output.setupLauncher,
    "workshop-setup-launcher",
    "Friendly setup launcher for adding software-update support with a programmer.",
  ));
  for (const [sourceName, kind, purpose] of [
    [
      "provision-bootloader.ps1",
      "workshop-setup-tool",
      "Advanced setup tool used by the friendly setup launcher.",
    ],
    [
      "test-bootloader-serial.ps1",
      "workshop-setup-tool",
      "Serial verification tool used after adding software-update support.",
    ],
  ]) {
    files.push(packageFile(
      join(workspaceRoot, sourceName),
      join(outputRoot, sourceName),
      kind,
      purpose,
    ));
  }

  const sourceCommit = git("rev-parse", "HEAD");
  const generatedUtc = git("show", "-s", "--format=%cI", "HEAD");
  const sourceTreeDirty = Boolean(
    git("status", "--porcelain", "--", "Software/AVR128DA28"),
  );
  const releaseInfo = {
    format: "signalslinger-release-info-v1",
    product,
    version,
    board,
    generatedUtc,
    gitCommit: sourceCommit,
    sourceTreeDirty,
    buildProfile: {
      status: "reference-version-match",
      compilerVersion: appEvidence.compilerVersion,
      dfpVersion: appEvidence.dfpVersion,
      mcu: appEvidence.mcu,
      applicationHexSha256: sha256(appHexPath),
      bootloaderHexSha256: sha256(bootHexPath),
    },
    update: {
      fileName: names.update,
      startAddress: `0x${appStart.toString(16).toUpperCase()}`,
      bytesInImage: appSummary.count,
    },
    firstInstall: {
      fileName: names.firstInstall,
      bytesInImage: firstInstallSummary.count,
    },
    serialSlinger: {
      appBaud: 9600,
      appInfoCommand: "INF",
      appUpdateCommand: "UPD",
      bootloaderEntryCommand: "U",
      updateBaud,
      pageBytes,
      protocolVersion,
      bootloaderVersion,
      appStartAddress: `0x${appStart.toString(16).toUpperCase()}`,
      flashBytes,
    },
    workshopSetup: {
      setupHelperFileName: names.bootloader,
      setupLauncherFileName: names.setupLauncher,
      provisioningScriptFileName: "provision-bootloader.ps1",
      serialValidationScriptFileName: "test-bootloader-serial.ps1",
      bootSectionPages,
      fuseBootSize: `0x${bootSectionPages.toString(16).padStart(2, "0").toUpperCase()}`,
      fuseCodeSize: "0x00",
      supportedProgrammers,
    },
    files,
    buildEvidence: {
      application: {
        status: appEvidence.status,
        compilerVersion: appEvidence.compilerVersion,
        dfpVersion: appEvidence.dfpVersion,
        mcu: appEvidence.mcu,
        hardwareTarget: appEvidence.hardwareTarget,
        applicationStart: appEvidence.applicationStart,
        textSectionBytes: appEvidence.textSectionBytes,
        flashImageBytes: appEvidence.flashImageBytes,
        warningCount: appEvidence.warningCount,
        artifacts: appEvidence.artifacts,
      },
      bootloader: {
        status: bootEvidence.status,
        compilerVersion: bootEvidence.compilerVersion,
        dfpVersion: bootEvidence.dfpVersion,
        mcu: bootEvidence.mcu,
        bootloaderBytes: bootEvidence.bootloaderBytes,
        maximumBootloaderBytes: bootEvidence.maximumBootloaderBytes,
        warningCount: bootEvidence.warningCount,
        artifacts: bootEvidence.artifacts,
      },
    },
  };
  writeFileSync(output.releaseInfo, `${JSON.stringify(releaseInfo, null, 2)}\n`);
  const releaseInfoEntry = packageFile(
    output.releaseInfo,
    output.releaseInfo,
    "release-info",
    "Information SerialSlinger reads to choose and verify files.",
  );

  const readme = `SignalSlinger ${friendlyVersion}

This folder is intended for release uploads and for the SerialSlinger app.

Normal update:
- ${names.update}

First-time workshop setup:
- ${names.firstInstall}
- ${names.bootloader}
- ${names.setupLauncher}

Metadata and integrity:
- ${names.releaseInfo}
- ${names.checksums}

Upload ${names.update} by itself for normal SerialSlinger updates.
Upload ${names.zip} for the complete release bundle.

Board: ${board}
Update speed: ${updateBaud} baud
Update verification: page-by-page CRC checks

For normal use, open SerialSlinger and let it choose the correct hardware file.
`;
  writeFileSync(output.readme, readme);
  const readmeEntry = packageFile(
    output.readme,
    output.readme,
    "readme",
    "Plain-language notes for the release folder.",
  );

  const checksumEntries = [...files, releaseInfoEntry, readmeEntry];
  writeFileSync(
    output.checksums,
    `${checksumEntries.map((entry) => `${entry.sha256}  ${entry.fileName}`).join("\n")}\n`,
    "ascii",
  );

  const archiveNames = [
    names.update,
    names.firstInstall,
    names.bootloader,
    names.setupLauncher,
    "provision-bootloader.ps1",
    "test-bootloader-serial.ps1",
    names.releaseInfo,
    names.checksums,
    names.readme,
  ];
  const timestamp = new Date(generatedUtc);
  for (const name of archiveNames) {
    utimesSync(join(outputRoot, name), timestamp, timestamp);
  }
  run("zip", ["-X", "-q", names.zip, ...archiveNames], { cwd: outputRoot });

  if (!options.skipValidate) {
    run(process.execPath, [
      join(workspaceRoot, "scripts", "validate-avr-release-package.mjs"),
      "--package-dir",
      outputRoot,
    ]);
  }

  process.stdout.write(`Release package complete: ${outputRoot}\n`);
  process.stdout.write(`${sha256(output.zip)}  ${names.zip}\n`);
}

try {
  main();
} catch (error) {
  process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
  process.exitCode = 2;
}
