#!/usr/bin/env node

import { readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";
import {
  applicationStart,
  artifactEvidence,
  expectedCompilerVersion,
  expectedDfpVersion,
  fail,
  loadBuildEnvironment,
  mcu,
  prepareOutputDirectory,
  reportCompletion,
  runTool,
  workspaceRoot,
  writeEvidence,
} from "./avr-build-support.mjs";

const label = "SignalSlinger AVR bootloader build";
const bootloaderRoot = join(workspaceRoot, "bootloader");

function textStart(mapText) {
  const match = mapText.match(/^\.text\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)/imu);
  if (!match) {
    fail(label, "link map does not contain a .text section summary");
  }
  return Number.parseInt(match[1], 16);
}

function main() {
  if (process.argv.length > 2) {
    fail(label, `unknown argument: ${process.argv[2]}`);
  }
  const environment = loadBuildEnvironment(label, bootloaderRoot);
  const outputRoot = join(workspaceRoot, "tmp", "avr-bootloader-release");
  prepareOutputDirectory(outputRoot);

  const deviceFlags = ["-mmcu=avr128da28", "-B", environment.dfpDevice];
  const mainObject = join(outputRoot, "main.o");
  const protectedIoObject = join(outputRoot, "protected_io.o");
  const elf = join(outputRoot, "SignalSlingerBootloader.elf");
  const hex = join(outputRoot, "SignalSlingerBootloader.hex");
  const binary = join(outputRoot, "SignalSlingerBootloader.bin");
  const map = join(outputRoot, "SignalSlingerBootloader.map");
  const lss = join(outputRoot, "SignalSlingerBootloader.lss");

  runTool(environment, environment.compiler, [
    "-funsigned-char",
    "-funsigned-bitfields",
    "-DNDEBUG",
    "-DF_CPU=24000000UL",
    "-I",
    "include",
    "-I",
    "../SignalSlinger/include",
    "-I",
    "../SignalSlinger/utils",
    "-I",
    "../SignalSlinger/utils/assembler",
    "-I",
    "../SignalSlinger/Config",
    "-I",
    environment.dfpInclude,
    "-O1",
    "-ffunction-sections",
    "-fdata-sections",
    "-fpack-struct",
    "-fshort-enums",
    "-Wall",
    ...deviceFlags,
    "-fno-threadsafe-statics",
    "-MD",
    "-MP",
    "-c",
    "-o",
    mainObject,
    "src/main.cpp",
  ]);
  runTool(environment, environment.compiler, [
    "-x",
    "assembler-with-cpp",
    "-c",
    ...deviceFlags,
    "-I",
    "../SignalSlinger/utils",
    "-I",
    environment.dfpInclude,
    "-o",
    protectedIoObject,
    "../SignalSlinger/src/protected_io.S",
  ]);
  runTool(environment, environment.compiler, [
    "-o",
    elf,
    mainObject,
    protectedIoObject,
    `-Wl,-Map=${map}`,
    "-Wl,--gc-sections",
    ...deviceFlags,
  ]);
  runTool(environment, environment.objcopy, [
    "-O",
    "ihex",
    "-R",
    ".eeprom",
    "-R",
    ".fuse",
    "-R",
    ".lock",
    "-R",
    ".signature",
    "-R",
    ".user_signatures",
    elf,
    hex,
  ]);
  runTool(environment, environment.objcopy, [
    "-O",
    "binary",
    "-R",
    ".eeprom",
    "-R",
    ".fuse",
    "-R",
    ".lock",
    "-R",
    ".signature",
    "-R",
    ".user_signatures",
    elf,
    binary,
  ]);
  const listing = runTool(
    environment,
    environment.objdump,
    ["-h", "-S", elf],
    { quiet: true },
  );
  writeFileSync(lss, listing.stdout);
  const sizeResult = runTool(environment, environment.size, [elf]);

  const bootloaderBytes = readFileSync(binary).length;
  if (textStart(readFileSync(map, "utf8")) !== 0) {
    fail(label, "bootloader .text section does not start at address zero");
  }
  if (bootloaderBytes > applicationStart) {
    fail(
      label,
      `bootloader is ${bootloaderBytes} bytes and overlaps application start `
      + `0x${applicationStart.toString(16)}`,
    );
  }

  const artifacts = artifactEvidence([elf, hex, binary, map, lss]);
  writeEvidence(outputRoot, {
    status: environment.status,
    compilerVersion: environment.compilerVersion,
    expectedCompilerVersion,
    dfpVersion: environment.dfpVersion,
    expectedDfpVersion,
    mcu,
    bootloaderBytes,
    maximumBootloaderBytes: applicationStart,
    applicationStart,
    applicationStartHex: `0x${applicationStart.toString(16)}`,
    sizeOutput: sizeResult.stdout.trim(),
    warningCount: environment.warnings.length,
    warnings: environment.warnings,
    artifacts,
  });
  reportCompletion(label, outputRoot, environment.warnings, artifacts);
}

try {
  main();
} catch (error) {
  process.stderr.write(`${error.message}\n`);
  process.exitCode = 2;
}
