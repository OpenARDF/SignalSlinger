#!/usr/bin/env node

import {
  mkdirSync,
  readFileSync,
  readdirSync,
  writeFileSync,
} from "node:fs";
import { dirname, join } from "node:path";
import {
  applicationStart as bootApplicationStart,
  artifactEvidence,
  expectedCompilerVersion,
  expectedDfpVersion,
  fail,
  flashBytes,
  loadBuildEnvironment,
  mcu,
  prepareOutputDirectory,
  reportCompletion,
  reportDoctor,
  runTool,
  workspaceRoot,
  writeEvidence,
} from "./avr-build-support.mjs";

const label = "SignalSlinger AVR Release build";
const projectRoot = join(workspaceRoot, "SignalSlinger");
const cppSources = [
  "atmel_start.cpp",
  "driver_isr.cpp",
  "main.cpp",
  "src/adc.cpp",
  "src/binio.cpp",
  "src/bod.cpp",
  "src/CircularStringBuff.cpp",
  "src/clkctrl.cpp",
  "src/cpuint.cpp",
  "src/driver_init.cpp",
  "src/eeprommanager.cpp",
  "src/i2c.cpp",
  "src/leds.cpp",
  "src/morse.cpp",
  "src/rtc.cpp",
  "src/serialbus.cpp",
  "src/si5351.cpp",
  "src/slpctrl.cpp",
  "src/tcb.cpp",
  "src/timeutil.cpp",
  "src/transmitter.cpp",
  "src/usart_basic.cpp",
  "src/util.cpp",
];
const assemblySources = ["src/protected_io.S"];

function verifySourceList() {
  const discovered = [
    ...readdirSync(projectRoot)
      .filter((name) => name.endsWith(".cpp"))
      .map((name) => name),
    ...readdirSync(join(projectRoot, "src"))
      .filter((name) => name.endsWith(".cpp"))
      .map((name) => `src/${name}`),
  ].sort();
  const expected = [...cppSources].sort();
  if (JSON.stringify(discovered) !== JSON.stringify(expected)) {
    fail(
      label,
      "portable source list differs from the project tree; update and review "
      + "cppSources before building",
    );
  }
  for (const source of assemblySources) {
    try {
      readFileSync(join(projectRoot, source));
    } catch {
      fail(label, `assembly source is missing: ${source}`);
    }
  }
}

function parseArguments(argv) {
  const options = {
    doctor: false,
    hardware: null,
    applicationStart: 0,
  };
  for (let index = 0; index < argv.length; index += 1) {
    const argument = argv[index];
    if (argument === "--doctor") {
      options.doctor = true;
    } else if (argument === "--hardware") {
      options.hardware = argv[index + 1];
      index += 1;
    } else if (argument === "--application-start") {
      options.applicationStart = Number(argv[index + 1]);
      index += 1;
    } else {
      fail(label, `unknown argument: ${argument}`);
    }
  }

  if (options.hardware !== null && !["3.4", "3.5"].includes(options.hardware)) {
    fail(label, "--hardware must be 3.4 or 3.5");
  }
  if (
    !Number.isInteger(options.applicationStart)
    || ![0, bootApplicationStart].includes(options.applicationStart)
  ) {
    fail(
      label,
      `--application-start must be 0 or 0x${bootApplicationStart.toString(16)}`,
    );
  }
  return options;
}

function textSection(mapText) {
  const match = mapText.match(/^\.text\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)/imu);
  if (!match) {
    fail(label, "link map does not contain a .text section summary");
  }
  return {
    start: Number.parseInt(match[1], 16),
    bytes: Number.parseInt(match[2], 16),
  };
}

function main() {
  const options = parseArguments(process.argv.slice(2));
  const environment = loadBuildEnvironment(label, projectRoot);
  if (options.doctor) {
    reportDoctor(environment);
    return;
  }
  verifySourceList();

  const relocationSuffix = options.applicationStart ? "-relocated" : "";
  const hardwareSuffix = options.hardware
    ? `-hw-${options.hardware.replace(".", "-")}`
    : "-active-hw";
  const outputRoot = join(
    workspaceRoot,
    "tmp",
    `avr-release${relocationSuffix}${hardwareSuffix}`,
  );
  prepareOutputDirectory(outputRoot);

  const includeDirectories = [
    "Config",
    "include",
    "utils",
    "utils/assembler",
    ".",
    environment.dfpInclude,
  ];
  const deviceFlags = ["-mmcu=avr128da28", "-B", environment.dfpDevice];
  const hardwareFlags = options.hardware
    ? [`-DHW_TARGET_${options.hardware.replace(".", "_")}`]
    : [];
  const objectFiles = [];

  for (const source of cppSources) {
    const object = join(outputRoot, source.replace(/\.cpp$/u, ".o"));
    mkdirSync(dirname(object), { recursive: true });
    objectFiles.push(object);
    runTool(environment, environment.compiler, [
      "-funsigned-char",
      "-funsigned-bitfields",
      "-DNDEBUG",
      ...hardwareFlags,
      ...includeDirectories.flatMap((directory) => ["-I", directory]),
      "-O1",
      "-ffunction-sections",
      "-fdata-sections",
      "-fpack-struct",
      "-fshort-enums",
      "-Wall",
      ...deviceFlags,
      "-c",
      "-fno-threadsafe-statics",
      "-MD",
      "-MP",
      "-o",
      object,
      source,
    ]);
  }

  for (const source of assemblySources) {
    const object = join(outputRoot, source.replace(/\.S$/u, ".o"));
    mkdirSync(dirname(object), { recursive: true });
    objectFiles.push(object);
    runTool(environment, environment.compiler, [
      "-Wa,-gdwarf2",
      "-x",
      "assembler-with-cpp",
      "-c",
      ...deviceFlags,
      "-I",
      "utils",
      "-I",
      environment.dfpInclude,
      "-MD",
      "-MP",
      "-o",
      object,
      source,
    ]);
  }

  const elf = join(outputRoot, "SignalSlinger.elf");
  const map = join(outputRoot, "SignalSlinger.map");
  const hex = join(outputRoot, "SignalSlinger.hex");
  const eep = join(outputRoot, "SignalSlinger.eep");
  const lss = join(outputRoot, "SignalSlinger.lss");
  const srec = join(outputRoot, "SignalSlinger.srec");
  const binary = join(outputRoot, "SignalSlinger.bin");

  runTool(environment, environment.compiler, [
    "-o",
    elf,
    ...objectFiles,
    `-Wl,-Map=${map}`,
    "-Wl,--start-group",
    "-Wl,-lm",
    "-Wl,--end-group",
    "-Wl,--gc-sections",
    ...(options.applicationStart
      ? [`-Wl,--section-start=.text=0x${options.applicationStart.toString(16)}`]
      : []),
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
    "-j",
    ".eeprom",
    "--set-section-flags=.eeprom=alloc,load",
    "--change-section-lma",
    ".eeprom=0",
    "--no-change-warnings",
    "-O",
    "ihex",
    elf,
    eep,
  ], { allowFailure: true });
  const listing = runTool(
    environment,
    environment.objdump,
    ["-h", "-S", elf],
    { quiet: true },
  );
  writeFileSync(lss, listing.stdout);
  runTool(environment, environment.objcopy, [
    "-O",
    "srec",
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
    srec,
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
  const sizeResult = runTool(environment, environment.size, [elf]);

  const section = textSection(readFileSync(map, "utf8"));
  if (section.start !== options.applicationStart) {
    fail(
      label,
      `.text starts at 0x${section.start.toString(16)}, expected `
      + `0x${options.applicationStart.toString(16)}`,
    );
  }
  if (section.start + section.bytes > flashBytes) {
    fail(
      label,
      `.text ends beyond flash: 0x${(section.start + section.bytes).toString(16)}`,
    );
  }
  const flashImageBytes = readFileSync(binary).length;
  if (section.start + flashImageBytes > flashBytes) {
    fail(
      label,
      `flash image ends beyond flash: 0x${(section.start + flashImageBytes).toString(16)}`,
    );
  }

  const artifacts = artifactEvidence([elf, hex, eep, map, lss, srec, binary]);
  writeEvidence(outputRoot, {
    status: environment.status,
    compilerVersion: environment.compilerVersion,
    expectedCompilerVersion,
    dfpVersion: environment.dfpVersion,
    expectedDfpVersion,
    mcu,
    hardwareTarget: options.hardware || "selected-by-defs.h",
    applicationStart: options.applicationStart,
    applicationStartHex: `0x${options.applicationStart.toString(16)}`,
    textSectionBytes: section.bytes,
    flashImageBytes,
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
