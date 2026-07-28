import { createHash } from "node:crypto";
import {
  existsSync,
  mkdirSync,
  readFileSync,
  rmSync,
  writeFileSync,
} from "node:fs";
import { basename, dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { spawnSync } from "node:child_process";

export const workspaceRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
export const expectedCompilerVersion = "7.3.0";
export const expectedDfpVersion = "1.10.114";
export const mcu = "avr128da28";
export const flashBytes = 131072;
export const applicationStart = 0x2000;

const localToolsRoot = join(workspaceRoot, "tmp", "avr-tools");
const defaultToolchainRoot = join(localToolsRoot, "avr8-gnu-toolchain-darwin_x86_64");
const defaultDfpRoot = join(localToolsRoot, "AVR-Dx_DFP", expectedDfpVersion);

export function fail(label, message) {
  throw new Error(`${label}: ${message}`);
}

function configuredRoot(label, environmentName, defaultRoot) {
  const configured = process.env[environmentName];
  const root = resolve(configured || defaultRoot);
  if (!existsSync(root)) {
    const setupHint = process.platform === "darwin"
      ? "Run `just avr-setup-macos`, or set the environment variable explicitly."
      : `Set ${environmentName} to the required installed directory.`;
    fail(label, `${environmentName} path does not exist: ${root}. ${setupHint}`);
  }
  return root;
}

function resolveTool(label, toolchainRoot, name) {
  const suffix = process.platform === "win32" ? ".exe" : "";
  const candidate = join(toolchainRoot, "bin", `${name}${suffix}`);
  if (!existsSync(candidate)) {
    fail(label, `required tool is missing: ${candidate}`);
  }
  return candidate;
}

function formatArgument(argument) {
  return /[\s"]/u.test(argument) ? JSON.stringify(argument) : argument;
}

export function runTool(context, command, args, options = {}) {
  if (!options.quiet) {
    process.stdout.write(`+ ${basename(command)} ${args.map(formatArgument).join(" ")}\n`);
  }
  const result = spawnSync(command, args, {
    cwd: options.cwd || context.cwd,
    encoding: "utf8",
    maxBuffer: 64 * 1024 * 1024,
  });

  if (result.error) {
    fail(context.label, `${basename(command)} could not run: ${result.error.message}`);
  }
  if (result.stdout && !options.quiet) {
    process.stdout.write(result.stdout);
  }
  if (result.stderr) {
    process.stderr.write(result.stderr);
    context.warnings.push(
      ...result.stderr.split(/\r?\n/u).filter((line) => /warning:/iu.test(line)),
    );
  }
  if (result.status !== 0 && !options.allowFailure) {
    fail(context.label, `${basename(command)} exited with status ${result.status}`);
  }
  return result;
}

export function loadBuildEnvironment(label, cwd) {
  const toolchainRoot = configuredRoot(
    label,
    "AVR_TOOLCHAIN_ROOT",
    defaultToolchainRoot,
  );
  const dfpRoot = configuredRoot(label, "AVR_DFP_ROOT", defaultDfpRoot);
  const dfpInclude = join(dfpRoot, "include");
  const dfpDevice = join(dfpRoot, "gcc", "dev", mcu);
  const pdscCandidates = [
    join(dfpRoot, "Atmel.AVR-Dx_DFP.pdsc"),
    join(dfpRoot, "Microchip.AVR-Dx_DFP.pdsc"),
  ];
  const pdscPath = pdscCandidates.find((candidate) => existsSync(candidate));

  for (const requiredPath of [dfpInclude, dfpDevice]) {
    if (!existsSync(requiredPath)) {
      fail(label, `AVR-Dx_DFP content is missing: ${requiredPath}`);
    }
  }
  if (!pdscPath) {
    fail(
      label,
      `AVR-Dx_DFP manifest is missing; checked ${pdscCandidates.join(" and ")}`,
    );
  }

  const context = {
    label,
    cwd,
    warnings: [],
    toolchainRoot,
    dfpRoot,
    compiler: resolveTool(label, toolchainRoot, "avr-g++"),
    objcopy: resolveTool(label, toolchainRoot, "avr-objcopy"),
    objdump: resolveTool(label, toolchainRoot, "avr-objdump"),
    size: resolveTool(label, toolchainRoot, "avr-size"),
    dfpInclude,
    dfpDevice,
  };

  const compilerVersion = runTool(
    context,
    context.compiler,
    ["-dumpversion"],
    { quiet: true },
  ).stdout.trim();
  const dfpVersion = basename(dfpRoot);
  const pdsc = readFileSync(pdscPath, "utf8");
  const pdscMatches = pdsc.includes(`<release version="${expectedDfpVersion}"`)
    && pdsc.includes("<vendor>Atmel</vendor>");
  const versionProblems = [];
  if (compilerVersion !== expectedCompilerVersion) {
    versionProblems.push(
      `compiler ${compilerVersion || "unknown"} (expected ${expectedCompilerVersion})`,
    );
  }
  if (dfpVersion !== expectedDfpVersion || !pdscMatches) {
    versionProblems.push(
      `device pack ${dfpVersion}${pdscMatches ? "" : " with mismatched PDSC"} `
      + `(expected ${expectedDfpVersion})`,
    );
  }

  const allowVersionMismatch = process.env.AVR_ALLOW_VERSION_MISMATCH === "1";
  if (versionProblems.length && !allowVersionMismatch) {
    fail(
      label,
      `${versionProblems.join("; ")}. Set AVR_ALLOW_VERSION_MISMATCH=1 only `
      + "for a non-baseline exploratory build.",
    );
  }

  return {
    ...context,
    compilerVersion,
    dfpVersion,
    status: versionProblems.length
      ? "exploratory-version-mismatch"
      : "reference-version-match",
  };
}

export function prepareOutputDirectory(outputRoot) {
  rmSync(outputRoot, { recursive: true, force: true });
  mkdirSync(outputRoot, { recursive: true });
}

export function sha256(path) {
  return createHash("sha256").update(readFileSync(path)).digest("hex");
}

export function artifactEvidence(paths) {
  return paths
    .filter((path) => existsSync(path))
    .map((path) => ({
      file: basename(path),
      bytes: readFileSync(path).length,
      sha256: sha256(path),
    }));
}

export function writeEvidence(outputRoot, evidence) {
  writeFileSync(
    join(outputRoot, "build-evidence.json"),
    `${JSON.stringify(evidence, null, 2)}\n`,
  );
}

export function reportCompletion(label, outputRoot, warnings, artifacts) {
  process.stdout.write(`${label} complete: ${outputRoot}\n`);
  process.stdout.write(`Warnings: ${warnings.length}\n`);
  for (const artifact of artifacts) {
    process.stdout.write(`${artifact.sha256}  ${artifact.file}\n`);
  }
}

export function reportDoctor(environment) {
  process.stdout.write("SignalSlinger AVR build environment is ready.\n");
  process.stdout.write(`Compiler: ${environment.compilerVersion} (${environment.compiler})\n`);
  process.stdout.write(`Device pack: ${environment.dfpVersion} (${environment.dfpRoot})\n`);
  process.stdout.write(`Target MCU: ${mcu}\n`);
  process.stdout.write(`Status: ${environment.status}\n`);
}
