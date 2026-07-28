#!/usr/bin/env node

import { existsSync, readFileSync } from "node:fs";
import { dirname, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { spawnSync } from "node:child_process";

const firmwareRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(firmwareRoot, "../..");
const result = spawnSync(
  "rg",
  [
    "--files",
    "-g",
    "*.md",
    "-g",
    "!.git/**",
    "-g",
    "!.venv/**",
    "-g",
    "!**/*-backups/**",
    "-g",
    "!**/tmp/**",
    "-g",
    "!**/release-packages/**",
  ],
  { cwd: repositoryRoot, encoding: "utf8" },
);

if (result.status !== 0) {
  process.stderr.write(result.stderr || "Unable to enumerate Markdown files.\n");
  process.exit(2);
}

const markdownFiles = result.stdout.trim().split(/\r?\n/u).filter(Boolean);
const linkPattern = /!?\[[^\]]*\]\(([^)]+)\)/gu;
const failures = [];
let checkedLinks = 0;

for (const relativePath of markdownFiles) {
  const markdownFile = resolve(repositoryRoot, relativePath);
  const content = readFileSync(markdownFile, "utf8");
  for (const match of content.matchAll(linkPattern)) {
    let target = match[1].trim();
    if (target.startsWith("<") && target.endsWith(">")) {
      target = target.slice(1, -1);
    }
    if (/^(?:[a-z]+:|#)/iu.test(target)) {
      continue;
    }

    const pathWithoutAnchor = target.split("#", 1)[0];
    if (!pathWithoutAnchor) {
      continue;
    }
    checkedLinks += 1;

    let decodedPath;
    try {
      decodedPath = decodeURIComponent(pathWithoutAnchor);
    } catch {
      failures.push(`${relativePath}: invalid encoded link ${target}`);
      continue;
    }
    if (!existsSync(resolve(dirname(markdownFile), decodedPath))) {
      failures.push(`${relativePath}: missing ${target}`);
    }
  }
}

if (failures.length > 0) {
  process.stderr.write(`${failures.join("\n")}\n`);
  process.exit(1);
}

process.stdout.write(
  `Checked ${checkedLinks} local links in ${markdownFiles.length} Markdown files `
    + `under ${relative(process.cwd(), repositoryRoot) || "."}.\n`,
);
