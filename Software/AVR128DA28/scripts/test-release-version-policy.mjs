#!/usr/bin/env node

import assert from "node:assert/strict";
import {
  parseVersion,
  releaseSuccessorPolicy,
} from "./release-support.mjs";

const previous = parseVersion("2.0.7");

assert.equal(releaseSuccessorPolicy(previous, "2.0.8"), "next-patch");
assert.equal(releaseSuccessorPolicy(previous, "2.1.0"), "next-minor");

for (const rejected of ["2.0.7", "2.0.9", "2.1.1", "2.2.0", "3.0.0"]) {
  assert.equal(
    releaseSuccessorPolicy(previous, rejected),
    null,
    `${rejected} must not bypass the guarded successor policy`,
  );
}

console.log("PASS release version policy: exact next patch or next minor only");
