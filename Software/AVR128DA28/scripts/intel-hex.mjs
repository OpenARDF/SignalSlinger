import { readFileSync } from "node:fs";

function fail(filePath, lineNumber, message) {
  throw new Error(`${filePath}:${lineNumber}: ${message}`);
}

export function parseIntelHex(filePath) {
  const memory = new Map();
  let baseAddress = 0;
  let sawEndOfFile = false;
  const lines = readFileSync(filePath, "utf8").split(/\r?\n/u);

  for (const [index, rawLine] of lines.entries()) {
    const line = rawLine.trim();
    if (!line) {
      continue;
    }
    if (!/^:[0-9A-Fa-f]+$/u.test(line) || line.length % 2 !== 1) {
      fail(filePath, index + 1, "invalid Intel HEX record");
    }

    const record = Buffer.from(line.slice(1), "hex");
    const byteCount = record[0];
    if (record.length !== byteCount + 5) {
      fail(filePath, index + 1, "record length does not match byte count");
    }
    if (record.reduce((sum, byte) => (sum + byte) & 0xff, 0) !== 0) {
      fail(filePath, index + 1, "record checksum failed");
    }

    const offset = (record[1] << 8) | record[2];
    const recordType = record[3];
    const payload = record.subarray(4, 4 + byteCount);

    if (recordType === 0x00) {
      for (const [payloadIndex, value] of payload.entries()) {
        const address = baseAddress + offset + payloadIndex;
        const existing = memory.get(address);
        if (existing !== undefined && existing !== value) {
          fail(filePath, index + 1, `conflicting byte at 0x${address.toString(16)}`);
        }
        memory.set(address, value);
      }
    } else if (recordType === 0x01) {
      if (byteCount !== 0) {
        fail(filePath, index + 1, "EOF record contains data");
      }
      sawEndOfFile = true;
      break;
    } else if (recordType === 0x02) {
      if (byteCount !== 2) {
        fail(filePath, index + 1, "invalid extended segment address record");
      }
      baseAddress = payload.readUInt16BE(0) << 4;
    } else if (recordType === 0x04) {
      if (byteCount !== 2) {
        fail(filePath, index + 1, "invalid extended linear address record");
      }
      baseAddress = payload.readUInt16BE(0) << 16;
    } else if (![0x03, 0x05].includes(recordType)) {
      fail(filePath, index + 1, `unsupported record type ${recordType}`);
    }
  }

  if (!sawEndOfFile) {
    throw new Error(`${filePath}: missing Intel HEX EOF record`);
  }
  if (memory.size === 0) {
    throw new Error(`${filePath}: Intel HEX contains no data bytes`);
  }
  return memory;
}

function encodeRecord(address, type, payload) {
  const bytes = [
    payload.length,
    (address >> 8) & 0xff,
    address & 0xff,
    type,
    ...payload,
  ];
  const checksum = (-bytes.reduce((sum, byte) => sum + byte, 0)) & 0xff;
  return `:${Buffer.from([...bytes, checksum]).toString("hex").toUpperCase()}`;
}

export function encodeIntelHex(memory) {
  const addresses = [...memory.keys()].sort((left, right) => left - right);
  if (addresses.length === 0) {
    throw new Error("cannot encode an empty Intel HEX image");
  }

  const lines = [];
  let currentUpper = -1;
  let index = 0;
  while (index < addresses.length) {
    const startAddress = addresses[index];
    const upper = startAddress >>> 16;
    if (upper !== currentUpper) {
      currentUpper = upper;
      lines.push(encodeRecord(0, 0x04, [(upper >> 8) & 0xff, upper & 0xff]));
    }

    const payload = [];
    let address = startAddress;
    while (
      index < addresses.length
      && addresses[index] === address
      && (address >>> 16) === currentUpper
      && payload.length < 16
    ) {
      payload.push(memory.get(address));
      index += 1;
      address += 1;
    }
    lines.push(encodeRecord(startAddress & 0xffff, 0x00, payload));
  }

  lines.push(encodeRecord(0, 0x01, []));
  return `${lines.join("\n")}\n`;
}

export function mergeHexImages(...images) {
  const merged = new Map();
  for (const image of images) {
    for (const [address, value] of image) {
      const existing = merged.get(address);
      if (existing !== undefined && existing !== value) {
        throw new Error(`Intel HEX images conflict at 0x${address.toString(16)}`);
      }
      merged.set(address, value);
    }
  }
  return merged;
}

export function summarizeHex(memory) {
  const addresses = [...memory.keys()].sort((left, right) => left - right);
  if (addresses.length === 0) {
    throw new Error("Intel HEX image contains no data bytes");
  }
  return {
    count: addresses.length,
    first: addresses[0],
    last: addresses.at(-1),
  };
}
