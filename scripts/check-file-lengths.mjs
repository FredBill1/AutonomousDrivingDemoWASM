#!/usr/bin/env node
// Checks that no source file exceeds 400 lines and the average is under 200 lines.

import console from 'console';
import { readdirSync, readFileSync } from 'fs';
import { extname, join } from 'path';
import process from 'process';
import { fileURLToPath, URL } from 'url';

const MAX_FILE_LINES = 400;
const MAX_AVERAGE_LINES = 200;

const ROOT = fileURLToPath(new URL('..', import.meta.url));
console.log(`Checking file lengths in ${ROOT}`);

function collectFiles(dir, extensions) {
  const results = [];
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const fullPath = join(dir, entry.name);
    if (entry.isDirectory()) {
      results.push(...collectFiles(fullPath, extensions));
    } else if (extensions.includes(extname(entry.name))) {
      results.push(fullPath);
    }
  }
  return results;
}

function countLines(filePath) {
  return readFileSync(filePath, 'utf8').split('\n').length;
}

const rsFiles = collectFiles(join(ROOT, 'wasm-core', 'src'), ['.rs']);
const tsFiles = collectFiles(join(ROOT, 'src'), ['.ts', '.tsx']);
const allFiles = [...rsFiles, ...tsFiles];

if (allFiles.length === 0) {
  console.error('No source files found.');
  process.exit(1);
}

const fileLengths = allFiles.map((f) => ({ path: f.replace(ROOT, ''), lines: countLines(f) }));
const overLimit = fileLengths.filter((f) => f.lines > MAX_FILE_LINES);
const totalLines = fileLengths.reduce((sum, f) => sum + f.lines, 0);
const averageLines = totalLines / fileLengths.length;

console.log(`Checked ${allFiles.length} files. Average: ${averageLines.toFixed(1)} lines.`);

if (overLimit.length > 0) {
  console.error(`\nFiles exceeding ${MAX_FILE_LINES}-line limit:`);
  for (const f of overLimit) {
    console.error(`  ${f.path}: ${f.lines} lines`);
  }
}

let failed = false;

if (overLimit.length > 0) {
  console.error(`\nFAIL: ${overLimit.length} file(s) exceed the ${MAX_FILE_LINES}-line limit.`);
  failed = true;
}

if (averageLines > MAX_AVERAGE_LINES) {
  console.error(`FAIL: Average file length ${averageLines.toFixed(1)} exceeds the ${MAX_AVERAGE_LINES}-line limit.`);
  failed = true;
}

if (!failed) {
  console.log('PASS: All file length checks passed.');
} else {
  process.exit(1);
}
