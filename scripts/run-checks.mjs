#!/usr/bin/env node

import { spawn } from 'child_process';
import console from 'console';
import process from 'process';
import { URL, fileURLToPath } from 'url';

const ROOT = fileURLToPath(new URL('..', import.meta.url));
const NPM_COMMAND = 'npm';

const checkScripts = [
  'check:wasm',
  'check:tsc',
  'check:lint:ts',
  'check:lint:rs',
  'check:format:ts',
  'check:format:rs',
  'check:lengths',
  'check:duplication:ts',
  'check:duplication:tsx',
  'check:duplication:rs',
];

function runCheck(scriptName) {
  return new Promise((resolve) => {
    let settled = false;

    const finish = (result) => {
      if (!settled) {
        settled = true;
        resolve(result);
      }
    };

    console.log(`\n=== Running ${scriptName} ===`);

    const child = spawn(NPM_COMMAND, ['run', scriptName], {
      cwd: ROOT,
      stdio: 'inherit',
      shell: true,
    });

    child.on('error', (error) => {
      finish({
        scriptName,
        ok: false,
        reason: `failed to start: ${error.message}`,
      });
    });

    child.on('close', (code, signal) => {
      if (code === 0) {
        finish({ scriptName, ok: true });
        return;
      }

      finish({
        scriptName,
        ok: false,
        reason: signal ? `terminated by ${signal}` : `exit code ${code ?? 'unknown'}`,
      });
    });
  });
}

const failures = [];

for (const scriptName of checkScripts) {
  const result = await runCheck(scriptName);
  if (!result.ok) {
    failures.push(result);
  }
}

if (failures.length > 0) {
  console.error('\nCheck summary:');
  for (const failure of failures) {
    console.error(`- ${failure.scriptName}: ${failure.reason}`);
  }
  process.exit(1);
}

console.log('\nAll checks passed.');
