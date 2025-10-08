#!/usr/bin/env node
import { readFileSync } from 'fs';
import path from 'path';
import process from 'process';
import { solve } from '../src/solver.js';

function readJson(filePath) {
  const absolute = path.resolve(filePath);
  return JSON.parse(readFileSync(absolute, 'utf8'));
}

function classifyArgs(args) {
  if (args.length === 2) {
    return {
      caseFiles: [args[0]],
      inputFiles: [args[1]],
    };
  }
  const caseFiles = [];
  const inputFiles = [];
  for (const arg of args) {
    if (arg.includes(`${path.sep}cases${path.sep}`)) {
      caseFiles.push(arg);
    } else if (arg.includes(`${path.sep}inputs${path.sep}`)) {
      inputFiles.push(arg);
    } else if (arg.toLowerCase().includes('case')) {
      caseFiles.push(arg);
    } else {
      inputFiles.push(arg);
    }
  }
  return { caseFiles, inputFiles };
}

function resolveInputPath(caseEntry, inputFiles) {
  const declared = caseEntry.data.input;
  const byName = new Map();
  for (const file of inputFiles) {
    byName.set(path.basename(file), file);
  }
  if (declared) {
    if (byName.has(declared)) {
      return byName.get(declared);
    }
    const caseDir = path.dirname(caseEntry.path);
    const candidate = path.resolve(caseDir, '..', 'inputs', declared);
    return candidate;
  }
  const caseBase = path.basename(caseEntry.path).replace(/_case.*$/i, '');
  for (const file of inputFiles) {
    const base = path.basename(file).replace(/_inputs.*$/i, '');
    if (base === caseBase) {
      return file;
    }
  }
  return null;
}

function tolerance(expected) {
  return Math.max(5, Math.abs(expected) * 0.02);
}

function formatDiff(actual, expected) {
  return `expected ${expected.toFixed(3)}, got ${actual.toFixed(3)}`;
}

function main() {
  const args = process.argv.slice(2);
  if (args.length < 2) {
    console.error('Usage: node qc/run.mjs <case.json ...> <input.json ...>');
    process.exit(1);
  }
  const { caseFiles, inputFiles } = classifyArgs(args);
  if (!caseFiles.length) {
    console.error('No case files provided.');
    process.exit(1);
  }
  const caseEntries = caseFiles.map((file) => ({ path: file, data: readJson(file) }));
  const results = [];
  const failures = [];

  for (const caseEntry of caseEntries) {
    const inputPath = resolveInputPath(caseEntry, inputFiles);
    if (!inputPath) {
      console.error(`Unable to locate input file for case ${caseEntry.path}`);
      process.exit(1);
    }
    const inputData = readJson(inputPath);
    const solution = solve(inputData);
    const caseFailures = [];
    for (const [caseId, metrics] of Object.entries(caseEntry.data.metrics)) {
      const solved = solution.results.find((r) => r.id === caseId);
      if (!solved) {
        caseFailures.push(`Case ${caseId} missing in solver output.`);
        continue;
      }
      const comparisons = {
        FL_TD: solved.touchdown.x,
        wire_gnd: solved.grounded.wire,
        chain_gnd: solved.grounded.chain,
        anchor: solved.anchorDistance,
        H_anchor_t: solved.H_anchor,
      };
      for (const [metric, expected] of Object.entries(metrics)) {
        const actual = comparisons[metric];
        if (typeof actual !== 'number') {
          caseFailures.push(`Metric ${metric} unavailable for ${caseId}.`);
          continue;
        }
        const diff = Math.abs(actual - expected);
        const limit = tolerance(expected);
        if (diff > limit) {
          caseFailures.push(`${caseId} ${metric} ${formatDiff(actual, expected)} (limit ±${limit.toFixed(3)})`);
        }
      }
    }
    if (caseFailures.length) {
      failures.push({ case: caseEntry.data.caseId || caseEntry.path, issues: caseFailures });
    } else {
      results.push({
        case: caseEntry.data.caseId || caseEntry.path,
        input: inputPath,
      });
    }
  }

  if (failures.length) {
    console.error('QC comparison failed.');
    for (const failure of failures) {
      console.error(`- ${failure.case}`);
      for (const issue of failure.issues) {
        console.error(`  • ${issue}`);
      }
    }
    process.exit(1);
  }

  for (const success of results) {
    console.log(`QC OK: ${success.case} (input ${path.basename(success.input)})`);
  }
  process.exit(0);
}

main();

