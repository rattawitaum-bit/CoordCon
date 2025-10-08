#!/usr/bin/env node
import { readFileSync } from 'fs';
import path from 'path';
import process from 'process';
import { solve } from '../src/solver.js';

const TOL_M = parseFloat(process.env.QC_TOL_M ?? '5');
const TOL_P = parseFloat(process.env.QC_TOL_P ?? '2.0') / 100;
const TOL_H = parseFloat(process.env.QC_TOL_H ?? '0.10');
const METRIC_NAMES = ['FL_TD', 'wire_gnd', 'chain_gnd', 'anchor', 'H_anchor_t'];

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

function absoluteTolerance(metric) {
  return metric === 'H_anchor_t' ? TOL_H : TOL_M;
}

function formatDiff(actual, expected) {
  return `expected ${expected.toFixed(3)}, got ${actual.toFixed(3)}`;
}

function relativeProportion(actual, expected) {
  if (!Number.isFinite(expected) || Math.abs(expected) < 1e-9) {
    return Math.abs(actual) < 1e-6 ? 0 : Infinity;
  }
  return Math.abs(actual - expected) / Math.abs(expected);
}

function evaluateMetric(metric, expected, actual) {
  const absTol = absoluteTolerance(metric);
  const diff = actual - expected;
  const absDiff = Math.abs(diff);
  const rel = relativeProportion(actual, expected);
  const pass = absDiff <= absTol || rel <= TOL_P;
  return { pass, absDiff, absTol, rel };
}

function toleranceSummary(metric) {
  const absTol = absoluteTolerance(metric);
  const pctTol = (TOL_P * 100).toFixed(2);
  return `±${absTol.toFixed(3)} or ±${pctTol}%`;
}

function toNumber(value) {
  if (typeof value === 'number') {
    return value;
  }
  if (typeof value === 'string') {
    const parsed = Number(value);
    return Number.isNaN(parsed) ? undefined : parsed;
  }
  if (value && typeof value === 'object') {
    if ('pdf' in value) {
      return toNumber(value.pdf);
    }
    if ('value' in value) {
      return toNumber(value.value);
    }
  }
  return undefined;
}

function extractExpected(row, metric) {
  if (!row || typeof row !== 'object') {
    return undefined;
  }
  const direct = row[metric];
  if (direct !== undefined) {
    const val = toNumber(direct);
    if (val !== undefined) {
      return val;
    }
  }
  const suffixed = row[`${metric}_pdf`];
  if (suffixed !== undefined) {
    const val = toNumber(suffixed);
    if (val !== undefined) {
      return val;
    }
  }
  if (row.metrics && typeof row.metrics === 'object') {
    const nested = row.metrics[metric];
    if (nested !== undefined) {
      const val = toNumber(nested);
      if (val !== undefined) {
        return val;
      }
    }
    const nestedSuffixed = row.metrics[`${metric}_pdf`];
    if (nestedSuffixed !== undefined) {
      const val = toNumber(nestedSuffixed);
      if (val !== undefined) {
        return val;
      }
    }
  }
  return undefined;
}

function approxEqual(a, b, eps = 1e-6) {
  return Math.abs(a - b) <= eps;
}

function findResultForRow(row, solutionResults) {
  if (!row) {
    return undefined;
  }
  if (row.id) {
    const match = solutionResults.find((res) => res.id === row.id);
    if (match) {
      return match;
    }
  }
  if (row.caseId) {
    const match = solutionResults.find((res) => res.id === row.caseId);
    if (match) {
      return match;
    }
  }
  if (row.mode && row.tension_t !== undefined) {
    const target = toNumber(row.tension_t);
    if (target !== undefined) {
      const match = solutionResults.find((res) => res.mode === row.mode && approxEqual(res.input, target, 1e-3));
      if (match) {
        return match;
      }
    }
  }
  if (row.mode && row.tension !== undefined) {
    const target = toNumber(row.tension);
    if (target !== undefined) {
      const match = solutionResults.find((res) => res.mode === row.mode && approxEqual(res.input, target, 1e-3));
      if (match) {
        return match;
      }
    }
  }
  if (row.label) {
    const match = solutionResults.find((res) => res.label === row.label);
    if (match) {
      return match;
    }
  }
  return undefined;
}

function rowIdentifier(row, fallback) {
  if (!row || typeof row !== 'object') {
    return fallback;
  }
  return row.label || row.id || row.caseId || (row.mode && (row.tension_t ?? row.tension) !== undefined
    ? `${row.mode}@${row.tension_t ?? row.tension}`
    : fallback);
}

function extractRowsFromCase(caseData) {
  if (Array.isArray(caseData)) {
    return caseData;
  }
  if (caseData && typeof caseData === 'object') {
    if (Array.isArray(caseData.rows)) {
      return caseData.rows;
    }
    if (Array.isArray(caseData.data)) {
      return caseData.data;
    }
  }
  return null;
}

function buildComparisons(result) {
  if (!result) {
    return {};
  }
  return {
    FL_TD: result.touchdown?.x,
    wire_gnd: result.grounded?.wire,
    chain_gnd: result.grounded?.chain,
    anchor: result.anchorDistance,
    H_anchor_t: result.H_anchor,
  };
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
    const rows = extractRowsFromCase(caseEntry.data);
    if (rows && rows.length) {
      rows.forEach((row, index) => {
        const identifier = rowIdentifier(row, `row ${index + 1}`);
        const solved = findResultForRow(row, solution.results || []);
        if (!solved) {
          caseFailures.push(`${identifier} missing in solver output.`);
          return;
        }
        const comparisons = buildComparisons(solved);
        for (const metric of METRIC_NAMES) {
          const expected = extractExpected(row, metric);
          if (expected === undefined) {
            continue;
          }
          const actual = comparisons[metric];
          if (typeof actual !== 'number') {
            caseFailures.push(`${identifier} ${metric} unavailable in solver output.`);
            continue;
          }
          const evaluation = evaluateMetric(metric, expected, actual);
          if (!evaluation.pass) {
            const relPct = Number.isFinite(evaluation.rel) ? `${(evaluation.rel * 100).toFixed(2)}%` : '∞';
            const tolText = toleranceSummary(metric);
            const details = [`|Δ|=${evaluation.absDiff.toFixed(3)}`, `tol ${tolText}`, `rel=${relPct}`];
            caseFailures.push(`${identifier} ${metric} ${formatDiff(actual, expected)} (${details.join('; ')})`);
          }
        }
      });
    } else if (caseEntry.data.metrics) {
      for (const [caseId, metrics] of Object.entries(caseEntry.data.metrics)) {
        const solved = (solution.results || []).find((r) => r.id === caseId);
        if (!solved) {
          caseFailures.push(`Case ${caseId} missing in solver output.`);
          continue;
        }
        const comparisons = buildComparisons(solved);
        for (const [metric, expectedRaw] of Object.entries(metrics)) {
          const expected = toNumber(expectedRaw);
          if (expected === undefined) {
            caseFailures.push(`${caseId} ${metric} has invalid expected value.`);
            continue;
          }
          const actual = comparisons[metric];
          if (typeof actual !== 'number') {
            caseFailures.push(`Metric ${metric} unavailable for ${caseId}.`);
            continue;
          }
          const evaluation = evaluateMetric(metric, expected, actual);
          if (!evaluation.pass) {
            const relPct = Number.isFinite(evaluation.rel) ? `${(evaluation.rel * 100).toFixed(2)}%` : '∞';
            const tolText = toleranceSummary(metric);
            const details = [`|Δ|=${evaluation.absDiff.toFixed(3)}`, `tol ${tolText}`, `rel=${relPct}`];
            caseFailures.push(`${caseId} ${metric} ${formatDiff(actual, expected)} (${details.join('; ')})`);
          }
        }
      }
    } else {
      caseFailures.push('Case file missing metrics definition.');
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

