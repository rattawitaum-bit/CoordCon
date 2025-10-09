const EPS = 1e-9;

function buildContext(config) {
  const wire = config.wire || {};
  const chain = config.chain || {};
  const wireLength = Number.isFinite(wire.length) ? Number(wire.length) : 0;
  const chainLength = Number.isFinite(chain.length) ? Number(chain.length) : 0;
  if (wireLength <= 0 && chainLength <= 0) {
    throw new Error('No suspended line; anchor dragging risk.');
  }

  const segments = [];
  let cumulative = 0;
  if (wireLength > 0) {
    segments.push({
      id: 'wire',
      length: wireLength,
      weight: wire.weight,
      friction: wire.friction || 0,
      start: cumulative,
      end: cumulative + wireLength,
    });
    cumulative += wireLength;
  }
  if (chainLength > 0) {
    segments.push({
      id: 'chain',
      length: chainLength,
      weight: chain.weight,
      friction: chain.friction || 0,
      start: cumulative,
      end: cumulative + chainLength,
    });
    cumulative += chainLength;
  }

  const totalLength = cumulative;
  const buoys = (config.buoys || []).map((b, index) => ({
    arcLength: b.arcLength,
    force: b.force,
    pennantLength: b.pennantLength || 0,
    name: b.name || `buoy-${index + 1}`,
    surfaceFirst: b.surfaceFirst !== false,
  })).sort((a, b) => a.arcLength - b.arcLength);

  return {
    waterDepth: config.waterDepth,
    fairleadDepth: config.fairleadDepth,
    targetDepth: config.waterDepth - config.fairleadDepth, // Touchdown at y = WD - df; no sign flip.
    segments,
    totalLength,
    wire: { length: wireLength, weight: wire.weight, friction: wire.friction || 0 },
    chain: { length: chainLength, weight: chain.weight, friction: chain.friction || 0 },
    buoys,
  };
}

function deriv(u, w, H) {
  const coshU = Math.cosh(u);
  const invCosh = 1 / coshU;
  return {
    du: w / (H * coshU),
    dy: Math.tanh(u),
    dx: invCosh,
  };
}

function rk4Step(state, ds, w, H) {
  const k1 = deriv(state.u, w, H);
  const k2 = deriv(state.u + 0.5 * ds * k1.du, w, H);
  const k3 = deriv(state.u + 0.5 * ds * k2.du, w, H);
  const k4 = deriv(state.u + ds * k3.du, w, H);

  const du = (k1.du + 2 * k2.du + 2 * k3.du + k4.du) * ds / 6;
  const dy = (k1.dy + 2 * k2.dy + 2 * k3.dy + k4.dy) * ds / 6;
  const dx = (k1.dx + 2 * k2.dx + 2 * k3.dx + k4.dx) * ds / 6;

  return {
    s: state.s + ds,
    u: state.u + du,
    y: state.y + dy,
    x: state.x + dx,
  };
}

function integrateLine({ context, H, u0, stopAtTarget }) {
  const { segments, totalLength, targetDepth, buoys } = context;
  const points = [];
  const buoyStates = [];
  const dsBase = Math.max(0.1, totalLength / 2000);
  let state = { s: 0, u: u0, y: 0, x: 0 };
  points.push({ ...state, segment: segments[0] ? segments[0].id : null });

  let currentSegment = 0;
  let currentSegmentEnd = segments.length ? segments[0].end : totalLength;
  let buoyIndex = 0;
  let nextBuoyArc = buoyIndex < buoys.length ? buoys[buoyIndex].arcLength : Infinity;
  let touchdown = null;

  while (state.s < totalLength - EPS) {
    let dsLimit = Math.min(dsBase, totalLength - state.s);
    if (currentSegmentEnd - state.s < dsLimit) {
      dsLimit = currentSegmentEnd - state.s;
    }
    if (nextBuoyArc - state.s < dsLimit) {
      dsLimit = nextBuoyArc - state.s;
    }
    if (dsLimit < EPS) {
      if (Math.abs(currentSegmentEnd - state.s) < EPS && currentSegment < segments.length - 1) {
        currentSegment += 1;
        currentSegmentEnd = segments[currentSegment].end;
        points.push({ ...state, segment: segments[currentSegment].id });
        continue;
      }
      if (Math.abs(nextBuoyArc - state.s) < EPS && buoyIndex < buoys.length) {
        const buoy = buoys[buoyIndex];
        const attachDepth = state.y;
        const attachX = state.x;
        const verticalBelow = H * Math.sinh(state.u);
        const taut = attachDepth >= buoy.pennantLength - 1e-9;
        const applied = taut ? Math.min(buoy.force, verticalBelow) : 0;
        const verticalAbove = Math.max(0, verticalBelow - applied);
        state = { ...state, u: verticalAbove === 0 ? 0 : Math.asinh(verticalAbove / H) };
        const buoyDepth = Math.max(0, attachDepth - buoy.pennantLength);
        buoyStates.push({
          name: buoy.name,
          arcLength: buoy.arcLength,
          attach: { x: attachX, y: attachDepth },
          buoyDepth,
          taut,
          appliedForce: applied,
        });
        buoyIndex += 1;
        nextBuoyArc = buoyIndex < buoys.length ? buoys[buoyIndex].arcLength : Infinity;
        points.push({ ...state, segment: segments[currentSegment] ? segments[currentSegment].id : null });
        continue;
      }
      if (Math.abs(totalLength - state.s) < EPS) {
        break;
      }
      dsLimit = dsBase;
    }

    const segment = segments[currentSegment] || { weight: 0, id: null };
    const nextState = rk4Step(state, dsLimit, segment.weight || 0, H);

    if (!touchdown && nextState.y >= targetDepth) {
      const ratio = (targetDepth - state.y) / (nextState.y - state.y);
      const sTouch = state.s + dsLimit * ratio;
      const xTouch = state.x + (nextState.x - state.x) * ratio;
      const uTouch = state.u + (nextState.u - state.u) * ratio;
      touchdown = {
        s: sTouch,
        x: xTouch,
        y: targetDepth,
        u: uTouch,
        segment: segment.id,
      };
      const touchState = { s: sTouch, x: xTouch, y: targetDepth, u: uTouch };
      points.push({ ...touchState, segment: segment.id });
      if (stopAtTarget) {
        state = touchState;
        break;
      }
    }

    state = nextState;
    points.push({ ...state, segment: segment.id });
  }

  return {
    points,
    touchdown,
    end: state,
    buoyStates,
  };
}

function computeSuspended(context, arcLength) {
  const suspended = { wire: 0, chain: 0 };
  let remaining = Math.max(0, arcLength);
  if (context.wire.length > 0) {
    const take = Math.min(context.wire.length, remaining);
    suspended.wire = take;
    remaining -= take;
  }
  if (context.chain.length > 0 && remaining > 0) {
    const take = Math.min(context.chain.length, remaining);
    suspended.chain = take;
    remaining -= take;
  }
  return suspended;
}

function computeGrounded(context, touchdownArc) {
  const grounded = { wire: 0, chain: 0 };
  const suspended = computeSuspended(context, touchdownArc);
  if (context.wire.length > 0) {
    grounded.wire = Math.max(0, context.wire.length - suspended.wire);
  }
  if (context.chain.length > 0) {
    grounded.chain = Math.max(0, context.chain.length - suspended.chain);
  }
  return grounded;
}

function frictionDrop(context, grounded) {
  const { segments } = context;
  let drop = 0;
  for (const segment of segments) {
    const length = grounded[segment.id] || 0;
    if (length > 0) {
      drop += (segment.friction || 0) * (segment.weight || 0) * length;
    }
  }
  return drop;
}

function solveTMode(context, caseDef) {
  const Tfair = caseDef.value;
  if (!Number.isFinite(Tfair) || Tfair <= 0) {
    throw new Error('Total fairlead tension must be positive.');
  }

  const evalAt = (u0) => {
    if (!Number.isFinite(u0) || u0 <= 0) {
      return { u0, invalid: true };
    }
    const cosh = Math.cosh(u0);
    if (!Number.isFinite(cosh) || cosh <= 1) {
      return { u0, invalid: true };
    }
    const H = Tfair / cosh;
    const solution = integrateLine({ context, H, u0, stopAtTarget: false });
    const touchdown = solution.touchdown || null;
    const depthErr = solution.end.y - context.targetDepth;
    return { u0, H, touchdown, depthErr, solution };
  };

  let uLow = 1e-6;
  let lowEval = evalAt(uLow);
  let uHigh = 0.2;
  let highEval = evalAt(uHigh);

  let guard = 0;
  while ((!highEval.touchdown || highEval.depthErr < 0) && guard < 80) {
    uHigh *= 2;
    highEval = evalAt(uHigh);
    guard += 1;
    if (uHigh > 50) break;
  }
  if (!highEval.touchdown || highEval.depthErr < 0) {
    throw new Error('Unable to reach seabed under supplied total tension.');
  }

  guard = 0;
  while (lowEval.touchdown && lowEval.depthErr > 0 && guard < 80) {
    uLow *= 0.5;
    lowEval = evalAt(uLow);
    guard += 1;
    if (uLow < 1e-8) break;
  }

  let iterations = 0;
  let best = highEval;
  while (iterations < 100 && Math.abs(uHigh - uLow) > 1e-6) {
    const midU = 0.5 * (uHigh + uLow);
    const midEval = evalAt(midU);
    if (!midEval.touchdown) {
      uLow = midU;
      iterations += 1;
      continue;
    }
    best = midEval;
    if (midEval.depthErr > 0) {
      uHigh = midU;
      highEval = midEval;
    } else {
      uLow = midU;
      lowEval = midEval;
    }
    iterations += 1;
  }

  const final = best.touchdown ? best : highEval;
  return finalizeCase(context, {
    mode: 'T',
    input: Tfair,
    H: final.H,
    u0: final.u0,
    totalTension: Tfair,
  });
}

function solveHMode(context, caseDef) {
  const H = caseDef.value;
  if (H <= 0) {
    throw new Error('Horizontal tension must be positive.');
  }
  const evalAt = (u0) => {
    return { u0, solution: integrateLine({ context, H, u0, stopAtTarget: false }) };
  };
  const lowEval = evalAt(0);
  let gLow = lowEval.solution.end.y - context.targetDepth;
  if (gLow >= 0) {
    return finalizeCase(context, { mode: 'H', input: H, H, u0: 0 });
  }
  let uHigh = 0.5;
  let highEval = evalAt(uHigh);
  let gHigh = highEval.solution.end.y - context.targetDepth;
  let attempts = 0;
  while (gHigh < 0 && attempts < 60) {
    uHigh *= 2;
    highEval = evalAt(uHigh);
    gHigh = highEval.solution.end.y - context.targetDepth;
    attempts += 1;
    if (uHigh > 50) {
      break;
    }
  }
  if (gHigh < 0) {
    throw new Error('Horizontal tension too low to reach seabed.');
  }
  let uLow = 0;
  let iterations = 0;
  let midU = uHigh;
  while (iterations < 80 && Math.abs(uHigh - uLow) > 1e-6) {
    midU = 0.5 * (uHigh + uLow);
    const midEval = evalAt(midU);
    const gMid = midEval.solution.end.y - context.targetDepth;
    if (gMid > 0) {
      uHigh = midU;
      highEval = midEval;
      gHigh = gMid;
    } else {
      uLow = midU;
      gLow = gMid;
    }
    iterations += 1;
  }
  return finalizeCase(context, { mode: 'H', input: H, H, u0: uHigh });
}

function finalizeCase(context, params) {
  const { H, u0 } = params;
  const integration = integrateLine({ context, H, u0, stopAtTarget: true });
  const touchdown = integration.touchdown || {
    s: context.totalLength,
    x: integration.end.x,
    y: integration.end.y,
    u: integration.end.u,
  };
  const suspended = computeSuspended(context, touchdown.s);
  const grounded = computeGrounded(context, touchdown.s);
  const totalGrounded = grounded.wire + grounded.chain;
  const frictionLoss = frictionDrop(context, grounded);
  const H_anchor = Math.max(0, H - frictionLoss);
  const anchorDistance = touchdown.x + totalGrounded;
  const V0 = H * Math.sinh(u0);
  const totalTension = params.totalTension ?? H * Math.cosh(u0);
  const warnings = [];
  if (totalGrounded <= EPS) {
    warnings.push('No grounded length: anchor-drag risk');
  }
  if (totalGrounded > 0 && H_anchor < 1e-3) {
    warnings.push('dragging risk');
  }
  return {
    mode: params.mode,
    input: params.input,
    H,
    u0,
    totalTension,
    verticalFairlead: V0,
    touchdown,
    suspended,
    grounded,
    totalGrounded,
    anchorDistance,
    H_anchor,
    geometry: buildGeometry(context, integration, touchdown, totalGrounded),
    buoyStates: integration.buoyStates,
    warnings,
  };
}

function buildGeometry(context, integration, touchdown, totalGrounded) {
  const suspendedPoints = integration.points.map((pt) => ({ x: pt.x, y: pt.y }));
  const seabedPoints = [];
  if (totalGrounded > 0) {
    seabedPoints.push({ x: touchdown.x, y: touchdown.y });
    seabedPoints.push({ x: touchdown.x + totalGrounded, y: touchdown.y });
  }
  return {
    suspended: suspendedPoints,
    seabed: seabedPoints,
  };
}

export function solve(config) {
  const context = buildContext(config);
  if (context.totalLength <= 0) {
    throw new Error('No suspended line; anchor dragging risk.');
  }
  const results = [];
  const warnings = [];
  for (const caseDef of config.tensionCases || []) {
    let caseResult;
    if (caseDef.mode === 'T') {
      caseResult = solveTMode(context, caseDef);
    } else if (caseDef.mode === 'H') {
      caseResult = solveHMode(context, caseDef);
    } else {
      throw new Error(`Unsupported tension mode: ${caseDef.mode}`);
    }
    caseResult.id = caseDef.id || caseDef.mode;
    caseResult.label = caseDef.label || caseResult.id;
    if (caseResult.warnings.length) {
      warnings.push({ caseId: caseResult.id, warnings: caseResult.warnings });
    }
    results.push(caseResult);
  }
  return {
    results,
    warnings,
    context,
  };
}

export default { solve };
