# Solver assumptions and conventions

## Coordinate system

* The still water surface is the global origin in the vertical direction: `y = 0` at the surface and `y = waterDepth` at the seabed. Positive `y` points downward.
* The fairlead is located at depth `fairleadDepth` below the surface. Internal integration coordinates start at the fairlead, so the suspended-line ordinate that the solver integrates (`y_local`) is zero at the fairlead and increases downward. The absolute depth of any point along the line is therefore `y_abs = fairleadDepth + y_local`.
* Horizontal distance `x` is measured from the fairlead toward the anchor. The touchdown point is the first location where `y_abs` reaches the seabed (`waterDepth`).

## Governing equations

* Arc-length (`s`) is used as the marching coordinate. The state variable `u` is the standard catenary parameter, so that the differential system is
  * `dy/ds = tanh(u)`
  * `du/ds = + w / (H * cosh(u))`
  * `dx/ds = 1 / cosh(u)`
  where `w` is the submerged weight per unit length for the current segment and `H` is the horizontal component of the tension. These equations are integrated with a fourth-order Runge–Kutta scheme with an adaptive step cap at segment and buoy boundaries.

## Segments and buoy handling

* The line is assembled as two in-line segments in this order: wire first, chain second. Segment-specific weight (`w`) and seabed friction (`μ`) properties switch instantaneously at their arc-length boundaries.
* Buoys (if any) are defined by their along-line attachment (`arcLength` measured from the anchor toward the fairlead), upward force (`force`), and pennant length. When the integration arrives at the buoy location the vertical component of tension is reduced by the applied buoyancy, clamped to non-negative values:
  * `V_above = max(0, V_below − F_buoy)`
  * The vertical drop is only applied when the pennant is taut. The pennant is considered taut when the attached point on the line is deeper than the pennant length below the buoy. The buoy is drawn at depth `y_abs = fairleadDepth + (attachDepth − pennantLength)` but never above the surface.

## Touchdown and grounded allocation

* Touchdown is detected when the absolute depth equals the seabed depth: `fairleadDepth + y_local = waterDepth`. The integration stops at the touchdown point when solving for the final geometry.
* Any remaining length after touchdown is placed on the seabed. Grounded length is allocated to the chain segment first; if additional length remains it is assigned to the wire segment.
* Seabed friction is only applied on grounded portions. The horizontal tension that arrives at the anchor is computed as `H_anchor = max(0, H_touchdown − Σ μ_i w_i L_grounded_i)`.

## Tension modes

* **T-mode** (`mode: "T"`): the total fairlead tension magnitude is prescribed. The solver brackets and bisects the fairlead catenary parameter `u₀`; each trial uses `H = T / cosh(u₀)` so the prescribed total tension is honoured while searching for a suspended profile that just reaches the seabed. This guarantees the longest fairlead-to-touchdown span that is compatible with the prescribed total tension.
* **H-mode** (`mode: "H"`): the fairlead horizontal tension component is prescribed. The solver brackets and bisects the initial catenary parameter `u₀` to reach the same limiting geometry (the suspended solution reaches the seabed exactly at the end of the full length).
* The vertical component at the fairlead is determined from `u₀` by `V₀ = H sinh(u₀)`; the total fairlead tension is `T = H cosh(u₀)`.

## API contract

The physics solver is exposed via `solve(config)` from `src/solver.js`. The configuration object must contain:

```jsonc
{
  "waterDepth": number,          // metres, > fairleadDepth
  "fairleadDepth": number,       // metres below surface
  "wire": { "length": number, "weight": number, "friction": number },
  "chain": { "length": number, "weight": number, "friction": number },
  "buoys": [
    {
      "arcLength": number,       // metres along the line from the anchor
      "force": number,           // upward force
      "pennantLength": number    // slack length, metres
    }
  ],
  "tensionCases": [
    {
      "id": string,
      "mode": "T" | "H",
      "value": number            // total tension (T-mode) or horizontal component (H-mode)
    }
  ]
}
```

The return value is `{ results, warnings, context }`, where each entry in `results` has

* `id`, `mode`, and the input value for traceability
* `touchdown` `{ s, x, y }` (arc-length, horizontal reach, and local depth at touchdown)
* `grounded` and `suspended` lengths for wire and chain
* `anchorDistance` (fairlead-to-anchor horizontal distance)
* `H_anchor` (horizontal tension that reaches the anchor)
* `geometry` (arrays of points for suspended and grounded portions)
* `buoyStates` (taut/slack status and drawing data)
* `warnings` (e.g. "dragging risk" when the anchor tension decays to zero with grounded length present, "No grounded length: anchor-drag risk" when the suspended solution consumes the entire line)

The solver raises an error when both the wire and chain have zero length (`"No suspended line; anchor dragging risk."`) and when the prescribed tension cannot deliver a seabed touchdown.
