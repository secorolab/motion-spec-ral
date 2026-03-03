# Find and Unscrew — Motion Specification

This folder contains the declarative motion specification models for a
**find-and-unscrew** task. The tool (end-effector) is already placed on the
screw head surface and pressed against the workpiece.

## Motions

### Phase 1 — Find

Rotate around the tool axis (z) at a constant angular velocity to locate the
screw engagement point (e.g., hex or slot). Detection criteria:

| Condition | Meaning | Response |
|-----------|---------|----------|
| `torque-ee-z ≥ 4.0 N·m` | Screw engagement found | Proceed to loosen |
| `rotation-find ≥ 2.0944 rad` (~120°) | Screw not found within search range | Abort |
| `torque-ee-z ≥ 12.0 N·m` | Safety torque limit exceeded | Emergency stop |

While searching, all other 5 DoF are held at zero velocity to maintain the
tool's position and alignment on the screw head.

### Phase 2 — Loosen

Pre-condition: engagement torque confirmed (`torque-ee-z ≥ 0.5 N·m`).

Reverse-rotate to unscrew. Termination when both conditions are within bounds:
- Rotation within `[1.4835, 1.6581]` rad
- Torque within `[0.05, 8.4]` N·m

Same 5 supporting constraints hold position and alignment.

## File Structure

| File | Contents |
|------|----------|
| `00-misc.json` | Geometric primitives: frames, points, links, kinematic chain |
| `01-world-model.json` | All quantities: twist, wrench, pose, velocities, torques, thresholds, errors, acceleration energies |
| `02-map.json` | Views (3D→1D from twist/wrench) and operators (rotation from pose) |
| `03-constraints.json` | Equality, unilateral, bilateral constraints for both phases |
| `04-motion-specification.json` | Guarded motions composing constraints into find and loosen phases |
| `05-constraint-handler.json` | Evaluators, PID controllers, monitors; assembled into handlers |
| `06-solver-specification.json` | Acceleration constraints, motion drivers, prioritization, solver |

## Constrained Dimensions

Both phases constrain all 6 end-effector DoF:

| DoF | Find | Loosen |
|-----|------|--------|
| Angular z | `ω_z = 1.0 rad/s` | `ω_z = -1.0 rad/s` |
| Angular x | `ω_x = 0` | `ω_x = 0` |
| Angular y | `ω_y = 0` | `ω_y = 0` |
| Linear x | `v_x = 0` | `v_x = 0` |
| Linear y | `v_y = 0` | `v_y = 0` |
| Linear z | `v_z = 0` | `v_z = 0` |
