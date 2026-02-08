# CRF (Contact-Rich Fastening) Screwing Motion Specifications

This folder contains JSON-LD motion specifications for two screwing motions used in contact-rich fastening tasks.

## Motions

### 1. "Find" Motion (Clockwise Rotation)
**Objective:** Rotate clockwise to lock the screwing tip with the screw-head.

**Physical Constraints:**
- **Rotation Limit (Shutoff Angle):** 120 degrees (2.0944 rad)
- **Torque at Shutoff (Shutoff Torque):** 4 Nm
- **Upper Torque Limit:** 12 Nm

**Motion Structure:**
- **when (preconditions):** Empty - motion can start immediately
- **while (control):**
  - Apply clockwise rotation up to 120°
- **until (termination):**
  - Shutoff angle (120°) reached, OR
  - Shutoff torque (4 Nm) reached, OR
  - Upper torque limit (12 Nm) exceeded

**Files:**
- Constraint: `cstr-rotation-find` (equality constraint to reach 120°)
- Constraints monitored: shutoff angle, shutoff torque, upper torque limit

### 2. "Loosen to Angle" Motion (Counterclockwise Rotation)
**Objective:** Rotate counterclockwise to loosen the screw to approximately 90 degrees.

**Physical Constraints:**
- **Target Rotation:** ~90 degrees with tolerance range [85°, 95°] (1.4835-1.6581 rad)
- **Minimum Torque (Lock Detection):** 0.05 Nm (must exceed this to ensure proper contact)
- **Maximum Torque:** 8.4 Nm (blocking screw detection)

**Motion Structure:**
- **when (preconditions):**
  - "Find" motion has completed (shutoff angle AND shutoff torque reached)
  - Ensures we start from a locked state
- **while (control):**
  - Maintain counterclockwise rotation within [85°, 95°] target range
- **until (termination):**
  - Target rotation reached (85°-95° range), AND
  - Torque within limits [0.05, 8.4] Nm

**Files:**
- Constraint: `cstr-rotation-loosen-bilateral` (bilateral constraint on target range)
- Constraint: `cstr-torque-loosen-bilateral` (torque within limits)
- Precondition: Requires "Find" motion completion

## Files Structure

### Core Model Files

1. **00-misc.json**
   - Structural entities:
     - Points: `point-ee-origin`
     - Frames: `frame-ee`, `frame-world`, `frame-base`
     - Links: `link-ee`, `link-world`
     - Kinematic chain: `chain-arm`
     - Gravitational field: `earth-uniform-gravitational-field`

2. **01-world-model.json**
   - Physical quantities:
     - Rotations: `rotation-find`, `rotation-loosen` (measured values)
     - Rotation references: `rotation-find-ref` (120°), tolerance bounds (85°-95°)
     - Torques: `torque-find`, `torque-loosen` (measured values)
     - Torque limits: shutoff (4 Nm), upper (12 Nm), min contact (0.05 Nm), max (8.4 Nm)
   - Kinematic quantities:
     - Twist: `twist-ee-ee` (end-effector 3D velocity twist)
     - Wrench: `wrench-ee-ee` (end-effector 3D force-torque)
     - Angular velocity: `angvel-ee-z` (1D rotation velocity)
     - Torque: `torque-ee-z` (1D torque around Z-axis)

3. **02-map.json**
   - Views (3D → 1D mappings):
     - `view-angvel-ee-z`: Extract Z-axis angular velocity from twist
     - `view-torque-ee-z`: Extract Z-axis torque from wrench
   - Operators (Integration):
     - `compute-rotation-find`: Integrate angular velocity to cumulative rotation for Find
     - `compute-rotation-loosen`: Integrate angular velocity to cumulative rotation for Loosen

4. **03-constraints.json**
   - Defines 10 constraint entities:
     - Rotation control for Find motion
     - Rotation monitoring (exceeded/not exceeded)
     - Torque monitoring for Find (shutoff, upper limit)
     - Bilateral rotation constraint for Loosen
     - Bilateral torque constraint for Loosen
     - Contact torque precondition for Loosen

5. **04-motion-specification.json**
   - Defines the two `GuardedMotion` specifications
   - Specifies when/while/until phases for each motion
   - References constraint entities

6. **05-constraint-handler.json**
   - Evaluators: compute errors for all constraints
   - Error signals: quantify how far from desired state
   - Controllers:
     - PID for Find rotation control (P=10.0, I=2.0, D=1.5)
     - PID with decaying integral for Loosen rotation (P=8.0, I=1.5, D=1.0, decay=0.99)
   - Monitors: track shutoff/terminal conditions
   - Handlers: assemble evaluators, monitors, controllers for each motion

7. **06-solver-specification.json**
   - Specifies torque commands to be applied to end-effector
   - Hybrid dynamics solver for kinematic chain
   - Prioritization levels (single level per motion)
   - References kinematic chain and gravity from 00-misc.json

## Key Design Decisions

1. **Sequencing:** Loosen motion depends on Find motion completion through precondition constraints
2. **Torque Control:** PID controllers compute required torque based on rotation error
3. **Monitoring:** Multiple unilateral constraints monitor upper limits and shutoff conditions
4. **Bilateral Constraints:** Used for target ranges (rotation ±tolerance, torque bounds)
5. **Decaying Integral:** Loosen motion controller uses decaying integral to reduce command once target reached

## Quantity Units

- **Rotation:** Radians (RAD)
- **Torque:** Newton-meters (N-M)
- **Angular velocity:** Radians per second (RAD-PER-SEC)

## Values Reference

### Find Motion
- Shutoff angle: 2.0944 rad (120°)
- Shutoff torque: 4.0 N⋅m
- Upper torque limit: 12.0 N⋅m

### Loosen Motion
- Lower rotation: 1.4835 rad (85°)
- Upper rotation: 1.6581 rad (95°)
- Minimum torque (contact): 0.05 N⋅m
- Maximum torque: 8.4 N⋅m
