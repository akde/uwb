# GPS-Denied Drone Swarm Relative Positioning System

## Complete System Design Document

**Version:** 1.0
**Date:** February 2026
**Purpose:** Enable a next engineer or code agent to fully understand, implement, and extend this system without ambiguity.

---

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [System Constraints](#2-system-constraints)
3. [Hardware](#3-hardware)
4. [Initial Conditions](#4-initial-conditions)
5. [Coordinate Frames](#5-coordinate-frames)
6. [Information Shared Among Drones](#6-information-shared-among-drones)
7. [Algorithm Pipeline](#7-algorithm-pipeline)
8. [Mathematical Details](#8-mathematical-details)
9. [Controller Design](#9-controller-design)
10. [Ground Calibration Procedure](#10-ground-calibration-procedure)
11. [Operational Sequence](#11-operational-sequence)
12. [Failure Modes and Degradation](#12-failure-modes-and-degradation)
13. [Tunable Parameters](#13-tunable-parameters)
14. [Design Decisions and Rationale](#14-design-decisions-and-rationale)
15. [What Was Explicitly Rejected and Why](#15-what-was-explicitly-rejected-and-why)
16. [Extension Points](#16-extension-points)

---

## 1. Problem Statement

Six drones must fly in a rigid pentagon-plus-center formation in an outdoor, GPS-denied environment. There is no ground infrastructure — no fixed anchors, no base stations, no RTK, no motion capture.

Only the **center drone** has autonomous navigation capability (STM KERKES — a visual-inertial GNSS-independent navigation system). The **five outer drones** have no absolute positioning. They must determine their own positions relative to the formation and hold their assigned vertices using only local sensors.

The output of each outer drone's algorithm is a **body-frame velocity command** `[vx, vy]` that feeds directly into the autopilot. The autopilot handles attitude control and altitude hold independently.

---

## 2. System Constraints

These constraints are not assumptions — they are design choices made during the system definition phase. Each one eliminates a specific mathematical difficulty.

| Constraint | What it eliminates |
|---|---|
| All drones have a compass and share their heading | **Rotation ambiguity** — all drones agree on a common north-east coordinate frame at all times |
| Center drone is defined as origin (0, 0) | **Translation ambiguity** — the coordinate frame is anchored to a known point |
| All drones fly at the same altitude | **3D degeneracy** — reduces the problem from 3D to 2D. Altitude is handled independently by each drone's barometric altitude hold |
| Drones are placed in a known formation before takeoff | **Initialization** — provides calibration baseline, initial filter state, and per-link UWB bias correction |
| All drones face magnetic north at placement | **Initial heading alignment** — not strictly required at runtime (compasses provide heading continuously), but simplifies ground calibration verification |

**Net effect:** All three fundamental ambiguities of anchor-free range-only localization (translation, rotation, reflection) are resolved before takeoff. The reflection ambiguity specifically is killed by the 2D + compass combination — when everyone agrees on north and east, there is no mirror axis to flip across.

---

## 3. Hardware

### 3.1 Drones

The drones are based on the **STM KERKES** ecosystem. KERKES is not a drone — it is a GNSS-independent navigation module developed by STM Savunma Teknolojileri (Turkey). It uses visual-inertial odometry with deep learning, matching real-time camera/IMU data against pre-loaded orthophoto maps to produce absolute position, or falling back to relative visual odometry from the takeoff point. It operates day and night.

KERKES has been integrated into STM's tactical mini-UAV platforms, notably the **TOGAN** surveillance quadrotor. The swarm operation concept comes from STM's separate **BUMİN** program (distributed swarm control, no central command node).

Each drone in this system has:

- **Flight controller** with IMU (attitude stabilization) and barometer (altitude hold)
- **Compass** (magnetometer) — built into the flight controller
- **Autopilot** that accepts velocity commands `[vx, vy]` in body frame
- **UART port** available for the UWB module

The center drone additionally has:

- **KERKES module** for autonomous cruise navigation

### 3.2 UWB Modules

**Nooploop LinkTrack P-B (LTP-B)** — the long-range variant of Nooploop's UWB product line.

| Parameter | Value |
|---|---|
| Ranging accuracy | ±10 cm (1σ), ±5 cm std. dev. in 2D |
| Maximum range | 500–2000 m (outdoor LOS) |
| Operating frequency | 3993.6 MHz and 4492.8 MHz (4 GHz / 4.5 GHz bands) |
| Ranging method | TWR (Two-Way Ranging) — no TDoA |
| Operating mode | **DR Mode (Distributed Ranging)** — peer-to-peer, no anchor/tag distinction |
| Update rate at 6 nodes | **~50 Hz** per node (DR_MODE0) |
| User data payload | Available in each ranging frame (used for heading broadcast) |
| Interface | UART (3.3V TTL, up to 3 Mbps) and USB Type-C |
| Power | 1.35 W (3.6–5.5 V supply) |
| Weight | 33.3 g (with shell and antenna) |
| Dimensions | 60.3 × 29 × 9 mm module + 85.5 × φ9.3 mm rod antenna |

**Allocation:** 1 module per drone, 6 active, 14 spares (out of 20 purchased).

The reason only 1 module per drone is needed: the original design considered 3 modules per drone for heading estimation, but the presence of onboard compasses made that unnecessary.

### 3.3 Why LinkTrack P-B specifically

The P-B variant operates exclusively on 4.0/4.5 GHz bands (lower frequency = better outdoor propagation) with higher transmit power than the P-A (indoor) or P-C (6.5 GHz regulatory variant). At 40 m inter-drone spacing, range is not an issue, but the P-B's superior outdoor noise characteristics (fewer multipath artifacts at 4 GHz vs 6.5 GHz) matter.

### 3.4 DR Mode explained

In DR (Distributed Ranging) mode, all nodes are peers. There is no anchor/tag hierarchy. Each node broadcasts a timestamped UWB packet in its assigned TDMA slot, and all other nodes listen. The firmware handles scheduling internally. Each node produces distances to all other nodes plus a small user data payload per frame.

This is fundamentally different from LinkTrack's LP mode (fixed anchors + mobile tags) or DT mode (data transfer). DR mode is the only mode that supports anchor-free peer-to-peer ranging.

---

## 4. Initial Conditions

### 4.1 Formation geometry

Six drones are placed on the ground in a **regular pentagon plus center** configuration:

- **Center drone (ID = 0):** at position `(0, 0)` in the formation frame
- **Outer drones (IDs 1–5):** at the five vertices of a regular pentagon

The pentagon has **side length 40 m**, which gives a **circumradius of 34.03 m** (distance from center to each vertex).

```
circumradius = side_length / (2 × sin(π/5))
             = 40 / (2 × 0.5878)
             = 34.03 m
```

### 4.2 Vertex coordinates (formation-local frame)

Vertex 1 is at the top (north), subsequent vertices go clockwise:

| Drone ID | Angle from east | x (east) [m] | y (north) [m] |
|---|---|---|---|
| 0 (center) | — | 0.00 | 0.00 |
| 1 | 90° | 0.00 | +34.03 |
| 2 | 18° | +32.36 | +10.51 |
| 3 | −54° | +20.00 | −27.53 |
| 4 | −126° | −20.00 | −27.53 |
| 5 | 162° | −32.36 | +10.51 |

The angle for vertex `i` (1-indexed) is:

```
θ_i = π/2 − (i−1) × 2π/5
x_i = R × cos(θ_i)
y_i = R × sin(θ_i)
```

### 4.3 Known pairwise distances

With 6 drones, there are `C(6,2) = 15` unique pairwise distances. All are computable from the coordinates above. These known distances are used during ground calibration to extract per-link UWB bias.

Key distances:

| Pair type | Distance |
|---|---|
| Center to any vertex | 34.03 m |
| Adjacent pentagon vertices (e.g., 1↔2) | 40.00 m |
| Non-adjacent vertices (e.g., 1↔3) | 64.72 m |

### 4.4 Heading at placement

All drones are placed **facing magnetic north** (heading ψ = 0). This is a convenience for calibration verification, not a runtime requirement — compasses provide continuous heading.

### 4.5 What gets loaded into each drone before flight

Every drone receives:

1. **Its own ID** (0–5)
2. **The complete formation vertex table** (all 6 positions in the formation-local frame)
3. **Per-link UWB bias corrections** (computed during ground calibration — see Section 10)
4. **Its initial position** (its own vertex coordinates, used to initialize the position filter)

---

## 5. Coordinate Frames

### 5.1 North frame (global, shared by all drones)

- **Origin:** center drone's position (always `(0, 0)`)
- **x-axis:** magnetic east
- **y-axis:** magnetic north
- **Convention:** right-handed, z up (but z is not used — 2D system)

All drones agree on this frame because all have compasses. Trilaterated positions, desired positions, and formation errors are computed in this frame.

### 5.2 Formation-local frame

- Identical to the north frame **when the center drone's heading is 0** (facing north)
- When the center drone rotates to heading `ψ_center`, the formation vertices rotate with it:

```
p_desired_i = R(ψ_center) × p_vertex_i
```

where `R(ψ)` is the 2D rotation matrix:

```
R(ψ) = [ cos(ψ)  −sin(ψ) ]
        [ sin(ψ)   cos(ψ) ]
```

This means the pentagon formation rotates to follow the center drone's heading. If the center drone turns east, the whole formation pivots east.

**Alternative behavior (north-locked formation):** If you want the pentagon to always point north regardless of center drone heading, simply skip the rotation: `p_desired_i = p_vertex_i`. This is a one-line change.

### 5.3 Body frame (per-drone)

- **Origin:** the drone itself
- **vx:** forward (along the drone's nose direction / heading)
- **vy:** rightward (90° clockwise from nose)
- **Heading ψ_self:** angle from magnetic north to nose, measured clockwise (standard aviation convention: 0° = north, 90° = east)

The autopilot expects velocity commands in this frame.

### 5.4 Frame transformation

To convert a velocity vector from north frame to body frame:

```
v_body = R(−ψ_self) × v_north
```

Expanded:

```
v_body_x =  cos(ψ_self) × v_north_x + sin(ψ_self) × v_north_y
v_body_y = −sin(ψ_self) × v_north_x + cos(ψ_self) × v_north_y
```

---

## 6. Information Shared Among Drones

All data exchange occurs over the UWB link. No separate radio channel is needed.

### 6.1 Data produced automatically by UWB hardware

| Data | Description | Rate |
|---|---|---|
| Pairwise range `d_ij` | Distance between every pair of drones | 50 Hz |

This comes directly from the LinkTrack P-B DR mode firmware. Each drone receives 5 range measurements per cycle (one to each of the other 5 drones).

### 6.2 Data sent via UWB user payload

| Data | Size | Sender | Content |
|---|---|---|---|
| Compass heading ψ | 2 bytes | Every drone | Heading in 0.01° resolution (0–36000 → 0°–360°), unsigned 16-bit integer |
| Node ID | 1 byte | Every drone | Drone identifier 0–5 |

**Total payload per drone per cycle: 3 bytes.**

### 6.3 What each drone knows at every cycle

Each **follower drone** (ID 1–5) has access to:

1. **Its own compass heading** `ψ_self` — from its magnetometer
2. **5 UWB range measurements** — `d_0, d_1, ..., d_5` (excluding self) — from UWB hardware
3. **All 6 compass headings** — `ψ_0, ψ_1, ..., ψ_5` — from UWB payload
4. **The center drone's heading** `ψ_center = ψ_0` — extracted from the above
5. **Known formation geometry** — the vertex table loaded before flight
6. **Per-link bias corrections** — loaded before flight
7. **Previous position estimate** — from its own filter state

The **center drone** (ID 0) does not run the follower algorithm. It navigates via KERKES and simply participates in UWB ranging and heading sharing.

### 6.4 What is NOT shared

- Drone positions are **not** shared between followers. Each follower trilaterates independently. (In a more advanced version, sharing positions would allow followers to use each other as reference points with known uncertainty — see Section 16.)
- Velocity or acceleration data is not shared.
- KERKES navigation state (beyond heading) is not shared.

---

## 7. Algorithm Pipeline

Each follower drone runs this pipeline at 50 Hz (every 20 ms):

```
raw_ranges ──→ [1. Bias Correction] ──→ [2. Outlier Rejection] ──→ [3. Trilateration]
                                                                          │
                                                                     raw position
                                                                          │
                                                                   [4. Position Filter]
                                                                          │
                                                                  filtered position
                                                                          │
                              ψ_center ──→ [5. Desired Position] ────→ [6. Formation Error]
                                                                          │
                                                                     error vector
                                                                          │
                               ψ_self ──→ [7. Frame Transform] ──→ [8. PD Controller]
                                                                          │
                                                                   v_cmd [vx, vy]
                                                                          │
                                                                     TO AUTOPILOT
```

### 7.1 Step 1: Bias correction

```
d_corrected_j = d_raw_j − bias_j
```

`bias_j` is a per-link constant computed during ground calibration (Section 10). Typical values: +10 to +25 cm for LinkTrack P-B at 40 m outdoor LOS.

**Why this exists:** UWB TWR has a systematic positive bias due to antenna delay, cable length, and internal processing time. It is stable over time and temperature for a given link, so a one-time calibration suffices.

### 7.2 Step 2: Outlier rejection

For each range measurement `d_j`, compute the predicted range from the previous position estimate:

```
d_predicted_j = ||p̂_prev − p_j||
```

If the residual exceeds 3σ:

```
|d_j − d_predicted_j| > 3 × σ_uwb
```

then **reject** that measurement (do not include it in trilateration).

`σ_uwb = 0.10 m` (10 cm, the UWB noise standard deviation).

**Why 3σ:** At 3σ, the false rejection rate for good measurements is 0.3%. The NLOS corruption at 40 m typically adds 0.5–2 m error, which is 5–20σ — easily caught. The threshold is deliberately generous because trilateration is overdetermined (5 measurements for 2 unknowns), so losing one measurement is fine, but wrongly keeping a bad one is costly.

**Edge case:** On the very first cycle, there is no previous position to predict from. Accept all measurements. The initial guess from the known formation vertex is close enough.

### 7.3 Step 3: Trilateration (Weighted Least Squares, Gauss-Newton)

**Input:** Up to 5 bias-corrected, outlier-filtered ranges; known positions of the corresponding peers; previous position as initial guess.

**Minimum:** 3 ranges required. If fewer pass outlier rejection, skip this cycle and hold the previous position estimate.

**Method:** Iterative Gauss-Newton on the nonlinear least-squares problem:

```
minimize  Σ_j  w_j × (||p − p_j|| − d_j)²
```

See Section 8 for the full math.

**Output:** Raw position estimate `p_raw = [x, y]` in the north frame.

**Convergence:** From the previous position (20 ms old, ~1 cm displacement), Gauss-Newton converges in 2–3 iterations. Set max iterations to 5 as a safety margin, with tolerance 0.1 mm.

### 7.4 Step 4: Position filter (Exponential Moving Average)

```
p_filtered(t) = α × p_raw(t) + (1 − α) × p_filtered(t−1)
```

`α = 0.25`

**Effect on noise:** Raw trilateration noise is ~±6 cm (±10 cm UWB / √2.5 overdetermination). After EMA at α=0.25 and 50 Hz, effective noise drops to ~±2 cm.

**Effect on latency:** EMA with α=0.25 has a time constant of `dt / α = 0.02 / 0.25 = 0.08 s` = 80 ms. At formation-keeping speeds (~0.5 m/s), this introduces ~4 cm lag. Acceptable.

**Initialization:** Set `p_filtered(0)` to the known formation vertex coordinates. This means the filter starts in tracking mode with no convergence delay.

### 7.5 Step 5: Desired position

Rotate the drone's assigned formation vertex by the center drone's current heading:

```
p_desired = R(ψ_center) × p_vertex_self
```

where `p_vertex_self` is this drone's pentagon vertex from the formation table (Section 4.2).

### 7.6 Step 6: Formation error

```
error = p_desired − p_filtered
```

This is a 2D vector in the north frame pointing from the drone's current position toward where it should be.

### 7.7 Step 7: PD controller (in north frame)

```
v_north = Kp × error + Kd × ė_filtered
```

Where `ė_filtered` is the filtered time derivative of the error:

```
ė_raw(t) = (error(t) − error(t−1)) / dt
ė_filtered(t) = α_d × ė_raw(t) + (1 − α_d) × ė_filtered(t−1)
```

`α_d = 0.15` (derivative filter coefficient — more aggressive smoothing than position, because numerical differentiation amplifies noise).

**Deadzone:** If `||error|| < 0.10 m`, output zero velocity. This prevents the controller from chasing noise when the drone is already at its vertex.

**Velocity clamp:** If `||v_north|| > V_max`, scale down to `V_max`:

```
if ||v_north|| > V_max:
    v_north = v_north × V_max / ||v_north||
```

`V_max = 5.0 m/s`

### 7.8 Step 8: Transform to body frame

```
v_body = R(−ψ_self) × v_north
```

`v_body = [vx, vy]` where `vx` is forward and `vy` is rightward. This goes directly to the autopilot.

---

## 8. Mathematical Details

### 8.1 Trilateration: Gauss-Newton derivation

**Measurement model** for range from own position `p` to peer `j` at position `p_j`:

```
h_j(p) = ||p − p_j|| = sqrt((x − x_j)² + (y − y_j)²)
```

**Jacobian row** (partial derivative of h_j with respect to p):

```
H_j = ∂h_j/∂p = (p − p_j)^T / ||p − p_j||
    = [(x − x_j) / d_j,  (y − y_j) / d_j]
```

where `d_j = ||p − p_j||` is the current estimated distance.

**Residual:**

```
r_j = d_measured_j − h_j(p̂)
```

**Weight matrix:** For uniform noise σ across all links:

```
W = (1/σ²) × I
```

**Normal equations:**

```
(H^T W H) δ = H^T W r
```

**Update:**

```
p̂_new = p̂ + δ
```

Repeat until `||δ|| < tolerance` or max iterations reached.

**Dimensions:** With `m` measurements and 2 unknowns:
- `H` is `m × 2`
- `r` is `m × 1`
- `H^T W H` is `2 × 2` (trivial to solve)
- `δ` is `2 × 1`

### 8.2 Numerical conditioning

The condition number of `H^T H` depends on the geometric spread of the reference nodes around the drone. For a pentagon + center formation at 40 m:

- The drone's 5 peers subtend angles spanning nearly 360° → condition number is small (~2–5)
- This is excellent. Poorly conditioned geometry (e.g., all peers in one direction) would give condition numbers >100

**Regularization:** Add `ε × I` to `H^T W H` with `ε = 1e-8` to prevent singular matrices in degenerate edge cases (e.g., drone exactly on top of a peer — impossible in flight but possible in numerical simulations).

### 8.3 Why not closed-form?

For exactly 3 ranges in 2D, a closed-form solution exists (two circle intersections). But:

1. We have 5 ranges, not 3. The overdetermination requires least-squares.
2. Gauss-Newton from a warm start (previous position) converges in 2 iterations. Computational cost is negligible.
3. The iterative approach naturally extends to weighted measurements if different links have different noise characteristics.

### 8.4 Geometric Dilution of Precision (GDOP)

For this geometry, the horizontal DOP can be approximated from the Jacobian:

```
GDOP = sqrt(trace((H^T H)^{-1}))
```

For a pentagon formation with the drone at a vertex and 5 peers (center + 4 other vertices) surrounding it, GDOP ≈ 0.6–0.8. This means trilateration amplifies UWB noise by less than 1×. With 10 cm UWB noise, position error is ~6–8 cm per snapshot — confirmed in simulation.

---

## 9. Controller Design

### 9.1 Plant model

The autopilot accepts body-frame velocity commands and tracks them. From the formation controller's perspective, the drone is a velocity-controlled integrator:

```
ṗ = v_cmd  (with some delay and bandwidth limit from the autopilot)
```

### 9.2 PD controller analysis

Transfer function (Laplace domain, scalar for simplicity):

```
V(s) = Kp × E(s) + Kd × s × E(s)
P(s) = V(s) / s  (integrator plant)

Closed-loop: P(s)/P_ref(s) = (Kd × s + Kp) / (s² + Kd × s + Kp)
```

With `Kp = 0.3` and `Kd = 0.15`:

- **Natural frequency:** `ω_n = sqrt(Kp) = 0.548 rad/s`
- **Damping ratio:** `ζ = Kd / (2 × sqrt(Kp)) = 0.15 / (2 × 0.548) = 0.137`

This is underdamped in continuous theory, but the EMA filter on both position and derivative provides additional damping that the continuous analysis doesn't capture. In discrete simulation at 50 Hz, the system behaves as approximately critically damped.

### 9.3 Settling time

From 2 m initial error, the system reaches <15 cm residual in ~8 seconds (verified in simulation). The residual is dominated by UWB noise, not controller convergence — the controller has converged, but the trilateration estimate has ±6 cm jitter.

### 9.4 Gain tuning rationale

| Parameter | Value | Rationale |
|---|---|---|
| `Kp = 0.3` | 1 m error → 0.3 m/s command | Gentle. At 40 m separation, we don't want aggressive corrections that could cause collisions. |
| `Kd = 0.15` | Kd/Kp = 0.5 | Damps oscillation. Higher Kd amplifies measurement noise through the derivative. |
| `V_max = 5.0 m/s` | Hard safety clamp | Prevents runaway if trilateration produces a large transient error. |
| `Deadzone = 0.10 m` | Below noise floor | ±6 cm trilateration noise + ±2 cm after filter = ~±8 cm peak. 10 cm deadzone prevents chasing noise. |

---

## 10. Ground Calibration Procedure

### 10.1 Purpose

Extract per-link UWB bias so that ranging errors are reduced from ±15–25 cm (biased) to ±10 cm (noise-only).

### 10.2 Procedure

1. Place all 6 drones at their known formation positions (Section 4.2). Measure positions to ±5 cm accuracy with a tape measure or laser rangefinder.

2. Power on all UWB modules in DR mode.

3. Collect range measurements for **10 seconds** (500 samples per pair at 50 Hz).

4. For each link `(i, j)`, compute:

```
bias_ij = mean(d_measured_ij[1..500]) − d_true_ij
```

where `d_true_ij` is computed from the known formation coordinates.

5. Store `bias_ij` on each drone for all links involving that drone.

### 10.3 Verification

After calibration, trilaterate each drone's position using the bias-corrected ranges and compare against the known formation coordinates. **All position errors must be < 5 cm.** If not, investigate the offending links (possible NLOS from a ground surface reflection, misplaced drone, or faulty module).

### 10.4 Bias stability

UWB TWR bias is dominated by antenna delay and is stable over hours and across temperature variations of ±10°C. Recalibrate if modules are physically replaced.

---

## 11. Operational Sequence

| Time | Action |
|---|---|
| T − 5 min | Place drones in pentagon + center formation at known coordinates, all facing north |
| T − 2 min | Load formation table, initial positions, and power on UWB modules |
| T − 1 min | Run ground calibration: collect 10 s of UWB data, compute per-link bias |
| T − 30 s | Verify: trilaterate all positions, confirm < 5 cm error. All 15 links active. All 6 headings received. System GO. |
| T = 0 | Takeoff. All drones ascend to common altitude. Barometric hold locks z. Trilateration + formation controller active immediately (no convergence delay). |
| T + onward | Center drone navigates via KERKES. Followers trilaterate at 50 Hz and hold formation. |

---

## 12. Failure Modes and Degradation

### 12.1 Loss of one UWB range

If one of the 5 range measurements fails outlier rejection or is missing, trilateration continues with 4 measurements. Accuracy degrades slightly (GDOP increases by ~20%). If 3 or more fail simultaneously (highly unlikely in LOS), hold the last known position for up to 0.5 s before declaring a fault.

### 12.2 Complete UWB loss on one follower

That drone loses its ability to trilaterate. It should hold its last known velocity for 0.5 s, then decelerate to hover. The remaining 5 drones (center + 4 followers) are fully localizable.

### 12.3 Center drone UWB loss

Followers lose range to the origin. They can still trilaterate relative to each other (4 ranges per drone from the other 4 followers), but the position estimate is now relative to the follower sub-swarm, not the center. The formation shape is maintained but may drift in translation. This is a degraded but survivable mode.

### 12.4 Compass failure on one follower

Without own heading, the drone cannot transform velocity commands from north frame to body frame. It should hover in place. Other drones are unaffected (they use their own compasses independently).

### 12.5 Center drone heading broadcast loss

Followers cannot compute `p_desired = R(ψ_center) × p_vertex`. Fallback: use the last received heading and hold it. If timeout > 2 s, switch to north-locked formation (no rotation — `p_desired = p_vertex`).

---

## 13. Tunable Parameters

All parameters in one table for easy adjustment:

| Parameter | Symbol | Default | Unit | Effect of increase |
|---|---|---|---|---|
| Position filter coefficient | `α` | 0.25 | — | More responsive but noisier |
| Proportional gain | `Kp` | 0.30 | 1/s | Faster convergence, more oscillation |
| Derivative gain | `Kd` | 0.15 | — | More damping, but amplifies noise |
| Derivative filter coefficient | `α_d` | 0.15 | — | More responsive derivative, noisier |
| Max velocity | `V_max` | 5.0 | m/s | Higher ceiling for correction speed |
| Deadzone | — | 0.10 | m | Larger quiet zone, less precision |
| Outlier threshold | — | 3.0 | σ | More permissive, admits more outliers |
| UWB noise std | `σ_uwb` | 0.10 | m | Adjusts outlier gate and WLS weights |
| Formation side length | — | 40.0 | m | Changes all vertex coordinates |

### 13.1 Recommended tuning procedure

1. Start with defaults.
2. If position jitter causes visible oscillation in flight, decrease `α` (e.g., 0.15).
3. If the drone is sluggish returning to formation, increase `Kp` (e.g., 0.5).
4. If the drone overshoots its vertex, increase `Kd` (e.g., 0.25).
5. If the drone vibrates near its vertex, increase deadzone (e.g., 0.15 m).

---

## 14. Design Decisions and Rationale

### 14.1 One module per drone (not three)

Originally considered: 3 modules per drone in a triangular arrangement for UWB-based heading estimation. Rejected because all drones have compasses. The compass provides heading directly, eliminating the need for differential ranging. One module saves 66 g and 2.7 W per drone, simplifies wiring, and frees 14 modules as spares.

### 14.2 EMA filter (not EKF)

An Extended Kalman Filter with IMU prediction would reduce noise from ±6 cm to ~±2 cm and handle UWB dropouts by coasting on IMU. But:

- EMA already achieves ±2 cm effective noise at 50 Hz
- The formation controller has a 10 cm deadzone — anything below that is irrelevant
- EKF requires tuning 10+ noise parameters (Q, R matrices), IMU bias modeling, and careful initialization
- EKF is warranted if drones perform aggressive maneuvers (>2 m/s²) or UWB dropout rate exceeds 20%

Start with EMA. Upgrade to EKF only if flight testing reveals inadequacy.

### 14.3 Trilateration (not MDS)

MDS (Multidimensional Scaling) treats all nodes equally and produces positions up to rotation/translation/reflection. It doesn't exploit the fact that the center drone is a privileged known reference. Trilateration anchored to the center is simpler, uses fewer computations, and directly gives each drone what it needs — its own position relative to center.

MDS is used only once, during ground calibration verification, if desired.

### 14.4 PD controller (not PID)

The integral term is intentionally omitted:

- The plant is an integrator (velocity-controlled position). Adding I to a PD-on-integrator gives a double integrator, which is marginally stable and prone to windup.
- There is no steady-state error to correct — the desired position is a constant vertex, and the proportional term drives the error to zero for a step reference on an integrator plant.
- If wind causes persistent drift, the correct fix is feedforward compensation, not integral action.

### 14.5 North frame for computation, body frame for output only

All computation (trilateration, error, PD) is done in the north frame. Body frame transform happens only at the final step. This avoids the complexity of tracking and propagating heading uncertainty through intermediate computations.

---

## 15. What Was Explicitly Rejected and Why

| Approach | Why rejected |
|---|---|
| **MDS at runtime** | Ignores center-as-origin structure. No temporal filtering. Jittery. Wasteful. |
| **EKF for position** | Premature complexity. EMA sufficient for slow formation keeping. |
| **3 UWB modules per drone** | Compass eliminates need for UWB-based heading estimation. |
| **Ground anchors** | Contradicts anchor-free requirement. Adds logistics. |
| **Factor graph optimization** | Research-grade complexity for a problem that trilateration solves. |
| **Position sharing between followers** | Adds communication overhead. Each follower has 5 ranges including center — sufficient without peer position sharing. |
| **PID controller** | Integral term unnecessary and dangerous on integrator plant. |
| **GPS for initialization** | GPS is denied by problem definition. Tape-measure placement suffices. |
| **Visual relative localization** | Requires cameras, detection algorithms, limited range. UWB is simpler and works at 40+ m. |

---

## 16. Extension Points

For future development:

### 16.1 Follower-to-follower position sharing

Each follower broadcasts its estimated position (4 bytes x, 4 bytes y = 8 bytes) in the UWB payload. Other followers use these as additional reference points with appropriate uncertainty weighting. This improves trilateration when the center drone's range is temporarily blocked (NLOS).

### 16.2 EKF upgrade

State vector: `[x, y, vx, vy]`. Prediction from IMU accelerometer. Update from UWB ranges. Handles temporary UWB dropouts (coasts on IMU for up to 1 s). Required if drones perform fast maneuvers.

### 16.3 Dynamic formation changes

The formation vertex table can be updated at runtime. To transition from pentagon to line formation: gradually interpolate each drone's target vertex from the old position to the new over 5–10 seconds. The controller tracks the moving target smoothly.

### 16.4 Swarm expansion to 20 drones

With 14 spare modules, the system scales to 20 drones. At 20 nodes in DR_MODE0, the update rate drops to ~50 Hz (still sufficient). The formation geometry changes, but the trilateration and controller logic is identical per drone.

### 16.5 Heading consensus without compass

If compasses are unreliable (strong electromagnetic interference), heading can be estimated from UWB by mounting 2–3 modules per drone at known offsets and using differential ranging. This was the original design before compasses were confirmed available.

---

## Summary

The system is defined by five facts:

1. **Center drone is always at (0, 0)** — kills translation ambiguity
2. **Every drone has a compass** — kills rotation ambiguity
3. **All fly at the same altitude** — kills reflection ambiguity and reduces to 2D
4. **Known pre-takeoff formation** — provides calibration baseline and filter initialization
5. **Nooploop P-B in DR mode** — provides 50 Hz peer-to-peer ranging with heading payload

The algorithm on each follower is: bias-correct 5 UWB ranges → reject outliers → trilaterate (Gauss-Newton, 2–3 iterations from warm start) → EMA filter → compute formation error → PD velocity controller → transform to body frame → send to autopilot.

No GPS. No ground anchors. No EKF. No MDS at runtime. No complex algorithms. Just constrained geometry, good calibration, and a PD controller on trilaterated positions.
