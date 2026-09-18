# Actuator Configuration & Gain Tuning

How to turn a motor datasheet + a URDF into the `<robot>_constants.py` actuator block,
for any robot. Written against `mjlab==1.2.0` / `mujoco-warp==3.5.0`.

**Contents**

1. [What the simulator actually needs](#1-what-the-simulator-actually-needs)
2. [Which datasheet numbers you need](#2-which-datasheet-numbers-you-need)
3. [Step 1 — armature (reflected inertia)](#3-step-1--armature-reflected-inertia)
4. [Step 2 — measure the load inertia](#4-step-2--measure-the-load-inertia)
5. [Step 3 — choose ω and ζ](#5-step-3--choose-ω-and-ζ)
6. [Step 4 — effort limit](#6-step-4--effort-limit)
7. [Step 5 — action scale](#7-step-5--action-scale)
8. [Sanity-check list](#8-sanity-check-list)
9. [Worked example — tahiti_c1](#9-worked-example--tahiti_c1)
10. [Reference — how G1's numbers were chosen](#10-reference--how-g1s-numbers-were-chosen)
11. [Audit script](#11-audit-script)

---

## 1. What the simulator actually needs

Each `BuiltinPositionActuatorCfg` group ends up as five numbers:

```python
BuiltinPositionActuatorCfg(
    target_names_expr=(".*_knee_joint",),  # regex over joint names
    stiffness=...,     # Kp   [Nm/rad]
    damping=...,       # Kd   [Nm·s/rad]
    effort_limit=...,  # τmax [Nm]
    armature=...,      # reflected drivetrain inertia [kg·m²]
)
```

mjlab writes them into MuJoCo like this (`mjlab/utils/spec.py::create_position_actuator`):

| field | lands in MuJoCo as |
|---|---|
| `stiffness` | `actuator.gainprm[0] = Kp`, `actuator.biasprm[1] = -Kp` |
| `damping` | `actuator.biasprm[2] = -Kd` |
| `effort_limit` | `forcelimited = True`, `forcerange = ±τmax` |
| `armature` | `joint.armature = armature` — **overwrites** any value in your XML |
| `frictionloss` | `joint.frictionloss` (defaults to 0) |

Producing the control law:

```
τ = clamp( Kp·(q_target − q) − Kd·q̇ ,  ±τmax )
```

and the joint-space dynamics:

```
M_effective(q) = M_link(q)   +   armature
                 ↑                ↑
       from the MJCF <inertial>   from this file
       configuration-dependent    constant, drivetrain only
```

> **The single most important idea in this document.**
> `armature` is the motor's own inertia. `M_link` is the limb's inertia, and MuJoCo
> already has it from your URDF/MJCF. They are complementary halves — putting the
> limb into `armature` double-counts it.

Two fields the policy never sees but you should still fill in:

- `frictionloss` — dry friction. Set it from the datasheet's **backdrive torque** if you
  have it; it makes the sim noticeably more honest about small motions. Default 0.
- `velocity_limit` on `ElectricActuator` — **currently unused by this pipeline.**
  `BuiltinPositionActuatorCfg` has no velocity field. Record it for documentation and
  for the Kd check in §8, but MuJoCo will not enforce a speed ceiling. If your motor's
  speed limit genuinely binds, you need `DCMotorActuatorCfg` instead.

---

## 2. Which datasheet numbers you need

Only five rows matter. Everything else on a datasheet is electrical and irrelevant here.

| # | What to look for | Datasheet aliases | Used for | Units you need |
|---|---|---|---|---|
| 1 | **Reduction ratio** | gear ratio, `N`, "1:20" | armature | – |
| 2 | **Moment of inertia** | rotor inertia, `Jm`, `J_rotor` | armature | kg·m² |
| 3 | **Peak torque** | max/instantaneous/stall torque | `effort_limit`, action scale | Nm |
| 4 | **Rated torque** | continuous/nominal torque | thermal budget (§6) | Nm |
| 5 | **No-load speed** | max speed, rated speed | Kd sanity check (§8) | rad/s |

Optional but useful: **backdrive torque** → `frictionloss`; **backlash** → tells you the
useful upper bound on Kp; **torque constant Kt** → lets you cross-check rows 3 and 4.

### Unit conversions (this is where people get burned)

```
1 kg·cm²  = 1e-4 kg·m²          ← most common on servo-module datasheets
1 g·cm²   = 1e-7 kg·m²
1 kg·mm²  = 1e-6 kg·m²
1 rpm     = 2π/60 = 0.10472 rad/s
1 arcmin  = 2.909e-4 rad
```

### Cross-checks that catch a misread datasheet

Run these before trusting anything:

```
τ_peak  ≈ Kt × I_peak                   (torque constant × peak phase current)
ω_noload ≈ V_bus / Ke / N               (Ke in V/krpm → rpm, then ÷ gear ratio)
P_rated ≈ τ_rated × ω_rated             (should match the listed rated power)
```

If all three land within ~10%, your reading of the table is right.

---

## 3. Step 1 — armature (reflected inertia)

### The physics

Inertia seen through a reduction of `N` is multiplied by `N²`, because torque scales by
`N` and speed divides by `N`. Each rotating mass is reflected by **only the gearing
downstream of it**.

**Single stage** (harmonic drive, cycloidal, belt, most servo modules):

```python
from mjlab.utils.actuator import reflected_inertia
armature = reflected_inertia(rotor_inertia=J_m, gear_ratio=N)      # = J_m · N²
```

**Two-stage planetary** (Unitree-style):

```python
from mjlab.utils.actuator import reflected_inertia_from_two_stage_planetary
armature = reflected_inertia_from_two_stage_planetary(
    rotor_inertia=(J_rotor, J_stage1, J_stage2),
    gear_ratio=(1, n1, n2),      # element 0 must be 1 — the rotor has no gearing before it
)
# = J_rotor·(n1·n2)² + J_stage1·n2² + J_stage2
```

For a planetary stage with sun input and carrier output, `n = 1 + Z_ring / Z_sun`.

**The rotor dominates.** On every G1 motor the rotor accounts for ~98.6% of the total:

```
5020:   rotor 0.003558 (98.6%)   stage1 0.000034 (1.0%)   stage2 0.000017 (0.5%)
7520-22: rotor 0.024756 (98.6%)  stage1 0.000273 (1.1%)   stage2 0.000074 (0.3%)
```

So if you only know the rotor inertia and the total ratio, `J_m · N²` is within ~1.5%.
Do not stall your project hunting for per-stage carrier inertias.

### ⚠ Rotor-side or output-side? — resolve this first

Datasheets are inconsistent about whether "moment of inertia" is referred to the **motor
shaft** (before the gearbox) or to the **output** (after). The two differ by `N²` —
a factor of 400 on a 20:1 unit. Get it wrong and every gain is wrong by 400×.

**Three tests that settle it without asking the vendor:**

**Test A — implied rotor size.** If the number were output-side, the implied rotor
inertia is `I_listed / N²`. Ask whether that is a plausible rotor for a motor of this
power and mass. A 900 W, 2.4 kg module cannot have a 3.2e-6 kg·m² rotor — that's a
hobby-servo shaft.

**Test B — torque-to-inertia.** Compute `τ_peak / armature`. A geared actuator lands
in **500–8000 rad/s²**. Above ~50,000 means you've under-counted the inertia
(you read an output-side number as if it were the full story, or forgot `N²`).

**Test C — the standing test.** Size a knee gain with `Kp = armature · ω²` at your
control-rate ceiling, then divide the joint's static gravity torque by it. If the
implied sag exceeds a few degrees, the armature is too small by orders of magnitude.

| | rotor-side reading | output-side reading |
|---|---|---|
| armature | `I_listed · N²` | `I_listed` |
| τ/armature | 500–8000 rad/s² ✓ | 10⁵–10⁶ rad/s² ✗ |
| implied rotor | the listed number ✓ | `I_listed/N²` — usually absurd ✗ |

**If in doubt, the rotor-side reading is nearly always the correct one**, because
moment of inertia is a *motor* spec while torque and speed are *module* specs.
When the two readings disagree, §5 tells you what changes: with a large armature the
drivetrain sets the gains; with a small one the **load** does, and you must use the
general formula rather than G1's shortcut.

---

## 4. Step 2 — measure the load inertia

You need this for two reasons: to know whether armature or load dominates, and to
compute the gains correctly when it's the load. **Do not estimate it by hand** — get it
out of the compiled model. See the [audit script](#11-audit-script).

There are two meaningful numbers, and they differ a lot:

| measure | formula | meaning |
|---|---|---|
| **locked** | `M[i,i]` | inertia felt when all other joints are held rigid |
| **free** | `1 / (M⁻¹)[i,i]` | inertia felt when the neighbours can move |

**Use the free (operational-space) value.** A swinging leg is not a locked chain, and
`M[i,i]` overstates it badly — on G1's hip pitch, locked gives 0.90 kg·m² against a free
value of 0.026 kg·m², a 34× difference that would send you to completely wrong gains.

Evaluate it at your **nominal standing pose**, not at `qpos = 0`.

---

## 5. Step 3 — choose ω and ζ

### What they are

`Kp` and `Kd` are hardware numbers — their meaning depends on the joint. `Kp = 40` is
bone-rigid on a wrist and floppy on a knee. `ω` and `ζ` are behaviour numbers — they mean
the same thing on every joint of every robot.

```
      you think in                the simulator needs
   ┌──────────────────┐         ┌──────────────────┐
   │  ω = how fast    │  ────►  │  Kp = I·ω²       │
   │  ζ = how bouncy  │         │  Kd = 2ζIω       │
   └──────────────────┘         └──────────────────┘
                      ▲
                      └── I supplies the per-joint hardware scale
```

From the standard second-order system `I·q̈ + Kd·q̇ + Kp·q = 0`:

```
ω = √(Kp / I)              →   Kp = I·ω²
ζ = Kd / (2·√(Kp·I))       →   Kd = 2·ζ·I·ω
```

**Which `I`?** This is the part G1's file glosses over. The correct reference inertia is
the **total** at the joint:

```
I_ref = armature + I_load
```

`g1_constants.py` uses `armature` alone. That works *only because* G1's armature is
comparable to its load (ratios of 0.3–7). It is a shortcut, not the general rule.
**Use `I_ref = armature + I_load` and the formula is correct in every regime**, degrading
gracefully to G1's version when the armature dominates.

### What each parameter does

**ω — speed and stiffness.** `Kp ∝ ω²`, `Kd ∝ ω`. Higher ω = snappier tracking, less sag
under load, crisper footstrike. Costs: bigger torque spikes, motor whine and chatter on
real hardware, more sensitivity to latency and sensor noise, wider sim2real gap.
Lower ω = compliant, safe, impact-absorbing, forgiving of model error — at the cost of
sag and sloppy tracking. Note it is **quadratic** in Kp; it's a strong knob.

**ζ — approach character.** `Kd ∝ ζ`.

```
ζ < 1   underdamped   ──╮ ╭─╮ ╭──   overshoot, ringing, bouncy legs
                        ╰─╯ ╰─╯
ζ = 1   critical      ──╭────────   fastest arrival without overshoot
                        ╯
ζ > 1   overdamped    ──╭╌╌╌╌╌╌──   no overshoot, sluggish, "through honey"
                      ╯
```

Damping is also your shock absorber at footstrike. Under-damped legs on a hard floor
chatter, and the policy learns strange stomping gaits to exploit it.

### Choosing ω — three ceilings, take the lowest

**Ceiling 1 — control rate (hard).** A discrete PD loop needs ~5 samples per closed-loop
period:

```
ω_max = 2π · f_control / 5
```

With this repo's defaults (`timestep=0.005`, `decimation=4` → 50 Hz) that is **10 Hz**.
Recompute it if you change either.

**Ceiling 2 — actuator torque-to-inertia (usually the real one).** Define the
**saturation error** — the tracking error at which the PD demands full torque:

```
θ_sat = τmax / Kp = τmax / (I_ref·ω²) = (τmax / I_ref) / ω²
```

so

```
        θ_sat · ω²  =  τmax / I_ref   ←  fixed by your hardware
```

**You can trade stiffness for range, but not escape the product.** If `θ_sat` is small,
the motor saturates on tracking errors that occur constantly in normal walking, and the
usable action range collapses. Target **θ_sat ≈ 1.0–2.0 rad**, which gives

```
ω_actuator = √( (τmax / I_ref) / 1.4 )
```

**Ceiling 3 — what the real hardware tolerates.** Backlash, belt compliance, encoder
resolution, bus latency. If your vendor ships working gains, back out what ω they imply
(`ω = √(Kp_vendor / I_ref)`) and treat that as the ceiling. Don't train at a stiffness
your robot cannot execute.

Then:

```
ω = min(ceiling 1, ceiling 2, ceiling 3)
```

Robot-scale sanity check (big things move slowly, roughly `ω ∝ 1/√L`):

| robot class | typical ω |
|---|---|
| small quadruped / low-inertia arm | 12–20 Hz |
| human-scale humanoid | 8–12 Hz |
| large or heavy humanoid | 3–8 Hz |

### Choosing ζ

Your target is the **loaded** damping ratio, not the number you type. If you size gains
with `I_ref = armature + I_load` as above, then `ζ_typed = ζ_effective` directly and you
should type **ζ ≈ 1.0–1.3**.

> G1's `DAMPING_RATIO = 2.0` looks wrong until you notice it sizes gains against
> `armature` alone, ignoring the load. The load then drags the effective ratio down by
> `√(I_a/(I_a+I_l))` ≈ 0.5, landing at ζ_eff ≈ 1.0. **The 2.0 is pre-compensation for a
> term the formula omitted.** If you include the load term, don't also inflate ζ — you'd
> be correcting twice.

Push ζ toward 1.3 for joints that absorb impact (ankles, knees); pull toward 1.0 if the
Kd torque check in §8 is tight.

### Per-motor or per-robot?

**ω and ζ are per-robot behavioural targets. Motor differences enter through
`armature`, not through ω.** Giving each motor its own ω undoes the normalization.

Split them per joint group only when there's a concrete reason:

- **Two motor types with very different `τ/I`.** If they differ by more than ~2×, one
  global ω leaves one group too soft or the other too stiff. Give each group the ω that
  puts it at the same `θ_sat`.
- **Different transmission types.** A belt or cable joint has real compliance and cannot
  hold the same bandwidth as a harmonic drive.
- **Arms vs legs.** Arms take no ground impact and can be softer.

---

## 6. Step 4 — effort limit

Use the **peak** torque as `effort_limit`. It is the true hardware clamp, it matches what
the URDF usually carries, and it gives the policy the headroom to recover from
disturbances.

**But peak is a burst rating** — typically a couple of seconds before thermal limits
bite. Rated torque is what you can hold forever. So:

1. Set `effort_limit = τ_peak`.
2. Compute the static gravity torque at your nominal pose (the audit script does this)
   and compare it to **τ_rated**. If standing alone eats more than ~50% of rated, make
   the default pose shallower.
3. During training, watch the actual torques. If the policy lives near saturation, add a
   torque or power penalty reward rather than lowering `effort_limit` —
   lowering it also shrinks the action scale (§7) and couples two unrelated decisions.

---

## 7. Step 5 — action scale

The policy outputs a normalized action `a`; the joint target is
`q_target = q_default + scale · a`.

```python
scale = 0.25 · τmax / Kp        #  = 0.25 · θ_sat
```

**Why:** `τ = Kp · scale · a = 0.25·τmax·a`. The Kp cancels, so a ±1 action commands ±25%
of peak torque on *every* joint regardless of its gains. The knee and the wrist become
equally "loud" to PPO. The 0.25 leaves 4× headroom to saturate when needed.

**Two caps to apply afterwards:**

```python
scale = min(0.25 · τmax / Kp,  0.4 · joint_range)
```

- **Never exceed ~40% of the joint's range of motion.** A scale of 3.2 rad on a joint
  with a ±0.17 rad limit is meaningless — the policy spends its whole action range
  commanding targets outside the hard stops.
- **Keep it above ~0.1 rad.** Below that the policy cannot reach the joint angles a gait
  needs, and PPO's exploration (calibrated around `std ≈ 1`) is badly scaled. If you land
  under 0.1 rad, your Kp is too high — go back to §5.

Healthy range: **0.15–0.5 rad**.

---

## 8. Sanity-check list

Run all of these *before* burning GPU hours.

| # | Check | Pass condition | If it fails |
|---|---|---|---|
| 1 | `τ_peak / armature` | 500–8000 rad/s² | misread inertia (§3) |
| 2 | `I_load / armature` | 0.1–10 (light distal joints may sit lower) | one term is wrong by `N²` |
| 3 | `θ_sat = τmax/Kp` | 1.0–2.0 rad | adjust ω |
| 4 | `ω ≤ 2π·f_control/5` | – | lower Kp |
| 5 | `action_scale` | 0.15–0.5 rad, ≤ 0.4× joint range | §7 |
| 6 | **Kd torque cost:** `Kd · v_gait` (`v_gait` ≈ 5 rad/s) | < 0.5·τmax | lower ζ |
| 7 | **Kd at top speed:** `Kd · v_noload` | < τmax | lower ζ, or accept a speed ceiling |
| 8 | Static gravity torque at nominal pose | < 0.5 · τ_rated | shallower default pose |
| 9 | Backlash torque step: `Kp · backlash_rad` | < 0.05 · τmax | Kp too high for this gearbox |
| 10 | **Base-welded hold test** (weld the root to the world, settle 5 s) | joint errors < 1°, no buzz, no solver blowup | see below |
| 11 | `play.py --agent zero` | quadruped: stands. **biped: topples — expected**, see below | fix only if it explodes or self-collides |

### Why a biped falls over under zero actions — and why that is fine

A joint-space PD controller has no balance feedback. The robot is an inverted pendulum
pinned at the ankles, so it is statically stable only if

```
2 · Kp_ankle  >  M · g · h_com
```

Almost no real biped satisfies this — the ankle motor would have to be enormous:

| robot | `2·Kp_ankle` | `M·g·h` | ratio |
|---|---|---|---|
| tahiti_c1 | 85.8 Nm/rad | 357.0 Nm/rad | 0.24 |
| Unitree G1 | 57.0 Nm/rad | 206.0 Nm/rad | 0.28 |

**Toppling under zero actions is correct behaviour** — balance is what the policy learns.
Do not raise your ankle gains trying to fix it; you would only saturate the ankle motor.

So for a biped, use the **base-welded** test (check 10) to validate the gains, and judge
check 11 on *how* it falls, not *that* it falls:

- ✅ tips over smoothly as a rigid body, joint errors staying small
- ❌ explodes, jitters, limbs interpenetrate, or a joint runs away → a real bug

A useful ratio: if yours is far **below** ~0.2, your robot is unusually top-heavy or
ankle-starved and the policy will struggle. Around 0.25 (both robots above) is normal.

**Symptom → knob**

| What you see | Fix |
|---|---|
| Sags, can't hold pose, crouches | ↑ ω |
| High-frequency jitter, buzzing, huge action-rate penalty | ↓ ω |
| Legs bounce/ring after footstrike | ↑ ζ |
| Motion sluggish, robot "wades" | ↓ ζ |
| Fine in sim, chatters on hardware | ↓ ω (latency/backlash the sim omits) |
| Policy saturates torque constantly | ↑ θ_sat (↓ ω), or add a torque penalty |

Also revisit the **gait period** (`foot_gait.period` and the `phase` observation,
default 0.6 s). Your loaded bandwidth should be ≥ 2× the gait frequency. A heavy robot
with 3 Hz bandwidth wants 0.75–0.85 s, not 0.6 s.

---

## 9. Worked example — tahiti_c1

**Robot:** 12 DOF biped, 53.52 kg, no upper body. Thigh 0.402 m, shank 0.402 m,
hip-to-ankle 0.913 m. Base sits 0.891 m above the ankle at the nominal pose.
75% of mass is in the legs (a human is ~32%) — expect swing-leg inertia to be the
dominant trunk disturbance, so weight `angular_momentum` and `body_ang_vel` harder
than G1 does.

**Actuators** (MyActuator), assigned by the URDF's `effort` field:

| | X12-320 → hip yaw/pitch/roll, knee | X6-60 → ankle pitch/roll |
|---|---|---|
| Reduction | 20 : 1 | 19.612 : 1 |
| Listed inertia | 12.9 kg·cm² = 1.29e-3 kg·m² | 0.66 kg·cm² = 6.6e-5 kg·m² |
| Rated / peak torque | 85 / 320 Nm | 20 / 60 Nm |
| No-load speed | 125 rpm = 13.09 rad/s | 176 rpm = 18.43 rad/s |
| Backlash | ≤15 arcmin = 0.0044 rad | ≤15 arcmin |
| Backdrive torque | 3.8 Nm | 1.6 Nm |

Datasheet cross-checks pass: `320/100 A = 3.2` vs listed Kt 3.3 ✓;
`48V / 17.9 V·krpm⁻¹ / 20 = 134 rpm` vs listed 125 ✓.

### The inertia reading is taken as rotor-side

**`armature = 12.9e-4 · 20² = 0.5160` (X12) and `0.66e-4 · 19.612² = 0.02539` (X6).**

This is the reading all three tests in §3 support:

| test | rotor-side (adopted) | output-side (rejected) |
|---|---|---|
| A — implied rotor inertia | 12.9 kg·cm², plausible for a 2.37 kg / 900 W module | 0.032 kg·cm² — a hobby-servo shaft |
| B — `τ_peak / armature` | 620 and 2364 rad/s² ✓ in band | 248,000 and 909,000 rad/s² — better than direct drive |
| C — standing test | knee holds with 3° sag | knee Kp 5.09 Nm/rad vs 51 Nm load → 574° sag |

If MyActuator ever contradicts this, nothing in §1–§8 changes — only this section. Swap
the two `armature` values in the audit script and rerun: `Kp` barely moves (it is set by
torque and range, not inertia), `Kd` drops ~2.5×, and the clean two-group split below
fragments into six per-joint groups.

### Measured inertia at the nominal pose

Pose: `hip_pitch −0.225, knee +0.45, ankle_pitch +0.225` (a shallow crouch — a 0.6 rad
crouch puts the knee at 60% of its *rated* torque just standing).

```
joint                armature    I_load     I_ref    I_load/I_a    gravity
hip_yaw               0.51600    0.0411    0.5571      0.08          0.0 Nm
hip_pitch             0.51600    0.1482    0.6642      0.29         15.4 Nm
hip_roll              0.51600    0.6210    1.1370      1.20          0.0 Nm
knee                  0.51600    0.0819    0.5979      0.16         35.8 Nm
ankle_pitch           0.02539    0.0088    0.0341      0.35         11.8 Nm
ankle_roll            0.02539    0.0011    0.0265      0.04          0.0 Nm
```

**The motor dominates the load at every joint.** Consequence: `I_ref` spans only 2.0×
within the X12 group and 1.3× within the X6 group, so **one (Kp, Kd) pair per motor type
works** — the armature is what makes joints in a group behave alike. (Contrast: with an
output-side armature these ratios would span 500× and you'd need six separate groups.)

This is the opposite regime from G1, where the load dominated. It means `ζ_typed ≈ ζ_eff`
here, so **do not inflate ζ the way `g1_constants.py` does** — see §5.

### Resulting gains — θ_sat target 1.4 rad, ζ = 1.0, two groups

`Kp = τmax / 1.4`, `Kd = 2ζ·√(Kp·I_ref_median)`, `scale = min(0.25·τ/Kp, 0.4·range)`

| group | joints | Kp | Kd | effort | armature | frictionloss | scale |
|---|---|---|---|---|---|---|---|
| **X12-320** | hip yaw/pitch/roll, knee | **228.6** | **24.02** | 320 | 0.51600 | 3.8 | 0.350 |
| **X6-60** | ankle pitch/roll | **42.9** | **2.28** | 60 | 0.02539 | 1.6 | 0.350\* |

\* `ankle_roll` caps to **0.140** on its ±0.17 rad range (§7).

Per-joint behaviour that results:

| joint | ω | ζ_eff | θ_sat |
|---|---|---|---|
| hip_yaw | 3.22 Hz | 1.06 | 1.40 rad |
| hip_pitch | 2.95 Hz | 0.97 | 1.40 rad |
| hip_roll | 2.26 Hz | 0.74 | 1.40 rad |
| knee | 3.11 Hz | 1.03 | 1.40 rad |
| ankle_pitch | 5.65 Hz | 0.94 | 1.40 rad |
| ankle_roll | 6.40 Hz | 1.07 | 1.40 rad |

(Run the [audit script](#11-audit-script) rather than copying these — they track your
model as it changes.)

**Notes and caveats**

- **All checks pass.** Check 4: every ω is ≤ 6.4 Hz, well under the 10 Hz ceiling.
  Check 6: `24.02 × 5 = 120 < 160` ✓. Check 7: `24.02 × 13.09 = 314 < 320` ✓ — only just,
  so **do not raise ζ above 1.0 on the X12 group**. Check 9: `228.6 × 0.0044 = 1.0 Nm`
  backlash step, 0.3% of peak ✓.
- **`hip_roll` runs at ζ_eff = 0.74** — slightly under-damped, because it carries 2× the
  group's median inertia. This is deliberate: giving it proper damping needs Kd = 32.2,
  which fails check 7 (`32.2 × 13.09 = 422 > 320`). Mild under-damping beats torque
  saturation. If it rings during training, split it into its own group and accept a
  lower ω instead.
- `hip_yaw` and `ankle_roll` show `I_load/I_a` of 0.08 and 0.04 — the motor almost
  entirely dominates. Normal for light distal joints; not a problem.
- **Ankle pitch is the tightest actuator on the robot:** 11.8 Nm static out of 20 Nm
  rated, and push-off / CoP shifting needs `W × foot_half_length` ≈ 30 Nm — over rated,
  inside peak. Check this against your intended foot length.
- **Gait period:** loaded bandwidth is 2.3–3.2 Hz at the hip. At the default 0.6 s
  period (1.67 Hz) the hip_roll margin is only 1.35×. Set `foot_gait.period` and the
  `phase` observation to **0.8–0.85 s** (1.18–1.25 Hz), giving ~1.9× margin.
- Set `frictionloss` to 3.8 (X12) and 1.6 (X6) from the backdrive torque.

### ⚠ The foot is missing from the mesh

`scripts/check_actuators.py` flags this automatically:

```
left_ankle_roll_link   inertia 0.235 x 0.096 x 0.034   mesh 0.067 x 0.120 x 0.290  <-- MISMATCH
```

The URDF's **inertial tensor** for `ankle_roll_link` is equivalent to a solid box
**0.235 × 0.096 × 0.034 m** — a proper foot, toe 148 mm forward and heel 87 mm behind the
ankle. The **STL contains no such geometry**: its largest x extent is 0.067 m, and its
long axis is vertical. The foot plate was left out of the mesh export.

Consequences:

- **All gains in this document are unaffected.** They derive from inertials and the mass
  matrix, never from meshes.
- The MJCF's foot collision box is reconstructed from that inertia tensor
  (`FOOT_CENTER` / `FOOT_HALF` in `scripts/urdf_to_mjcf.py`). It is a well-founded
  estimate, not a guess — but **replace it with real CAD dimensions when available**,
  since the support polygon drives everything about balance.
- Standing height follows from it: sole at **0.920 m** below the base at the nominal
  pose, so `init_state.pos = (0, 0, 0.92)`.

### MJCF conversion — done

`python scripts/urdf_to_mjcf.py` generates `xmls/tahiti_c1.xml`:
14 bodies, 18 DOF, 38 geoms, 3 sites, 4 sensors, 53.52 kg.

- collision geoms `*_collision`; feet as `left_foot1_collision` / `right_foot1_collision`
  (box, condim 3, priority 1, friction 0.6) matching G1's `foot[1-7]` regex convention
- sites `imu_in_pelvis`, `left_foot`, `right_foot` (at the sole)
- sensors `imu_ang_vel`, `imu_lin_vel`, `imu_lin_acc`, `root_angmom`
- **all collision geoms are `contype=0, conaffinity=1`** — terrain-only. The capsules are
  fitted to visual-mesh bounding boxes and adjacent joint housings overlap (`hip_yaw`
  interpenetrates `hip_roll` by 28 mm, and they are not a parent-child pair so MuJoCo
  does not exclude them). Tighten the capsules to real CAD envelopes before enabling
  self-collision.

Verified: compiles standalone; no contacts at the nominal pose; base-welded settle gives
joint errors < 0.3°; free-base topple is smooth and rigid-body (see §8 check 11).

### Still needed before this robot can train

1. **Real foot CAD dimensions**, replacing the inertia-derived box above.
2. Confirm the nominal home pose with mechanical (currently hip_pitch −0.225,
   knee +0.45, ankle_pitch +0.225).
3. `c1_constants.py` — the actuator groups from this section, `InitialStateCfg` at
   z = 0.92, and a `CollisionCfg`.
4. The three task-config files under `src/tasks/velocity/config/tahiti_c1/`, with
   `foot_gait.period` raised to 0.8–0.85 s.

---

## 10. Reference — how G1's numbers were chosen

Useful as a calibration target, and it shows the method working backwards.

```
motor      total_gear   armature       Kp      Kd  τ_peak  action_scale  θ_sat
5020            16.00    0.00361    14.25   0.907    25.0        0.4386   1.75 rad
7520_14         14.32    0.01018    40.18   2.558    88.0        0.5475   2.19 rad
7520_22         22.50    0.02510    99.10   6.309   139.0        0.3507   1.40 rad
4010            25.00    0.00425    16.78   1.068     5.0        0.0745   0.30 rad
2×5020          16.00    0.00722    28.50   1.814    50.0        0.4386   1.75 rad
```

**G1's 10 Hz was itself reverse-engineered from θ_sat.** Its knee motor has
`τ/I = 139/0.0251 = 5538 rad/s²`, and `5538 / (2π·10)² = 1.40 rad`. Every leg joint lands
at 1.4–2.2 rad. The 10 Hz is not a physics constant — it is the ω that produces a ~1.4 rad
saturation error given those motors, and it happens to coincide with the 50 Hz
control-rate ceiling.

Loaded behaviour, measured at G1's home pose with the free-inertia measure:

```
joint                    armature   I_free   f_eff    ζ_eff
hip_pitch                  0.0102   0.0264   5.27 Hz   1.05
hip_roll                   0.0251   0.0514   5.73 Hz   1.15
knee                       0.0251   0.0259   7.02 Hz   1.40
ankle_pitch                0.0072   0.0022   8.76 Hz   1.75
shoulder_pitch             0.0036   0.0260   3.49 Hz   0.70
elbow                      0.0036   0.0042   6.78 Hz   1.36
```

ζ_eff clusters at 1.0–1.7 — lightly overdamped, with impact-bearing joints crisper and
distal joints cushioned. That is the distribution you are aiming to reproduce.

---

## 11. Audit script

`scripts/check_actuators.py` prints everything §8 asks for. Robot presets live in the
`ROBOTS` dict at the top; design targets (`F_CONTROL`, `THETA_SAT_TARGET`, `ZETA`,
`V_GAIT`) are module constants.

```bash
python scripts/check_actuators.py --robot tahiti_c1
python scripts/check_actuators.py --robot g1          # reference numbers
```

It reports three blocks:

1. **Per-joint** — armature, measured `I_load`, `I_ref`, the ideal Kp/Kd/ω/θ_sat/scale
   for that joint alone, the static support torque, and any failed checks as flags.
2. **Grouped** — what actually goes into `<robot>_constants.py`: one Kp/Kd per motor
   type using the group's median `I_ref`, with the resulting per-joint ω and ζ_eff.
   It warns when a group's `I_ref` spread exceeds 3×, meaning you should split it.
3. **Mesh / inertia consistency** — the solid box equivalent to each link's inertia
   tensor next to its mesh bounding box. A `MISMATCH` means the STL and the inertial
   data describe different objects; see the tahiti_c1 foot in §9 for a live example.

**Per-joint vs grouped.** Block 1 shows the ideal Kd for each joint in isolation; your
config takes one Kd per group. That is why tahiti_c1 flags `Kd-vmax` on hip_pitch and
hip_roll in block 1 while the grouped `Kd = 24.02` passes — the group value is lower
than the per-joint ideal for the heavy joints, exactly the trade §9 describes.

**Re-run it whenever the model changes.** `I_load` is measured from the compiled model,
so every gain goes stale silently when a mass, a link or the home pose moves.

---

## Quick reference

```
armature   = J_rotor · N²                         (§3, verify rotor- vs output-side)
I_ref      = armature + I_load                    (§4, free/operational inertia)
ω          = min( 2π·f_ctrl/5 ,  √((τmax/I_ref)/1.4) ,  hardware limit )
ζ          = 1.0 – 1.3                            (when I_ref includes the load)
Kp         = I_ref · ω²
Kd         = 2 · ζ · I_ref · ω
θ_sat      = τmax / Kp                            → want 1.0–2.0 rad
scale      = min( 0.25·τmax/Kp , 0.4·joint_range) → want 0.15–0.5 rad
```
