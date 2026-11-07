# 0010. Hover thrust: normalised collective, battery-compensated mixer, online estimate

- **Status:** Proposed
- **Date:** 2026-11-07

## Context

The vertical loop (`App/control/alt_ctrl.c`) computes

$$
T = \frac{T_h + k_{p,v}\,(v_{sp} - v) + I}{\cos\theta}
$$

where $T_h$ is the hover collective and $I$ the integrator. The flight core
(`fc_core.c`) also uses $T_h$ in three other places:

- the centre of the throttle stick in the manual modes (`collective()`);
- the "airborne" test ($T > 0.8\,T_h$);
- the open-loop LAND ($0.85\,T_h$).

Today:

| Item | Value | Where |
|---|---|---|
| Collective unit | PWM duty 0…1 (the mixer of phase 3 is not written yet) | `fc_core.h` |
| Initial hover | `FC_HOVER_DEFAULT` = 0.45 | `fc_core.h` |
| Learned hover range | `FC_HOVER_MIN` … `FC_HOVER_MAX` = 0.15 … 0.75 | `fc_core.h` |
| Learning | first-order low-pass of $T\cos\theta$, τ = 3 s, only while $\lvert v\rvert$ < 0.15 m/s | `learn_hover()` |
| Integrator | $\lvert I\rvert \le$ 0.35, collective clamped to 0.05 … 0.90 | `alt_ctrl.c` |

**What the drone needs** (Mech A1, 2026-11-07; 4 × 8520 motors, 55 mm propellers, ≈ 140 g of static thrust on a full
1S pack; thrust ≈ quadratic in $u V_{bat}$, [vehicle model](../theory/vehicle-model.md#brushed-motor-dynamics)):

| Configuration | Mass | T/W | Hover thrust / max | Hover duty, 4.2 V | Hover duty, 3.7 V | Hover duty, 3.5 V |
|---|---|---|---|---|---|---|
| T400 indoor | 71 g | 1.97 | 0.51 | 0.71 | 0.81 | 0.86 |
| T800 indoor | 82 g | 1.71 | 0.58 | 0.76 | 0.86 | 0.91 |
| T1200 indoor | 90 g | 1.55 | 0.65 | 0.80 | 0.91 | 0.96 |

The duty is $u_h = \sqrt{T_h/T_{max}}\cdot 4.2/V_{bat}$: speed ∝ $uV$, thrust ∝ speed², 140 g measured at 4.2 V. These are
estimates until the thrust stand gives $k_T$ and the speed curve.

Problems with the current scheme:

1. **Default far from the truth.** At the first take-off the integrator has to cover 0.45 → 0.71 (T400). That is
   0.26 of the 0.35 limit, and it ends in a slow, sagging lift-off.
2. **The clamp is below the real hover.** `FC_HOVER_MAX` = 0.75 is below the hover duty of T800 and T1200 at any
   charge, and of T400 below ≈ 3.95 V. The learned value then stops at 0.75:
   - in ANGLE the stick centre gives less than hover, so the drone sinks with the stick centred;
   - "airborne" and the open-loop LAND scale wrongly.
3. **The learning is biased.** A low-pass of the command is right only in steady hover. Each climb or descent
   inside the 0.15 m/s window pulls the estimate, and so does the ground effect at take-off. It also learns
   nothing outside the window (gusts, manoeuvres).
4. **The loop gain changes with the battery and the thrust level.** In duty units the thrust per unit of command
   ($\partial T/\partial u \propto u V^2$) changes by about 40 % from a full to an empty pack. The same $k_{p,v}$ is
   then sluggish on a full pack and aggressive on an empty one.
5. **Little margin.** With `thr_max` = 0.90 in duty, T1200 below 3.7 V has no authority left to climb or reject a
   gust.

## Decision

### 1. Collective in normalised thrust, linearised in the mixer

The controllers work in **normalised thrust** $t \in [0, 1]$: thrust divided by the maximum thrust at the reference
voltage $V_{ref}$ = 4.2 V. The mixer converts each rotor thrust $t_i$ to a duty:

$$
u_i = \underbrace{f^{-1}(t_i)}_{\text{thrust curve}} \cdot \underbrace{\frac{V_{ref}}{\bar V_{bat}}}_{\text{battery}},
\qquad f(u') = \theta\,u'^2 + (1-\theta)\,u'
$$

- $\theta$ (new parameter **`thrCurve`**, stable id **81**, 0…1, default **1.0** = quadratic) is identified on the
  thrust stand. Inverse:
  $u' = \dfrac{-(1-\theta) + \sqrt{(1-\theta)^2 + 4\theta t}}{2\theta}$ (or $u' = t$ for $\theta = 0$).
- $\bar V_{bat}$ is the BattTask voltage **under load**, low-passed (τ ≈ 0.5 s), clamped to 3.0 … 4.35 V. If the
  battery is not known, no compensation is applied.
- $u_i$ is then saturated to 0 … 1. When a rotor saturates, roll/pitch keep priority over the collective, as in the
  usual airmode mixers.

Everything above the mixer (`alt_ctrl`, `collective()`, the airborne test, LAND) then uses $t$, which is
proportional to thrust and independent of the battery. The loop gain stays constant over the flight.

### 2. Online hover-thrust estimate (1-state Kalman filter)

New module `App/filters/hover_est/hover_est.c`, host-testable like `alt_est`. State: $\hat t_h$, the hover
thrust in normalised units. In the vertical axis, with $t_c = t\cos\theta$ the vertical part of the commanded
thrust:

$$
a_{up} = g\left(\frac{t_c}{t_h} - 1\right) \quad\Rightarrow\quad
z = a_{up},\quad h(\hat t_h) = g\left(\frac{t_c}{\hat t_h} - 1\right),\quad
H = -\frac{g\,t_c}{\hat t_h^{2}}
$$

- **Measurement $a_{up}$:** the vertical acceleration already computed for `alt_est` (`alt_est_accel_up()`),
  minus the estimated accelerometer bias $b_a$ (`alt_est.x[3]`), low-passed at ≈ 10 Hz.
- **Command $t_c$:** the collective sent to the mixer one actuator lag earlier ($\tau_m$, tens of ms), so that the
  command and its effect are aligned.
- **Predict:** $P \mathrel{+}= q\,\Delta t$, with $\sqrt q$ = 0.0036 s$^{-1/2}$. A battery sag is mostly removed by the
  mixer compensation, so what is left is slow.
- **Update at 50 Hz** with $R = \sigma_a^2$, $\sigma_a$ ≈ 2 m/s² (vibration with the props on; tune from the logs).
  - Innovation gate at NIS > 9: reject the sample and inflate R ×4 for 0.5 s, so impacts and hard manoeuvres do not
    pull the estimate.
- **Run only when the model holds:**
  - in air, and more than 0.5 s after the take-off spool-up;
  - not on the ToF ground flag;
  - tilt < 30°, $t$ not saturated, below `gfCeiling`.
  
  The ground effect below ≈ 2 propeller diameters (11 cm) biases $t_h$ low. Skip the update when the ToF height
  is under 0.15 m.
- **Clamp** $\hat t_h$ to 0.25 … 0.85. If it leaves 0.35 … 0.75, send a WARN event: wrong tray parameter, lost
  propeller, or a damaged motor.

The cost is about 30 floating-point operations per update.

`learn_hover()` is replaced by this estimator. `alt_ctrl` uses $\hat t_h$ as its hover term, and its integrator limit
drops from 0.35 to **0.15**: it only has to cover the model error left over, not the whole hover value.

### 3. Initial value and persistence

- New parameter **`hoverThr`** (stable id **80**; 70 is `logMode`), default **0.51** (T400 indoor), range 0.25 … 0.85, unit:
  normalised thrust.
  - The app sets it from the tray model: T400 0.51, T800 0.58, T1200 0.65, and +0.04 with the GPS board (≈ +5 g).
  - It is the seed at every arm.
- At disarm after a flight of ≥ 20 s, the learned $\hat t_h$ is stored in RAM and used as the seed of the next
  flight in the same power cycle, as today. It is **not** written to flash: a tray swap needs the parameter, not a
  stale learned value.

### 4. Limits

- Collective limits in normalised thrust: `thr_min` 0.05, `thr_max` **0.95**. The duty saturation of the mixer is
  the real limit.
- Pre-flight CHECKS item "thrust margin": WARN if
  $\hat t_h \cdot (V_{ref}/\bar V_{bat})^2 > 0.80$, that is less than 25 % of the thrust left for control. With the
  table above this flags T1200 below ≈ 3.8 V and T800 below ≈ 3.6 V; T400 stays above the limit down to 3.35 V.

### 5. Logging

Add to the flight log: $\hat t_h$, $\sqrt P$, NIS, the battery compensation factor $V_{ref}/\bar V_{bat}$ and the
duty of each motor. With these, the thrust curve and $\sigma_a$ can be checked from real flights.

## Consequences

- The take-off reaches the hover thrust within the first second instead of winding up the integrator.
  Mode switches between ANGLE and ALT_HOLD are smooth over the whole battery, because the stick centre follows
  $\hat t_h$ in thrust units.
- The vertical gains are tuned once and hold from 4.2 V down to the critical voltage.
- A heavier tray, the GPS board or a damaged propeller show up as a change in $\hat t_h$, logged and warned.
- **Needs the thrust stand.** $\theta$, the voltage dependence and $\tau_m$ must be measured with the real motors
  and propellers (phase 3, [vehicle model](../theory/vehicle-model.md#brushed-motor-dynamics)). Until then:
  $\theta$ = 1, $V_{ref}$ = 4.2 V.
- Two more parameters (`hoverThr`, `thrCurve`; to add to HANDOVER_FW §8 and the app), one more module, one CHECKS item, a few log fields.

## Alternatives considered

- **Keep the low-pass learning and only fix the numbers** (default 0.70, max 0.95). This is the minimal change and
  it fixes problems 1 and 2. It leaves the bias (3) and the battery-dependent gain (4).
  - Worth doing at once if phase 3 starts before this record is implemented: `FC_HOVER_DEFAULT` 0.70,
    `FC_HOVER_MAX` 0.95, `thr_max` 0.95.
- **Hover thrust as a fifth state of `alt_est`.** It is coupled with $b_a$: both explain a constant vertical
  acceleration, so they are only separable with the baro/ToF height. It also makes the validated linear filter
  nonlinear. Rejected; the separate 1-state filter uses the bias-corrected acceleration instead.
- **PX4's approach.** PX4 does the same: `MPC_THR_HOVER` as the seed, the `hover_thrust_estimator` module
  (`MPC_USE_HTE`) and `THR_MDL_FAC` for the thrust curve. This record follows that design, adapted to brushed
  motors: the battery compensation sits in the mixer, because duty is applied to the battery voltage directly.

## Validation

1. **Host test** (`Tests/host`) with a 1-D vehicle: $m\ddot z = T_{max}\,t\,(V/V_{ref})^2_{\text{after mixer}} - mg$,
   motor lag $\tau_m$ = 40 ms, accelerometer noise 2 m/s². Check that:
   - the seed off by ±30 % converges to within 3 % in < 2 s;
   - a battery-sag ramp (4.2 → 3.5 V in 300 s) gives no drift in $\hat t_h$;
   - a mass step +15 g (tray change) is tracked in < 3 s;
   - a step climb of 1 m/s does not bias $\hat t_h$ by more than 2 %.
2. **Thrust stand:** $T(u)$ at 4.2, 3.9 and 3.6 V, giving $\theta$ and the voltage exponent; $\tau_m$ from a step.
3. **Tethered hover** (indoor, T400): $\hat t_h$ against the thrust-stand value, and the duty at the end of the pack.
