# Validation against the reference


## Principle

The firmware records, for every 240 Hz window, the **exact arguments** of every
filter call (960 Hz angle increments, rotated acc mean, latency-compensated mag)
and the **resulting state** (q, bias, diag P) — the REPLAY packet.
`compareEmbeddedVsMatlab.m` rearranges those inputs into the batch format
expected by the **unmodified** MATLAB filters (`iekfAttitude.m` and
`iekfAttitudeEmb.m`):

- each propagation becomes a sample: `gyro(:,k-1) = dθ/h`, `dt(k) = h`;
- acc/mag sit in the samples where the firmware updated, zero elsewhere (the
  original skips an update when |v| < eps);
- a mag arriving after the end-of-window acc update gets an auxiliary 1 ns sample,
  so states are compared at the same instant;
- the INIT packet supplies $R_0$, $\hat b_0$, $P_0$, references and every parameter;
- for the extended filter, the stillness flag of every acc update and the
  mag-age variance of every mag update are passed per sample.

Inputs are identical bit for bit: any difference comes from the filter.

## Two validation tests

| `APP_COMPARE_ORIGINAL` | Firmware behaviour | Reference filter | Meaning of the test |
|---|---|---|---|
| **1** | extensions of [Magnetometer through the sensor hub, with latency compensation](../theory/iekf-embedded.md#magnetometer-through-the-sensor-hub-with-latency-compensation)–4.7 that the original lacks are disabled | original `iekfAttitude.m` | **base equations**: the C port of the original filter is exact → MATCH / DIVERGE |
| **0** | full configuration (as flown) | extended `iekfAttitudeEmb.m` | **every improvement**: NIS inflation and memory, dip check, stillness detector, mag-age variance, error reset, lockout protection → MATCH / DIVERGE. The original is also run, to show how much the extensions change the estimate. |

## Procedure

1. `app_config.h`: `APP_TLM_REPLAY 1`, `APP_COMPARE_ORIGINAL 1`; build and flash.
2. Record **without the viewer** (it cannot keep up with ~44 kB/s):
   ```matlab
   bin = tlmCapture("COM6", 60);   % then reset the board immediately
   ```
   Keep the board still for 3–4 s, then move it (rotations, tilts, some shaking)
   for the rest of the time. `tlmCapture` reports the rate over the second half
   (~44 kB/s expected).
3. Compare:
   ```matlab
   out = compareEmbeddedVsMatlab(bin);
   ```
4. Repeat with `APP_COMPARE_ORIGINAL 0` (full configuration) to validate the
   extended filter. Include some shaking and, if possible, a stillness phase in
   the middle, so that the inflation and stillness paths are exercised; the
   report shows how many times each was used.
5. Set `APP_TLM_REPLAY 0` again for normal use.

A previously recorded file is analysed the same way:
`compareEmbeddedVsMatlab('C:\...\acwing_cap_YYYYMMDD_HHMMSS.bin')`.

Python, same procedure and same report:

```bash
acwing-tools capture COM6 --duration 60          # then reset the board immediately
acwing-tools compare acwing_cap_YYYYMMDD_HHMMSS.bin
```

The Python port of both filters matches the MATLAB code to ~1e-14, and
`compare` reproduces the figures of `compareEmbeddedVsMatlab` on the same
captures (`02 - Tools/01 - Python/tests`).

## Reading the result

Console report:

- **capture integrity:** frames per type, firmware drop counters, REPLAY windows
  received/lost. The comparison stops at the first lost window (the filter state
  cannot be rebuilt across a gap). The INIT must belong to the same boot as
  window 0 (detected through the STATUS uptime).
- **absolute differences** (attitude, bias, diag P) and the same differences **in
  units of the filter's own standard deviation** $\sqrt{\mathrm{diag}\,P}$.
- **verdict** (embedded vs the reference filter of the mode):

| Normalised difference | Verdict |
|---|---|
| < 0.10 σ (and diag P relative < 10⁻³) | **MATCH** — float32 vs double only |
| > 0.5 σ from the first windows | different inputs / initial conditions (e.g. lost INIT) |
| > 0.5 σ appearing later | inspect the reported window |
| small and growing | numerical drift |

  Absolute thresholds do not work: the bias estimate integrates round-off, so the
  float32/double gap grows slowly with capture length and dynamics.

- mode 1: the same calls replayed through `iekfCore*` as a cross-check;
  mode 0: the differences w.r.t. the original (during acc inflation and
  elsewhere) and a **coverage line** — stillness-trusted acc updates, windows
  with acc inflated and maximum inflation, lockout events, mag updates with
  age variance — so it is clear which extensions the capture actually tested.

Figures (`doPlot`, default true):

1. **Angles** — roll, pitch, yaw of the embedded filter and of the reference
   (mode 0 also shows the original, dotted), plus embedded − reference.
   Angles are wrapped to ±180° (pitch ±90°); lines are broken at wraps, and
   samples with |pitch| > 80° (gimbal lock: roll and yaw genuinely flip) are
   marked.
2. **Quaternion** — the four components of the same filters, sign made continuous: no wrap and no
   singularity, the best view near the vertical.
3. **Geodesic difference and bias** — attitude difference on a log scale (red
   marks: acc de-weighted) and gyro bias, embedded vs reference.

The output struct contains `t`, `eulEmb`, `eulMatlab` (original), `eulRef`
(reference of the mode), `dAng`, `dBias`, `dP` (vs original), `extended`
(metrics vs the extended filter, mode 0), `ok`, `adapted`, `init`.

## Results

| Data | Attitude diff. | Bias diff. | diag P (rel.) | Verdict |
|---|---|---|---|---|
| Simulated capture, mode 1 (21 s, up to 200 °/s, 0.4 g shaking) | 1.1·10⁻⁴ ° | 9.8·10⁻⁵ °/s | 1.5·10⁻⁴ | MATCH (0.001 σ) |
| **Board capture, mode 1** (20 s, 4711 windows, no gaps) | 3.2·10⁻⁴ ° (RMS 6.5·10⁻⁵ °) | 3.3·10⁻⁴ °/s | 1.1·10⁻⁴ | **MATCH** (attitude 0.002 σ, bias 0.003 σ) |
| Simulated, mode 0 vs **extended** `iekfAttitudeEmb` (same capture as row 1 but full configuration) | 1.7·10⁻⁴ ° | 2.7·10⁻⁵ °/s | 1.5·10⁻⁴ | **MATCH** (0.001 σ) |
| Simulated, mode 0 vs extended, calibration under a 0.3 g push (exercises the lockout protection) | 2.0·10⁻⁴ ° | 3.4·10⁻⁵ °/s | 2.0·10⁻⁴ | **MATCH** (0.001 σ) |
| Simulated, mode 0 vs **original** | up to 20.6° while shaking, ≤ 0.37° elsewhere | | | expected: there the original is the one that is wrong |
| *Negative control*: extended filter with the dip tolerance changed by 20 % | | | | DIVERGE (0.35 σ) — the test detects small deviations |
| **Board capture, mode 0 vs extended** (21.7 s, 5137 windows, no gaps) | 5.6·10⁻⁴ ° (RMS 1.5·10⁻⁴ °) | 1.2·10⁻⁴ °/s | 1.5·10⁻⁴ | **MATCH** (attitude 0.002 σ, bias 0.001 σ) |
| Board capture, mode 0 vs **original** | max 91.0° (RMS 23.2°); during acc inflation max 78.2°, elsewhere max 91.0° (RMS 13.9°) | | | expected, see below |

Largest angle differences in the board mode-0 capture, embedded vs extended:
roll 7.8·10⁻⁴ °, pitch 1.7·10⁻⁴ °, yaw 8.5·10⁻⁴ °.

**Coverage.**

| Capture | Stillness-trusted acc updates | Windows with acc inflated | Lockout events | Mag updates with age variance |
|---|---|---|---|---|
| Simulated mode 0 (21 s, up to 200 °/s, 0.4 g shaking for 6 s) | 168 | ~1900–2100 (cap ×100 reached) | 0 / 1 (calibration-push scenario) | 2118 |
| **Board mode 0** (21.7 s, hand-held) | **958** | **3777** of 5137 (cap ×100 reached) | **3** | **2288** |

The board capture therefore exercised every active extension, the lockout
protection included, and the C code matches the extended MATLAB filter on all of
them. Not exercised: `IEKF_DECOUPLE` and `IEKF_MAG_NIS_GATE`, both disabled in the
firmware.

**Why the original differs by up to 91° on the board.** The hand-held motion was
much more aggressive than in simulation: the acc was de-weighted in 74 % of the
windows. The original filter has no NIS inflation, so it trusts every
accelerometer sample during shaking. It then needs time to reconverge, which is
why large differences also appear outside the inflated windows (RMS 13.9°). The
difference is not an error of the embedded filter, which matches its own
reference at 0.002 σ. It measures how much the [Robustness to external accelerations](../theory/iekf-embedded.md#robustness-to-external-accelerations) extensions change the
estimate in real use.

**Phase 1 validation: complete.**

- The C port of the original equations is exact (mode 1, board).
- Every embedded improvement is implemented as specified (mode 0, board).

In both tests the only residual is float32 vs double.

> After validation, set `APP_TLM_REPLAY 0` and `APP_COMPARE_ORIGINAL 0` again in
> `app_config.h`: this is the configuration for normal use and for the live viewer.
