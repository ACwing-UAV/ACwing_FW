# Embedded Invariant EKF for attitude estimation on STM32WB55

*Technical notebook — phase 1: attitude estimation only (control hooks in place, motors disabled).*

| | |
|---|---|
| **Target** | P-NUCLEO-WB55 (NUCLEO-WB55RG: Cortex-M4F @ 64 MHz + Cortex-M0+ radio core) |
| **Sensors** | X-NUCLEO-IKS4A1 in **Mode 3**: LSM6DSV16X (gyro + acc) on the MCU I²C bus, LIS2MDL (mag) read through the LSM6DSV16X sensor hub |
| **Software** | STM32CubeIDE / CubeMX, FreeRTOS (CMSIS-RTOS2), CMSIS-DSP, MATLAB R2019b+ |
| **Status** | **Phase 1 concluded.** Both tests validated **on the board**: base equations vs the original `iekfAttitude.m` (≤ 0.003 σ) and full extended filter vs `iekfAttitudeEmb.m` (≤ 0.002 σ), see §10.5 |

---

## Contents

1. [Introduction](#1-introduction)
2. [Repository layout](#2-repository-layout)
3. [Theory: the original IEKF](#3-theory-the-original-iekf)
4. [From MATLAB to embedded: improvements](#4-from-matlab-to-embedded-improvements)
5. [Firmware architecture](#5-firmware-architecture)
6. [Telemetry protocol](#6-telemetry-protocol)
7. [Building and flashing](#7-building-and-flashing)
8. [MATLAB tools: setup and live viewer](#8-matlab-tools-setup-and-live-viewer)
9. [Magnetometer calibration](#9-magnetometer-calibration)
10. [Telemetry capture and embedded-vs-MATLAB comparison](#10-telemetry-capture-and-embedded-vs-matlab-comparison)
11. [Troubleshooting](#11-troubleshooting)
12. [Lessons learned](#12-lessons-learned)
13. [Next steps](#13-next-steps)
14. [References](#14-references)

---

## 1. Introduction

The goal of the project is attitude control of a small quadrotor (4 × brushed
coreless 8520 motors, 1S). Position and trajectory are out of scope and there is
no GPS: the estimator only has to provide **attitude**, using a gyroscope, an
accelerometer and a magnetometer.

The estimator is an **Invariant Extended Kalman Filter (IEKF)** on the Lie group
SO(3). It was first developed and benchmarked in MATLAB
(`Matlab/FiltersComparison/IEKF_originalVersion/iekfAttitude.m`) and then ported to C for the
STM32WB55, inside a multi-rate FreeRTOS architecture that already hosts the
(empty) control loops.

Phase 1 delivers:

- the embedded IEKF (attitude + gyro bias, 6 error states) running at 960 Hz
  propagation / 240 Hz accelerometer update / ~100 Hz magnetometer update;
- a telemetry link to a PC over the ST-LINK virtual COM port;
- MATLAB tools to visualise the attitude in real time, calibrate the
  magnetometer and **compare the embedded filter with the original MATLAB one on
  identical inputs**.

---

## 2. Repository layout

```
.
├── App/                          application code (hardware-independent where possible)
│   ├── app/                      app_main (tasks, IRQ glue), app_config.h (ALL parameters),
│   │                             est_pipeline (multi-rate estimation), tlm_build (frames)
│   ├── iekf/                     so3, iekf, accwin (rotated acc mean), calib (static window)
│   ├── drivers/                  sensor_bus.h (abstract bus) + imu_lsm6dsv16x (IMU + sensor hub + FIFO parser)
│   ├── platform/                 imu_i2c: STM32 HAL I²C + DMA binding
│   ├── telemetry/                packet format, CRC, UART+DMA transport
│   ├── control/                  control hooks (weak, empty) and motor PWM (disabled)
│   └── third_party/              ST lsm6dsv16x-pid driver (BSD-3), CMSIS-DSP
├── Core/, Drivers/, Middlewares/ CubeMX-generated code (HAL, CMSIS, FreeRTOS)
├── IEKFcfg.ioc                   CubeMX configuration
├── IEKFcfg.launch                STM32CubeIDE debug configuration
├── STM32WB55RGVX_*.ld            linker scripts
└── Matlab/
    ├── initPath.m                adds every folder below to the path
    ├── Viewer/                   iekfLiveViewer.m (3D triad + logging), quat2rotmLocal.m
    ├── Telemetry/                parseTelemetry, decode*, crc16ccitt, statusDiagnosis
    ├── MagCalib/                 calibMagAxes.m (axis map + hard-iron), fitHardIron.m
    └── FiltersComparison/        tlmCapture, tlmSniffer, compareEmbeddedVsMatlab
        ├── IEKF_embeddedVersion/ EXTENDED filter: iekfAttitudeEmb.m + iekfCore*.m (MATLAB mirror of the C code)
        └── IEKF_originalVersion/ ORIGINAL filter developed in MATLAB: iekfAttitude.m, expSO3.m, logSO3.m, skewSym.m
```

`IEKF_originalVersion` is the filter as first developed and benchmarked in
MATLAB, kept unchanged: it is the reference of the base-equation test (§10.2).
`IEKF_embeddedVersion` is the MATLAB mirror of what runs on the board.

---

## 3. Theory: the original IEKF

This section documents the filter as implemented in
`Matlab/FiltersComparison/IEKF_originalVersion/iekfAttitude.m`. The embedded filter implements the
same equations (§4 lists what was added on top).

### 3.1 State and error definition

The attitude $R \in SO(3)$ rotates body-frame vectors into the world frame.
$SO(3)$ is not a vector space, so the estimation error is defined through the
group operation as a **left-invariant (body-frame) error**:

$$
R = \hat R\,\mathrm{Exp}(\delta), \qquad \delta = \mathrm{Log}(\hat R^\top R) \in \mathbb{R}^3 \cong \mathfrak{so}(3)
$$

where $\mathrm{Exp}$ is Rodrigues' formula
$\mathrm{Exp}(\phi) = I + \sin\theta\,[u]_\times + (1-\cos\theta)\,[u]_\times^2$,
$\theta = \|\phi\|$, $u = \phi/\theta$, and $[\cdot]_\times$ is the skew-symmetric
matrix. All Gaussian machinery (mean, covariance, linear update) is applied to
$\delta$, which lives in a vector space.

With gyro-bias estimation the error state is

$$
x = \begin{bmatrix}\delta \\ \beta\end{bmatrix}, \qquad \beta = b - \hat b .
$$

### 3.2 Process model

The gyroscope measures $\omega_m = \omega + b + n_g$; the bias is a random walk,
$\dot b = n_b$. The error dynamics are

$$
\dot\delta = -[\hat\omega]_\times\,\delta - \beta - n_g, \qquad \dot\beta = n_b,
\qquad \hat\omega = \omega_m - \hat b .
$$

Because the kinematics are *group-affine*, these dynamics are **autonomous**: the
Jacobian depends only on the measured rate, never on the estimate $\hat R$.
This is what distinguishes the IEKF from a classical multiplicative EKF and
gives it consistent covariance even when the estimate is far off (fast
convergence from a bad initialisation).

### 3.3 Propagation (discrete, step $h$)

$$
\hat R \leftarrow \hat R\,\mathrm{Exp}(\hat\omega h)
$$

$$
F = \begin{bmatrix}\mathrm{Exp}(-\hat\omega h) & -h I \\ 0 & I\end{bmatrix},
\qquad
Q = \begin{bmatrix}\sigma_g^2 h\, I & 0 \\ 0 & \sigma_b^2 h\, I\end{bmatrix},
\qquad
P \leftarrow F P F^\top + Q .
$$

### 3.4 Vector measurements

Accelerometer and magnetometer are both treated as observations of a known
world-frame direction $v$ (gravity $g_{ref}$, magnetic field $m_{ref}$):

$$
y = R^\top v + n, \qquad \hat y = \hat R^\top v .
$$

Measurements are normalised ($y \leftarrow y/\|y\|$). Linearising with the
left-invariant error, $R^\top v = \mathrm{Exp}(-\delta)\hat R^\top v \approx \hat y + [\hat y]_\times \delta$, so

$$
y - \hat y \approx [\hat y]_\times\,\delta
\quad\Longrightarrow\quad
H = \begin{bmatrix}[\hat y]_\times & 0\end{bmatrix}.
$$

**Adaptive measurement noise.** When the measured magnitude departs from its
nominal value (external acceleration, magnetic disturbance) the measurement is
de-weighted rather than rejected:

$$
R_m = \sigma^2 I \left(1 + \left(\frac{\Delta}{\tau}\right)^2\right),
\qquad \Delta = \frac{\big|\,\|y_{raw}\| - n_{nom}\big|}{n_{nom}} .
$$

**Tilt-compensated magnetometer.** With `tiltCompensatedMag = true` the
predicted field is built from the measurement itself, keeping only its
horizontal component, so the magnetic dip never enters the innovation and the
magnetometer can only correct yaw:

$$
h_w = \hat R y,\qquad b_w = \begin{bmatrix}\sqrt{h_{w,x}^2 + h_{w,y}^2} & 0 & h_{w,z}\end{bmatrix}^\top,\qquad \hat y = \hat R^\top b_w .
$$

### 3.5 Update and retraction

$$
S = H P H^\top + R_m, \qquad K = P H^\top S^{-1}, \qquad \delta x = K\,(y - \hat y)
$$

$$
\hat R \leftarrow \hat R\,\mathrm{Exp}(\delta x_{1:3}), \qquad \hat b \leftarrow \hat b + \delta x_{4:6}
$$

$$
P \leftarrow (I - KH)\,P\,(I - KH)^\top + K R_m K^\top \quad\text{(Joseph form)}, \qquad P \leftarrow \tfrac12 (P + P^\top).
$$

The retraction through $\mathrm{Exp}$ keeps $\hat R$ on $SO(3)$ at all times.
Per sample the order is: propagate with `gyro(:,k-1)`, accelerometer update,
magnetometer update.

---

## 4. From MATLAB to embedded: improvements

Each item lists *why* it was needed. Unless stated otherwise all numbers come
from the PC simulation of the full firmware pipeline (same C code, synthetic
LSM6DSV16X FIFO encoded byte by byte).

### 4.1 Multi-rate structure with angle increments

| Function | Rate |
|---|---|
| Gyro + acc sampling (FIFO) | 960 Hz (1920 Hz with I²C Fast-mode Plus) |
| FIFO-threshold interrupt → propagation, `ctrl_rate_step()` | 960 Hz |
| Accelerometer update, `ctrl_attitude_step()`, telemetry | 240 Hz |
| Magnetometer update (LIS2MDL 100 Hz, sensor hub 120 Hz) | on new data |

When a propagation step spans several gyro samples, the increments are summed
without loss of information:

$$
\Delta\theta = \sum_i \omega_{m,i}\,\Delta t_i, \qquad \phi = \Delta\theta - \hat b\,h, \qquad \hat R \leftarrow \hat R\,\mathrm{Exp}(\phi).
$$

$\Delta t$ is **measured** by the MCU (DWT cycle counter between interrupts),
which absorbs the tolerance of the sensor's internal oscillator.

### 4.2 Rotation-compensated accelerometer mean (`accwin`)

Averaging the acc over a 4 ms window while rotating at 360 °/s smears the
direction and lags by half a window (**0.66°** error). Each sample is moved to
the body frame at the start of the window with the already-propagated
increments $E_j$, averaged, and moved back:

$$
\bar a = D_N^\top\,\frac1N\sum_i D_i\,a_i, \qquad D_i = E_1 E_2 \cdots E_i .
$$

Error after compensation: **0.000°**.

### 4.3 Static calibration and levelled world frame

At start-up the board must stay still for ~2 s (`calib.c`, restarts if it moves):

- **bias:** $\hat b_0$ = mean gyro;
- **noise:** $\sigma_{acc}, \sigma_{mag}$ from the direction scatter
  (port of `directionNoiseStd.m`), with a floor `IEKF_SIGMA_VEC_MIN = 0.01`;
- **world frame:** levelled **NWU** (x = horizontal magnetic north, y = west, z = up)
  built by **TRIAD**, instead of "body at $t=0$" used by `runRealData.m`:

$$
z = \frac{\bar a}{\|\bar a\|},\quad
x = \frac{\bar m - (\bar m\cdot z)z}{\|\cdot\|},\quad
y = z\times x,\quad
R_0 = \begin{bmatrix}x & y & z\end{bmatrix}^\top .
$$

The tilt-compensated magnetometer update assumes a vertical world $z$; with
"body at $t=0$" and a board that is not perfectly level that assumption fails.

### 4.4 Magnetometer through the sensor hub, with latency compensation

In IKS4A1 Mode 3 the LIS2MDL is behind the LSM6DSV16X and its samples land in
the same FIFO, time-aligned with gyro and acc. The hub re-reads the LIS2MDL
faster than its ODR (duplicates discarded), so a new sample has a random age
$\tau \sim U(0, 1/f_{sh})$. The mean age is compensated by rotating the
measurement forward with the gyro, the random part enters the noise:

$$
y \leftarrow y - \bar\tau\,(\hat\omega \times y), \qquad
R_m \leftarrow R_m + \big(\|\hat\omega \times u\|\,\sigma_\tau\big)^2 I,
\qquad \bar\tau = \tfrac{1}{2f_{sh}},\ \sigma_\tau = \tfrac{1}{\sqrt{12}f_{sh}} .
$$

| Magnetometer | Attitude RMSE (fast rotations) |
|---|---|
| 50/60 Hz, no compensation | 1.150° |
| 50/60 Hz, compensated | 0.165° |
| **100/120 Hz, compensated** | **0.095°** |

### 4.5 Single-precision numerics

- `so3_exp` switches to a 4th-order Taylor series below 0.05 rad: in float32,
  $(1-\cos\theta)/\theta^2$ loses all digits for $\theta \sim 10^{-3}$, the typical
  $\omega h$.
- $\hat R$ is re-orthonormalised (Gram–Schmidt) after every product.
- **Covariance conditioning** after every propagation and update: exact
  symmetrisation and a floor on the diagonal ($10^{-12}$ rad², $10^{-16}$ (rad/s)²).
  The Joseph form guarantees $P \succeq 0$ only in exact arithmetic.
- The 6×6 covariance algebra uses CMSIS-DSP (`arm_mat_*`); static workspaces keep
  large matrices off the RTOS task stack.

### 4.6 Error reset after the retraction

After $\hat R \leftarrow \hat R\,\mathrm{Exp}(\delta\theta)$ the error is defined
w.r.t. the *new* estimate, $\delta' \approx \delta - \delta\theta - \tfrac12[\delta\theta]_\times\delta$, hence

$$
J = I - \tfrac12[\delta\theta]_\times, \qquad P_{aa} \leftarrow J P_{aa} J^\top, \qquad P_{ab} \leftarrow J P_{ab}.
$$

Negligible at steady state, relevant for large corrections (`IEKF_RESET_JACOBIAN`).

### 4.7 Robustness to external accelerations

The magnitude test of §3.4 misses accelerations **perpendicular** to gravity:
0.4 g sideways changes $\|a\|$ by a few percent but rotates the measured
direction by >20°. The tilt error then leaks into yaw through the
tilt-compensated compass (× tan of the magnetic dip, ~1.6 in Italy).

**NIS-driven inflation with memory** (accelerometer):

$$
\nu = e^\top S_0^{-1} e,\qquad
g_k = \min\!\Big(g_{max},\ \max\big(1,\ \lambda\,g_{k-1},\ \nu/\gamma\big)\Big),\qquad
R_{eff} = g_k R_m ,
$$

with $S_0$ computed with the nominal $R_m$, $\gamma$ = `IEKF_ACC_NIS_GATE` (4),
$\lambda = e^{-h_{acc}/\tau}$ with $\tau$ = `IEKF_INFL_TAU_S` (0.5 s),
$g_{max}$ = `IEKF_INFL_MAX` (100).

**Magnetic dip check** (magnetometer): the angle between the *measured* field and
the *measured* gravity is compared with its calibration value; it does not depend
on the estimate, so it cannot mistake a true yaw error for a disturbance:

$$
R_m \leftarrow R_m\left(1 + \left(\frac{|\angle(y_m, y_a) - \mathrm{dip}_{ref}|}{\mathrm{tol}_{dip}}\right)^2\right).
$$

**Stillness detector:** $\big|\|a\|/a_{nom} - 1\big| < 0.02$ and
$\|\omega_{raw}\| < 3$ °/s for 0.3 s ⇒ an external acceleration is impossible ⇒ the
next acc update is not inflated.

**Lockout protection.** If the estimate is truly wrong, the innovation is large
because of the estimate, NIS inflates $R$ forever and the filter stops
correcting (recovery > 16 s in simulation). With a correct gyro, a true attitude
error is *fixed in the world frame*, while a hand shaking the board produces a
zero-mean innovation. The world-frame innovation is tracked:

$$
m_k = \lambda_p m_{k-1} + (1-\lambda_p)\,\hat R e_k,\qquad
p_k = \lambda_p p_{k-1} + (1-\lambda_p)\,\|e_k\|^2 ,
$$

and if the acc has been inflated for `IEKF_LOCKOUT_S` (0.5 s) **and**
$\|m\|^2 > \rho\,p$ and $\|m\| > m_{min}$ (persistent), then
$P_{aa} \leftarrow P_{aa} + \sigma_{bump}^2 I$ (10°) and the acc is trusted for
`IEKF_GRACE_S` (0.25 s).

| Simulation (0.4 g lateral for 10 s) | yaw max | tilt max | recovery from 37° error in motion |
|---|---|---|---|
| magnitude test only (original) | 14.9° | 20.3° | 0.26 s |
| NIS without lockout protection | 2.56° | 1.72° | 16.6 s |
| **final configuration** | **2.40°** | **2.32°** | **0.95 s** |

Nominal accuracy is unchanged (RMSE 0.095° at 360 °/s peak).
**Known limit:** a *sustained, constant* external acceleration (steady turn) is as
persistent as a true error and is accepted after 0.5 s. Only a thrust model
(phase 3) can separate the two.

Tried and left disabled: gain projection so that acc corrects only tilt and mag
only yaw (`IEKF_DECOUPLE`: tilt max 3.3° vs 2.3°); NIS inflation on the
magnetometer (`IEKF_MAG_NIS_GATE`: a high mag NIS is often a true yaw error).

### 4.8 Tuning notes

- **6 states, not 3.** With a realistic bias: RMSE 0.095° (6 states) vs 0.184°
  (3 states). `IEKF_ESTIMATE_BIAS = 0` reduces the filter exactly to 3 states.
- **`IEKF_SIGMA_GYRO = 0.5 °/s/√Hz`.** It must cover what the model ignores
  (scale factor, misalignment, g-sensitivity), not just the sensor noise
  (~0.003 °/s/√Hz). With 1 % scale error and 1° misalignment: σ = 0.1 → RMSE 1.17°,
  σ = 0.5 → 0.42°. Lower it only after calibrating the gyro.

### 4.9 MATLAB mirror of the C code

`IEKF_embeddedVersion/iekfAttitudeEmb.m` is the **extended MATLAB filter**: same
signature and options as `iekfAttitude.m` plus the extensions above
(`accNisGate`, `inflTau`, `dipTol`, `decouple`, `resetJacobian`, `inflMax`,
`lockoutTime`, …). Its maths goes through `iekfCore*.m`, written line by line
after `iekf.c`. As in the firmware, the inflation memory and the lockout counters
run **per accelerometer update**; their exact per-update values (`inflDecay`,
`lockoutUpdates`, `graceUpdates`, `persistDecay`) and the per-sample inputs
produced by the firmware pipeline (`accTrusted` from the stillness detector,
`magExtraVar` from the mag-age model) can be passed directly. A 4th output
reports `lockouts` and `accInflMax`. With every extension off it matches the
original within 10⁻¹² deg.

---

## 5. Firmware architecture

```
INT1 LSM6DSV16X (FIFO threshold, 960 Hz) ── EXTI ISR: DWT timestamp, thread flag ──┐
                                                                                   v
ImuTask        osPriorityRealtime, 2 KB stack
   FIFO status (2 B) → I²C DMA burst N×7 B → imu_fifo_parse → est_process():
   propagation → ctrl_rate_step → [÷4] acc update → ctrl_attitude_step → telemetry frame
   [mag update whenever a new sample arrives]
        │ qTlm (16 frames, never blocking)
        v
TelemetryTask  osPriorityBelowNormal, 1 KB stack     UART1 DMA → ST-LINK VCP (921600 baud)
   + STATUS at 10 Hz, LED2 heartbeat
```

- A single high-priority task owns the I²C bus and the filter state: no mutex on
  the critical path.
- `ctrl_rate_step()` runs right after the propagation, **before** the Kalman
  corrections, to minimise gyro → motor latency.
- `est_pipeline.c`, `iekf.c`, `accwin.c`, `calib.c`, the FIFO parser and the
  telemetry builders have **no HAL/RTOS dependency**: the same code runs in a PC
  test harness.

**I²C budget.** Each interrupt reads ~2 status bytes and ~5 FIFO words (35 B):
~1 ms per cycle at 400 kHz (fine at 960 Hz, not at 1920 Hz), ~0.4 ms at 1 MHz
Fast-mode Plus. The DMA frees the CPU, but the transfer time is latency on the
rate loop. If unexpected FIFO tags appear (`ps.unknown`), switch from one burst to
word-by-word reads.

**Pin map (UM3239, UM2435)**

| Signal | IKS4A1 | Arduino | STM32WB55 |
|---|---|---|---|
| I²C SCL | CN5-10 | D15 | PB8 (I2C1) |
| I²C SDA | CN5-9 | D14 | PB9 (I2C1) |
| LSM6DSV16X INT1 | CN9-6 | D5 | PA15 (EXTI15, label `IMU_INT1`) |
| ST-LINK VCP | — | — | PB6/PB7 (USART1) |

IKS4A1 **Mode 3**: J4 and J5 on 5-6 only. PA15 is also JTDI → debug must be
*Serial Wire*.

**Motors (prepared, disabled):** `motors.c`, TIM1, 64 MHz / 2000 = 32 kHz PWM with
2000 levels, `MOTORS_ENABLE = 0`. A power stage is required (low-side
logic-level N-MOSFET, gate resistor, pull-down, Schottky freewheeling diode, bulk
capacitance). PA8 (TIM1_CH1) is wired to the LPS22DF INT1 on the IKS4A1: PWM pins
are to be chosen together with the power stage.

---

## 6. Telemetry protocol

Frame (little-endian): `A5 5A | TYPE | LEN | PAYLOAD[LEN] | CRC16 (LSB, MSB)`,
CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD.

| Type | Name | Size | Rate | Content |
|---|---|---|---|---|
| 0x01 | STATE | 71 B | 240 Hz (30 Hz with REPLAY) | seq, time, q, bias, acc, gyro, mag, flags |
| 0x02 | INIT | 137 B | after calibration, then ~1 Hz | initial state, references, all filter parameters |
| 0x03 | STATUS | 54 B | 10 Hz, **always** | IMU init result, IRQ/bus/FIFO counters, calibration progress, drops |
| 0x04 | REPLAY | ≤ 212 B | 240 Hz (only with `APP_TLM_REPLAY = 1`) | exact filter calls of the window + resulting q, b, diag P |

REPLAY operations: `'P' dθ[3] h` (propagation), `'A' acc[3] trusted`
(acc update), `'M' mag[3] extraVar` (mag update), all float32.

Bandwidth: ~19 kB/s normally, ~44 kB/s with REPLAY (921600 baud ≈ 92 kB/s).

> **Pitfall.** Telemetry structs are `packed`: float fields are not 4-byte
> aligned. Never pass the address of such a field to a function that writes
> floats: Cortex-M4F uses `VSTR`, which faults on unaligned addresses. Write to a
> local variable and `memcpy`.

---

## 7. Building and flashing

1. Open `IEKFcfg.ioc` in STM32CubeMX/IDE and generate the project for
   STM32CubeIDE (the `.project`/`.cproject` files are not part of this package).
   Relevant settings already in the `.ioc`: SYSCLK 64 MHz; I2C1 on PB8/PB9 with
   DMA RX and event/error IRQs; USART1 921600 baud with DMA TX (**Memory increment
   enabled**) and global IRQ; PA15 EXTI rising edge (`IMU_INT1`); FreeRTOS
   CMSIS_V2, 1 kHz tick, 20 KB heap; HAL timebase on a TIM; Serial Wire debug.
   Interrupts that call RTOS functions need priority ≥ 5.
2. Include paths (all configurations): `App/app`, `App/iekf`, `App/telemetry`,
   `App/platform`, `App/control`, `App/drivers/common`,
   `App/drivers/imu_lsm6dsv16x`, `App/third_party/st_lsm6dsv16x`,
   `App/third_party/cmsis_dsp/Include`.
3. Linker: library `arm_cortexM4lf_math` from `App/third_party/cmsis_dsp/Lib`
   (the **GCC** `.a`, not the Keil `.lib`). FPU: FPv4-SP-D16, hard ABI.
4. `main.c` already calls `app_start()` and forwards the HAL callbacks
   (`HAL_GPIO_EXTI_Callback`, `HAL_I2C_MemRxCpltCallback`,
   `HAL_I2C_ErrorCallback`, `HAL_UART_TxCpltCallback`).
5. Debug/flash: `IEKFcfg.launch` (ST-LINK GDB server, SWD, connect under reset).

Bring-up check in *Live Expressions*: `s_irqCount` grows by ~480/s at 960 Hz,
`s_busErr` stays 0, `s_est.running` becomes 1 about 2 s after reset with the board
still. `s_procCycMax / 64` = worst-case processing time in µs.

All parameters live in `App/app/app_config.h`. The switches that matter for the
procedures below:

| Macro | Normal use | Comparison with MATLAB |
|---|---|---|
| `APP_TLM_REPLAY` | 0 | 1 |
| `APP_COMPARE_ORIGINAL` | 0 | 1 (base-equation test vs original) or 0 (extended-filter test) |
| `MAG_AXIS_MAP`, `MAG_HARD_IRON` | from §9 | unchanged |

---

## 8. MATLAB tools: setup and live viewer

```matlab
run('<repo>/Matlab/initPath.m')   % once per session
```

`initPath.m` adds all folders (tools, original filter, extended filter) to the
path and prints which `iekfAttitude.m` is in use (it must be the one in
`FiltersComparison/IEKF_originalVersion`; `compareEmbeddedVsMatlab` warns if
another copy shadows it).

**Live viewer** (firmware with `APP_TLM_REPLAY = 0`):

```matlab
L = iekfLiveViewer("serial", "COM6");                 % 921600 baud by default
L = iekfLiveViewer("serial", "COM6", 921600, "C:\logs") % choose the log folder
```

1. Start the viewer, then reset the board and keep it **still for 2–4 s**
   (static calibration; it restarts if the board moves).
2. The 3D triad follows the board in the levelled NWU frame. The text panel shows
   roll/pitch/yaw, packet rate (~240 Hz), lost packets, **display lag** (warning
   above 0.5 s), gyro bias, flags (`ACC de-weighted`, `MAG de-weighted`, `STILL`)
   and the calibration result. Before the filter starts it shows the STATUS
   diagnosis instead.
3. Close the figure to stop. The log (`iekf_log_*.mat`, autosaved every 10 s) is
   also returned and copied into the workspace as `iekfLog`; a raw `.bin` capture
   is written next to it.

**Byte-level link check:**

```matlab
tlmSniffer("COM6")      % bytes -> frames -> STATUS -> diagnosis
```

---

## 9. Magnetometer calibration

On the IKS4A1 the LIS2MDL axes are **not** aligned with the LSM6DSV16X axes, and
the board carries a large hard-iron offset. Both are estimated from data by
`calibMagAxes.m`.

**Principle.** The magnetic field is fixed in the world; seen from the rotating
body it must rotate exactly as the integrated gyro says,
$m_j = D_{ij}^\top m_i$. This holds only with the right axis map $M$
($m_{body} = M\,m_{sensor} - h$). All 24 signed-permutation rotations are tried;
for each, the hard-iron $h$ is solved by least squares over pairs of samples
~0.2 s apart:

$$
(I - D^\top) M h = M m_j - D^\top M m_i .
$$

The candidate with the smallest residual wins, and must win by a clear margin.

**Procedure**

1. In `app_config.h` set the identity map and zero offset:
   ```c
   #define MAG_AXIS_MAP   { 1.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f,   0.0f, 0.0f, 1.0f }
   #define MAG_HARD_IRON  { 0.0f, 0.0f, 0.0f }
   ```
   Build with `APP_TLM_REPLAY = 0` and flash.
2. `iekfLiveViewer("serial","COM6")`, reset, keep still 2 s, then ~40 s of slow to
   moderate rotations about **all** axes (tilts included), away from metal objects
   and the USB cable. Close the viewer.
3. Run
   ```matlab
   calibMagAxes('iekf_log_YYYYMMDD_HHMMSS.mat')
   ```
   It prints the residuals of the 4 best hypotheses (warning if the best is not
   clearly separated) and the two lines to paste into `app_config.h`.
4. Rebuild, flash and verify with a new log while rotating in all directions:
   ```matlab
   S = load('iekf_log_....mat'); m = vecnorm(S.L.mag);
   fprintf('|B| mean %.1f uT, variation %.1f%%\n', mean(m), 100*std(m)/mean(m))
   ```
   Expected: |B| ≈ 40–50 µT (local Earth field), variation < 3–5 %. With the
   board level, a 90° turn about the vertical changes yaw by ~90° and tilting
   does not move yaw. A larger variation points to soft-iron effects (ellipsoid
   fit, not yet implemented).

Current calibration (2026-10-07): LIS2MDL rotated 180° about x (y, z flipped),
hard-iron (100.31, −0.56, −166.07) µT. The offset is ~4× the Earth field:
recalibrate whenever anything near the sensor changes.

`fitHardIron.m` (sphere fit) is kept for a quick hard-iron-only check once the
axis map is known.

---

## 10. Telemetry capture and embedded-vs-MATLAB comparison

### 10.1 Principle

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

### 10.2 Two validation tests

| `APP_COMPARE_ORIGINAL` | Firmware behaviour | Reference filter | Meaning of the test |
|---|---|---|---|
| **1** | extensions of §4.4–4.7 that the original lacks are disabled | original `iekfAttitude.m` | **base equations**: the C port of the original filter is exact → MATCH / DIVERGE |
| **0** | full configuration (as flown) | extended `iekfAttitudeEmb.m` | **every improvement**: NIS inflation and memory, dip check, stillness detector, mag-age variance, error reset, lockout protection → MATCH / DIVERGE. The original is also run, to show how much the extensions change the estimate. |

### 10.3 Procedure

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
`compareEmbeddedVsMatlab('C:\...\iekf_cap_YYYYMMDD_HHMMSS.bin')`.

### 10.4 Reading the result

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

### 10.5 Results

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
reference at 0.002 σ. It measures how much the §4.7 extensions change the
estimate in real use.

**Phase 1 validation: complete.**
- The C port of the original equations is exact (mode 1, board).
- Every embedded improvement is implemented as specified (mode 0, board).

In both tests the only residual is float32 vs double.

> After validation, set `APP_TLM_REPLAY 0` and `APP_COMPARE_ORIGINAL 0` again in
> `app_config.h`: this is the configuration for normal use and for the live viewer.

---

## 11. Troubleshooting

| Symptom | Cause / action |
|---|---|
| LED1 blinking every 200 ms | `imu_init` failing: jumpers (Mode 3), I²C pins, power |
| No bytes at all | wrong COM port, port held by another program (`clear all`), firmware halted |
| Only `A5 A5 A5 …` | USART1 TX DMA without *Memory increment* |
| Bytes but no valid frame | baud mismatch or SYSCLK ≠ 64 MHz |
| One frame, then nothing | UART TX-complete callback not wired / USART1 IRQ disabled |
| Only STATUS frames | filter never started: `statusDiagnosis` (IMU init, no INT1, no mag from hub, board moving during calibration) |
| HardFault ~2 s after reset | unaligned float write into a packed telemetry struct (§6) |
| Attitude "converges slowly" in the viewer | viewer lagging: check the *lag* field; disable REPLAY or use `tlmCapture` |
| Yaw moves when tilting | wrong `MAG_AXIS_MAP` / hard-iron → §9 |
| `iekfAttitude` not found | run `initPath.m` |
| Comparison: "INIT of this boot was lost" | UART saturated during the capture; repeat |
| Euler plots jump by 180° | gimbal lock (|pitch| ≈ 90°); use the quaternion figure |
| `undefined reference to arm_mat_*` | CMSIS-DSP GCC library not linked (§7) |

---

## 12. Lessons learned

- **Validate on identical inputs, judge in sigma.** Comparing filters on
  separately acquired data mixes sensor and algorithm differences; replaying the
  exact calls isolates the implementation, and normalising by $\sqrt{P}$ turns
  "how small is small" into a property of the filter itself.
- **Instrument the link before debugging the filter.** A viewer that falls behind
  looks exactly like a slowly converging filter. The STATUS packet and the display
  lag indicator now make link problems visible.
- **Robustness features need an exit.** Any mechanism that de-weights a sensor
  based on the innovation can lock the filter out of a true error; it needs a cap
  and an independent way to recognise a genuine error (here: world-frame
  persistence and stillness).
- **Process noise is a model-error budget.** The gyro σ that worked was 150×
  the datasheet noise density because it also covers scale and alignment errors.

---

## 13. Next steps

Phase 1 (embedded attitude estimation, telemetry, MATLAB tools and
validation) is concluded. Open items for the next phases:


- Gyro scale and misalignment calibration (turntable or known rotations), then
  lower `IEKF_SIGMA_GYRO`.
- Soft-iron (ellipsoid) magnetometer calibration.
- Verify the magnetometer latency $\bar\tau$ on the bench (constant-rate rotation
  about the vertical) — the LIS2MDL internal filter delay is not modelled.
- I²C Fast-mode Plus and 1920 Hz sampling (check bus edges on a scope first).
- Phase 2: BLE telemetry/commands on the Cortex-M0+ stack (IPCC/HCI priorities,
  no flash writes in flight).
- Phase 3: rate PID at 960 Hz with notch-filtered gyro, attitude loop at 240 Hz,
  motor power stage, arming and fail-safe, and a **thrust model** to reject
  sustained accelerations in the acc update.

---

## 14. References

- A. Barrau, S. Bonnabel, *The Invariant Extended Kalman Filter as a Stable
  Observer*, IEEE TAC, 2017.
- R. Mahony, T. Hamel, J.-M. Pflimlin, *Nonlinear Complementary Filters on the
  Special Orthogonal Group*, IEEE TAC, 2008.
- S. Madgwick, A. Harrison, R. Vaidyanathan, *Estimation of IMU and MARG
  orientation using a gradient descent algorithm*, ICORR, 2011.
- STMicroelectronics: UM3239 (X-NUCLEO-IKS4A1), UM2435 (NUCLEO-WB55),
  LSM6DSV16X and LIS2MDL datasheets, `lsm6dsv16x-pid` driver, AN5289 (STM32WB
  BLE).
