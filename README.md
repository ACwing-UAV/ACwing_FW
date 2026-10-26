# Embedded Invariant EKF for attitude estimation on STM32WB55

This repository contains the firmware of an **attitude estimator for a small
quadrotor**, together with the PC tools used to develop, calibrate and validate
it.

The estimator is an **Invariant Extended Kalman Filter (IEKF) on SO(3)** that
fuses a gyroscope, an accelerometer and a magnetometer. It runs on an
**STM32WB55** (P-NUCLEO-WB55) with the **X-NUCLEO-IKS4A1** sensor shield: the
LSM6DSV16X provides gyroscope + accelerometer, the LIS2MDL the magnetometer.
The filter was first developed and benchmarked in MATLAB, then ported to C on
FreeRTOS + CMSIS-DSP, inside a multi-rate architecture:
- 960 Hz propagation;
- 240 Hz accelerometer update;
- ~100 Hz magnetometer update.

The rate and attitude control loops are already wired in and empty.

Two things make the repository more than a firmware project:

- **Validation against the reference filter.** The firmware streams the
  *exact* inputs of every filter call. The PC replays them through the
  reference filter and checks that the embedded estimate is the same, up to
  float32 vs double. This was done on the board for the original equations
  and for every embedded improvement (§11).
- **Two equivalent toolchains.** All PC tools exist in **MATLAB**
  (`01 - Tools/00 - Matlab/`) and in **Python** (`01 - Tools/01 - Python/`, for
  users without MATLAB).
  The two give the same numbers on the same data, and their logs are
  interchangeable.

| | |
|---|---|
| **Vehicle** | quadrotor, 4 × brushed coreless 8520 motors, 1S LiPo; attitude control only (no GPS, no position control) |
| **Target** | P-NUCLEO-WB55 (NUCLEO-WB55RG: Cortex-M4F @ 64 MHz + Cortex-M0+ radio core) |
| **Sensors** | X-NUCLEO-IKS4A1 in **Mode 3**: LSM6DSV16X (gyro + acc) on the MCU I²C bus, LIS2MDL (mag) read through the LSM6DSV16X sensor hub |
| **Software** | STM32CubeIDE / CubeMX, FreeRTOS (CMSIS-RTOS2), CMSIS-DSP; PC tools: MATLAB R2019b+ or Python ≥ 3.9 |
| **Status** | **Phase 1 (attitude estimation) concluded.** Both validation tests passed **on the board**: base equations vs the original `iekfAttitude.m` (≤ 0.003 σ), full embedded filter vs `iekfAttitudeEmb.m` (≤ 0.002 σ), see §11.5. **Phase 2 (BLE telemetry)**: notifications working on the board; PC viewers over BLE ready, board test pending (§12). Motors disabled. |

---

## Contents

1. [Introduction](#1-introduction)
2. [Repository structure](#2-repository-structure)
3. [Vehicle model and dynamics](#3-vehicle-model-and-dynamics)
4. [Theory: the original IEKF](#4-theory-the-original-iekf)
5. [From MATLAB to embedded: improvements](#5-from-matlab-to-embedded-improvements)
6. [Firmware architecture](#6-firmware-architecture)
7. [Telemetry protocol](#7-telemetry-protocol)
8. [Building and flashing](#8-building-and-flashing)
9. [PC tools (MATLAB and Python): setup and live viewer](#9-pc-tools-matlab-and-python-setup-and-live-viewer)
10. [Magnetometer calibration](#10-magnetometer-calibration)
11. [Telemetry capture and embedded-vs-reference comparison](#11-telemetry-capture-and-embedded-vs-reference-comparison)
12. [BLE telemetry (phase 2)](#12-ble-telemetry-phase-2)
13. [Troubleshooting](#13-troubleshooting)
14. [Lessons learned](#14-lessons-learned)
15. [Next steps](#15-next-steps)
16. [References](#16-references)

---

## 1. Introduction

The goal of the project is **attitude control of a small quadrotor**
(4 × brushed coreless 8520 motors, 1S). Position and trajectory are out of
scope and there is no GPS: the estimator has to provide only the **attitude**
(and the gyroscope bias), from a gyroscope, an accelerometer and a
magnetometer.

The work is organised in phases:

| Phase | Content | Status |
|---|---|---|
| 1 | Embedded attitude estimation: IEKF on SO(3), multi-rate FreeRTOS pipeline, telemetry, PC tools, validation against the reference filter | **concluded** |
| 2 | Wireless telemetry (BLE on the Cortex-M0+ core) to the PC, then a mobile app | **in progress**: BLE link working on the board, PC tools ready (§12); mobile app planned |
| 3 | Control: rate loop at 960 Hz, attitude loop at 240 Hz, motor power stage, arming/fail-safe, thrust model in the estimator | planned |

How to read this document:

| Sections | Content |
|---|---|
| §3–§4 | the physics: the vehicle, its sensors, the filter |
| §5–§8 | how the filter was made to work on the microcontroller, and how to build it |
| §9–§11 | how to use the PC tools to see, calibrate and validate the estimate |

The theory sections give each equation in the form used by the code, with the
name of the file that implements it.

---

## 2. Repository structure

The repository has two numbered top-level folders: the firmware and the PC
tools.

```
stm32_iekf_attitude_fw/
├── 00 - STM32_FW/                  STM32CubeIDE project of the firmware (import this folder)
│   ├── App/                        application code, hardware-independent where possible
│   │   ├── app/                    app_main (FreeRTOS tasks, IRQ glue), app_config.h (ALL parameters),
│   │   │                           est_pipeline (multi-rate estimation), tlm_build (telemetry frames)
│   │   ├── iekf/                   IEKF on SO(3): so3 (Exp/Log, orthonormalisation), iekf (filter),
│   │   │                           accwin (rotation-compensated acc mean), calib (static calibration window)
│   │   ├── drivers/                sensor_bus.h (abstract bus) + imu_lsm6dsv16x (IMU, sensor hub, FIFO parser)
│   │   ├── platform/               imu_i2c: STM32 HAL I²C + DMA binding of the sensor bus;
│   │   │                           ble_tlm_svc: GATT service + TX thread on STM32_WPAN (BLE)
│   │   ├── telemetry/              frame format and CRC (telemetry.h), UART + DMA transport;
│   │   │                           tlm_ble: BLE queue + link state (host-testable)
│   │   ├── control/                control hooks (weak, empty) and motor PWM (disabled in phase 1)
│   │   └── third_party/            ST lsm6dsv16x-pid driver (BSD-3), CMSIS-DSP headers and library
│   ├── Core/                       CubeMX-generated: main.c, interrupts, HAL MSP, FreeRTOS config, startup,
│   │                               radio-core infrastructure (app_entry, hw_timerserver, app_conf.h)
│   ├── STM32_WPAN/App/             CubeMX-generated BLE application (app_ble.c with our 3 hooks, §12.5)
│   ├── Drivers/, Middlewares/      CubeMX-generated: STM32WB HAL, CMSIS, FreeRTOS, ST STM32_WPAN (BLE)
│   ├── Utilities/                  CubeMX-generated: low-power manager used by STM32_WPAN
│   ├── IEKFcfg.ioc                 CubeMX configuration (pins, clocks, DMA, FreeRTOS)
│   ├── IEKFcfg.launch              STM32CubeIDE debug configuration (ST-LINK, SWD)
│   ├── .project, .cproject, .mxproject, .settings/   STM32CubeIDE project files
│   └── STM32WB55RGVX_FLASH.ld, ..._RAM.ld   linker scripts
├── 01 - Tools/                     PC tools, in two equivalent flavours
│   ├── 00 - Matlab/                MATLAB tools (R2019b+)
│   │   ├── initPath.m              adds every folder below to the path
│   │   ├── Viewer/                 iekfLiveViewer.m (live 3D triad + logging, USB or BLE), quat2rotmLocal.m
│   │   ├── Telemetry/              parseTelemetry, decode*, crc16ccitt, statusDiagnosis
│   │   ├── MagCalib/               calibMagAxes.m (axis map + hard-iron), fitHardIron.m, magCheck.m
│   │   └── FiltersComparison/      tlmCapture, tlmSniffer, compareEmbeddedVsMatlab
│   │       ├── IEKF_originalVersion/   ORIGINAL filter developed in MATLAB: iekfAttitude.m, expSO3, logSO3, skewSym
│   │       └── IEKF_embeddedVersion/   EXTENDED filter: iekfAttitudeEmb.m + iekfCore*.m (MATLAB mirror of the C code)
│   └── 01 - Python/                Python equivalents of the MATLAB tools (package iekf_tools, CLI iekf-tools)
│       ├── iekf_tools/             viewer, capture + sniffer, compare, magcalib, telemetry,
│       │                           ble_io (BLE link, bleak), filters/ (original + embedded mirror),
│       │                           logio (.mat logs), simulate
│       ├── tests/                  pytest suite: parity with the MATLAB outputs, recorded captures
│       ├── pyproject.toml, requirements.txt
│       └── README.md               installation, MATLAB ↔ Python map, commands
├── .gitignore                      build outputs (Debug/, Release/) are not versioned
└── README.md                       this file: theory, firmware, procedures, results
```

**Paths in this document.** Short paths are relative to the folder they
belong to:

| Paths like | Are relative to |
|---|---|
| `App/…`, `Core/…`, `IEKFcfg.ioc` | `00 - STM32_FW/` |
| `Viewer/…`, `MagCalib/…`, `FiltersComparison/…`, `initPath.m` | `01 - Tools/00 - Matlab/` |
| `iekf_tools/…`, `tests/…` | `01 - Tools/01 - Python/` |

**Three implementations of the same filter**, each with its role:

| Implementation | Role |
|---|---|
| `App/iekf/` (C, float32) | what flies |
| `FiltersComparison/IEKF_originalVersion/` (MATLAB) | the filter as first designed, kept **unchanged** as the reference of the equations |
| `FiltersComparison/IEKF_embeddedVersion/` (MATLAB) | MATLAB mirror of the C code, written line by line after `iekf.c`, with every improvement of §5 |

`iekf_tools/filters/` (Python) ports both MATLAB filters, matching them to
~1e-14.

**Where to start:**

| I want to... | Where |
|---|---|
| understand the vehicle and its sensors | §3 |
| understand the filter | §4 (original IEKF), §5 (embedded improvements) |
| build and flash the firmware | §6 (architecture), §8 (build), `App/app/app_config.h` (parameters) |
| see the attitude live / record logs | §9: MATLAB `iekfLiveViewer`, Python `iekf-tools viewer` |
| calibrate or check the magnetometer | §10: MATLAB `calibMagAxes` / `magCheck`, Python `iekf-tools calib-mag` / `mag-check` |
| validate the embedded filter | §11: MATLAB `compareEmbeddedVsMatlab`, Python `iekf-tools compare` |
| use the BLE link | §12: MATLAB `iekfLiveViewer("ble")`, Python `iekf-tools viewer --ble` |
| use the tools without MATLAB | [`01 - Tools/01 - Python/README.md`](01%20-%20Tools/01%20-%20Python/README.md) |
| fix a problem | §13 (troubleshooting) |

---

## 3. Vehicle model and dynamics

The estimator itself uses only kinematics: the gyroscope is its input and no
vehicle parameter appears in the filter. The vehicle dynamics still matter for
three reasons:
- they decide what the accelerometer and the magnetometer *actually* measure
  in flight;
- they set the rates of the multi-rate design;
- they are the plant of the controllers of phase 3.

This section collects the model and the conventions shared by firmware, tools
and documentation.

### 3.1 Frames and conventions

- **World frame $\mathcal W$: NWU**. $x$ = magnetic north (horizontal), $y$ =
  west, $z$ = up. It is built by the firmware at start-up from the static
  calibration (§5.3): $z$ along the measured "up" (opposite to gravity), $x$
  along the horizontal component of the measured magnetic field.
- **Body frame $\mathcal B$**: the LSM6DSV16X axes, which are the axes of the
  board. With the board level and the components on top, $z_b$ is up, so at
  rest the accelerometer reads $+1\,g$ on $z$. On the vehicle the board is to
  be mounted with $x_b$ forward, $y_b$ left, $z_b$ up (FLU), consistent with
  NWU.
- **Attitude** $R \in SO(3)$ maps body vectors into the world frame,
  $v_{\mathcal W} = R\,v_{\mathcal B}$. The firmware transmits the equivalent
  unit quaternion $q = [w\;x\;y\;z]$ (body → world). Euler angles are used
  **only** for display: ZYX, yaw–pitch–roll, $R = R_z(\psi)R_y(\theta)R_x(\phi)$.
- **Normalised units.** In the filter, gravity and magnetic field are unit
  directions. The accelerometer is in $g$, the magnetometer in µT, both
  normalised before each update.

### 3.2 Attitude kinematics

$$
\dot R = R\,[\omega]_\times, \qquad
\dot q = \tfrac12\, q \otimes \begin{bmatrix}0\\ \omega\end{bmatrix},
$$

where $\omega$ is the angular velocity **in the body frame**, which is exactly
what the gyroscope measures. Over one step $h$ with constant $\omega$ the
exact solution is

$$
R_{k+1} = R_k\,\mathrm{Exp}(\omega h).
$$

The firmware applies it with the *angle increment* $\Delta\theta =
\sum \omega_i\,\Delta t$ of the gyro samples in the step (§5.1). These
kinematics are **group-affine**: this property is what makes the invariant
filter of §4 possible.

### 3.3 Rigid-body dynamics

With mass $m$, inertia $J$ (body frame, diagonal for a symmetric frame),
world position $p$ and total thrust $T$ along $z_b$:

$$
m\,\ddot p = -m g\,e_3 + R\,\begin{bmatrix}0\\0\\T\end{bmatrix} + f_{drag},
\qquad
J\dot\omega = -\,\omega\times J\omega + \tau + \tau_{dist},
$$

where $e_3 = [0\;0\;1]^\top$ (up), $g = 9.81\ \mathrm{m/s^2}$, and $\tau$ is the
control torque produced by the rotors. The terms $\omega\times J\omega$
(gyroscopic coupling of the body) and the rotor gyroscopic torques are small
on a symmetric micro-quad at the rates of interest. They are kept in the model
and neglected in a first controller design.

### 3.4 Rotors and mixer

Each rotor $i$ spins at $\Omega_i$. It produces a thrust along $z_b$ and a
reaction torque about $z_b$:

$$
T_i = k_T\,\Omega_i^2, \qquad Q_i = k_Q\,\Omega_i^2 = \kappa\,T_i,\quad \kappa = k_Q/k_T .
$$

The rotor sits at $r_i = d\,[\cos\psi_i\;\sin\psi_i\;0]^\top$, at distance $d$
from the centre and azimuth $\psi_i$. Let $\sigma_i = +1$ if the propeller
turns counter-clockwise seen from above (the reaction on the frame is
clockwise), $-1$ otherwise. Then

$$
T = \sum_i T_i, \qquad
\tau = \sum_i r_i \times \begin{bmatrix}0\\0\\T_i\end{bmatrix}
      - \kappa\sum_i \sigma_i T_i\,e_3 .
$$

For the X configuration, take rotors at $\psi = 45°, 135°, 225°, 315°$
(front-left, rear-left, rear-right, front-right in FLU), with diagonal rotors
turning the same way, $\sigma = (+1, -1, +1, -1)$:

$$
\begin{bmatrix}T\\ \tau_x\\ \tau_y\\ \tau_z\end{bmatrix} =
\underbrace{\begin{bmatrix}
1 & 1 & 1 & 1\\
\tfrac{d}{\sqrt2} & \tfrac{d}{\sqrt2} & -\tfrac{d}{\sqrt2} & -\tfrac{d}{\sqrt2}\\
-\tfrac{d}{\sqrt2} & \tfrac{d}{\sqrt2} & \tfrac{d}{\sqrt2} & -\tfrac{d}{\sqrt2}\\
-\kappa & \kappa & -\kappa & \kappa
\end{bmatrix}}_{M}
\begin{bmatrix}T_1\\T_2\\T_3\\T_4\end{bmatrix}.
$$

The **mixer** of the controller is $M^{-1}$: from the demanded $(T, \tau)$ to
the per-rotor thrusts, which are then saturated and converted to PWM. The rotor
numbering and the turning directions are a convention to be fixed when the
frame is assembled. The formula above holds for any choice once $\psi_i$ and
$\sigma_i$ are set.

### 3.5 Brushed motor dynamics

The 8520 coreless motors are driven by low-side PWM: duty $u \in [0,1]$ on a
1S battery of voltage $V_{bat}$ (4.2 → ~3.3 V during a flight).

- The electrical time constant is negligible against the mechanical one.
- The rotor speed follows a **first-order lag**,
  $$
  \dot\Omega_i = \frac{\Omega_{ss}(u_i V_{bat}) - \Omega_i}{\tau_m},
  $$
  with $\tau_m$ of the order of tens of milliseconds.
- The steady-state speed grows roughly linearly with the applied voltage
  $u V_{bat}$, so thrust grows roughly with $(u V_{bat})^2$.

Two consequences shape the control design of phase 3:
- the actuator lag $\tau_m$ limits the useful bandwidth of the rate loop;
- the same duty gives less thrust as the battery discharges, so the mixer
  output should be compensated with the measured $V_{bat}$.

$k_T$, $k_Q$, $\tau_m$ and the speed curve have to be **identified on a thrust
stand** with the actual propellers. No values are assumed here.

### 3.6 Sensor models

The firmware calibrates or estimates every term below, except noise and the
disturbances, which the filter has to tolerate.

| Sensor | Model (body frame) | Calibrated / estimated in |
|---|---|---|
| Gyroscope | $\omega_m = \omega + b + n_g$, $\dot b = n_b$ | bias $b$: static calibration (§5.3) + online in the filter state (§4) |
| Accelerometer | $a_m = R^\top(\ddot p + g\,e_3) + n_a = \dfrac{1}{m}\begin{bmatrix}0\\0\\T\end{bmatrix} + \dfrac{1}{m}R^\top f_{drag} + n_a$ | scale via nominal magnitude at rest (§5.3) |
| Magnetometer | $m_m = M_{map}^{-1}\big(R^\top m_{\mathcal W} + h + d(t)\big) + n_m$ | axis map $M_{map}$ and hard-iron $h$: §10; disturbances $d(t)$: dip check (§5.7) |

The accelerometer measures the **specific force**, not gravity:
- at rest or in steady hover, $\ddot p = 0$ and $a_m = R^\top g\,e_3$, the
  "up" direction in the body frame. This is the measurement the filter uses
  (§4.4);
- in any acceleration, $a_m$ is mostly thrust along $z_b$ plus drag. For an
  ideal quadrotor the $x,y$ components carry only the rotor drag
  $\approx -\mu\,v_{\mathcal B}$, whatever the tilt;
- a filter that trusted the accelerometer blindly would therefore estimate
  "level" during a sustained turn.

The firmware handles this in two ways. It de-weights the accelerometer when
the measured magnitude or the innovation are inconsistent with gravity
(adaptive noise and NIS inflation, §5.7). It also trusts it fully only when the
vehicle is detected still (§5.7). A thrust/drag model in the measurement
equation is a phase-3 item (§15).

The magnetometer sees, besides the Earth field:
- the constant field of the board and of the nearby parts (the **hard-iron**
  $h$, ~4× the Earth field on this board, §10);
- in flight, the field of the **motor currents**, which grows with thrust.

The dip check (angle between measured field and measured gravity) de-weights
the magnetometer when the field is disturbed.

### 3.7 Rates of the multi-rate design

| Loop / signal | Rate | Why |
|---|---|---|
| Gyro + acc sampling (LSM6DSV16X ODR) | 960 Hz | ~10× the bandwidth of a micro-quad rate loop; gyro integrated without aliasing the vibrations |
| Propagation (§4.3) and rate loop (phase 3) | 960 Hz | the rate loop needs the freshest bias-corrected gyro |
| Accelerometer update and attitude loop (phase 3) | 240 Hz | mean of 4 samples rotated into a common frame (§5.2): less vibration noise, 4× less CPU |
| Magnetometer update | ~100 Hz (LIS2MDL), on new data only | yaw is slow; the sample age is compensated (§5.4) |

---

## 4. Theory: the original IEKF

This section documents the filter as implemented in
`FiltersComparison/IEKF_originalVersion/iekfAttitude.m` (MATLAB tools). The embedded filter implements the
same equations (§5 lists what was added on top).

### 4.1 State and error definition

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

### 4.2 Process model

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

### 4.3 Propagation (discrete, step $h$)

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

### 4.4 Vector measurements

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

### 4.5 Update and retraction

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

### 4.6 Filter dynamics: what is observable, how fast it corrects

**Error dynamics as a linear system.** Between updates the error evolves as
$x_{k+1} = F_k x_k + w_k$, with $F_k$ from §4.3. At an update,
$x^+ = (I - K_k H_k)\,x^-$. Both $F_k$ and $H_k$ depend only on the measured
rate and on the *predicted* measurement direction $\hat y = \hat R^\top v$,
never on how wrong $\hat R$ is. The closed-loop error dynamics are therefore
the same for small and large errors. This is the practical benefit of the
invariant formulation: the covariance stays consistent and the filter
converges from any initial attitude without tuning for it.

**Observability.** $[\hat y]_\times$ has rank 2, and its null space is
$\hat y$ itself: a vector measurement corrects the two error components
orthogonal to the measured direction and says nothing about rotations
*about* it.

| Measurement | Corrects | Cannot see |
|---|---|---|
| gravity (accelerometer) | roll and pitch | yaw, i.e. rotations about the vertical |
| magnetic field (magnetometer) | rotations about axes orthogonal to the field: mainly yaw, plus part of the tilt unless tilt-compensated | rotations about the field direction |
| both, non-parallel | the full attitude | (singular only where the field is vertical, i.e. at the magnetic poles) |

The gyro bias enters through the coupling $-hI$ in $F$: a bias produces an
attitude drift that the measurements see, so it becomes observable.
- With gravity alone, the bias component about the vertical is unobservable,
  because the yaw drift it causes is invisible.
- The magnetometer makes the full bias observable.

The *tilt-compensated* magnetometer (§4.4) makes the separation explicit: the
magnetometer only acts on yaw, and a disturbed field cannot tilt the
estimate.

**Correction bandwidth.** The filter can be read as a complementary filter
whose crossover is set by the noise ratio. Take one axis, small angles, gyro
noise density $\sigma_g$ (rad/s/√Hz, $Q = \sigma_g^2 h$) and accelerometer
updates at $f_u$ with direction noise $\sigma_a$ (rad). The steady-state
Kalman gain gives a first-order correction of the gyro-integrated attitude
with bandwidth and steady-state uncertainty

$$
\omega_c \simeq \frac{\sigma_g\sqrt{f_u}}{\sigma_a},
\qquad
\sigma_{\delta,ss} \simeq \sqrt{\frac{\sigma_g\,\sigma_a}{\sqrt{f_u}}} .
$$

With the firmware values ($\sigma_g$ = 0.5 °/s/√Hz, $f_u$ = 240 Hz):

| $\sigma_a$ | $\omega_c$ | crossover | steady-state σ |
|---|---|---|---|
| 0.01 (floor of the static calibration, §5.3) | 13.5 rad/s | 2.2 Hz | 0.13° |
| 0.02 | 6.8 rad/s | 1.1 Hz | 0.19° |
| 0.2, i.e. $R$ inflated ×100 by the NIS logic (§5.7) | 0.68 rad/s | 0.11 Hz | 0.61° |

The values were checked against the iterated scalar Riccati equation.

Below the crossover the attitude follows the accelerometer and magnetometer,
above it the gyroscope. The adaptive noise and the NIS inflation of §5.7 work
by *moving the crossover down* while the measurement is not trustworthy, down
to ~0.1 Hz at the cap. In that window the estimate is essentially
gyro-integrated, and the gyro bias error sets how fast it drifts.

---

## 5. From MATLAB to embedded: improvements

Each item lists *why* it was needed. Unless stated otherwise all numbers come
from the PC simulation of the full firmware pipeline (same C code, synthetic
LSM6DSV16X FIFO encoded byte by byte).

### 5.1 Multi-rate structure with angle increments

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

### 5.2 Rotation-compensated accelerometer mean (`accwin`)

Averaging the acc over a 4 ms window while rotating at 360 °/s smears the
direction and lags by half a window (**0.66°** error). Each sample is moved to
the body frame at the start of the window with the already-propagated
increments $E_j$, averaged, and moved back:

$$
\bar a = D_N^\top\,\frac1N\sum_i D_i\,a_i, \qquad D_i = E_1 E_2 \cdots E_i .
$$

Error after compensation: **0.000°**.

### 5.3 Static calibration and levelled world frame

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

### 5.4 Magnetometer through the sensor hub, with latency compensation

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

### 5.5 Single-precision numerics

- `so3_exp` switches to a 4th-order Taylor series below 0.05 rad: in float32,
  $(1-\cos\theta)/\theta^2$ loses all digits for $\theta \sim 10^{-3}$, the typical
  $\omega h$.
- $\hat R$ is re-orthonormalised (Gram–Schmidt) after every product.
- **Covariance conditioning** after every propagation and update: exact
  symmetrisation and a floor on the diagonal ($10^{-12}$ rad², $10^{-16}$ (rad/s)²).
  The Joseph form guarantees $P \succeq 0$ only in exact arithmetic.
- The 6×6 covariance algebra uses CMSIS-DSP (`arm_mat_*`); static workspaces keep
  large matrices off the RTOS task stack.

### 5.6 Error reset after the retraction

After $\hat R \leftarrow \hat R\,\mathrm{Exp}(\delta\theta)$ the error is defined
w.r.t. the *new* estimate, $\delta' \approx \delta - \delta\theta - \tfrac12[\delta\theta]_\times\delta$, hence

$$
J = I - \tfrac12[\delta\theta]_\times, \qquad P_{aa} \leftarrow J P_{aa} J^\top, \qquad P_{ab} \leftarrow J P_{ab}.
$$

Negligible at steady state, relevant for large corrections (`IEKF_RESET_JACOBIAN`).

### 5.7 Robustness to external accelerations

The magnitude test of §4.4 misses accelerations **perpendicular** to gravity:
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

### 5.8 Tuning notes

- **6 states, not 3.** With a realistic bias: RMSE 0.095° (6 states) vs 0.184°
  (3 states). `IEKF_ESTIMATE_BIAS = 0` reduces the filter exactly to 3 states.
- **`IEKF_SIGMA_GYRO = 0.5 °/s/√Hz`.** It must cover what the model ignores
  (scale factor, misalignment, g-sensitivity), not just the sensor noise
  (~0.003 °/s/√Hz). With 1 % scale error and 1° misalignment: σ = 0.1 → RMSE 1.17°,
  σ = 0.5 → 0.42°. Lower it only after calibrating the gyro.

### 5.9 MATLAB mirror of the C code

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

## 6. Firmware architecture

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

## 7. Telemetry protocol

Frame (little-endian): `A5 5A | TYPE | LEN | PAYLOAD[LEN] | CRC16 (LSB, MSB)`,
CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD.

| Type | Name | Size | Rate | Content |
|---|---|---|---|---|
| 0x01 | STATE | 71 B | 240 Hz (30 Hz with REPLAY) | seq, time, q, bias, acc, gyro, mag, flags |
| 0x02 | INIT | 137 B | after calibration, then ~1 Hz | initial state, references, all filter parameters |
| 0x03 | STATUS | 62 B (54 B before phase 2) | 10 Hz, **always** | IMU init result, IRQ/bus/FIFO counters, calibration progress, drops, BLE link state/MTU/drops/last error |
| 0x04 | REPLAY | ≤ 212 B | 240 Hz (only with `APP_TLM_REPLAY = 1`) | exact filter calls of the window + resulting q, b, diag P |

REPLAY operations: `'P' dθ[3] h` (propagation), `'A' acc[3] trusted`
(acc update), `'M' mag[3] extraVar` (mag update), all float32.

Bandwidth: ~19 kB/s normally, ~44 kB/s with REPLAY (921600 baud ≈ 92 kB/s).
The same frames travel over BLE as a byte stream, STATE decimated and no
REPLAY (§12). The decoders read both STATUS lengths, so phase-1 captures
still work.

> **Pitfall.** Telemetry structs are `packed`: float fields are not 4-byte
> aligned. Never pass the address of such a field to a function that writes
> floats: Cortex-M4F uses `VSTR`, which faults on unaligned addresses. Write to a
> local variable and `memcpy`.

---

## 8. Building and flashing

1. The folder `00 - STM32_FW` is an STM32CubeIDE project: *File → Import →
   General → Existing Projects into Workspace*, select `00 - STM32_FW`. Build
   outputs (`Debug/`) are not versioned: the first build regenerates them. To change
   pins or clocks, open `IEKFcfg.ioc` in CubeMX and regenerate (user code is
   kept). Relevant settings already in the `.ioc`: SYSCLK 64 MHz; I2C1 on PB8/PB9 with
   DMA RX and event/error IRQs; USART1 921600 baud with DMA TX (**Memory increment
   enabled**) and global IRQ; PA15 EXTI rising edge (`IMU_INT1`); FreeRTOS
   CMSIS_V2, 1 kHz tick, 52 KB heap; HAL timebase on a TIM; Serial Wire debug;
   STM32_WPAN (BLE) with HSEM, IPCC, RTC, RNG, RF (§12.4).
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
6. BLE: the wireless stack must be flashed once on the Cortex-M0+ (§12.6). Without
   it, the radio core never starts. Set `APP_TLM_BLE 0` to run on USB only.

Bring-up check in *Live Expressions*: `s_irqCount` grows by ~480/s at 960 Hz,
`s_busErr` stays 0, `s_est.running` becomes 1 about 2 s after reset with the board
still. `s_procCycMax / 64` = worst-case processing time in µs.

All parameters live in `App/app/app_config.h`. The switches that matter for the
procedures below:

| Macro | Normal use | Comparison with MATLAB |
|---|---|---|
| `APP_TLM_REPLAY` | 0 | 1 |
| `APP_COMPARE_ORIGINAL` | 0 | 1 (base-equation test vs original) or 0 (extended-filter test) |
| `MAG_AXIS_MAP`, `MAG_HARD_IRON` | from §10 | unchanged |
| `APP_TLM_BLE` | 1 (BLE + USB) or 0 (USB only) | unchanged (REPLAY is USB-only) |
| `APP_BLE_STATE_DECIM` | 4 (STATE at 60 Hz over BLE) | — |

---

## 9. PC tools (MATLAB and Python): setup and live viewer

Every tool exists in MATLAB and in Python, with the same options and outputs.
MATLAB setup:

```matlab
run('<repo>/01 - Tools/00 - Matlab/initPath.m')   % once per session
```

Python setup (details in [`01 - Tools/01 - Python/README.md`](01%20-%20Tools/01%20-%20Python/README.md)):

```bash
cd "01 - Tools/01 - Python"
python -m pip install -e .        # installs the iekf-tools command
```

`initPath.m` adds all folders (tools, original filter, extended filter) to the
path and prints which `iekfAttitude.m` is in use (it must be the one in
`FiltersComparison/IEKF_originalVersion`; `compareEmbeddedVsMatlab` warns if
another copy shadows it).

**Live viewer** (firmware with `APP_TLM_REPLAY = 0`):

```matlab
L = iekfLiveViewer("serial", "COM6");                 % 921600 baud by default
L = iekfLiveViewer("serial", "COM6", 921600, "C:\logs") % choose the log folder
L = iekfLiveViewer("ble");                            % BLE, board found automatically (§12.8)
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

**Python equivalents** (installation in [`01 - Tools/01 - Python/README.md`](01%20-%20Tools/01%20-%20Python/README.md)):

```bash
iekf-tools viewer COM6                  # live viewer; --log-dir DIR, --baud B
iekf-tools viewer --replay file.bin     # replay a recorded session (no board needed)
iekf-tools viewer --ble                 # BLE (pip install bleak), board found automatically (§12.8)
iekf-tools sniff COM6                   # byte-level link check
```

The Python viewer shows the same panel and writes the same `.mat` log (variable
`L`) and raw `.bin`. Logs are interchangeable between MATLAB and Python.

---

## 10. Magnetometer calibration

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

**Without re-flashing.** A log recorded with an already calibrated firmware
can also be used: pass that firmware's values, and the raw field is
reconstructed. The printed values are always absolute: paste them as they are.

```matlab
calibMagAxes(log, [], [1 0 0, 0 -1 0, 0 0 -1], [100.31 -0.56 -166.07])
```

> **Pitfall.** On a log of a calibrated firmware, calling `calibMagAxes(log)`
> without the firmware values returns ≈ identity and ≈ 0: that is the
> residual *correction*, not the calibration. Pasting it removes the real map
> and offset. The yaw then locks onto a fixed body axis, the magnetometer is
> always de-weighted, and at rest the same axis points the same way whatever
> the orientation.

`fitHardIron.m` (sphere fit) is kept for a quick hard-iron-only check once the
axis map is known.

**Checking the calibration without re-flashing.** `magCheck.m` works on an
ordinary viewer log recorded with the current firmware: board still for 2 s,
then ~30 s of slow rotations about all axes.

```matlab
out = magCheck('iekf_log_YYYYMMDD_HHMMSS.mat', [100.31 -0.56 -166.07])  % current MAG_HARD_IRON
```

It fits a sphere to the logged (already corrected) field and compares how much
the field direction turns with how much the gyro turns. Possible verdicts:
- **consistent**: residual offset ≈ 0, |B| 25–65 µT;
- **`MAG_HARD_IRON` no longer matches**: it prints the corrected value; the axis
  map is unchanged;
- **constant field**: the LIS2MDL is not delivering new data.

**Python**, same algorithms and same numbers (checked on the same log against
the MATLAB functions), with logs from either viewer:

```bash
iekf-tools calib-mag iekf_log_....mat                       # firmware with identity map, zero offset
iekf-tools calib-mag iekf_log_....mat --map 1,0,0,0,-1,0,0,0,-1 --hard-iron 100.31,-0.56,-166.07
iekf-tools fit-hard-iron iekf_log_....mat
iekf-tools mag-check iekf_log_....mat --hard-iron 100.31,-0.56,-166.07
```

---

## 11. Telemetry capture and embedded-vs-reference comparison

### 11.1 Principle

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

### 11.2 Two validation tests

| `APP_COMPARE_ORIGINAL` | Firmware behaviour | Reference filter | Meaning of the test |
|---|---|---|---|
| **1** | extensions of §5.4–4.7 that the original lacks are disabled | original `iekfAttitude.m` | **base equations**: the C port of the original filter is exact → MATCH / DIVERGE |
| **0** | full configuration (as flown) | extended `iekfAttitudeEmb.m` | **every improvement**: NIS inflation and memory, dip check, stillness detector, mag-age variance, error reset, lockout protection → MATCH / DIVERGE. The original is also run, to show how much the extensions change the estimate. |

### 11.3 Procedure

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

Python, same procedure and same report:

```bash
iekf-tools capture COM6 --duration 60          # then reset the board immediately
iekf-tools compare iekf_cap_YYYYMMDD_HHMMSS.bin
```

The Python port of both filters matches the MATLAB code to ~1e-14, and
`compare` reproduces the figures of `compareEmbeddedVsMatlab` on the same
captures (`01 - Tools/01 - Python/tests`).

### 11.4 Reading the result

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

### 11.5 Results

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
reference at 0.002 σ. It measures how much the §5.7 extensions change the
estimate in real use.

**Phase 1 validation: complete.**
- The C port of the original equations is exact (mode 1, board).
- Every embedded improvement is implemented as specified (mode 0, board).

In both tests the only residual is float32 vs double.

> After validation, set `APP_TLM_REPLAY 0` and `APP_COMPARE_ORIGINAL 0` again in
> `app_config.h`: this is the configuration for normal use and for the live viewer.

---

## 12. BLE telemetry (phase 2)

**Goal.** Same behaviour as the USB link, over Bluetooth Low Energy:
- the PC receives the same STATE, INIT and STATUS frames;
- the same viewers, MATLAB and Python, show the attitude;
- the same `.mat` and `.bin` logs are written.

The USB link keeps working unchanged. REPLAY and the embedded-vs-reference
comparison (§11) stay USB-only. The GATT service is designed so that a mobile
app can use it as it is.

**Status:**

| Item | State |
|---|---|
| Notifications on the board | **working**, checked with nRF Connect (frames starting with `A5 5A`) |
| Firmware | compiles and links with the CubeIDE settings (§12.9) |
| PC viewers over BLE (MATLAB and Python) | tested with simulated notifications; test on the board pending |

### 12.1 Architecture

```
ImuTask (Realtime, unchanged)
   │ qTlm
   ▼
TelemetryTask (BelowNormal) ── tx_frame() ──► UART DMA ─► ST-LINK VCP      (all frames)
   │                                    └──► tlm_ble ring (2 KB, lock-free SPSC)
   │                                          STATE decimated, INIT, STATUS, no REPLAY
   ▼
BleTxThread (ble_tlm_svc.c, BelowNormal)
   every 10 ms or on TX_POOL_AVAILABLE: peek ≤ ATT_MTU−3 bytes
   → aci_gatt_update_char_value() → notification ─► IPCC ─► Cortex-M0+ BLE stack ─► radio
```

| File | Role |
|---|---|
| `App/telemetry/tlm_ble.c/.h` | BLE queue (lock-free ring), link state, STATE decimation, commands. No HAL, no BLE stack: host-testable |
| `App/platform/ble_tlm_svc.c/.h` | GATT service on STM32_WPAN, event handler, TX thread |
| `STM32_WPAN/App/app_ble.c` | CubeMX-generated BLE application; three one-line hooks in USER CODE sections (§12.5) |
| `Core/Src/app_entry.c`, `hw_timerserver.c`, … | CubeMX-generated infrastructure of the radio core (IPCC, timer server, system commands) |

Design choices:
- **Byte stream, not packets.** A notification carries up to ATT_MTU−3 bytes
  of the frame stream and may cut frames anywhere. The receiver concatenates
  notifications and re-synchronises on `A5 5A` + CRC with the usual parser.
  It works with any MTU (23 by default, 185 on iOS, up to 247 on PCs) without
  a fragmentation protocol. Only the first notification after subscribing is
  guaranteed to start at a frame boundary.
- **ImuTask is never involved.** The BLE work is in TelemetryTask (queueing)
  and BleTxThread (sending).
- **Flow control.** When the link is slower than the data,
  `aci_gatt_update_char_value()` returns `BLE_STATUS_INSUFFICIENT_RESOURCES`.
  The chunk is kept and retried on `ACI_GATT_TX_POOL_AVAILABLE`. If the ring
  fills up, *whole frames* are dropped and counted in `STATUS.ble_drop`.
- **Thread safety.** In the FreeRTOS version of `app_ble.c`, `BLE_StatusNot()`
  takes `MtxHciId` around every HCI command, so ACI calls from BleTxThread are
  serialised with the stack's. The GATT event handler, which runs in the
  stack's thread, only sets flags.

### 12.2 GATT service

| Item | UUID | Properties | Content |
|---|---|---|---|
| Service "IEKF Telemetry" | `7E4A0001-2C6B-4F8E-9A5D-3B1C0E1D2F30` | primary | |
| TLM | `7E4A0002-…` | Notify | frame byte stream, ≤ ATT_MTU−3 bytes per notification |
| CMD | `7E4A0003-…` | Write, Write w/o response | `[0x01]` resend INIT · `[0x02, N]` STATE every N acc updates (1…240) |

When a client enables notifications, the board:
1. clears the queue;
2. sends INIT immediately;
3. requests a 15–30 ms connection interval, a 4 s supervision timeout, an MTU
   exchange and 251-byte link-layer packets.

These values are within the iOS rules and are accepted by Windows, macOS and
Linux. The central has the last word.

### 12.3 Bandwidth

| Frame | Size on air | Rate over BLE | Bytes/s |
|---|---|---|---|
| STATE | 77 B | 60 Hz (`APP_BLE_STATE_DECIM` = 4) | 4620 |
| STATUS | 68 B | 10 Hz | 680 |
| INIT | 143 B | ~1 Hz, plus once on subscribe | ~143 |
| **Total** | | | **~5.4 kB/s** |

- With MTU 23 (20 B per notification), ~270 notifications/s. If `ble_drop`
  grows, use decimation 8 (30 Hz).
- With MTU ≥ 185, one notification carries 2–3 frames.

### 12.4 CubeMX configuration

The `.ioc` in the repository already has the BLE middleware enabled. For
reference, and to check after a regeneration:

1. **Clocks.**
   - HSE 32 MHz and LSE 32.768 kHz crystals on.
   - RTC clock and RF wake-up clock from **LSE**.
   - SYSCLK 64 MHz.
2. **Peripherals.**
   - **HSEM** and **IPCC** activated, with the IPCC C1 RX/TX interrupts.
   - **RTC**: clock source activated, internal wake-up (timer server of the
     stack).
   - **RNG** and **RF** activated.
3. **STM32_WPAN.**
   - *BLE* enabled, server profile, **Heart Rate** template. It is kept
     because, in the CubeMX version used, it is the template that exposes the
     advertised name (`LOCAL_NAME`); with the Custom Template the field is not
     available. The Heart Rate service stays registered alongside ours, unused.
   - The generated `hrs_app.c` has three unused functions (compiler warnings).
     They are outside USER CODE, so silence them for that file only:
     *Properties → C/C++ Build → Settings → MCU GCC Compiler → Miscellaneous*,
     add `-Wno-unused-function -Wno-unused-variable` (Debug and Release).
   - Our service is added in code (`ble_tlm_svc.c`), not in the *BLE GATT* tab.
   - Debug trace **off**: it would use USART1, our telemetry port.
   - Low power **off** (`CFG_LPM_SUPPORTED` = 0): stop mode would halt the
     I²C/DMA of the 960 Hz loop.
4. **FreeRTOS.** Heap ≥ 48 KB (52000 in the repository): the stack's threads
   take ~10 KB. Tickless idle **off**.
5. **NVIC.** Every interrupt that calls RTOS functions needs priority ≥ 5
   (IMU EXTI, IPCC, HSEM, RTC wake-up).

Settings to review in CubeMX. The repository still has the CubeMX defaults
for:
- the advertised **name**: `STM32WB`. Set it to **`IEKF`**: CubeMX accepts only
  5 characters here, because the name shares the 31-byte advertising packet
  with the template's data. The PC tools recognise any name starting with
  `IEKF`, and `STM32WB` too;
- `CFG_BLE_MAX_ATT_MTU`: **156** (247 gives fewer notifications on PCs);
- the **PLL source**: MSI (HSE is preferable, for the UART baud accuracy too).

### 12.5 Firmware hooks

`STM32_WPAN/App/app_ble.c` has three calls, all in USER CODE sections, so they
survive regeneration:

```c
/* USER CODE BEGIN Includes */
#include "ble_tlm_svc.h"

/* APP_BLE_Init(): after SVCCTL_Init(), before advertising starts */
/* USER CODE BEGIN APP_BLE_Init_3 */
  ble_tlm_svc_init();

/* SVCCTL_App_Notification(), case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
/* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */
  ble_tlm_on_connect(p_connection_complete_event->Connection_Handle);

/* SVCCTL_App_Notification(), case HCI_DISCONNECTION_COMPLETE_EVT_CODE */
/* USER CODE BEGIN EVT_DISCONN_COMPLETE */
  ble_tlm_on_disconnect();
```

`APP_TLM_BLE` in `app_config.h` switches the BLE path:
- **1** (repository default): the BLE path is active;
- **0**: everything compiles without using the stack, and STATUS reports
  `ble_state` = 0.

### 12.6 Wireless stack on the Cortex-M0+

The radio core runs a binary from ST that is flashed separately with
STM32CubeProgrammer:

1. Take its version from the **same** STM32CubeWB package that CubeMX uses: a
   mismatch is the most common reason why CPU2 never starts.
2. The files are in `Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x/` of
   that package:
   - the stack, `stm32wb5x_BLE_Stack_full_fw.bin`;
   - the FUS, if it is older than the stack requires.
3. In *Firmware Upgrade Services*:
   1. *Start FUS*, *Read FUS infos*. Update the FUS first if needed.
   2. *Delete FW*.
   3. *Firmware Upgrade* with the stack at the **start address given in
      `Release_Notes.html` for STM32WB5xxG (1 MB)**. It changes from version to
      version.
   4. *Start Wireless Stack*.
4. The application uses at most 512 KB of flash (`STM32WB55RGVX_FLASH.ld`),
   below the stack.

### 12.7 Bring-up checklist

1. **USB viewer first.** The text panel shows `BLE: advertising`: the service
   is registered and the stack is running.
   - No `BLE` line: the stack did not start. Check the CPU2 firmware (§12.6).
   - `err 0x..`: last stack error. `0x64`/`0x98` at start-up means too little
     GATT attribute space.
2. **Phone, with nRF Connect or LightBlue.**
   1. Scan **without filters**: scanner filters apply to the advertising
      packet, never to the notifications.
   2. Connect and enable notifications on `7E4A0002-…`: the data starts with
      `A5 5A`.
   3. The USB viewer now shows `BLE: streaming`, the MTU and `drop 0`.
3. **960 Hz loop.** `s_irqCount` (~480/s) and `s_procCycMax` in *Live
   Expressions* must behave as without BLE.
4. **Disconnect the phone**, then connect from the PC (§12.8). A peripheral
   accepts one central at a time.

### 12.8 PC tools over BLE

MATLAB (R2019b+, Windows 10+/macOS, Bluetooth 4.0+ adapter):

```matlab
L = iekfLiveViewer("ble");                 % board found automatically, STATE 60 Hz
L = iekfLiveViewer("ble", "IEKF", 8); % by name or address, STATE 30 Hz
blelist                                    % what is advertising
```

Python (`pip install bleak`, or `pip install -e ".[ble]"`; Windows, macOS,
Linux/BlueZ):

```bash
iekf-tools scan                            # boards advertising, marked "<-- IEKF board"
iekf-tools viewer --ble                    # board found automatically, STATE 60 Hz
iekf-tools viewer --ble IEKF --decim 8
iekf-tools capture --ble --duration 30     # no graphics: received rates every 2 s
```

How the tools behave:
- **Discovery.** It picks the strongest device whose name starts with `IEKF`
  or equals `STM32WB` (the CubeMX default).
- **On connection.** The tools subscribe to TLM and write the decimation to
  CMD.
- **Processing.** Notifications are appended as bytes and go through the same
  parser as the serial port. Log, `.bin` and panel are identical to USB; the
  panel adds a `BLE:` line with state, MTU, drops and notifications received.
- **`lost`.** It counts only real gaps of the decimated stream.

### 12.9 Verification

| Check | Result |
|---|---|
| Notifications on the board, nRF Connect | frames `A5 5A …` received |
| Full firmware build with ARM GCC 13.3, include paths and defines read from `.cproject` (87 C files + startup, linked with `STM32WB55RGVX_FLASH.ld` and CMSIS-DSP) | 0 errors; only warnings: 3 unused functions in the generated `hrs_app.c`. Debug flags (`-O0 -g3`): 107 KB flash, 73 KB RAM; `-O2`: 66 KB flash |
| `ble_tlm_svc.c` against the STM32_WPAN headers | all ACI/HCI functions, events and structs used exist with that signature |
| Host test of the BLE queue: 60 s of frames, random MTU 23–247, 30 % refused sends | bytes received = frames queued; overload drops whole frames only |
| MATLAB viewer, BLE path with simulated notifications (Octave) | 660 STATE at step 4, 0 lost, INIT/STATUS decoded, `.bin` identical, CMD `[2 4]` written |
| Python: 5 BLE tests (`tests/test_ble.py`) | STATUS 54/62 B, board discovery, reassembly from 20-byte notifications, viewer on a decimated stream (0 lost, BLE line), a real gap counted once |

### 12.10 Troubleshooting (BLE)

| Symptom | Cause / action |
|---|---|
| The phone does not list the board | scanner filter on data: remove it (§12.7); reset the board (after 60 s the advertising interval grows to 1–2.5 s) |
| No `BLE` line in the USB viewer | CPU2 stack missing or not matching the CubeWB package (§12.6) |
| USB telemetry garbled after enabling BLE | stack debug trace on USART1: trace off |
| SWD lost, IMU stops | low power on: `CFG_LPM_SUPPORTED` = 0, tickless idle off |
| HardFault / stack overflow at start-up | FreeRTOS heap too small |
| `multiple definition of HAL_GPIO_EXTI_Callback` after a regeneration | keep a single definition, calling `app_exti(pin)` |
| `ble_drop` grows | link slower than the data: decimation 8, larger MTU |
| PC viewer over BLE: rate below 60 Hz, `lag` growing, `lost 0` and `drop 0` | the PC delivers the notifications late (nothing is lost on the link). The Python tools receive BLE in a separate process for this reason (since the BLE support was added); update them. The `BLE:` line shows the notifications/s actually received, `draw` the redraw time. `iekf-tools capture --ble` measures the link without graphics: if it shows 60 STATE/s and a stable lag, the limit is the viewer's redraw on that PC |
| PC tools: board not found | it is connected to another central (phone): disconnect it. Python on Linux: `bluetoothd` running |

### 12.11 Towards a mobile app

The service is already what an iOS/Android app needs:
- 128-bit UUIDs;
- notify plus write-without-response;
- connection parameters within Apple's rules;
- an MTU-agnostic stream.

The app has to:
1. subscribe to `7E4A0002-…`;
2. append each notification to a buffer;
3. port `parseTelemetry` (about 40 lines);
4. decode STATE for the attitude, with STATUS and INIT for diagnostics.

CMD is the place for future commands: arming, gains, logging.

---

## 13. Troubleshooting

| Symptom | Cause / action |
|---|---|
| LED1 blinking every 200 ms | `imu_init` failing: jumpers (Mode 3), I²C pins, power |
| No bytes at all | wrong COM port, port held by another program (`clear all`), firmware halted |
| Only `A5 A5 A5 …` | USART1 TX DMA without *Memory increment* |
| Bytes but no valid frame | baud mismatch or SYSCLK ≠ 64 MHz |
| One frame, then nothing | UART TX-complete callback not wired / USART1 IRQ disabled |
| Only STATUS frames | filter never started: `statusDiagnosis` (IMU init, no INT1, no mag from hub, board moving during calibration) |
| HardFault ~2 s after reset | unaligned float write into a packed telemetry struct (§7) |
| Attitude "converges slowly" in the viewer | viewer lagging: check the *lag* field; disable REPLAY or use `tlmCapture` |
| Yaw moves when tilting | wrong `MAG_AXIS_MAP` / hard-iron → §10 |
| `iekfAttitude` not found | run `initPath.m` (in `01 - Tools/00 - Matlab`) |
| Build errors with paths that no longer exist, after moving or re-cloning the project | stale makefiles in the local `Debug/` folder: delete it (or *Project → Clean*), re-import `00 - STM32_FW` and build again |
| Comparison: "INIT of this boot was lost" | UART saturated during the capture; repeat |
| Euler plots jump by 180° | gimbal lock (|pitch| ≈ 90°); use the quaternion figure |
| `undefined reference to arm_mat_*` | CMSIS-DSP GCC library not linked (§8) |

---

## 14. Lessons learned

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

## 15. Next steps

Phase 1 (embedded attitude estimation, telemetry, MATLAB tools and
validation) is concluded. Open items for the next phases:


- Gyro scale and misalignment calibration (turntable or known rotations), then
  lower `IEKF_SIGMA_GYRO`.
- Soft-iron (ellipsoid) magnetometer calibration.
- Verify the magnetometer latency $\bar\tau$ on the bench (constant-rate rotation
  about the vertical) — the LIS2MDL internal filter delay is not modelled.
- I²C Fast-mode Plus and 1920 Hz sampling (check bus edges on a scope first).
- Phase 2: test the PC viewers over BLE on the board (§12.7), set the advertised
  name and the ATT MTU in CubeMX (§12.4), then the mobile app (§12.11).
- Phase 3: rate PID at 960 Hz with notch-filtered gyro, attitude loop at 240 Hz,
  motor power stage, arming and fail-safe, and a **thrust model** to reject
  sustained accelerations in the acc update.

---

## 16. References

- A. Barrau, S. Bonnabel, *The Invariant Extended Kalman Filter as a Stable
  Observer*, IEEE TAC, 2017.
- R. Mahony, T. Hamel, J.-M. Pflimlin, *Nonlinear Complementary Filters on the
  Special Orthogonal Group*, IEEE TAC, 2008.
- S. Madgwick, A. Harrison, R. Vaidyanathan, *Estimation of IMU and MARG
  orientation using a gradient descent algorithm*, ICORR, 2011.
- STMicroelectronics: UM3239 (X-NUCLEO-IKS4A1), UM2435 (NUCLEO-WB55),
  LSM6DSV16X and LIS2MDL datasheets, `lsm6dsv16x-pid` driver, AN5289 (STM32WB
  BLE).
