# Vehicle model and dynamics


The estimator itself uses only kinematics: the gyroscope is its input and no
vehicle parameter appears in the filter. The vehicle dynamics still matter for
three reasons:

- they decide what the accelerometer and the magnetometer *actually* measure
  in flight;
- they set the rates of the multi-rate design;
- they are the plant of the controllers of phase 3.

This section collects the model and the conventions shared by firmware, tools
and documentation.

## Frames and conventions

- **World frame $\mathcal W$: NWU**. $x$ = magnetic north (horizontal), $y$ =
  west, $z$ = up. It is built by the firmware at start-up from the static
  calibration ([Static calibration and levelled world frame](iekf-embedded.md#static-calibration-and-levelled-world-frame)): $z$ along the measured "up" (opposite to gravity), $x$
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

## Attitude kinematics

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
\sum \omega_i\,\Delta t$ of the gyro samples in the step ([Multi-rate structure with angle increments](iekf-embedded.md#multi-rate-structure-with-angle-increments)). These
kinematics are **group-affine**: this property is what makes the invariant
filter of [Attitude IEKF](iekf.md) possible.

## Rigid-body dynamics

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

## Rotors and mixer

Each rotor $i$ spins at $\Omega_i$. It produces a thrust along $z_b$ and a
reaction torque about $z_b$:

$$
T_i = k_T\,\Omega_i^2, \qquad Q_i = k_Q\,\Omega_i^2 = \kappa\,T_i,\quad \kappa = k_Q/k_T .
$$

The firmware uses the Betaflight quad-X numbering, the same as the app
(HANDOVER_FW §10.7). With $\ell = d/\sqrt2$ the lever of each rotor about the
$x$ and $y$ axes:

| Motor | Position (FLU) | Turns (seen from above) | Reaction on the frame |
|---|---|---|---|
| M1 | rear right $(-\ell, -\ell)$ | CW | CCW (+z) |
| M2 | front right $(+\ell, -\ell)$ | CCW | CW (−z) |
| M3 | rear left $(-\ell, +\ell)$ | CCW | CW (−z) |
| M4 | front left $(+\ell, +\ell)$ | CW | CCW (+z) |

$$
\begin{bmatrix}T\\ \tau_x\\ \tau_y\\ \tau_z\end{bmatrix} =
\underbrace{\begin{bmatrix}
1 & 1 & 1 & 1\\
-\ell & -\ell & \ell & \ell\\
\ell & -\ell & \ell & -\ell\\
\kappa & -\kappa & -\kappa & \kappa
\end{bmatrix}}_{M}
\begin{bmatrix}T_1\\T_2\\T_3\\T_4\end{bmatrix}
$$

with the FLU signs of the controller: $\tau_x > 0$ rolls right wing down
(left motors up), $\tau_y > 0$ pitches nose down (rear motors up), $\tau_z > 0$
yaws counter-clockwise (the CW propellers M1, M4 up).

**Normalised form (firmware).** The controller does not work in newtons. It
commands a collective $t$ and torques $u_{x,y,z}$ in **normalised thrust per
motor** (fraction of the maximum thrust $T_{max}$ of one motor at 4.2 V per
cell, ADR 0010 §1):

$$
t_i = t + \begin{bmatrix}-1 & -1 & 1 & 1\end{bmatrix}_i u_x
        + \begin{bmatrix}1 & -1 & 1 & -1\end{bmatrix}_i u_y
        + \begin{bmatrix}1 & -1 & -1 & 1\end{bmatrix}_i u_z
$$

so a unit of torque command produces $\tau_x = 4\ell T_{max} u_x$ and
$\tau_z = 4\kappa T_{max} u_z$. The gains of the rate loop are therefore
independent of the battery and of the motor curve, which the mixer removes
([Attitude control and mixer](../firmware/control.md)). Saturation keeps the attitude first: roll/pitch,
then yaw, then the collective is moved (airmode).

## Brushed motor dynamics

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

**Thrust model used by the firmware (ADR 0010 §1).** With $u' = u\,V_{cell}/4.2$
the duty referred to a full cell,

$$
\frac{T_i}{T_{max}} = f(u') = \theta\,u'^2 + (1-\theta)\,u',
$$

$\theta$ = `thrCurve` (1 = purely quadratic). The mixer inverts it:
$u_i = f^{-1}(t_i)\cdot 4.2/V_{cell}$, with $V_{cell}$ the sagged cell voltage
of the BattTask. The model neglects the rotor-speed dependence of the
efficiency and the change of $T_{max}$ with temperature; what is left is
absorbed by the hover-thrust estimate ([Hover-thrust Kalman filter](hover-kf.md)) and the
integrators.

Two consequences shape the control design:

- the actuator lag $\tau_m$ limits the useful bandwidth of the rate loop: the
  rate loop crosses over below $1/\tau_m$;
- the same duty gives less thrust as the battery discharges: compensated in
  the mixer, so the hover thrust stays constant over the pack.

## Design numbers (assumed)

The controllers and the host tests use the numbers below. They are **estimates
from similar 1S micro quads**, not measurements. $k_T$, $\kappa$, $\tau_m$,
$\theta$ and $T_{max}$ have to be identified on a thrust stand with the actual
propellers, and the gains re-tuned from flight logs (issue #42: calibration /
auto-calibration of every tunable parameter).

| Quantity | Value | Used in |
|---|---|---|
| mass $m$ | 71 / 82 / 90 g (T400 / T800 / T1200, ADR 0010); 75 g in the host tests | hover thrust |
| inertia $J_{xx}, J_{yy}, J_{zz}$ | 3.5e-5, 3.5e-5, 6e-5 kg m² | rate-loop gains, `test_ctrl.c` |
| lever per axis $\ell$ | 28 mm (40 mm arm / √2) | torque per unit command |
| yaw torque ratio $\kappa$ | 12 mm | yaw authority |
| maximum thrust per motor $T_{max}$ | 0.343 N at 4.2 V (thrust-to-weight ≈ 1.9) | hover thrust $mg/4T_{max}$ ≈ 0.54 in the test plant (seed 0.51, ADR 0010) |
| motor lag $\tau_m$ | 35 ms (pole ≈ 28 rad/s) | rate-loop crossover |
| thrust curve $\theta$ | 1 (quadratic) | mixer |
| roll/pitch acceleration per unit torque | $4\ell T_{max}/J_{xx}$ ≈ 1100 rad/s² | rate P |
| yaw acceleration per unit torque | $4\kappa T_{max}/J_{zz}$ ≈ 270 rad/s² | yaw P |
| vertical acceleration per unit collective | $g/t_h$ ≈ 19 m/s² | vertical-loop gains |
| hover thrust $t_h$ | 0.51 (T400), 0.58 (T800), 0.65 (T1200), +0.04 with GPS | `hoverThr` seed |

**Simplifications made in the controllers** (kept in the plant of the tests
where noted):

- rigid body; $\omega\times J\omega$ neglected in the design (in the test plant);
- rotor gyroscopic torques and blade flapping neglected;
- rotor drag $\approx -\mu v$ neglected in the controllers; it biases the
  hover-thrust model in fast climbs, hence its $|v_z| < 0.5$ m/s gate;
- ground effect neglected: the hover estimate is not updated below 0.15 m;
- thrust independent of the airspeed (no inflow model): valid at the low
  speeds of the indoor models.

## Sensor models

The firmware calibrates or estimates every term below, except noise and the
disturbances, which the filter has to tolerate.

| Sensor | Model (body frame) | Calibrated / estimated in |
|---|---|---|
| Gyroscope | $\omega_m = \omega + b + n_g$, $\dot b = n_b$ | bias $b$: static calibration ([Static calibration and levelled world frame](iekf-embedded.md#static-calibration-and-levelled-world-frame)) + online in the filter state ([Attitude IEKF](iekf.md)) |
| Accelerometer | $a_m = R^\top(\ddot p + g\,e_3) + n_a = \dfrac{1}{m}\begin{bmatrix}0\\0\\T\end{bmatrix} + \dfrac{1}{m}R^\top f_{drag} + n_a$ | scale via nominal magnitude at rest ([Static calibration and levelled world frame](iekf-embedded.md#static-calibration-and-levelled-world-frame)) |
| Magnetometer | $m_m = M_{map}^{-1}\big(R^\top m_{\mathcal W} + h + d(t)\big) + n_m$ | axis map $M_{map}$ and hard-iron $h$: [Magnetometer calibration](../tools/mag-calibration.md); disturbances $d(t)$: dip check ([Robustness to external accelerations](iekf-embedded.md#robustness-to-external-accelerations)) |

The accelerometer measures the **specific force**, not gravity:

- at rest or in steady hover, $\ddot p = 0$ and $a_m = R^\top g\,e_3$, the
  "up" direction in the body frame. This is the measurement the filter uses
  ([Vector measurements](iekf.md#vector-measurements));
- in any acceleration, $a_m$ is mostly thrust along $z_b$ plus drag. For an
  ideal quadrotor the $x,y$ components carry only the rotor drag
  $\approx -\mu\,v_{\mathcal B}$, whatever the tilt;
- a filter that trusted the accelerometer blindly would therefore estimate
  "level" during a sustained turn.

The firmware handles this in two ways. It de-weights the accelerometer when
the measured magnitude or the innovation are inconsistent with gravity
(adaptive noise and NIS inflation, [Robustness to external accelerations](iekf-embedded.md#robustness-to-external-accelerations)). It also trusts it fully only when the
vehicle is detected still ([Robustness to external accelerations](iekf-embedded.md#robustness-to-external-accelerations)). A thrust/drag model in the measurement
equation is a phase-3 item ([Roadmap](../project/roadmap.md)).

The magnetometer sees, besides the Earth field:

- the constant field of the board and of the nearby parts (the **hard-iron**
  $h$, ~4× the Earth field on this board, [Magnetometer calibration](../tools/mag-calibration.md));
- in flight, the field of the **motor currents**, which grows with thrust.

The dip check (angle between measured field and measured gravity) de-weights
the magnetometer when the field is disturbed.

## Rates of the multi-rate design

| Loop / signal | Rate | Why |
|---|---|---|
| Gyro + acc sampling (LSM6DSV16X ODR) | 960 Hz | ~10× the bandwidth of a micro-quad rate loop; gyro integrated without aliasing the vibrations |
| Propagation ([Propagation (discrete, step $h$)](iekf.md#propagation-discrete-step-h)) and rate loop (phase 3) | 960 Hz | the rate loop needs the freshest bias-corrected gyro |
| Accelerometer update and attitude loop (phase 3) | 240 Hz | mean of 4 samples rotated into a common frame ([Rotation-compensated accelerometer mean (`accwin`)](iekf-embedded.md#rotation-compensated-accelerometer-mean-accwin)): less vibration noise, 4× less CPU |
| Magnetometer update | ~100 Hz (LIS2MDL), on new data only | yaw is slow; the sample age is compensated ([Magnetometer through the sensor hub, with latency compensation](iekf-embedded.md#magnetometer-through-the-sensor-hub-with-latency-compensation)) |
