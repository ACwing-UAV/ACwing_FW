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

Two consequences shape the control design of phase 3:

- the actuator lag $\tau_m$ limits the useful bandwidth of the rate loop;
- the same duty gives less thrust as the battery discharges, so the mixer
  output should be compensated with the measured $V_{bat}$.

$k_T$, $k_Q$, $\tau_m$ and the speed curve have to be **identified on a thrust
stand** with the actual propellers. No values are assumed here.

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
