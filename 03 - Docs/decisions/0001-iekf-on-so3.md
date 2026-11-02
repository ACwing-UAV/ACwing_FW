# 0001. Attitude: invariant EKF on SO(3)

- **Status:** Accepted
- **Date:** phase 1 (recorded on 2026-10-27; revised on 2026-11-02 after the c4dynamics review)

## Context

The drone needs its attitude from a gyroscope, an accelerometer and a
magnetometer, on a Cortex-M4F at a rate high enough for a 960 Hz rate loop.
The attitude must stay right through fast rotations and large errors, for
example after a bump or a start-up in an unknown orientation.

## Decision

An **Invariant Extended Kalman Filter on SO(3)**:

- the state is the rotation $R$ and the gyroscope bias $b$;
- the error is left-invariant, $R = \hat R\,\mathrm{Exp}(\delta)$;
- the accelerometer and the magnetometer are vector measurements.

Theory: [Attitude IEKF](../theory/iekf.md). Implementation and the embedded
extensions: [IEKF on the microcontroller](../theory/iekf-embedded.md).

## Consequences

- The propagation of the error does not depend on the attitude estimate
  (group-affine kinematics). The measurement Jacobian does, through the
  predicted direction $\hat R^\top v$. Convergence from large errors is much
  better than with an Euler-angle EKF up to about 90°, slower beyond, and not
  guaranteed near 180° (measured by the cold-start test, see
  [IEKF on the microcontroller](../theory/iekf-embedded.md#cold-start-and-re-initialisation)).
  What matters is the group representation (a rotation matrix or a
  quaternion) instead of Euler angles, not the storage.
- A 6-state filter with 3-component vector updates fits easily in float32 at
  960 Hz.
- The reference is a MATLAB filter kept unchanged. The firmware streams the
  exact filter inputs so that the C code can be compared to it on the board
  ([Validation against the reference](../tools/validation.md)).

## Alternatives considered

- **Complementary / Mahony / Madgwick filters.** Lighter, but they have no
  covariance and give no consistent uncertainty for gating, and their gains
  are tuned by hand.
- **Multiplicative EKF on quaternions.** Equivalent in structure for an
  attitude-only filter with a body-frame error. Rejected for simplicity: no
  unit-norm constraint, no double cover, and the measurement model
  $y = R^\top v$ is direct.
- **Right-invariant error** $\eta = \hat R R^\top$. The Jacobian of a world
  direction seen in the body frame becomes $[v]_\times$, independent of
  $\hat R$. Kept as an option to test in MATLAB if large-error convergence
  becomes a requirement (hand launch).
- **Full navigation filter (attitude + velocity + position).** Out of scope
  for phase 1, and the expansion sensors did not exist yet.
