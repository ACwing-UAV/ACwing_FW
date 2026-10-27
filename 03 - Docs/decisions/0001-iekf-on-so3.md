# 0001. Attitude: invariant EKF on SO(3)

- **Status:** Accepted
- **Date:** phase 1 (recorded on 2026-10-27)

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

- The error dynamics do not depend on the estimate (invariance). The Jacobians
  are the same along any trajectory, and convergence from large errors is
  good, which a quaternion EKF cannot promise.
- A 6-state filter with 3-component vector updates fits easily in float32 at
  960 Hz.
- The reference is a MATLAB filter kept unchanged. The firmware streams the
  exact filter inputs so that the C code can be compared to it on the board
  ([Validation against the reference](../tools/validation.md)).

## Alternatives considered

- **Complementary / Mahony / Madgwick filters.** Lighter, but they have no
  covariance and give no consistent uncertainty for gating, and their gains
  are tuned by hand.
- **Multiplicative EKF on quaternions.** It works, but its linearisation
  depends on the estimate, and it is less robust to large initial errors.
- **Full navigation filter (attitude + velocity + position).** Out of scope
  for phase 1, and the expansion sensors did not exist yet.
