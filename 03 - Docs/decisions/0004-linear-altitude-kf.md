# 0004. Altitude: a linear Kalman filter in cascade on the attitude

- **Status:** Accepted
- **Date:** 2026-10-27

## Context

TAKEOFF, LAND, the hold modes and the ceiling need height and vertical speed.
The sources are a barometer (bench and FC), a ToF on the ACwing Flow board,
and the accelerometer. The attitude IEKF is validated against its MATLAB
reference and should not change.

## Decision

- A separate **linear** Kalman filter with the state $[h, v, b_b, b_a]$
  (`App/filters/altitude_kf/alt_est.c`).
- It is a cascade: the vertical acceleration is computed from the IEKF
  attitude, outside the filter.
- The pressure is turned into an ISA altitude, and the ToF range into a tilt
  corrected height, before the update. Every measurement is linear in the
  state.
- The barometer bias is frozen when the ToF is not available (it is not
  observable), and the floor is re-anchored when the ToF steps.

Theory: [Altitude Kalman filter](../theory/altitude-kf.md).

## Consequences

- The IEKF stays exactly the validated filter.
- No Jacobians and no linearisation error; about 150 multiply-adds per
  prediction.
- The same filter serves the models with and without GNSS.
- Cost: the height does not correct the attitude. This is negligible for a
  hovering multirotor.
- The folder is `altitude_kf`, not `altitude_ekf`, because the filter is not
  extended.

## Alternatives considered

- **Extend the IEKF state with height and vertical speed.** One filter, but
  the validated filter would change, and its cost would grow.
- **Complementary filter.** Simpler, but it has no bias estimation with ToF,
  no consistent gating and no uncertainty.
- **EKF with the raw pressure in the measurement.** Possible later, but it
  adds nothing while the ISA conversion is exact at these heights.
