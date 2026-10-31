# Decisions (ADR)

Each **Architecture Decision Record** explains one choice that shapes the
code: the context, what was decided, what it costs, and what was rejected.
The records are short and are not rewritten. When a decision changes, a new
record replaces the old one, and the old one is marked *Superseded*.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-iekf-on-so3.md) | Attitude: invariant EKF on SO(3) | Accepted |
| [0002](0002-multirate-pipeline.md) | Multi-rate estimation pipeline driven by the IMU FIFO | Accepted |
| [0003](0003-flight-core-in-imu-task.md) | The flight core runs in the ImuTask, isolated from BLE | Accepted |
| [0004](0004-linear-altitude-kf.md) | Altitude: a linear Kalman filter in cascade on the attitude | Accepted |
| [0005](0005-drivers-and-bsp.md) | Portable `drivers/` separate from board-specific `bsp/` | Accepted |
| [0006](0006-flow-bridge-mcu.md) | A bridge MCU for the PMW3901, firmware on registers | Accepted |
| [0007](0007-battery-task.md) | Battery monitor in its own task, decisions in the flight core | Accepted |
| [0008](0008-flash-log-and-usb-cdc.md) | Flight log as a sector ring, downloaded over USB CDC | Accepted |

**New record.** Copy the template below to `NNNN-short-title.md`, add it to
this table and to `nav` in `mkdocs.yml`.

```markdown
# NNNN. Title

- **Status:** Proposed | Accepted | Superseded by NNNN
- **Date:** YYYY-MM-DD

## Context
## Decision
## Consequences
## Alternatives considered
```
