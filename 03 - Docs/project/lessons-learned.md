# Lessons learned


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
- **A level interrupt on an edge-triggered EXTI needs a way out.** INT1 is high
  while the FIFO is above the watermark. If a read ends with the FIFO still
  above it, no new edge ever comes and the loop silently degrades to polling.
  Phase 1 never hit it; enabling BLE, with a slower start-up, did. The reader
  must drain the FIFO below the watermark.
- **The sensor's clock is not the MCU's clock.** The IMU ODR comes from its
  internal RC oscillator, ~5 % off on this board. The integration step must be
  measured against the MCU crystal over long windows, not assumed and not
  taken from single reads.
- **Process noise is a model-error budget.** The gyro σ that worked was 150×
  the datasheet noise density because it also covers scale and alignment errors.
