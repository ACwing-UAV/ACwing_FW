/**
 * @file  tlm_build.h
 * @brief Builds telemetry frames from the pipeline state.
 *        Portable (no HAL/RTOS): used by the firmware and by the PC test, which
 *        therefore produces capture files byte-identical to the board's.
 */
#ifndef TLM_BUILD_H
#define TLM_BUILD_H

#include <stddef.h>
#include <stdint.h>
#include "est_pipeline.h"
#include "telemetry.h"

size_t tlm_build_state (const est_t *e, uint8_t *frame, size_t cap);
size_t tlm_build_init  (const est_t *e, float fs_state_hz, uint8_t *frame, size_t cap);
size_t tlm_build_replay(const est_t *e, uint8_t *frame, size_t cap);

#endif /* TLM_BUILD_H */
