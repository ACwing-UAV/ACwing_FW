/**
 * @file  est_config.h
 * @brief The production configuration of the attitude estimator, built from
 *        app_config.h in one place, so the firmware and the host tests run the
 *        same numbers.
 */
#ifndef EST_CONFIG_H
#define EST_CONFIG_H

#include "est_pipeline.h"

/** Fills every field from app_config.h, including APP_COMPARE_ORIGINAL.
 *  Not set here (board data): mag_map, mag_hard_iron, mag_scale. */
void app_est_config(est_config_t *ec);

#endif /* EST_CONFIG_H */
