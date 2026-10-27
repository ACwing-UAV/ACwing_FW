/**
 * @file  alt_ctrl.h
 * @brief Vertical control: height -> climb rate -> collective thrust.
 *
 *   vz_sp  = clamp(kp_h * (h_sp - h), -vz_dn, +vz_up)     (or from the stick)
 *   thrust = (hover + kp_v * (vz_sp - vz) + I) / cos(tilt), I += ki_v * e * dt
 *
 * The integrator learns the hover thrust. Output 0..1 (collective), clamped.
 * Host-testable.
 */
#ifndef ALT_CTRL_H
#define ALT_CTRL_H

typedef struct {
    float hover;            /**< initial hover thrust guess (0..1)        */
    float kp_h, kp_v, ki_v;
    float thr_min, thr_max;
    float i_lim;
    float integ;
} alt_ctrl_t;

void  alt_ctrl_init(alt_ctrl_t *c, float hover);
void  alt_ctrl_reset(alt_ctrl_t *c, float integ);
float alt_ctrl_vz_sp(const alt_ctrl_t *c, float h_sp, float h, float vz_up, float vz_dn);
float alt_ctrl_step(alt_ctrl_t *c, float vz_sp, float vz, float cos_tilt, float dt);

#endif /* ALT_CTRL_H */
