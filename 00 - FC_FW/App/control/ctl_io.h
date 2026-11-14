/**
 * @file  ctl_io.h
 * @brief The firmware side of ctl_api.h (ADR 0013): what the producers feed, the dispatch to the active law, and the
 *        command the actuator layer takes. Laws do not include this header: they use ctl_api.h only.
 *
 *   fc_core       ctl_io_feed_vehicle(), ctl_io_feed_hover(), ctl_io_set_ref(), then ctl_io_outer()
 *   ctrl_loop     ctl_io_feed_attitude() + ctl_io_attitude() at 240 Hz,
 *                 ctl_io_feed_rates() + ctl_io_rate() + ctl_io_take_cmd() at 960 Hz,
 *                 ctl_io_feed_cell_voltage(), ctl_io_set_level_trim()
 * All in the flight-control context (ImuTask): no lock.
 */
#ifndef CTL_IO_H
#define CTL_IO_H

#include "ctl_api.h"

/** vehicle state from the estimators, fed by the flight manager at its rate */
typedef struct {
    uint8_t h_valid;
    float   h, vz, a_up;
    float   cos_tilt, yaw;
    uint8_t vel_valid;
    float   vel_b[2];
} ctl_vehicle_t;

/** the command of one rate step */
enum { CTL_CMD_NONE = 0, CTL_CMD_THRUST_TORQUE = 1, CTL_CMD_MOTORS = 2 };
typedef struct {
    uint8_t kind;
    float   collective, torque[3];
    float   motor[4];
} ctl_cmd_t;

/** registers the default law (index 0) and activates the one of the parameter ctlLaw */
void    ctl_io_init(void);

void    ctl_io_feed_vehicle(const ctl_vehicle_t *v);
void    ctl_io_feed_hover(float hover);
void    ctl_io_feed_attitude(const float q[4]);
void    ctl_io_feed_rates(const float w[3]);
void    ctl_io_feed_cell_voltage(float v_cell);
void    ctl_io_set_level_trim(float roll_rad, float pitch_rad);
void    ctl_io_set_ref(const ctl_ref_t *ref);

/** outer step of the active law. Disarmed, a change of ctlLaw activates the new law first (never armed) */
void    ctl_io_outer(float dt);
void    ctl_io_attitude(float dt);
void    ctl_io_rate(float dt);
/** the command set during the last rate step; CTL_CMD_NONE if the law set none (the actuators write duty 0) */
void    ctl_io_take_cmd(ctl_cmd_t *cmd);

#endif /* CTL_IO_H */
