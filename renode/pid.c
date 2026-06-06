/* pid.c - portable PID implementation
 * Derivative-on-measurement (no derivative kick on setpoint step)
 * Back-calculation anti-windup (unwinds integrator on output saturation)
 */
#include "pid.h"

void pid_init(PID *p, float kp, float ki, float kd,
              float v_limit, float ctrl_dt) {
    p->kp        = kp;
    p->ki        = ki;
    p->kd        = kd;
    p->v_limit   = v_limit;
    p->ctrl_dt   = ctrl_dt;
    p->integ     = 0.0f;
    p->prev_meas = 0.0f;
}

float pid_update(PID *p, float setpoint, float meas) {
    float err  = setpoint - meas;

    /* Proportional */
    float out_p = p->kp * err;

    /* Integral (pre-clamp accumulation) */
    p->integ += p->ki * err * p->ctrl_dt;

    /* Derivative on measurement - avoids kick on setpoint step */
    float out_d = -p->kd * (meas - p->prev_meas) / p->ctrl_dt;
    p->prev_meas = meas;

    float out = out_p + p->integ + out_d;

    /* Back-calculation anti-windup: unwind integrator by the excess */
    if (out >  p->v_limit) { p->integ -= (out - p->v_limit); out =  p->v_limit; }
    if (out < -p->v_limit) { p->integ -= (out + p->v_limit); out = -p->v_limit; }

    return out;
}
