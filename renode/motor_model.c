/* motor_model.c - DC motor RK4 integrator (same ODE as host-sim + Wokwi) */
#include "motor_model.h"
#include <math.h>

void motor_init(Motor *m) {
    m->i     = 0.0f;
    m->omega = 0.0f;
}

/* Compute ODE derivatives at state (i, w) with applied voltage v */
static void deriv(float i, float w, float v,
                  float *di_out, float *dw_out) {
    *di_out = (v - MOTOR_R * i - MOTOR_KE * w) / MOTOR_L;
    *dw_out = (MOTOR_KT * i - MOTOR_BV * w)   / MOTOR_J;
}

/* Advance plant by (micro_steps * MOTOR_DT) seconds using RK4 */
void motor_step(Motor *m, float v_cmd, uint32_t micro_steps) {
    /* Clamp to supply rail */
    if (v_cmd >  MOTOR_VS) v_cmd =  MOTOR_VS;
    if (v_cmd < -MOTOR_VS) v_cmd = -MOTOR_VS;

    float dt = MOTOR_DT;
    float ci = m->i, cw = m->omega;

    for (uint32_t s = 0; s < micro_steps; ++s) {
        float k1i, k1w, k2i, k2w, k3i, k3w, k4i, k4w;
        deriv(ci,               cw,               v_cmd, &k1i, &k1w);
        deriv(ci+0.5f*dt*k1i,  cw+0.5f*dt*k1w,  v_cmd, &k2i, &k2w);
        deriv(ci+0.5f*dt*k2i,  cw+0.5f*dt*k2w,  v_cmd, &k3i, &k3w);
        deriv(ci+dt*k3i,        cw+dt*k3w,        v_cmd, &k4i, &k4w);
        ci += (dt/6.0f)*(k1i+2.0f*k2i+2.0f*k3i+k4i);
        cw += (dt/6.0f)*(k1w+2.0f*k2w+2.0f*k3w+k4w);
        /* Safety: NaN guard */
        if (!isfinite(ci)) ci = 0.0f;
        if (!isfinite(cw)) cw = 0.0f;
    }
    m->i     = ci;
    m->omega = cw;
}
