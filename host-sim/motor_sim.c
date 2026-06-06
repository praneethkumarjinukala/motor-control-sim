/*
 * motor_sim.c  --  "golden" reference: DC-motor RK4 + PID, host simulation
 *
 * Build:  gcc -O2 motor_sim.c -lm -o motor_sim
 * Run:    ./motor_sim > response.csv
 * Plot:   python3 plot.py
 *
 * Outputs CSV:  t, setpoint, omega  (one row per control tick)
 * Same control law and plant as the Wokwi / Renode firmware ports.
 */

#include <stdio.h>
#include <math.h>

/* ---- motor parameters ---- */
#define R    1.0          /* armature resistance   [Ohm]    */
#define L    5.0e-4       /* armature inductance   [H]      */
#define KT   0.023        /* torque constant       [N.m/A]  */
#define KE   0.023        /* back-EMF constant     [V.s/rad]*/
#define J    1.0e-5       /* rotor inertia         [kg.m^2] */
#define BV   1.0e-4       /* viscous friction      [N.m.s]  */
#define VS   24.0         /* supply voltage        [V]      */

/* ---- sim parameters ---- */
#define PLANT_DT  1.0e-5  /* RK4 micro-step  [s]  */
#define CTRL_DT   1.0e-3  /* control period  [s]  */
#define CTRL_DIV  100     /* micro-steps / tick   */
#define T_END     3.0     /* simulation length [s]*/

/* ---- PID gains ---- */
#define KP   0.08
#define KI   6.0
#define KD   0.0008

/* ---- plant state (global for simplicity) ---- */
static double m_i = 0.0, m_w = 0.0;

static void deriv(double i, double w, double v,
                  double *di, double *dw) {
    *di = (v - R*i - KE*w) / L;
    *dw = (KT*i - BV*w)   / J;
}

static void plant_step(double v, double dt) {
    double k1i,k1w, k2i,k2w, k3i,k3w, k4i,k4w;
    if (v >  VS) v =  VS;
    if (v < -VS) v = -VS;
    deriv(m_i,               m_w,               v, &k1i, &k1w);
    deriv(m_i+0.5*dt*k1i,   m_w+0.5*dt*k1w,   v, &k2i, &k2w);
    deriv(m_i+0.5*dt*k2i,   m_w+0.5*dt*k2w,   v, &k3i, &k3w);
    deriv(m_i+dt*k3i,        m_w+dt*k3w,        v, &k4i, &k4w);
    m_i += (dt/6.0)*(k1i+2*k2i+2*k3i+k4i);
    m_w += (dt/6.0)*(k1w+2*k2w+2*k3w+k4w);
}

int main(void) {
    double integ = 0.0, prev_err = 0.0;
    double t = 0.0;

    /* Step setpoints: 0->150 at t=0.2 s, 150->80 at t=1.5 s */
    printf("t,setpoint,omega\n");

    for (; t < T_END; t += CTRL_DT) {
        double sp = (t < 0.2) ? 0.0 : (t < 1.5) ? 150.0 : 80.0;

        /* PID (derivative on measurement to avoid derivative kick) */
        double err  = sp - m_w;
        integ      += KI * err * CTRL_DT;
        double derv = -KD * (m_w - (sp - err)) / CTRL_DT; /* = -KD*d(meas)/dt */
        double out  = KP*err + integ + derv;
        if (out >  VS) { integ -= (out - VS); out =  VS; }
        if (out < -VS) { integ -= (out + VS); out = -VS; }

        for (int s = 0; s < CTRL_DIV; ++s)
            plant_step(out, PLANT_DT);

        printf("%.4f,%.1f,%.4f\n", t, sp, m_w);
    }
    return 0;
}
