#ifndef PID_H
#define PID_H
/* pid.h - portable PID controller (derivative-on-measurement, anti-windup)
 * Shared by renode/main.c and host-sim/motor_sim.c
 */

typedef struct {
    float kp, ki, kd;
    float v_limit;   /* symmetric output clamp (+/- V_SUPPLY) */
    float ctrl_dt;   /* control period [s]                    */
    float integ;     /* integral accumulator                  */
    float prev_meas; /* previous measurement for D term       */
} PID;

void  pid_init  (PID *p, float kp, float ki, float kd,
                 float v_limit, float ctrl_dt);
float pid_update(PID *p, float setpoint, float meas);

#endif /* PID_H */
