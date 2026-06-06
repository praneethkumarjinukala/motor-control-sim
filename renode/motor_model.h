#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H
/* motor_model.h - DC motor RK4 plant (same parameters as host-sim + Wokwi)
 *
 * di/dt = (V - R*i - Ke*w) / L
 * dw/dt = (Kt*i - Bv*w)   / J
 */
#include <stdint.h>

typedef struct {
    float i;      /* armature current  [A]     */
    float omega;  /* angular velocity  [rad/s] */
} Motor;

void motor_init(Motor *m);
void motor_step(Motor *m, float v_cmd, uint32_t micro_steps);

/* Motor parameters (SI) */
#define MOTOR_R    1.0f
#define MOTOR_L    5.0e-4f
#define MOTOR_KT   0.023f
#define MOTOR_KE   0.023f
#define MOTOR_J    1.0e-5f
#define MOTOR_BV   1.0e-4f
#define MOTOR_VS   24.0f
#define MOTOR_DT   1.0e-5f   /* RK4 micro-step [s] */

#endif /* MOTOR_MODEL_H */
