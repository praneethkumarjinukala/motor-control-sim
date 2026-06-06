/*
 * Real-Time Motor Control - LIVE closed-loop speed control demo (Wokwi)
 * Board: STM32 Blue Pill  (STM32F103C8, ARM Cortex-M3)
 *
 * - Turn the POTENTIOMETER  -> sets the target speed (0..300 rad/s)
 * - Watch the SERVO needle  -> live "speedometer" tracking the motor
 * - Open Serial Plotter     -> setpoint vs. measured speed, live
 *
 * Baud: 115200.  If the plot is blank, change Serial -> Serial1 below.
 */

#include <Servo.h>
#include <math.h>

// ---- pins ----
const int PIN_POT   = PA0;
const int PIN_SERVO = PA1;

// ---- motor parameters (SI) ----
const float R   = 1.0f;        // armature resistance   [Ohm]
const float L   = 5.0e-4f;    // armature inductance   [H]
const float Kt  = 0.023f;     // torque constant       [N.m/A]
const float Ke  = 0.023f;     // back-EMF constant     [V.s/rad]
const float J   = 1.0e-5f;    // rotor inertia         [kg.m^2]
const float Bv  = 1.0e-4f;    // viscous friction      [N.m.s/rad]
const float VS  = 24.0f;      // supply voltage        [V]

// ---- simulation step sizes ----
const float PLANT_DT = 1.0e-5f;   // RK4 micro-step  [s]
const int   CTRL_DIV = 100;        // micro-steps per control tick
const float CTRL_DT  = 1.0e-3f;   // control period  [s]

// ---- PID gains ----
const float KP = 0.08f;
const float KI = 6.0f;
const float KD = 0.0008f;

// ---- plant state ----
float m_i = 0.0f;   // armature current  [A]
float m_w = 0.0f;   // angular velocity  [rad/s]

// ---- PID state ----
float integ     = 0.0f;
float prev_meas = 0.0f;

Servo needle;

// DC-motor ODE derivatives
static void deriv(float i, float w, float v,
                  float *di_out, float *dw_out) {
  *di_out = (v - R * i - Ke * w) / L;
  *dw_out = (Kt * i - Bv * w) / J;
}

// RK4 plant integrator (one step)
static void plantStep(float v, float dt) {
  float k1i, k1w, k2i, k2w, k3i, k3w, k4i, k4w;
  if (v >  VS) v =  VS;
  if (v < -VS) v = -VS;

  deriv(m_i,                  m_w,                  v, &k1i, &k1w);
  deriv(m_i+0.5f*dt*k1i,     m_w+0.5f*dt*k1w,     v, &k2i, &k2w);
  deriv(m_i+0.5f*dt*k2i,     m_w+0.5f*dt*k2w,     v, &k3i, &k3w);
  deriv(m_i+dt*k3i,           m_w+dt*k3w,           v, &k4i, &k4w);

  m_i += (dt/6.0f)*(k1i+2.0f*k2i+2.0f*k3i+k4i);
  m_w += (dt/6.0f)*(k1w+2.0f*k2w+2.0f*k3w+k4w);

  if (!isfinite(m_i)) m_i = 0.0f;
  if (!isfinite(m_w)) m_w = 0.0f;
}

// PID: derivative-on-measurement + clamping anti-windup
static float pidUpdate(float sp, float meas) {
  float err  = sp - meas;
  integ     += KI * err * CTRL_DT;
  float d    = -KD * (meas - prev_meas) / CTRL_DT;
  prev_meas  = meas;

  float out = KP * err + integ + d;
  if (out >  VS) { integ -= (out - VS); out =  VS; }
  if (out < -VS) { integ -= (out + VS); out = -VS; }
  return out;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  needle.attach(PIN_SERVO);
  needle.write(0);
}

void loop() {
  float setpoint = (analogRead(PIN_POT) / 4095.0f) * 300.0f;

  float v_cmd = pidUpdate(setpoint, m_w);
  for (int s = 0; s < CTRL_DIV; ++s)
    plantStep(v_cmd, PLANT_DT);

  // Serial Plotter: two space-separated values
  Serial.print(setpoint, 1);
  Serial.print(' ');
  Serial.println(m_w, 1);

  // Servo needle: 0..300 rad/s -> 0..180 deg
  int ang = (int)(m_w / 300.0f * 180.0f);
  if (ang < 0)   ang = 0;
  if (ang > 180) ang = 180;
  needle.write(ang);

  delay(15);
}
