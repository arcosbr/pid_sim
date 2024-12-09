#ifndef PID_H
#define PID_H

// PID Controller Structure
typedef struct
{
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
} PIDConfig;

// PID Controller State Structure
typedef struct
{
    float previousError;
    float integral;
} PIDState;

// Initialize PID Controller State
void PID_Init(PIDState *state);

// Calculate PID Output
float PID_Control(PIDConfig *config, PIDState *state, float setpoint,
                  float actual, float dt);

// Set PID Gains with Saturation
void PID_SetGains(PIDConfig *config, float Kp, float Ki, float Kd, float Kp_min,
                  float Kp_max, float Ki_min, float Ki_max, float Kd_min,
                  float Kd_max);

#endif // PID_H
