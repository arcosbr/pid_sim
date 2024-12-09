#include "pid.h"

// Maximum and Minimum Integral Limits
static const float MAX_INTEGRAL = 30.0f;
static const float MIN_INTEGRAL = -30.0f;

// Initialize PID Controller State
void PID_Init(PIDState *state)
{
    state->previousError = 0.0f;
    state->integral = 0.0f;
}

// Calculate PID Output
float PID_Control(PIDConfig *config, PIDState *state, float setpoint,
                  float actual, float dt)
{
    float error = setpoint - actual;
    state->integral += error * dt;

    // Limit integral to prevent windup
    if (state->integral > MAX_INTEGRAL)
        state->integral = MAX_INTEGRAL;
    if (state->integral < MIN_INTEGRAL)
        state->integral = MIN_INTEGRAL;

    float derivative = (dt > 0.0f) ? (error - state->previousError) / dt : 0.0f;
    state->previousError = error;

    float output = (config->Kp * error) + (config->Ki * state->integral) +
                   (config->Kd * derivative);

    // Output saturation (e.g., 0 to 200 bar)
    if (output > 200.0f)
        output = 200.0f;
    if (output < 0.0f)
        output = 0.0f;

    return output;
}

// Set PID Gains with Saturation
void PID_SetGains(PIDConfig *config, float Kp, float Ki, float Kd, float Kp_min,
                  float Kp_max, float Ki_min, float Ki_max, float Kd_min,
                  float Kd_max)
{
    // Saturate Kp
    if (Kp < Kp_min)
        config->Kp = Kp_min;
    else if (Kp > Kp_max)
        config->Kp = Kp_max;
    else
        config->Kp = Kp;

    // Saturate Ki
    if (Ki < Ki_min)
        config->Ki = Ki_min;
    else if (Ki > Ki_max)
        config->Ki = Ki_max;
    else
        config->Ki = Ki;

    // Saturate Kd
    if (Kd < Kd_min)
        config->Kd = Kd_min;
    else if (Kd > Kd_max)
        config->Kd = Kd_max;
    else
        config->Kd = Kd;
}
