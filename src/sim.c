// sim.c
#include "sim.h"
#include <math.h>

// Initialize Simulation
void Simulation_Init(PhysicsConstants *physics, MotorParameters *motor,
                     SimulationState *state, float mass, float length)
{
    physics->gravity         = 9.8f;     // m/s²
    physics->length          = length;   // meters
    physics->mass            = mass;     // kg
    physics->viscousFriction = 0.02f;    // N·m·s
    physics->momentOfInertia = (1.0f / 3.0f) * mass * length * length;

    motor->maxPressure     = 400.0f;     // bar
    motor->controlPressure = 200.0f;     // bar
    motor->maxMotorTorque  = 50.0f;      // N·m
    motor->displacement    = 0.157e-6f;  // m³/rev
    motor->torqueScaling   = 40.0f;      // Arbitrary scaling factor

    // Initialize simulation state
    state->angle = 0.0f;
    state->omega = 0.0f;
    state->Irot  = physics->momentOfInertia;
}

// Reset Simulation State
void Simulation_Reset(SimulationState *state, float initialAngle,
                      float initialOmega)
{
    state->angle = initialAngle;
    state->omega = initialOmega;
}

// Compute Total Torque
float Simulation_ComputeTotalTorque(const PhysicsConstants *physics,
                                    float angle, float omega, float motorTorque,
                                    float externalTorque)
{
    float gravityTorque = -physics->mass * physics->gravity *
                          (physics->length / 2.0f) * sinf(angle);
    float frictionTorque = -physics->viscousFriction * omega;
    float totalTorque =
        motorTorque + gravityTorque + frictionTorque + externalTorque;
    return totalTorque;
}

// Integration using Euler's Method
void Simulation_IntegrateEuler(SimulationState *state,
                               const PhysicsConstants *physics, float dt,
                               float motorTorque, float externalTorque)
{
    float totalTorque = Simulation_ComputeTotalTorque(
        physics, state->angle, state->omega, motorTorque, externalTorque);
    float alpha = totalTorque / state->Irot;
    state->omega += alpha * dt;
    state->angle += state->omega * dt;
}

// Integration using Runge-Kutta 4th Order
void Simulation_IntegrateRK4(SimulationState *state,
                             const PhysicsConstants *physics, float dt,
                             float motorTorque, float externalTorque)
{
    // k1
    float k1_angle_dot = state->omega;
    float k1_omega_dot =
        Simulation_ComputeTotalTorque(physics, state->angle, state->omega,
                                      motorTorque, externalTorque) /
        state->Irot;

    // k2
    float angle_half   = state->angle + k1_angle_dot * (dt / 2.0f);
    float omega_half   = state->omega + k1_omega_dot * (dt / 2.0f);
    float k2_angle_dot = omega_half;
    float k2_omega_dot =
        Simulation_ComputeTotalTorque(physics, angle_half, omega_half,
                                      motorTorque, externalTorque) /
        state->Irot;

    // k3
    angle_half = state->angle + k2_angle_dot * (dt / 2.0f);
    omega_half = state->omega + k2_omega_dot * (dt / 2.0f);

    float k3_angle_dot = omega_half;
    float k3_omega_dot =
        Simulation_ComputeTotalTorque(physics, angle_half, omega_half,
                                      motorTorque, externalTorque) /
        state->Irot;

    // k4
    float angle_end    = state->angle + k3_angle_dot * dt;
    float omega_end    = state->omega + k3_omega_dot * dt;
    float k4_angle_dot = omega_end;
    float k4_omega_dot =
        Simulation_ComputeTotalTorque(physics, angle_end, omega_end,
                                      motorTorque, externalTorque) /
        state->Irot;

    // Update state
    state->angle += (dt / 6.0f) * (k1_angle_dot + 2.0f * k2_angle_dot +
                                   2.0f * k3_angle_dot + k4_angle_dot);
    state->omega += (dt / 6.0f) * (k1_omega_dot + 2.0f * k2_omega_dot +
                                   2.0f * k3_omega_dot + k4_omega_dot);
}
