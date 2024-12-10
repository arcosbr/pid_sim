// sim.h
#ifndef SIM_H
#define SIM_H

#include <stdbool.h>

// Physics Constants
typedef struct
{
    float gravity;             // Gravity (m/s²)
    float length;              // Length of the bar (m)
    float mass;                // Mass (kg)
    float viscousFriction;     // Viscous friction (N·m·s)
    float momentOfInertia;     // Moment of inertia
} PhysicsConstants;

// Motor Hydraulic Parameters
typedef struct
{
    float maxPressure;         // Maximum pressure (bar)
    float controlPressure;     // Control pressure range (0-200
                               // bar)
    float displacement;        // Displacement (m³/rev)
    float torqueScaling;       // Torque scaling factor
    float maxMotorTorque;      // Maximum motor torque (N·m)
} MotorParameters;

// Simulation State
typedef struct
{
    float angle;               // Current angle (rad)
    float omega;               // Angular velocity (rad/s)
    float Irot;                // Rotational inertia
} SimulationState;

// Pressure Control State Structure
typedef struct
{
    float pressureSetpoint;    // Manual pressure (0-200 bar)
} PressureControlState;

// External Disturbance Parameters
typedef struct
{
    bool enabled;              // Is external disturbance enabled
    float magnitude;           // Magnitude of external disturbance
} ExternalDisturbance;

// Initialize Simulation
void Simulation_Init(PhysicsConstants *physics, MotorParameters *motor,
                     SimulationState *state, float mass, float length);

// Reset Simulation State
void Simulation_Reset(SimulationState *state, float initialAngle,
                      float initialOmega);

// Compute Total Torque
float Simulation_ComputeTotalTorque(const PhysicsConstants *physics,
                                    float angle, float omega, float motorTorque,
                                    float externalTorque);

// Integration Methods
void Simulation_IntegrateEuler(SimulationState *state,
                               const PhysicsConstants *physics, float dt,
                               float motorTorque, float externalTorque);
void Simulation_IntegrateRK4(SimulationState *state,
                             const PhysicsConstants *physics, float dt,
                             float motorTorque, float externalTorque);

#endif // SIM_H
