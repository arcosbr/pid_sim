
// main.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "raylib.h"

#include "gui.h"
#include "log.h"
#include "pid.h"
#include "sim.h"
#include "utils.h"

// Define Maximum Data Points for Graphs
#define MAX_DATA_POINTS 1000

int main(void)
{
    // Define PID Gain Limits
    const float Kp_min = 0.0f, Kp_max = 1000.0f;
    const float Ki_min = 0.0f, Ki_max = 1.0f;
    const float Kd_min = 0.0f, Kd_max = 1000.0f;

    srand((unsigned int)time(NULL));

    // Initialize PID Controller
    PIDConfig pidConfig = {
        1.0f,  // Kp
        0.01f, // Ki
        5.0f   // Kd
    };

    PIDState pidState;
    PID_Init(&pidState);

    // Initialize Pressure Control State
    PressureControlState pressureControlState = {
        .pressureSetpoint = 10.0f // Initial pressure
    };

    // Histories for Graphs
    float angleHistory[MAX_DATA_POINTS] = {0.0f};
    float errorHistory[MAX_DATA_POINTS] = {0.0f};
    float pressureHistory[MAX_DATA_POINTS] = {0.0f};
    int historyIndex = 0;

    // Pressure Filtering
    float pressureFiltered = 0.0f;
    float pressureFilterAlpha = 0.1f; // Lower value = stronger filtering

    // Integration Method Flag
    bool useRK4 = false;

    // External Disturbance Parameters
    ExternalDisturbance disturbance = {
        .enabled = false, // External disturbance disabled
        .magnitude = 0.0f // Disturbance magnitude (optional)
    };

    // Angle Noise Amplitude
    float angleNoiseAmplitude = 0.005f;

    // Target Angle
    const float angleSetpoint = 0.0f;

    // Initialize Data Logger
    DataLogger logger;

    if (!DataLogger_Init(&logger, "sim_data.csv"))
    {
        printf("Failed to initialize data logger.\n");
        return 1;
    }

    // Initialize Window
    Graphics_InitWindow("PID Simulation - Hydraulic Motor", 1240, 768, 60);

    float simTime = 0.0f;

    // Data Logging State
    bool dataLogging = false;

    // Initialize Physics Constants and Motor Parameters
    PhysicsConstants physics;
    MotorParameters motor;
    SimulationState simState;

    // Initialize Simulation (mass = 0.5 kg, length = 1.0 m)
    Simulation_Init(&physics, &motor, &simState, 0.5f, 1.0f);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        simTime += dt;

        // Handle Keyboard Input
        Graphics_HandleManualPressureAdjustment(&pressureControlState,
                                                motor.controlPressure);
        Graphics_HandleSimulationReset(&simState, &pidState,
                                       &pressureControlState, &pressureFiltered,
                                       angleHistory, errorHistory,
                                       pressureHistory, &historyIndex, &logger);
        Graphics_HandlePIDGainAdjustments(&pidConfig, Kp_min, Kp_max, Ki_min,
                                          Ki_max, Kd_min, Kd_max);
        Graphics_HandleIntegrationMethodToggle(&useRK4);
        Graphics_HandleExternalDisturbanceToggle(&disturbance);
        Graphics_HandleExternalDisturbanceMagnitude(&disturbance);

        // Calculate External Torque if Enabled
        float externalTorque = 0.0f;

        if (disturbance.enabled)
        {
            externalTorque = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f *
                             disturbance.magnitude;
        }

        // Measured Angle with Noise
        float measuredAngle = AddNoise(simState.angle, angleNoiseAmplitude);

        // PID Control Output
        float pidOutput = PID_Control(&pidConfig, &pidState, angleSetpoint,
                                      measuredAngle, dt);

        // Desired Final Pressure
        float rawPressure = pressureControlState.pressureSetpoint + pidOutput;

        rawPressure = RClamp(rawPressure, 0.0f, motor.controlPressure);
        pressureFiltered = pressureFilterAlpha * pressureFiltered +
                           (1.0f - pressureFilterAlpha) * rawPressure;

        // Duty Cycle for Visualization
        float dutyCycle = MapValue(pressureFiltered, 0.0f,
                                   motor.controlPressure, 0.0f, 100.0f);

        // Pressure in Pascals
        float pressure_pa = pressureFiltered * 1e5f;

        // Motor Torque Calculation
        float motorTorqueCalc =
            ((pressure_pa * motor.displacement) / (2.0f * (float)M_PI)) *
            motor.torqueScalingFactor;

        // Saturate Motor Torque
        motorTorqueCalc =
            RClamp(motorTorqueCalc, -motor.maxMotorTorque, motor.maxMotorTorque);

        // Integration Substeps for Stability
        const int substeps = 20;
        float sub_dt = dt / substeps;

        // Substeps for stability
        for (int i = 0; i < substeps; i++)
        {
            if (useRK4)
                Simulation_IntegrateRK4(&simState, &physics, sub_dt,
                                        motorTorqueCalc, externalTorque);
            else
                Simulation_IntegrateEuler(&simState, &physics, sub_dt,
                                          motorTorqueCalc, externalTorque);
        }

        // Normalize Angle to [-PI, PI]
        simState.angle =
            fmodf(simState.angle + (float)M_PI, 2.0f * (float)M_PI) -
            (float)M_PI;

        // Update Histories
        float error = angleSetpoint - measuredAngle;

        Graphics_UpdateHistory(angleHistory, &historyIndex, simState.angle);
        Graphics_UpdateHistory(errorHistory, &historyIndex, error);
        Graphics_UpdateHistory(pressureHistory, &historyIndex,
                               pressureFiltered);

        // Calculate Torques for Display
        float gravityTorque = -physics.mass * physics.gravity *
                              (physics.length / 2.0f) * sinf(simState.angle);
        float frictionTorque = -physics.viscousFriction * simState.omega;

        // Render Graphics
        BeginDrawing();
        ClearBackground((Color){43, 43, 43, 255});

        // Draw Graphs
        Graphics_DrawSimulationGraphs(angleHistory, errorHistory,
                                      pressureHistory, historyIndex);

        // Draw Compass and Pendulum
        Graphics_DrawCompassAndPendulum(simState.angle, physics.length);

        // Display Simulation Information
        Graphics_DisplayInfo(simTime, angleSetpoint, measuredAngle,
                             pressureFiltered, dutyCycle, motorTorqueCalc,
                             gravityTorque, frictionTorque, externalTorque,
                             pidConfig.Kp, pidConfig.Ki, pidConfig.Kd, useRK4);

        // Draw UI Elements
        Graphics_DrawUI(&pidConfig, &dataLogging);

        EndDrawing();

        // Handle Data Logging
        if (dataLogging)
        {
            DataLogger_LogData(&logger, simTime, simState.angle, error,
                               pressureFiltered);
        }
    }

    // Close Data Logger
    DataLogger_Close(&logger);

    CloseWindow();
    return 0;
}
