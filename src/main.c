#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "raylib.h"
#include "pid.h"
#include "sim.h"
#include "graph.h"
#include "utils.h"

// Pressure Control State Structure
typedef struct
{
    float pressureSetpoint; // Manual pressure (0-200 bar)
} PressureControlState;

int main(void)
{
    srand((unsigned int)time(NULL));

    // Initialize Physics Constants and Motor Parameters
    PhysicsConstants physics;
    MotorParameters motor;
    SimulationState simState;
    Simulation_Init(&physics, &motor, &simState, 0.5f,
                    1.0f); // mass = 0.5 kg, length = 1.0 m

    // Initialize PID Controller
    PIDConfig pidConfig = {1.0f, 0.01f, 5.0f}; // Kp, Ki, Kd
    PIDState pidState;
    PID_Init(&pidState);

    // Define PID Gain Limits
    const float Kp_min = 0.0f, Kp_max = 1000.0f;
    const float Ki_min = 0.0f, Ki_max = 1.0f;
    const float Kd_min = 0.0f, Kd_max = 1000.0f;

    // Initialize Pressure Control State
    PressureControlState pressureControlState = {
        .pressureSetpoint = 10.0f // Initial pressure
    };

    // Simulation States
    float angle = 0.0f;
    float omega = 0.0f;
    float Irot = simState.Irot;

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
    ExternalDisturbance disturbance = {.enabled = false, .magnitude = 0.0f};

    // Angle Noise Amplitude
    float angleNoiseAmplitude = 0.005f;

    // Target Angle
    const float angleSetpoint = 0.0f;

    // Initialize Window
    Graphics_InitWindow("PID Simulation - Hydraulic Motor", 1240, 768, 60);

    float simTime = 0.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        simTime += dt;

        // Manual Pressure Adjustment with UP/DOWN Keys
        if (IsKeyPressed(KEY_UP) &&
            pressureControlState.pressureSetpoint < motor.controlPressure)
            pressureControlState.pressureSetpoint += 1.0f;
        if (IsKeyPressed(KEY_DOWN) &&
            pressureControlState.pressureSetpoint > 0.0f)
            pressureControlState.pressureSetpoint -= 1.0f;

        // Reset Simulation with R Key
        if (IsKeyPressed(KEY_R))
        {
            simTime = 0.0f;
            Simulation_Reset(&simState, 0.0f, 0.0f);
            pidState.integral = 0.0f;
            pidState.previousError = 0.0f;
            pressureControlState.pressureSetpoint = 10.0f;
            pressureFiltered = 0.0f;
            disturbance.magnitude = 0.0f;
            disturbance.enabled = false;

            for (int i = 0; i < MAX_DATA_POINTS; i++)
            {
                angleHistory[i] = 0.0f;
                errorHistory[i] = 0.0f;
                pressureHistory[i] = 0.0f;
            }

            historyIndex = 0;
        }

        // Adjust PID Parameters with Function Keys
        if (IsKeyPressed(KEY_F2))
            PID_SetGains(&pidConfig, pidConfig.Kp - 10.0f, pidConfig.Ki,
                         pidConfig.Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                         Kd_max);

        if (IsKeyPressed(KEY_F3))
            PID_SetGains(&pidConfig, pidConfig.Kp + 10.0f, pidConfig.Ki,
                         pidConfig.Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                         Kd_max);

        if (IsKeyPressed(KEY_F4))
            PID_SetGains(&pidConfig, pidConfig.Kp, pidConfig.Ki - 0.01f,
                         pidConfig.Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                         Kd_max);

        if (IsKeyPressed(KEY_F5))
            PID_SetGains(&pidConfig, pidConfig.Kp, pidConfig.Ki + 0.01f,
                         pidConfig.Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                         Kd_max);

        if (IsKeyPressed(KEY_F6))
            PID_SetGains(&pidConfig, pidConfig.Kp, pidConfig.Ki,
                         pidConfig.Kd - 10.0f, Kp_min, Kp_max, Ki_min, Ki_max,
                         Kd_min, Kd_max);

        if (IsKeyPressed(KEY_F7))
            PID_SetGains(&pidConfig, pidConfig.Kp, pidConfig.Ki,
                         pidConfig.Kd + 10.0f, Kp_min, Kp_max, Ki_min, Ki_max,
                         Kd_min, Kd_max);

        // Toggle Integration Method with TAB Key
        if (IsKeyPressed(KEY_TAB))
            useRK4 = !useRK4;

        // Toggle External Disturbance with P Key
        if (IsKeyPressed(KEY_P))
            disturbance.enabled = !disturbance.enabled;

        // Adjust External Disturbance Magnitude with J/K Keys
        if (IsKeyPressed(KEY_J))
            disturbance.magnitude += 0.5f;

        if (IsKeyPressed(KEY_K))
            disturbance.magnitude -= 0.5f;

        if (disturbance.magnitude < 0.0f)
            disturbance.magnitude = 0.0f;

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

        if (rawPressure > motor.controlPressure)
            rawPressure = motor.controlPressure;

        if (rawPressure < 0.0f)
            rawPressure = 0.0f;

        // Pressure Filtering
        pressureFiltered = pressureFilterAlpha * pressureFiltered +
                           (1.0f - pressureFilterAlpha) * rawPressure;

        // Duty Cycle for Visualization
        float dutyCycle = MapValue(pressureFiltered, 0.0f,
                                   motor.controlPressure, 0.0f, 100.0f);

        // Pressure in Pascals
        float pressure_pa = pressureFiltered * 1e5f;

        // Motor Torque Calculation
        float motorTorque =
            ((pressure_pa * motor.displacement) / (2.0f * (float)M_PI)) *
            motor.torqueScalingFactor;

        // Saturate Motor Torque
        if (motorTorque > motor.maxMotorTorque)
            motorTorque = motor.maxMotorTorque;

        if (motorTorque < -motor.maxMotorTorque)
            motorTorque = -motor.maxMotorTorque;

        // Integration Substeps for Stability
        const int substeps = 20;
        float sub_dt = dt / substeps;

        for (int i = 0; i < substeps; i++)
        {
            if (useRK4)
                Simulation_IntegrateRK4(&simState, &physics, sub_dt,
                                        motorTorque, externalTorque);
            else
                Simulation_IntegrateEuler(&simState, &physics, sub_dt,
                                          motorTorque, externalTorque);
        }

        // Normalize Angle to [-PI, PI]
        simState.angle =
            fmodf(simState.angle + (float)M_PI, 2.0f * (float)M_PI) -
            (float)M_PI;

        // Update Histories
        float error = angleSetpoint - measuredAngle;

        UpdateGraph(angleHistory, &historyIndex, simState.angle);
        UpdateGraph(errorHistory, &historyIndex, error);
        UpdateGraph(pressureHistory, &historyIndex, pressureFiltered);

        // Calculate Torques for Display
        float gravityTorque = -physics.mass * physics.gravity *
                              (physics.length / 2.0f) * sinf(simState.angle);
        float frictionTorque = -physics.viscousFriction * simState.omega;

        // Begin Drawing
        BeginDrawing();
        ClearBackground((Color){43, 43, 43, 255});

        // Define Graph Positions and Sizes
        float graphX = 10.0f;
        float graphY = 400.0f;
        float graphWidth = 400.0f;
        float graphHeight = 200.0f;

        float errorGraphX = 420.0f;
        float errorGraphY = 400.0f;
        float errorGraphWidth = 400.0f;
        float errorGraphHeight = 200.0f;

        float pressureGraphX = 830.0f;
        float pressureGraphY = 400.0f;
        float pressureGraphWidth = 400.0f;
        float pressureGraphHeight = 200.0f;

        // Draw Graph Borders and Labels
        DrawRectangleLines((int)graphX, (int)graphY, (int)graphWidth,
                           (int)graphHeight, (Color){245, 222, 179, 255});
        DrawText("Angle (rad)", (int)graphX, (int)(graphY - 30), 20,
                 (Color){245, 222, 179, 255});

        DrawRectangleLines((int)errorGraphX, (int)errorGraphY,
                           (int)errorGraphWidth, (int)errorGraphHeight,
                           (Color){245, 222, 179, 255});
        DrawText("Error (rad)", (int)errorGraphX, (int)(errorGraphY - 30), 20,
                 (Color){245, 222, 179, 255});

        DrawRectangleLines((int)pressureGraphX, (int)pressureGraphY,
                           (int)pressureGraphWidth, (int)pressureGraphHeight,
                           (Color){245, 222, 179, 255});
        DrawText("Pressure (bar)", (int)pressureGraphX,
                 (int)(pressureGraphY - 30), 20, (Color){245, 222, 179, 255});

        // Draw Graphs
        Graphics_DrawGraph(angleHistory, historyIndex, MAX_DATA_POINTS, graphX,
                           graphY, graphWidth, graphHeight,
                           (Color){204, 85, 0, 255}, 3.0f);
        Graphics_DrawGraph(errorHistory, historyIndex, MAX_DATA_POINTS,
                           errorGraphX, errorGraphY, errorGraphWidth,
                           errorGraphHeight, (Color){150, 150, 250, 255}, 3.0f);
        Graphics_DrawGraph(pressureHistory, historyIndex, MAX_DATA_POINTS,
                           pressureGraphX, pressureGraphY, pressureGraphWidth,
                           pressureGraphHeight, (Color){50, 200, 50, 255},
                           200.0f);

        // Draw Compass
        float compassCenterX = 830.0f;
        float compassCenterY = 190.0f;
        float compassRadius = 150.0f;

        Graphics_DrawCompass(compassCenterX, compassCenterY,
                             compassRadius - 50.0f);

        // Draw Pendulum Bar
        float px = compassCenterX +
                   physics.length * sinf(simState.angle) * (compassRadius);
        float py = compassCenterY +
                   physics.length * cosf(simState.angle) * (compassRadius);

        DrawLineEx((Vector2){compassCenterX, compassCenterY}, (Vector2){px, py},
                   5.0f, (Color){204, 85, 0, 255});
        DrawCircle((int)compassCenterX, (int)compassCenterY, 10,
                   (Color){255, 200, 100, 255});
        DrawCircle((int)px, (int)py, 10, (Color){255, 200, 100, 255});

        // Display Simulation Information
        Graphics_DisplayInfo(simTime, angleSetpoint, measuredAngle,
                             pressureFiltered, dutyCycle, motorTorque,
                             gravityTorque, frictionTorque, externalTorque,
                             pidConfig.Kp, pidConfig.Ki, pidConfig.Kd, useRK4);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
