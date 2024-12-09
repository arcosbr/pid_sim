// gui.h
#ifndef GUI_H
#define GUI_H

#include "log.h"
#include "pid.h"
#include "sim.h"
#include "raylib.h"

// Define Maximum Data Points for Graphs
#define MAX_DATA_POINTS 1000

// Graph Types
typedef enum
{
    GRAPH_ANGLE,
    GRAPH_ERROR,
    GRAPH_PRESSURE
} GraphType;

// Initialize Graphics
void Graphics_InitWindow(const char *title, int width, int height, int fps);

// Render Graphics
void Graphics_Render(float simTime, SimulationState *simState,
                     PIDConfig *pidConfig, int historyIndex,
                     float *angleHistory, float *errorHistory,
                     float *pressureHistory);

// Draw Compass
void Graphics_DrawCompass(float centerX, float centerY, float radius);

// Draw Compass and Pendulum
void Graphics_DrawCompassAndPendulum(float angle, float length);

// Draw Graph
void Graphics_DrawGraph(float *history, int historyIndex, int maxPoints,
                        float graphX, float graphY, float graphWidth,
                        float graphHeight, Color color, float scale);

// Draw Simulation Graphs
void Graphics_DrawSimulationGraphs(float *angleHistory, float *errorHistory,
                                   float *pressureHistory, int historyIndex);

// Display Simulation Information
void Graphics_DisplayInfo(float simTime, float angleSetpoint,
                          float measuredAngle, float pressureFiltered,
                          float dutyCycle, float motorTorque,
                          float gravityTorque, float frictionTorque,
                          float externalTorque, float Kp, float Ki, float Kd,
                          bool useRK4);

// Draw UI Elements
void Graphics_DrawUI(PIDConfig *pidConfig, bool *dataLogging);

// Update UI Elements
void Graphics_UpdateUI(PIDConfig *pidConfig, bool *dataLogging);

// Update History for Graphs
void Graphics_UpdateHistory(float *history, int *index, float value);

// Handle Manual Pressure Adjustment with UP/DOWN Keys
void Graphics_HandleManualPressureAdjustment(PressureControlState *state,
                                             float maxPressure);

// Handle Simulation Reset with R Key
void Graphics_HandleSimulationReset(SimulationState *simState,
                                    PIDState *pidState,
                                    PressureControlState *pressureControlState,
                                    float *pressureFiltered,
                                    float angleHistory[], float errorHistory[],
                                    float pressureHistory[], int *historyIndex,
                                    DataLogger *logger);

// Handle PID Gain Adjustments with F2-F7 Keys
void Graphics_HandlePIDGainAdjustments(PIDConfig *pidConfig, float Kp_min,
                                       float Kp_max, float Ki_min, float Ki_max,
                                       float Kd_min, float Kd_max);

// Handle Integration Method Toggle with TAB Key
void Graphics_HandleIntegrationMethodToggle(bool *useRK4);

// Handle External Disturbance Toggle with P Key
void Graphics_HandleExternalDisturbanceToggle(ExternalDisturbance *disturbance);

// Handle External Disturbance Magnitude with J/K Keys
void Graphics_HandleExternalDisturbanceMagnitude(
    ExternalDisturbance *disturbance);

#endif // GUI_H
