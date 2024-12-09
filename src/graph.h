#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "raylib.h"

// Graphical Constants
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

// Draw Compass
void Graphics_DrawCompass(float centerX, float centerY, float radius);

// Draw Graph
void Graphics_DrawGraph(float *history, int historyIndex, int maxPoints,
                        float graphX, float graphY, float graphWidth,
                        float graphHeight, Color color, float scale);

// Display Simulation Information
void Graphics_DisplayInfo(float simTime, float angleSetpoint,
                          float measuredAngle, float pressureFiltered,
                          float dutyCycle, float motorTorque,
                          float gravityTorque, float frictionTorque,
                          float externalTorque, float Kp, float Ki, float Kd,
                          bool useRK4);

#endif // GRAPHICS_H
