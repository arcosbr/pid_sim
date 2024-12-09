#include "graph.h"
#include "utils.h"
#include <math.h>

// Initialize Graphics Window
void Graphics_InitWindow(const char *title, int width, int height, int fps)
{
    InitWindow(width, height, title);
    SetTargetFPS(fps);
}

// Draw Compass
void Graphics_DrawCompass(float centerX, float centerY, float radius)
{
    DrawCircleLines((int)centerX, (int)centerY, (int)radius,
                    (Color){245, 222, 179, 255});

    const float angles[] = {0.0f, M_PI_2, M_PI, 3.0f * M_PI_2};
    const char *labels[] = {"90°", "180°", "270°", "0°"};
    const float margin[] = {20.0f, 16.0f, 40.0f, 20.0f};
    const float dif[] = {10.0f, 15.0f, 10.0f, 5.0f};

    for (int i = 0; i < 4; i++)
    {
        float x = centerX + cosf(angles[i]) * (radius + margin[i]);
        float y = centerY - sinf(angles[i]) * (radius + margin[i]);
        DrawText(labels[i], (int)(x - dif[i]), (int)(y - 10), 20,
                 (Color){255, 200, 100, 255});
    }
}

// Draw Graph
void Graphics_DrawGraph(float *history, int historyIndex, int maxPoints,
                        float graphX, float graphY, float graphWidth,
                        float graphHeight, Color color, float scale)
{
    BeginScissorMode((int)graphX, (int)graphY, (int)graphWidth,
                     (int)graphHeight);
    for (int i = 1; i < maxPoints; i++)
    {
        int prevIndex = (historyIndex + i - 1) % maxPoints;
        int currIndex = (historyIndex + i) % maxPoints;
        float val1 = history[prevIndex];
        float val2 = history[currIndex];

        float x1 =
            graphX + (float)(i - 1) * (graphWidth / (float)(maxPoints - 1));
        float x2 = graphX + (float)i * (graphWidth / (float)(maxPoints - 1));

        float yCenter = graphY + graphHeight / 2.0f;
        float y1 = yCenter - (val1 / scale) * (graphHeight / 2.0f);
        float y2 = yCenter - (val2 / scale) * (graphHeight / 2.0f);

        DrawLineEx((Vector2){x1, y1}, (Vector2){x2, y2}, 2.0f, color);
    }
    EndScissorMode();
}

// Display Simulation Information
void Graphics_DisplayInfo(float simTime, float angleSetpoint,
                          float measuredAngle, float pressureFiltered,
                          float dutyCycle, float motorTorque,
                          float gravityTorque, float frictionTorque,
                          float externalTorque, float Kp, float Ki, float Kd,
                          bool useRK4)
{
    const Color textColor = (Color){245, 222, 179, 255};

    DrawText("PID Simulation - Hydraulic Motor", 10, 10, 20, textColor);
    DrawText(TextFormat("Time: %.2f s", simTime), 10, 40, 20, textColor);
    DrawText(TextFormat("Target Angle: %.2f rad", angleSetpoint), 10, 70, 20,
             textColor);
    DrawText(TextFormat("Measured Angle: %.3f rad", measuredAngle), 10, 100, 20,
             textColor);
    DrawText(TextFormat("Filtered Pressure (bar): %.1f", pressureFiltered), 10,
             130, 20, textColor);
    DrawText(TextFormat("Duty Cycle: %.1f%%", dutyCycle), 10, 160, 20,
             textColor);
    DrawText(TextFormat("Motor Torque (N·m): %.5f", motorTorque), 10, 190, 20,
             textColor);

    DrawText(TextFormat("Gravity Torque (N·m): %.3f", gravityTorque), 10, 220,
             20, textColor);
    DrawText(TextFormat("Friction Torque (N·m): %.3f", frictionTorque), 10, 250,
             20, textColor);
    DrawText(TextFormat("External Torque (N·m): %.3f", externalTorque), 10, 280,
             20, textColor);

    DrawText(TextFormat("Kp: %.1f | Ki: %.3f | Kd: %.1f", Kp, Ki, Kd), 10, 310,
             20, textColor);

    DrawText("Adjust Kp (F2: -10 | F3: +10), Ki (F4: -0.01 | F5: +0.01), Kd "
             "(F6: -10 | F7: +10)",
             10, 610, 20, (Color){255, 200, 100, 255});
    DrawText("Use UP/DOWN ARROWS to adjust manual pressure (+/-1 bar)", 10, 640,
             20, (Color){255, 200, 100, 255});
    DrawText("Press R to reset simulation", 10, 670, 20,
             (Color){255, 200, 100, 255});
    DrawText(
        "Press P to toggle disturbance, J/K to adjust disturbance magnitude",
        10, 700, 20, (Color){255, 200, 100, 255});
    DrawText(TextFormat("Integration Method: %s (Press TAB to toggle)",
                        useRK4 ? "RK4" : "Euler"),
             10, 740, 20, (Color){204, 85, 0, 255});
}
