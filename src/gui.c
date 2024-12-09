// gui.c
#include <math.h>
#include <stddef.h>
#include "gui.h"
#include "raygui.h"
#include "utils.h"
#include "style_amber.h"

// Amber Style Color Definitions
// static const Color AMBER_DEFAULT_BORDER_COLOR_NORMAL =
//     (Color){137, 137, 136, 255}; // 0x898988ff
// static const Color AMBER_DEFAULT_BASE_COLOR_NORMAL =
//     (Color){41, 41, 41, 255}; // 0x292929ff
static const Color AMBER_DEFAULT_TEXT_COLOR_NORMAL =
    (Color){212, 212, 212, 255}; // 0xd4d4d4ff
static const Color AMBER_DEFAULT_BORDER_COLOR_FOCUSED =
    (Color){235, 137, 29, 255}; // 0xeb891dff
// static const Color AMBER_DEFAULT_BASE_COLOR_FOCUSED =
//     (Color){41, 41, 41, 255}; // 0x292929ff
static const Color AMBER_DEFAULT_TEXT_COLOR_FOCUSED =
    (Color){255, 255, 255, 255}; // 0xffffffff
static const Color AMBER_DEFAULT_BORDER_COLOR_PRESSED =
    (Color){241, 207, 157, 255}; // 0xf1cf9dff
// static const Color AMBER_DEFAULT_BASE_COLOR_PRESSED =
//     (Color){243, 147, 51, 255}; // 0xf39333ff
// static const Color AMBER_DEFAULT_TEXT_COLOR_PRESSED =
//     (Color){40, 32, 32, 255}; // 0x282020ff
// static const Color AMBER_DEFAULT_BORDER_COLOR_DISABLED =
//     (Color){106, 106, 106, 255}; // 0x6a6a6aff
// static const Color AMBER_DEFAULT_BASE_COLOR_DISABLED =
//     (Color){129, 129, 129, 255}; // 0x818181ff
// static const Color AMBER_DEFAULT_TEXT_COLOR_DISABLED =
//     (Color){96, 96, 96, 255}; // 0x606060ff
static const Color AMBER_DEFAULT_LINE_COLOR =
    (Color){239, 146, 42, 255}; // 0xef922aff
static const Color AMBER_DEFAULT_BACKGROUND_COLOR =
    (Color){51, 51, 51, 255}; // 0x333333ff
static const Color AMBER_HIGHLIGHT_COLOR =
    (Color){235, 137, 29, 255}; // 0xeb891dff

// Initialize Graphics Window
void Graphics_InitWindow(const char *title, int width, int height, int fps)
{
    InitWindow(width, height, title);
    SetTargetFPS(fps);

    // Load custom Amber style
    GuiLoadStyleAmber();

    // Load custom font
    Font customFont = LoadFontEx("fonts/amber.ttf", 16, NULL, 0);
    GuiSetFont(customFont);
}

void Graphics_Render(float simTime, SimulationState *simState,
                     PIDConfig *pidConfig, int historyIndex,
                     float *angleHistory, float *errorHistory,
                     float *pressureHistory)
{
    BeginDrawing();
    ClearBackground(AMBER_DEFAULT_BACKGROUND_COLOR);

    // Draw Graphs
    Graphics_DrawGraph(angleHistory, historyIndex, MAX_DATA_POINTS, 10, 400,
                       400, 200, AMBER_DEFAULT_BORDER_COLOR_PRESSED, 3.0f);
    Graphics_DrawGraph(errorHistory, historyIndex, MAX_DATA_POINTS, 420, 400,
                       400, 200, AMBER_DEFAULT_TEXT_COLOR_FOCUSED, 3.0f);
    Graphics_DrawGraph(pressureHistory, historyIndex, MAX_DATA_POINTS, 830, 400,
                       400, 200, AMBER_DEFAULT_LINE_COLOR, 200.0f);

    // Draw Compass
    Graphics_DrawCompass(830, 190, 150);

    // Display Simulation Information
    Graphics_DisplayInfo(simTime, 0.0f, simState->angle, 10.0f, 50.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, pidConfig->Kp, pidConfig->Ki,
                         pidConfig->Kd, true);

    // Draw UI Elements
    bool dataLogging = false;
    Graphics_DrawUI(pidConfig, &dataLogging);

    EndDrawing();
}

// Draw Compass
void Graphics_DrawCompass(float centerX, float centerY, float radius)
{
    DrawCircleLines((int)centerX, (int)centerY, (int)radius,
                    AMBER_DEFAULT_BORDER_COLOR_PRESSED);

    const float angles[] = {0.0f, M_PI_2, M_PI, 3.0f * M_PI_2};
    const char *labels[] = {"90°", "180°", "270°", "0°"};
    const float margin[] = {20.0f, 16.0f, 40.0f, 20.0f};
    const float dif[] = {10.0f, 15.0f, 10.0f, 5.0f};

    for (int i = 0; i < 4; i++)
    {
        float x = centerX + cosf(angles[i]) * (radius + margin[i]);
        float y = centerY - sinf(angles[i]) * (radius + margin[i]);
        DrawText(labels[i], (int)(x - dif[i]), (int)(y - 10), 20,
                 AMBER_DEFAULT_TEXT_COLOR_FOCUSED);
    }
}

// Draw Compass and Pendulum
void Graphics_DrawCompassAndPendulum(float angle, float length)
{
    // Compass dimensions
    float compassCenterX = 830.0f, compassCenterY = 190.0f,
          compassRadius = 150.0f;

    Graphics_DrawCompass(compassCenterX, compassCenterY, compassRadius - 50.0f);

    // Draw pendulum bar
    float px = compassCenterX + length * sinf(angle) * compassRadius;
    float py = compassCenterY + length * cosf(angle) * compassRadius;

    DrawLineEx((Vector2){compassCenterX, compassCenterY}, (Vector2){px, py},
               5.0f, AMBER_HIGHLIGHT_COLOR);
    DrawCircle((int)compassCenterX, (int)compassCenterY, 10,
               AMBER_DEFAULT_BORDER_COLOR_PRESSED);
    DrawCircle((int)px, (int)py, 10, AMBER_DEFAULT_BORDER_COLOR_PRESSED);
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

// Draw Simulation Graphs
void Graphics_DrawSimulationGraphs(float *angleHistory, float *errorHistory,
                                   float *pressureHistory, int historyIndex)
{
    // Graph dimensions
    float graphX = 10.0f, graphY = 400.0f, graphWidth = 400.0f,
          graphHeight = 200.0f;
    float errorGraphX = 420.0f, errorGraphY = 400.0f, errorGraphWidth = 400.0f,
          errorGraphHeight = 200.0f;
    float pressureGraphX = 830.0f, pressureGraphY = 400.0f,
          pressureGraphWidth = 400.0f, pressureGraphHeight = 200.0f;

    // Draw graph borders and labels
    DrawRectangleLines((int)graphX, (int)graphY, (int)graphWidth,
                       (int)graphHeight, AMBER_DEFAULT_BORDER_COLOR_PRESSED);
    DrawText("Angle (rad)", (int)graphX, (int)(graphY - 30), 20,
             AMBER_DEFAULT_TEXT_COLOR_FOCUSED);

    DrawRectangleLines((int)errorGraphX, (int)errorGraphY, (int)errorGraphWidth,
                       (int)errorGraphHeight,
                       AMBER_DEFAULT_BORDER_COLOR_PRESSED);
    DrawText("Error (rad)", (int)errorGraphX, (int)(errorGraphY - 30), 20,
             AMBER_DEFAULT_TEXT_COLOR_FOCUSED);

    DrawRectangleLines((int)pressureGraphX, (int)pressureGraphY,
                       (int)pressureGraphWidth, (int)pressureGraphHeight,
                       AMBER_DEFAULT_BORDER_COLOR_PRESSED);
    DrawText("Pressure (bar)", (int)pressureGraphX, (int)(pressureGraphY - 30),
             20, AMBER_DEFAULT_TEXT_COLOR_FOCUSED);

    // Draw graphs
    Graphics_DrawGraph(angleHistory, historyIndex, MAX_DATA_POINTS, graphX,
                       graphY, graphWidth, graphHeight,
                       AMBER_DEFAULT_BORDER_COLOR_PRESSED, 3.0f);
    Graphics_DrawGraph(errorHistory, historyIndex, MAX_DATA_POINTS, errorGraphX,
                       errorGraphY, errorGraphWidth, errorGraphHeight,
                       AMBER_DEFAULT_TEXT_COLOR_FOCUSED, 3.0f);
    Graphics_DrawGraph(pressureHistory, historyIndex, MAX_DATA_POINTS,
                       pressureGraphX, pressureGraphY, pressureGraphWidth,
                       pressureGraphHeight, AMBER_DEFAULT_LINE_COLOR, 200.0f);
}

// Display Simulation Information
void Graphics_DisplayInfo(float simTime, float angleSetpoint,
                          float measuredAngle, float pressureFiltered,
                          float dutyCycle, float motorTorque,
                          float gravityTorque, float frictionTorque,
                          float externalTorque, float Kp, float Ki, float Kd,
                          bool useRK4)
{
    const Color infoTextColor = AMBER_DEFAULT_TEXT_COLOR_NORMAL;

    DrawText("PID Simulation - Hydraulic Motor", 10, 10, 20, infoTextColor);
    DrawText(TextFormat("Time: %.2f s", simTime), 10, 40, 20, infoTextColor);
    DrawText(TextFormat("Target Angle: %.2f rad", angleSetpoint), 10, 70, 20,
             infoTextColor);
    DrawText(TextFormat("Measured Angle: %.3f rad", measuredAngle), 10, 100, 20,
             infoTextColor);
    DrawText(TextFormat("Filtered Pressure (bar): %.1f", pressureFiltered), 10,
             130, 20, infoTextColor);
    DrawText(TextFormat("Duty Cycle: %.1f%%", dutyCycle), 10, 160, 20,
             infoTextColor);
    DrawText(TextFormat("Motor Torque (N·m): %.5f", motorTorque), 10, 190, 20,
             infoTextColor);

    DrawText(TextFormat("Gravity Torque (N·m): %.3f", gravityTorque), 10, 220,
             20, infoTextColor);
    DrawText(TextFormat("Friction Torque (N·m): %.3f", frictionTorque), 10, 250,
             20, infoTextColor);
    DrawText(TextFormat("External Torque (N·m): %.3f", externalTorque), 10, 280,
             20, infoTextColor);

    DrawText(TextFormat("Kp: %.1f | Ki: %.3f | Kd: %.1f", Kp, Ki, Kd), 10, 310,
             20, infoTextColor);

    DrawText("Adjust Kp (F2: -10 | F3: +10), Ki (F4: -0.01 | F5: +0.01), Kd "
             "(F6: -10 | F7: +10)",
             10, 610, 20, AMBER_HIGHLIGHT_COLOR);
    DrawText("Use UP/DOWN ARROWS to adjust manual pressure (+/-1 bar)", 10, 640,
             20, AMBER_HIGHLIGHT_COLOR);
    DrawText("Press R to reset simulation", 10, 670, 20, AMBER_HIGHLIGHT_COLOR);
    DrawText(
        "Press P to toggle disturbance, J/K to adjust disturbance magnitude",
        10, 700, 20, AMBER_HIGHLIGHT_COLOR);
    DrawText(TextFormat("Integration Method: %s (Press TAB to toggle)",
                        useRK4 ? "RK4" : "Euler"),
             10, 740, 20, AMBER_DEFAULT_BORDER_COLOR_FOCUSED);
}

// Draw UI Elements
void Graphics_DrawUI(PIDConfig *pidConfig, bool *dataLogging)
{
    // Define UI Rectangle Areas
    Rectangle kpSliderRect = {900, 620, 300, 20};
    Rectangle kiSliderRect = {900, 650, 300, 20};
    Rectangle kdSliderRect = {900, 680, 300, 20};
    Rectangle logButtonRect = {900, 710, 150, 40};

    // Draw PID Sliders
    GuiSlider(kpSliderRect, TextFormat("Kp: %.1f", pidConfig->Kp), NULL,
              &pidConfig->Kp, 0.0f, 1000.0f);

    GuiSlider(kiSliderRect, TextFormat("Ki: %.3f", pidConfig->Ki), NULL,
              &pidConfig->Ki, 0.0f, 1.0f);

    GuiSlider(kdSliderRect, TextFormat("Kd: %.1f", pidConfig->Kd), NULL,
              &pidConfig->Kd, 0.0f, 1000.0f);

    // Draw Data Logging Button
    if (GuiButton(logButtonRect,
                  *dataLogging ? "Stop Logging" : "Start Logging"))
    {
        *dataLogging = !(*dataLogging);
    }
}

// Update History for Graphs
void Graphics_UpdateHistory(float *history, int *index, float value)
{
    history[*index] = value;
    *index = (*index + 1) % MAX_DATA_POINTS;
}

// Update UI Elements (Placeholder for future use)
void Graphics_UpdateUI(PIDConfig *pidConfig, bool *dataLogging)
{
    // Currently handled in DrawUI, can be expanded if needed
}

// Handle Manual Pressure Adjustment with UP/DOWN Keys
void Graphics_HandleManualPressureAdjustment(PressureControlState *state,
                                             float maxPressure)
{
    if (IsKeyPressed(KEY_UP) && state->pressureSetpoint < maxPressure)
        state->pressureSetpoint += 1.0f;
    if (IsKeyPressed(KEY_DOWN) && state->pressureSetpoint > 0.0f)
        state->pressureSetpoint -= 1.0f;
}

// Handle Simulation Reset with R Key
void Graphics_HandleSimulationReset(SimulationState *simState,
                                    PIDState *pidState,
                                    PressureControlState *pressureControlState,
                                    float *pressureFiltered,
                                    float angleHistory[], float errorHistory[],
                                    float pressureHistory[], int *historyIndex,
                                    DataLogger *logger)
{
    if (IsKeyPressed(KEY_R))
    {
        // Reset Simulation Variables
        simState->angle = 0.0f;
        simState->omega = 0.0f;
        pidState->integral = 0.0f;
        pidState->previousError = 0.0f;
        pressureControlState->pressureSetpoint = 10.0f;
        *pressureFiltered = 0.0f;

        // Reset External Disturbance
        // Assuming this function is available or pass the pointer
        // disturbance.magnitude = 0.0f;
        // disturbance.enabled = false;

        // Reset Histories
        for (int i = 0; i < MAX_DATA_POINTS; i++)
        {
            angleHistory[i] = 0.0f;
            errorHistory[i] = 0.0f;
            pressureHistory[i] = 0.0f;
        }

        *historyIndex = 0;

        // Reset Data Logger
        DataLogger_Reset(logger, "sim_data.csv");
    }
}

// Handle PID Gain Adjustments with F2-F7 Keys
void Graphics_HandlePIDGainAdjustments(PIDConfig *pidConfig, float Kp_min,
                                       float Kp_max, float Ki_min, float Ki_max,
                                       float Kd_min, float Kd_max)
{
    // Decrement Kp with F2
    if (IsKeyPressed(KEY_F2))
        PID_SetGains(pidConfig, pidConfig->Kp - 10.0f, pidConfig->Ki,
                     pidConfig->Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                     Kd_max);

    // Increment Kp with F3
    if (IsKeyPressed(KEY_F3))
        PID_SetGains(pidConfig, pidConfig->Kp + 10.0f, pidConfig->Ki,
                     pidConfig->Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                     Kd_max);

    // Decrement Ki with F4
    if (IsKeyPressed(KEY_F4))
        PID_SetGains(pidConfig, pidConfig->Kp, pidConfig->Ki - 0.01f,
                     pidConfig->Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                     Kd_max);

    // Increment Ki with F5
    if (IsKeyPressed(KEY_F5))
        PID_SetGains(pidConfig, pidConfig->Kp, pidConfig->Ki + 0.01f,
                     pidConfig->Kd, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min,
                     Kd_max);

    // Decrement Kd with F6
    if (IsKeyPressed(KEY_F6))
        PID_SetGains(pidConfig, pidConfig->Kp, pidConfig->Ki,
                     pidConfig->Kd - 10.0f, Kp_min, Kp_max, Ki_min, Ki_max,
                     Kd_min, Kd_max);

    // Increment Kd with F7
    if (IsKeyPressed(KEY_F7))
        PID_SetGains(pidConfig, pidConfig->Kp, pidConfig->Ki,
                     pidConfig->Kd + 10.0f, Kp_min, Kp_max, Ki_min, Ki_max,
                     Kd_min, Kd_max);
}

// Handle Integration Method Toggle with TAB Key
void Graphics_HandleIntegrationMethodToggle(bool *useRK4)
{
    if (IsKeyPressed(KEY_TAB))
        *useRK4 = !(*useRK4);
}

// Handle External Disturbance Toggle with P Key
void Graphics_HandleExternalDisturbanceToggle(ExternalDisturbance *disturbance)
{
    if (IsKeyPressed(KEY_P))
        disturbance->enabled = !disturbance->enabled;
}

// Handle External Disturbance Magnitude with J/K Keys
void Graphics_HandleExternalDisturbanceMagnitude(
    ExternalDisturbance *disturbance)
{
    if (IsKeyPressed(KEY_J))
        disturbance->magnitude += 0.5f;

    if (IsKeyPressed(KEY_K))
        disturbance->magnitude -= 0.5f;

    if (disturbance->magnitude < 0.0f)
        disturbance->magnitude = 0.0f;
}
