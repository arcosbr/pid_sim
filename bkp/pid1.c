#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "raylib.h"

//------------------------------------------------------------------------------
// Configurações Gerais do Sistema
//------------------------------------------------------------------------------
#define MAX_DATA_POINTS 1000
#define SIM_FPS 60

// Ajustes do PID
static float Kp = 500.0f; // Ganho Proporcional maior para resposta mais rápida
static float Ki = 0.1f;   // Ganho Integral menor para reduzir acúmulo
static float Kd = 250.0f; // Ganho Derivativo ajustado para amortecer oscilações

// Constantes físicas
static const float g = 9.8f;  // Gravidade (m/s²)
static const float L = 1.0f;  // Comprimento da barra (m)
static const float m = 0.5f;  // Massa (kg)
static const float c = 0.02f; // Atrito viscoso (N·m·s)

// Parâmetros do Motor Hidráulico
static const float maxPressure = 400.0f;        // Pressão máxima (bar)
static const float controlPressure = 200.0f;    // Faixa de controle (0-200 bar)
static const float displacement = 0.157e-6f;    // Deslocamento em m³/rev
static const float torqueScalingFactor = 30.0f; // Fator de escala do torque

// Limites da integral do PID
static const float maxIntegral = 30.0f;
static const float minIntegral = -30.0f;

// Ruído no ângulo
static float angleNoiseAmplitude = 0.005f;

// Estruturas de controle
typedef struct
{
    float pressureSetpoint; // Pressão manual (0-200 bar)
} PressureControlState;

typedef struct
{
    float previousError;
    float integral;
} PIDState;

static PressureControlState pressureControlState = {
    .pressureSetpoint = 20.0f // Pressão inicial
};

static PIDState pidState = {0};

// Estados do sistema
static float angle = 0.0f;
static float omega = 0.0f;
static float Irot = 0.0f; // Momento de inércia
static float angleHistory[MAX_DATA_POINTS] = {0.0f};
static float errorHistory[MAX_DATA_POINTS] = {0.0f};
static int historyIndex = 0;

//------------------------------------------------------------------------------
// Funções Auxiliares
//------------------------------------------------------------------------------
static float PIDControl(float setpoint, float actual, float dt)
{
    float error = setpoint - actual;
    pidState.integral += error * dt;

    // Limite integral
    if (pidState.integral > maxIntegral)
        pidState.integral = maxIntegral;
    if (pidState.integral < minIntegral)
        pidState.integral = minIntegral;

    float derivative =
        (dt > 0.0f) ? (error - pidState.previousError) / dt : 0.0f;
    pidState.previousError = error;

    float output = (Kp * error) + (Ki * pidState.integral) + (Kd * derivative);

    // Limite de saída (0 a 200 bar)
    if (output > controlPressure)
        output = controlPressure;
    if (output < 0.0f)
        output = 0.0f;

    return output;
}

static void UpdateGraph(float *history, int *index, float value)
{
    history[*index] = value;
    *index = (*index + 1) % MAX_DATA_POINTS;
}

static float MapValue(float value, float in_min, float in_max, float out_min,
                      float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DrawCompass(float centerX, float centerY, float radius)
{
    DrawCircleLines((int)centerX, (int)centerY, (int)radius, WHITE);

    const float angles[] = {0, (float)M_PI_2, (float)M_PI, (float)(3 * M_PI_2)};
    const char *labels[] = {"90°", "180°", "270°", "0°"};

    for (int i = 0; i < 4; i++)
    {
        float x = centerX + cosf(angles[i]) * (radius + 20);
        float y = centerY - sinf(angles[i]) * (radius + 20);
        DrawText(labels[i], (int)(x - 15), (int)(y - 10), 20, WHITE);
    }
}

static float AddNoise(float signal, float amplitude)
{
    float noise =
        ((float)rand() / (float)RAND_MAX) * 2.0f * amplitude - amplitude;
    return signal + noise;
}

// Função para resetar a simulação
static void ResetSimulation(void)
{
    angle = 0.0f;
    omega = 0.0f;
    pidState.integral = 0.0f;
    pidState.previousError = 0.0f;
    pressureControlState.pressureSetpoint = 50.0f;

    for (int i = 0; i < MAX_DATA_POINTS; i++)
    {
        angleHistory[i] = 0.0f;
        errorHistory[i] = 0.0f;
    }

    historyIndex = 0;
}

//------------------------------------------------------------------------------
// Função Principal
//------------------------------------------------------------------------------
int main(void)
{
    srand((unsigned int)time(NULL));

    const float angleSetpoint = 0.0f;
    Irot = (1.0f / 3.0f) * m * L * L;

    InitWindow(1000, 700, "Simulacao PID - Motor Hidraulico");
    SetTargetFPS(SIM_FPS);

    float simTime = 0.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        simTime += dt;

        // Teclas de ajuste de pressão manual
        // Agora incrementa e decrementa em 1 bar
        if (IsKeyPressed(KEY_UP) &&
            pressureControlState.pressureSetpoint < controlPressure)
            pressureControlState.pressureSetpoint += 1.0f;
        if (IsKeyPressed(KEY_DOWN) &&
            pressureControlState.pressureSetpoint > 0.0f)
            pressureControlState.pressureSetpoint -= 1.0f;

        // Tecla de reset da simulação
        if (IsKeyPressed(KEY_R))
        {
            simTime = 0.0f;
            ResetSimulation();
        }

        // Leitura do ângulo com ruído
        float measuredAngle = AddNoise(angle, angleNoiseAmplitude);

        // Controle PID
        float pidOutput = PIDControl(angleSetpoint, measuredAngle, dt);

        // Pressão final
        float pressure = pressureControlState.pressureSetpoint + pidOutput;
        if (pressure > controlPressure)
            pressure = controlPressure;
        if (pressure < 0.0f)
            pressure = 0.0f;

        // Duty Cycle
        float dutyCycle =
            MapValue(pressure, 0.0f, controlPressure, 0.0f, 100.0f);

        // Pressão em Pa
        float pressure_pa = pressure * 1e5f;
        // Torque do motor
        float motorTorque =
            ((pressure_pa * displacement) / (2.0f * (float)M_PI)) *
            torqueScalingFactor;

        // Torques
        float gravityTorque = -m * g * (L / 2.0f) * sinf(angle);
        float frictionTorque = -c * omega;
        float totalTorque = motorTorque + gravityTorque + frictionTorque;

        // Integração numérica
        const int substeps = 20;
        float sub_dt = dt / substeps;
        for (int i = 0; i < substeps; i++)
        {
            float alpha = totalTorque / Irot;
            omega += alpha * sub_dt;
            angle += omega * sub_dt;
        }

        // Atualiza histórico
        float error = angleSetpoint - measuredAngle;
        UpdateGraph(angleHistory, &historyIndex, angle);
        UpdateGraph(errorHistory, &historyIndex, error);

        // Desenho
        BeginDrawing();
        ClearBackground((Color){43, 43, 43, 255});

        const Color textColor = (Color){245, 222, 179, 255};

        DrawText("Simulacao PID - Motor Hidraulico", 10, 10, 20, textColor);
        DrawText(TextFormat("Tempo: %.2f s", simTime), 10, 40, 20, textColor);
        DrawText(TextFormat("Angulo Alvo: %.2f rad", angleSetpoint), 10, 70, 20,
                 textColor);
        DrawText(TextFormat("Angulo Medido: %.3f rad", measuredAngle), 10, 100,
                 20, textColor);
        DrawText(TextFormat("Pressao Comandada (bar): %.1f", pressure), 10, 130,
                 20, textColor);
        DrawText(TextFormat("Duty Cycle: %.1f%%", dutyCycle), 10, 160, 20,
                 textColor);
        DrawText(TextFormat("Torque Motor (N·m): %.5f", motorTorque), 10, 190,
                 20, textColor);
        DrawText(TextFormat("Torque Gravidade (N·m): %.3f", gravityTorque), 10,
                 220, 20, textColor);
        DrawText(TextFormat("Torque Atrito (N·m): %.3f", frictionTorque), 10,
                 250, 20, textColor);

        // Exibe os ganhos do PID
        DrawText(TextFormat("Kp: %.1f | Ki: %.2f | Kd: %.1f", Kp, Ki, Kd), 10,
                 280, 20, textColor);

        // Gráfico do Ângulo
        float graphX = 410.0f;
        float graphY = 40.0f;
        float graphWidth = 550.0f;
        float graphHeight = 200.0f;

        DrawRectangleLines((int)graphX, (int)graphY, (int)graphWidth,
                           (int)graphHeight, textColor);
        DrawText("Angulo (rad)", (int)graphX, (int)(graphY - 30), 20,
                 textColor);

        for (int i = 1; i < MAX_DATA_POINTS; i++)
        {
            int prevIndex = (historyIndex + i - 1) % MAX_DATA_POINTS;
            int currIndex = (historyIndex + i) % MAX_DATA_POINTS;

            float val1 = angleHistory[prevIndex];
            float val2 = angleHistory[currIndex];

            float x1 = graphX + (float)(i - 1) *
                                    (graphWidth / (float)(MAX_DATA_POINTS - 1));
            float x2 =
                graphX + (float)i * (graphWidth / (float)(MAX_DATA_POINTS - 1));

            float yCenter = graphY + graphHeight / 2.0f;
            float y1 = yCenter - (val1 / 3.0f) * (graphHeight / 2.0f);
            float y2 = yCenter - (val2 / 3.0f) * (graphHeight / 2.0f);

            DrawLineEx((Vector2){x1, y1}, (Vector2){x2, y2}, 2.0f,
                       (Color){204, 85, 0, 255});
        }

        // Gráfico do Erro
        float errorGraphX = 410.0f;
        float errorGraphY = 300.0f;
        float errorGraphWidth = 550.0f;
        float errorGraphHeight = 200.0f;

        DrawRectangleLines((int)errorGraphX, (int)errorGraphY,
                           (int)errorGraphWidth, (int)errorGraphHeight,
                           textColor);
        DrawText("Erro (rad)", (int)errorGraphX, (int)(errorGraphY - 30), 20,
                 textColor);

        for (int i = 1; i < MAX_DATA_POINTS; i++)
        {
            int prevIndex = (historyIndex + i - 1) % MAX_DATA_POINTS;
            int currIndex = (historyIndex + i) % MAX_DATA_POINTS;

            float val1 = errorHistory[prevIndex];
            float val2 = errorHistory[currIndex];

            float x1 = errorGraphX +
                       (float)(i - 1) *
                           (errorGraphWidth / (float)(MAX_DATA_POINTS - 1));
            float x2 = errorGraphX + (float)i * (errorGraphWidth /
                                                 (float)(MAX_DATA_POINTS - 1));

            float yCenter = errorGraphY + errorGraphHeight / 2.0f;
            float y1 = yCenter - (val1 / 3.0f) * (errorGraphHeight / 2.0f);
            float y2 = yCenter - (val2 / 3.0f) * (errorGraphHeight / 2.0f);

            DrawLineEx((Vector2){x1, y1}, (Vector2){x2, y2}, 2.0f,
                       (Color){100, 200, 255, 255});
        }

        // Desenha a barra (pêndulo) com a bússola
        float centerX = 200.0f;
        float centerY = 470.0f;
        float radius = 150.0f;

        DrawCompass(centerX, centerY, radius - 50);

        float px = centerX + L * sinf(angle) * radius;
        float py = centerY + L * cosf(angle) * radius;

        DrawLineEx((Vector2){centerX, centerY}, (Vector2){px, py}, 5.0f,
                   (Color){204, 85, 0, 255});
        DrawCircle((int)centerX, (int)centerY, 10, (Color){194, 178, 128, 255});
        DrawCircle((int)px, (int)py, 10, (Color){119, 140, 85, 255});

        DrawText(
            "Use SETAS CIMA/BAIXO para ajustar a pressao manual (+/-1 bar)", 10,
            640, 20, (Color){204, 85, 0, 255});
        DrawText("Pressione R para RESETAR a simulacao", 10, 670, 20,
                 (Color){204, 85, 0, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
