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

// Ajustes do PID (Inicial)
static float Kp = 1.0f;
static float Ki = 0.01f;
static float Kd = 5.0f; // Ajuste conforme necessário

// Constantes físicas
static const float g = 9.8f;  // Gravidade (m/s²)
static const float L = 1.0f;  // Comprimento da barra (m)
static const float m = 0.5f;  // Massa (kg)
static const float c = 0.02f; // Atrito viscoso (N·m·s)

// Parâmetros do Motor Hidráulico
static const float maxPressure = 400.0f;        // Pressão máxima (bar)
static const float controlPressure = 200.0f;    // Faixa de controle (0-200 bar)
static const float displacement = 0.157e-6f;    // Deslocamento em m³/rev
static const float torqueScalingFactor = 40.0f; // Fator de escala do torque

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
    .pressureSetpoint = 10.0f // Pressão inicial
};

static PIDState pidState = {0};

// Estados do sistema
static float angle = 0.0f;
static float omega = 0.0f;
static float Irot = 0.0f; // Momento de inércia

// Históricos
static float angleHistory[MAX_DATA_POINTS] = {0.0f};
static float errorHistory[MAX_DATA_POINTS] = {0.0f};
static float pressureHistory[MAX_DATA_POINTS] = {0.0f};
static int historyIndex = 0;

// Pressão filtrada para reduzir oscilações bruscas
static float pressureFiltered = 0.0f;
static float pressureFilterAlpha = 0.1f; // Valor pequeno = filtragem mais forte

// Flag para uso do RK4 ou Euler simples
static bool useRK4 = false; // Ajuste para false caso deseje Euler simples.

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
    DrawCircleLines((int)centerX, (int)centerY, (int)radius,
                    (Color){245, 222, 179, 255});

    const float angles[] = {0, (float)M_PI_2, (float)M_PI, (float)(3 * M_PI_2)};
    const char *labels[] = {"90°", "180°", "270°", "0°"};
    const float margin[] = {20, 16, 40, 20};
    const float dif[] = {10, 15, 10, 5};

    for (int i = 0; i < 4; i++)
    {
        float x = centerX + cosf(angles[i]) * (radius + margin[i]);
        float y = centerY - sinf(angles[i]) * (radius + margin[i]);
        DrawText(labels[i], (int)(x - dif[i]), (int)(y - 10), 20,
                 (Color){255, 200, 100, 255});
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
    pressureControlState.pressureSetpoint = 10.0f;
    pressureFiltered = 0.0f;

    for (int i = 0; i < MAX_DATA_POINTS; i++)
    {
        angleHistory[i] = 0.0f;
        errorHistory[i] = 0.0f;
        pressureHistory[i] = 0.0f;
    }

    historyIndex = 0;
}

// Função para calcular o torque total dado ângulo e velocidade angular
static float ComputeTotalTorque(float angleLocal, float omegaLocal,
                                float motorTorque, float externalTorque)
{
    float gravityTorque = -m * g * (L / 2.0f) * sinf(angleLocal);
    float frictionTorque = -c * omegaLocal;
    float totalTorque =
        motorTorque + gravityTorque + frictionTorque + externalTorque;
    return totalTorque;
}

// Integração Euler simples
static void IntegrateEuler(float *angleRef, float *omegaRef, float dt,
                           float motorTorque, float externalTorque)
{
    float totalTorque =
        ComputeTotalTorque(*angleRef, *omegaRef, motorTorque, externalTorque);
    float alpha = totalTorque / Irot;
    *omegaRef += alpha * dt;
    *angleRef += (*omegaRef) * dt;
}

// Integração Runge-Kutta de 4ª Ordem
// Sistema:
// d(angle)/dt = omega
// d(omega)/dt = totalTorque(angle,omega)/Irot
static void IntegrateRK4(float *angleRef, float *omegaRef, float dt,
                         float motorTorque, float externalTorque)
{
    float angle0 = *angleRef;
    float omega0 = *omegaRef;

    // k1
    float k1_angle_dot = omega0;
    float k1_omega_dot =
        ComputeTotalTorque(angle0, omega0, motorTorque, externalTorque) / Irot;

    float angle_half = angle0 + k1_angle_dot * (dt / 2.0f);
    float omega_half = omega0 + k1_omega_dot * (dt / 2.0f);

    // k2
    float k2_angle_dot = omega_half;
    float k2_omega_dot = ComputeTotalTorque(angle_half, omega_half, motorTorque,
                                            externalTorque) /
                         Irot;

    angle_half = angle0 + k2_angle_dot * (dt / 2.0f);
    omega_half = omega0 + k2_omega_dot * (dt / 2.0f);

    // k3
    float k3_angle_dot = omega_half;
    float k3_omega_dot = ComputeTotalTorque(angle_half, omega_half, motorTorque,
                                            externalTorque) /
                         Irot;

    float angle_end = angle0 + k3_angle_dot * dt;
    float omega_end = omega0 + k3_omega_dot * dt;

    // k4
    float k4_angle_dot = omega_end;
    float k4_omega_dot =
        ComputeTotalTorque(angle_end, omega_end, motorTorque, externalTorque) /
        Irot;

    float angle_new =
        angle0 + (dt / 6.0f) * (k1_angle_dot + 2.0f * k2_angle_dot +
                                2.0f * k3_angle_dot + k4_angle_dot);
    float omega_new =
        omega0 + (dt / 6.0f) * (k1_omega_dot + 2.0f * k2_omega_dot +
                                2.0f * k3_omega_dot + k4_omega_dot);

    *angleRef = angle_new;
    *omegaRef = omega_new;
}

int main(void)
{
    srand((unsigned int)time(NULL));

    const float angleSetpoint = 0.0f;
    Irot = (1.0f / 3.0f) * m * L * L;

    InitWindow(1240, 768, "Simulacao PID - Motor Hidraulico (Melhorias)");
    SetTargetFPS(SIM_FPS);

    float simTime = 0.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        simTime += dt;

        // Teclas de ajuste de pressão manual (+/-1 bar)
        if (IsKeyPressed(KEY_UP) &&
            pressureControlState.pressureSetpoint < controlPressure)
            pressureControlState.pressureSetpoint += 1.0f;
        if (IsKeyPressed(KEY_DOWN) &&
            pressureControlState.pressureSetpoint > 0.0f)
            pressureControlState.pressureSetpoint -= 1.0f;

        // Tecla de reset da simulação (R)
        if (IsKeyPressed(KEY_R))
        {
            simTime = 0.0f;
            ResetSimulation();
        }

        // Ajuste dinâmico de Kp, Ki, Kd
        if (IsKeyPressed(KEY_F2))
            Kp -= 10.0f;
        if (IsKeyPressed(KEY_F3))
            Kp += 10.0f;

        if (IsKeyPressed(KEY_F4))
            Ki -= 0.01f;
        if (IsKeyPressed(KEY_F5))
            Ki += 0.01f;

        if (IsKeyPressed(KEY_F6))
            Kd -= 10.0f;
        if (IsKeyPressed(KEY_F7))
            Kd += 10.0f;

        // Tecla para alternar RK4/Euler (opcional)
        if (IsKeyPressed(KEY_TAB))
            useRK4 = !useRK4;

        // Perturbação externa (P)
        float externalTorque = 0.0f;
        if (IsKeyPressed(KEY_P))
        {
            externalTorque = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f;
        }

        // Leitura do ângulo com ruído
        float measuredAngle = AddNoise(angle, angleNoiseAmplitude);

        // Controle PID
        float pidOutput = PIDControl(angleSetpoint, measuredAngle, dt);

        // Pressão final desejada
        float rawPressure = pressureControlState.pressureSetpoint + pidOutput;
        if (rawPressure > controlPressure)
            rawPressure = controlPressure;
        if (rawPressure < 0.0f)
            rawPressure = 0.0f;

        // Filtragem da pressão
        pressureFiltered = pressureFilterAlpha * pressureFiltered +
                           (1.0f - pressureFilterAlpha) * rawPressure;

        // Duty Cycle
        float dutyCycle =
            MapValue(pressureFiltered, 0.0f, controlPressure, 0.0f, 100.0f);

        // Pressão em Pa
        float pressure_pa = pressureFiltered * 1e5f;
        // Torque do motor
        float motorTorque =
            ((pressure_pa * displacement) / (2.0f * (float)M_PI)) *
            torqueScalingFactor;

        // Integração do sistema
        // Subdividindo o passo para maior estabilidade
        const int substeps = 20;
        float sub_dt = dt / substeps;

        for (int i = 0; i < substeps; i++)
        {
            if (useRK4)
                IntegrateRK4(&angle, &omega, sub_dt, motorTorque,
                             externalTorque);
            else
                IntegrateEuler(&angle, &omega, sub_dt, motorTorque,
                               externalTorque);
        }

        // Normaliza ângulo para [-π, π]
        angle = fmodf(angle + (float)M_PI, 2.0f * (float)M_PI) - (float)M_PI;

        // Atualiza históricos
        float error = angleSetpoint - measuredAngle;
        UpdateGraph(angleHistory, &historyIndex, angle);
        UpdateGraph(errorHistory, &historyIndex, error);
        UpdateGraph(pressureHistory, &historyIndex, pressureFiltered);

        BeginDrawing();
        ClearBackground((Color){43, 43, 43, 255});
        const Color textColor = (Color){245, 222, 179, 255};

        // Parâmetros dos gráficos
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

        // Desenha contornos dos gráficos
        DrawRectangleLines((int)graphX, (int)graphY, (int)graphWidth,
                           (int)graphHeight, textColor);
        DrawText("Angulo (rad)", (int)graphX, (int)(graphY - 30), 20,
                 textColor);

        DrawRectangleLines((int)errorGraphX, (int)errorGraphY,
                           (int)errorGraphWidth, (int)errorGraphHeight,
                           textColor);
        DrawText("Erro (rad)", (int)errorGraphX, (int)(errorGraphY - 30), 20,
                 textColor);

        DrawRectangleLines((int)pressureGraphX, (int)pressureGraphY,
                           (int)pressureGraphWidth, (int)pressureGraphHeight,
                           textColor);
        DrawText("Pressao (bar)", (int)pressureGraphX,
                 (int)(pressureGraphY - 30), 20, textColor);

        // Gráfico do Ângulo
        BeginScissorMode((int)graphX, (int)graphY, (int)graphWidth,
                         (int)graphHeight);

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

        EndScissorMode();

        // Gráfico do Erro
        BeginScissorMode((int)errorGraphX, (int)errorGraphY,
                         (int)errorGraphWidth, (int)errorGraphHeight);

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
                       (Color){150, 150, 250, 255});
        }

        EndScissorMode();

        // Gráfico da Pressão
        BeginScissorMode((int)pressureGraphX, (int)pressureGraphY,
                         (int)pressureGraphWidth, (int)pressureGraphHeight);

        for (int i = 1; i < MAX_DATA_POINTS; i++)
        {
            int prevIndex = (historyIndex + i - 1) % MAX_DATA_POINTS;
            int currIndex = (historyIndex + i) % MAX_DATA_POINTS;

            float val1 = pressureHistory[prevIndex];
            float val2 = pressureHistory[currIndex];

            float x1 = pressureGraphX +
                       (float)(i - 1) *
                           (pressureGraphWidth / (float)(MAX_DATA_POINTS - 1));
            float x2 =
                pressureGraphX +
                (float)i * (pressureGraphWidth / (float)(MAX_DATA_POINTS - 1));

            float yCenter = pressureGraphY + pressureGraphHeight / 2.0f;
            float y1 = yCenter - (val1 / 200.0f) * (pressureGraphHeight / 2.0f);
            float y2 = yCenter - (val2 / 200.0f) * (pressureGraphHeight / 2.0f);

            DrawLineEx((Vector2){x1, y1}, (Vector2){x2, y2}, 2.0f,
                       (Color){50, 200, 50, 255});
        }

        EndScissorMode();

        // Desenha a barra (pêndulo) com a bússola
        float centerX = 830.0f;
        float centerY = 190.0f;
        float radius = 150.0f;

        DrawCompass(centerX, centerY, radius - 50);

        float px = centerX + L * sinf(angle) * radius;
        float py = centerY + L * cosf(angle) * radius;

        DrawLineEx((Vector2){centerX, centerY}, (Vector2){px, py}, 5.0f,
                   (Color){204, 85, 0, 255});
        DrawCircle((int)centerX, (int)centerY, 10, (Color){255, 200, 100, 255});
        DrawCircle((int)px, (int)py, 10, (Color){255, 200, 100, 255});

        DrawText("Simulação PID - Motor Hidráulico", 10, 10, 20, textColor);
        DrawText(TextFormat("Tempo: %.2f s", simTime), 10, 40, 20, textColor);
        DrawText(TextFormat("Angulo Alvo: %.2f rad", angleSetpoint), 10, 70, 20,
                 textColor);
        DrawText(TextFormat("Angulo Medido: %.3f rad", measuredAngle), 10, 100,
                 20, textColor);
        DrawText(TextFormat("Pressão Filtrada (bar): %.1f", pressureFiltered),
                 10, 130, 20, textColor);
        DrawText(TextFormat("Duty Cycle: %.1f%%", dutyCycle), 10, 160, 20,
                 textColor);
        DrawText(TextFormat("Torque Motor (N·m): %.5f", motorTorque), 10, 190,
                 20, textColor);

        float gravityTorque = -m * g * (L / 2.0f) * sinf(angle);
        float frictionTorque = -c * omega;
        DrawText(TextFormat("Torque Gravidade (N·m): %.3f", gravityTorque), 10,
                 220, 20, textColor);
        DrawText(TextFormat("Torque Atrito (N·m): %.3f", frictionTorque), 10,
                 250, 20, textColor);
        DrawText(TextFormat("Torque Externo (N·m): %.3f", externalTorque), 10,
                 280, 20, textColor);

        // Exibe os ganhos do PID
        DrawText(TextFormat("Kp: %.1f | Ki: %.3f | Kd: %.1f", Kp, Ki, Kd), 10,
                 310, 20, textColor);

        // Instruções adicionais
        DrawText("Ajuste Kp (F2: -10 | F3: +10), Ki (F4: -0.01 | F5: +0.01), "
                 "Kd (F6: -10 | F7: +10)",
                 10, 610, 20, (Color){255, 200, 100, 255});
        DrawText("Use SETAS para CIMA/BAIXO para ajustar a pressão manual "
                 "(+/-1 bar)",
                 10, 640, 20, (Color){255, 200, 100, 255});
        DrawText("Pressione R para RESETAR a simulacao, P para perturbar", 10,
                 670, 20, (Color){255, 200, 100, 255});
        DrawText(TextFormat("Integração: %s (Pressione TAB para alternar)",
                            useRK4 ? "RK4" : "Euler"),
                 10, 740, 20, (Color){204, 85, 0, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
