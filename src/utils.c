#include <stdlib.h>
#include "utils.h"

// Update History for Graphs
void UpdateGraph(float *history, int *index, float value)
{
    history[*index] = value;
    *index = (*index + 1) % MAX_DATA_POINTS;
}

// Map Value from One Range to Another
float MapValue(float value, float in_min, float in_max, float out_min,
               float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Add Noise to a Signal
float AddNoise(float signal, float amplitude)
{
    float noise =
        ((float)rand() / (float)RAND_MAX) * 2.0f * amplitude - amplitude;
    return signal + noise;
}
